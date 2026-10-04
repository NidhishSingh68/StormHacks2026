// span_gpu.v - draws frames from command lists in DDR (minecraft/gpu.h)
//
// The CPU builds each frame as a list of commands: one per block-face
// polygon and 16-row band (its edges and its 1/z plane), plus colour fills
// and tints for sky, weather and HUD. This block fetches the list through
// the ARM's coherency port and does everything else, one pixel per clock:
//
//   fetch     Avalon bursts from the frame's command buffer (virtual
//             addresses, translated by a page table the CPU loads) into a
//             512-word FIFO; region r of buffer b starts at b*8M + r*256K
//   parse     commands are assembled while the previous one draws
//   rows      the row engine walks a polygon's edges: each row's first and
//             last pixel, 1/z and the view ray at its first pixel
//   draw      a 16-row band at a time, on chip: depth buffer (16 x 640 x
//             24-bit 1/z, invalidated per band by a 6-bit tag), colour
//             buffer (8 banks so 8 pixels can be read at once), and a
//             40-stage pixel pipeline: depth test and write; 1/z by three
//             pipelined dividers (the pixel and its neighbours to the right
//             and below); the world position on the face (perspective
//             correct); how big a pixel is on the face (grid lines fade out,
//             then the face goes plain); fog; block shade hash, highlight,
//             blending, ordered dither
//   flush     when the next band starts, the finished band is copied to
//             the framebuffer, but only after the display has scanned past
//             it, so a frame is never shown half drawn
//
// gpu_sw.c in the game is the C model of exactly this; sim/ compares the
// two on real frames.
//
// CSR (32-bit words, lightweight bridge 0xFF200100):
//   0 ID        "GPU1"
//   1 CTRL  W   [0] queue the frame in buffer [1]; [31] reset
//   2 STATUS R  [15:0] frames done, [16] busy, [17] a frame is queued
//   3 PT_INDEX  page table index for PT_DATA
//   4 PT_DATA W physical page number (address >> 12); the index advances
//   5 BEAM  R   [15:5] display scan count, [4:0] bands scanned this scan
//   6 FRAME R   clocks from start to end of the last frame
//   7 DRAW  R   of those, clocks not spent waiting for the display
//   8 LATE  R   bands copied out after the display had reached them

module span_gpu (
    input  wire        clk,
    input  wire        reset,

    // CSR slave
    input  wire [5:0]  csr_address,         // word address
    input  wire        csr_read,
    input  wire        csr_write,
    input  wire [31:0] csr_writedata,
    output reg  [31:0] csr_readdata,
    output reg         csr_readdatavalid,

    // command fetch: Avalon-MM burst read master (DDR physical addresses)
    output reg  [31:0] cmd_address,
    output reg         cmd_read,
    output wire [2:0]  cmd_burstcount,
    input  wire [63:0] cmd_readdata,
    input  wire        cmd_waitrequest,
    input  wire        cmd_readdatavalid,

    // framebuffer: Avalon-MM write master (byte addresses, 8 pixels a word)
    output wire [18:0] fb_address,
    output wire        fb_write,
    output wire [63:0] fb_writedata,
    output wire [7:0]  fb_byteenable,
    input  wire        fb_waitrequest,

    // display position, synchronous to clk
    input  wire [10:0] beam_scan,           // counts frames scanned
    input  wire [4:0]  beam_band            // bands fully scanned in this one (30 = vblank)
);

    localparam OP_NOP = 4'd0, OP_BAND = 4'd1, OP_COLORS = 4'd2, OP_FRAME = 4'd3,
               OP_FILL = 4'd4, OP_TINT = 4'd5, OP_INVERT = 4'd6, OP_MATS = 4'd7,
               OP_END = 4'd8, OP_POLY = 4'd9;
    // pixel kinds in the pipeline
    localparam PK_FILL = 3'd1, PK_TINT = 3'd2, PK_INVERT = 3'd3,
               PK_SOLID = 3'd4, PK_GLASS = 3'd6, PK_WATER = 3'd7;
    localparam [63:0] IZ_MIN = 64'd1 << 20;
    localparam [31:0] FOOT_MAX = 32'd19661;    // 0.3 in 16.16
    localparam NST = 39;                        // pipeline stages; colour written from the last
    localparam LAST_REGION = 5'd30;
    localparam FIFO_DEPTH = 512;
    localparam BURST = 4;

    integer i;

    // =========================================================================
    // control registers
    // =========================================================================

    reg         soft_reset;
    reg         pend, pend_buf;
    reg         busy, cur_buf;
    reg  [15:0] frames_done;
    reg  [11:0] pt_index;
    reg  [31:0] frame_cycles, draw_cycles, cyc_frame, cyc_wait;
    reg  [31:0] late_count;
    wire        kick_accept;                // the queued frame starts

    always @(posedge clk) begin
        csr_readdatavalid <= csr_read;
        case (csr_address)
            6'd0:    csr_readdata <= 32'h47505531;
            6'd2:    csr_readdata <= {14'd0, pend, busy, frames_done};
            6'd3:    csr_readdata <= {20'd0, pt_index};
            6'd5:    csr_readdata <= {16'd0, beam_scan, beam_band};
            6'd6:    csr_readdata <= frame_cycles;
            6'd7:    csr_readdata <= draw_cycles;
            6'd8:    csr_readdata <= late_count;
            default: csr_readdata <= 32'd0;
        endcase
    end

    wire        csr_ctrl   = csr_write && csr_address == 6'd1;
    wire        csr_pt_idx = csr_write && csr_address == 6'd3;
    wire        csr_pt_dat = csr_write && csr_address == 6'd4;

    always @(posedge clk) begin
        soft_reset <= reset || (csr_ctrl && csr_writedata[31]);
        if (reset || (csr_ctrl && csr_writedata[31])) begin
            pend <= 1'b0;
        end else if (csr_ctrl && csr_writedata[0]) begin
            pend     <= 1'b1;
            pend_buf <= csr_writedata[1];
        end else if (kick_accept) begin
            pend <= 1'b0;
        end
        if (csr_pt_idx)
            pt_index <= csr_writedata[11:0];
        else if (csr_pt_dat)
            pt_index <= pt_index + 12'd1;
    end

    // page table: virtual page {buffer, region, page in region} -> physical
    reg  [17:0] pt [0:4095];
    reg  [17:0] pt_q;

    always @(posedge clk)
        if (csr_pt_dat)
            pt[pt_index] <= csr_writedata[17:0];

    // =========================================================================
    // command FIFO (first word visible at the output)
    // =========================================================================

    reg  [63:0] fifo_mem [0:FIFO_DEPTH-1];
    reg  [8:0]  fifo_wp, fifo_rp;
    reg  [63:0] fifo_q;
    reg         fifo_qv;
    wire        fifo_pop;
    reg         fifo_clear;
    wire        fifo_push;
    wire        fifo_mem_empty = fifo_wp == fifo_rp;
    wire        fifo_load = !fifo_mem_empty && (!fifo_qv || fifo_pop);
    wire [9:0]  fifo_count = {1'b0, fifo_wp - fifo_rp} + {9'd0, fifo_qv};

    always @(posedge clk) begin
        if (fifo_push)
            fifo_mem[fifo_wp] <= cmd_readdata;
        if (fifo_load)
            fifo_q <= fifo_mem[fifo_rp];
    end

    always @(posedge clk) begin
        if (soft_reset || fifo_clear) begin
            fifo_wp <= 9'd0;
            fifo_rp <= 9'd0;
            fifo_qv <= 1'b0;
        end else begin
            if (fifo_push)
                fifo_wp <= fifo_wp + 9'd1;
            if (fifo_load)
                fifo_rp <= fifo_rp + 9'd1;
            if (fifo_load)
                fifo_qv <= 1'b1;
            else if (fifo_pop)
                fifo_qv <= 1'b0;
        end
    end

    // =========================================================================
    // fetcher
    // =========================================================================

    localparam F_IDLE = 3'd0, F_PT = 3'd1, F_LOOKUP = 3'd2, F_ISSUE = 3'd3, F_DRAIN = 3'd4;

    reg  [2:0]  f_state;
    reg  [4:0]  region;                     // region being fetched / parsed
    reg  [17:0] f_off;                      // byte offset in the region
    reg         f_done;                     // whole region requested
    reg  [9:0]  inflight;                   // beats asked for, not yet back
    reg         f_restart;                  // after draining: next region
    reg         f_go;                       // dispatcher: start a frame
    reg         f_req, f_req_next;          // dispatcher: stop (and go on with
                                            // the next region)
    wire        f_room = fifo_count + inflight + BURST <= FIFO_DEPTH - 1;

    assign cmd_burstcount = BURST;
    assign fifo_push = cmd_readdatavalid && f_state != F_DRAIN && f_state != F_IDLE;

    always @(posedge clk) begin
        if (cmd_read && !cmd_waitrequest)
            inflight <= inflight + BURST - (cmd_readdatavalid ? 10'd1 : 10'd0);
        else if (cmd_readdatavalid)
            inflight <= inflight - 10'd1;
        fifo_clear <= 1'b0;

        if (reset) begin
            cmd_read <= 1'b0;
            inflight <= 10'd0;
            f_state  <= F_IDLE;
        end else if (soft_reset) begin
            // reads already asked for still come back: drain them
            if (!(cmd_read && cmd_waitrequest))
                cmd_read <= 1'b0;
            f_state   <= F_DRAIN;
            f_restart <= 1'b0;
        end else begin
            case (f_state)
            F_IDLE:
                if (f_go) begin
                    region  <= 5'd0;
                    f_off   <= 18'd0;
                    f_done  <= 1'b0;
                    f_state <= F_PT;
                end
            F_PT:                           // page table lookup of f_off
                f_state <= F_LOOKUP;
            F_LOOKUP:
                if (f_req) begin
                    f_state   <= F_DRAIN;
                    f_restart <= f_req_next;
                end else if (!f_done && f_room) begin
                    cmd_address <= {2'b00, pt_q, f_off[11:0]};
                    cmd_read    <= 1'b1;
                    f_state     <= F_ISSUE;
                end
            F_ISSUE:
                if (!cmd_waitrequest) begin
                    cmd_read <= 1'b0;
                    f_off    <= f_off + 18'd32;
                    if (f_off == 18'h3FFE0)
                        f_done <= 1'b1;
                    f_state  <= F_PT;
                end
            default:                        // F_DRAIN
                if (cmd_read) begin
                    if (!cmd_waitrequest)
                        cmd_read <= 1'b0;   // finish a request caught by a reset
                end else if (inflight == 10'd0 && !cmd_readdatavalid) begin
                    fifo_clear <= 1'b1;
                    f_off      <= 18'd0;
                    f_done     <= 1'b0;
                    if (f_restart)
                        region <= region + 5'd1;
                    f_state    <= f_restart ? F_PT : F_IDLE;
                    f_restart  <= 1'b0;
                end
            endcase
        end
    end

    always @(posedge clk)
        pt_q <= pt[{cur_buf, region, f_off[17:12]}];

    // nothing more is coming for this region
    wire        f_exhausted = f_state == F_LOOKUP && f_done && inflight == 10'd0 &&
                              !cmd_readdatavalid && fifo_count == 10'd0;

    // =========================================================================
    // parser: assembles the next command while the current one draws
    // =========================================================================

    reg  [63:0] st_w [0:10];
    reg  [3:0]  st_need, st_have;
    reg         st_busy;                    // collecting words
    reg         st_valid;                   // complete command waiting
    wire        st_take;                    // dispatcher takes it
    wire [3:0]  st_op = st_w[0][3:0];
    reg  [5:0]  mats_left;                  // MATS words still to come
    reg  [4:0]  mats_i;
    reg  [47:0] mats [0:31];                // see-through colours by fog level

    function [3:0] cmd_words;
        input [63:0] w;
        case (w[3:0])
            OP_COLORS: cmd_words = 4'd3;
            OP_FRAME:  cmd_words = 4'd9;
            OP_FILL:   cmd_words = 4'd2;
            OP_TINT:   cmd_words = 4'd2;
            OP_POLY:   cmd_words = 4'd4 + {1'b0, w[7:5]};
            default:   cmd_words = 4'd1;
        endcase
    endfunction

    reg         p_hold;                     // after END: wait for the next region
    assign fifo_pop = fifo_qv && !p_hold && !soft_reset && (mats_left != 6'd0 || !st_valid);

    always @(posedge clk) begin
        if (soft_reset) begin
            st_busy   <= 1'b0;
            st_valid  <= 1'b0;
            mats_left <= 6'd0;
        end else if (mats_left != 6'd0) begin
            // MATS goes straight into its table (only in the prologue)
            if (fifo_pop) begin
                mats[mats_i] <= fifo_q[47:0];
                mats_i       <= mats_i + 5'd1;
                mats_left    <= mats_left - 6'd1;
            end
        end else if (st_valid) begin
            if (st_take)
                st_valid <= 1'b0;
        end else if (fifo_pop) begin
            if (!st_busy) begin
                if (fifo_q[3:0] == OP_MATS) begin
                    mats_left <= 6'd32;
                    mats_i    <= 5'd0;
                end else begin
                    st_w[0] <= fifo_q;
                    st_have <= 4'd1;
                    st_need <= cmd_words(fifo_q);
                    if (cmd_words(fifo_q) == 4'd1)
                        st_valid <= 1'b1;
                    else
                        st_busy <= 1'b1;
                end
            end else begin
                st_w[st_have] <= fifo_q;
                st_have <= st_have + 4'd1;
                if (st_have + 4'd1 == st_need) begin
                    st_busy  <= 1'b0;
                    st_valid <= 1'b1;
                end
            end
        end else if (f_exhausted && !p_hold) begin
            // ran off the end of the region without an END: end it here
            st_w[0]  <= {60'd0, OP_END};
            st_busy  <= 1'b0;
            st_valid <= 1'b1;
        end
    end

    wire        r_ready;                    // row engine (below) can take a polygon

    // =========================================================================
    // frame constants (FRAME)
    // =========================================================================

    reg  [31:0] fr_n0 [0:2], fr_nx [0:2], fr_ny [0:2], fr_eye [0:2];
    reg  [31:0] fr_fog_start, fr_fog_scale;
    reg  [15:0] fr_sel [0:2];
    reg  [7:0]  fr_alpha_water, fr_alpha_glass;

    always @(posedge clk) begin
        if (st_take && st_op == OP_FRAME) begin
            fr_n0[0]  <= st_w[1][31:0];  fr_nx[0]  <= st_w[1][63:32];
            fr_ny[0]  <= st_w[2][31:0];  fr_n0[1]  <= st_w[2][63:32];
            fr_nx[1]  <= st_w[3][31:0];  fr_ny[1]  <= st_w[3][63:32];
            fr_n0[2]  <= st_w[4][31:0];  fr_nx[2]  <= st_w[4][63:32];
            fr_ny[2]  <= st_w[5][31:0];  fr_eye[0] <= st_w[5][63:32];
            fr_eye[1] <= st_w[6][31:0];  fr_eye[2] <= st_w[6][63:32];
            fr_fog_start <= st_w[7][31:0];
            fr_fog_scale <= st_w[7][63:32];
            fr_sel[0] <= st_w[8][15:0];
            fr_sel[1] <= st_w[8][31:16];
            fr_sel[2] <= st_w[8][47:32];
            fr_alpha_water <= st_w[8][55:48];
            fr_alpha_glass <= st_w[8][63:56];
        end
    end

    // the face's two in-plane axes: horizontal faces (x, z), x faces (y, z),
    // z faces (x, y)
    function [1:0] axis_a;
        input [1:0] axis;
        axis_a = axis == 2'd0 ? 2'd1 : 2'd0;
    endfunction
    function [1:0] axis_b;
        input [1:0] axis;
        axis_b = axis == 2'd2 ? 2'd1 : 2'd2;
    endfunction

    // =========================================================================
    // colour sets: 2048 x 5 RGB888 tones
    // =========================================================================

    reg  [119:0] cset [0:2047];
    reg  [119:0] cset_q;
    reg  [10:0]  cset_ra;
    wire         cset_we = st_valid && st_take && st_op == OP_COLORS;

    always @(posedge clk) begin
        if (cset_we)
            cset[st_w[0][45:35]] <= {st_w[2][55:0], st_w[1]};
        cset_q <= cset[cset_ra];
    end

    // =========================================================================
    // band memories
    // =========================================================================

    // depth: {tag, z}; an entry is valid only if its tag is the band's
    reg  [29:0] zmem [0:10239];
    reg  [29:0] z_q;
    reg  [13:0] z_ra, z_wa;
    reg  [29:0] z_wd;
    reg         z_we;

    always @(posedge clk) begin
        if (z_we)
            zmem[z_wa] <= z_wd;
        z_q <= zmem[z_ra];
    end

    // colour: 8 banks, bank = x & 7
    wire [63:0] cb_qv;                      // all 8 banks at cb_ra
    reg  [10:0] cb_ra;
    reg  [13:0] cb_wa;
    reg  [7:0]  cb_wd;
    reg         cb_we;

    genvar g;
    generate
        for (g = 0; g < 8; g = g + 1) begin : banks
            reg [7:0] mem [0:1279];
            reg [7:0] q;
            always @(posedge clk) begin
                if (cb_we && cb_wa[2:0] == g)
                    mem[cb_wa[13:3]] <= cb_wd;
                q <= mem[cb_ra];
            end
            assign cb_qv[8*g +: 8] = q;
        end
    endgenerate

    // =========================================================================
    // dispatcher: frame sequencing, bands, regions
    // =========================================================================

    localparam D_IDLE = 3'd0, D_RUN = 3'd1, D_DRAIN = 3'd2, D_FLUSH_WAIT = 3'd3,
               D_FLUSH = 3'd4, D_CLEAR = 3'd5, D_DONE = 3'd6;

    reg  [2:0]  d_state;
    reg         band_open;
    reg  [4:0]  band;                       // band being drawn
    reg  [5:0]  zseq;                       // depth tag of this band
    reg  [13:0] clear_a;
    reg         scan_valid, last_valid;
    reg  [10:0] frame_scan, last_scan;
    reg  [4:0]  next_band;
    reg         d_band;                     // draining for a BAND (else the final END)

    // flush: copy the band's 1280 words to the framebuffer
    reg  [10:0] fl_w;
    reg         fl_prime;

    wire        pipe_empty;
    wire        eng_ready;                  // the engine can take a command now
    wire        rows_idle;                  // the row engine has nothing left
    wire        is_pixel_op = st_op == OP_FILL || st_op == OP_TINT || st_op == OP_INVERT;

    assign st_take = st_valid && d_state == D_RUN &&
                     (is_pixel_op ? eng_ready && rows_idle :
                      st_op == OP_POLY ? r_ready : 1'b1);
    assign kick_accept = d_state == D_IDLE && pend && f_state == F_IDLE && !soft_reset;

    // has the display scanned past the band, in this frame's scan?
    wire        first_scan_ok = !last_valid || beam_scan != last_scan;
    wire        beam_past = scan_valid ? (beam_scan != frame_scan || beam_band > band)
                                       : (first_scan_ok && beam_band > band);
    wire        beam_late = scan_valid && beam_scan != frame_scan && beam_band >= band;

    wire [18:0] fb_band_base = {1'b0, band, 13'd0} + {3'd0, band, 11'd0};   // * 10240
    assign fb_address    = fb_band_base + {5'd0, fl_w, 3'd0};
    assign fb_write      = d_state == D_FLUSH && !fl_prime;
    assign fb_writedata  = cb_qv;
    assign fb_byteenable = 8'hFF;
    wire        fl_accept = fb_write && !fb_waitrequest;

    always @(posedge clk) begin
        f_go <= 1'b0;
        if (f_state == F_LOOKUP)
            f_req <= 1'b0;
        if (fifo_clear)
            p_hold <= 1'b0;
        if (busy) begin
            cyc_frame <= cyc_frame + 32'd1;
            if (d_state == D_FLUSH_WAIT && !beam_past)
                cyc_wait <= cyc_wait + 32'd1;
        end

        if (soft_reset) begin
            d_state     <= D_IDLE;
            busy        <= 1'b0;
            band_open   <= 1'b0;
            zseq        <= 6'd63;           // first band clears the depth memory
            last_valid  <= 1'b0;
            frames_done <= 16'd0;
            late_count  <= 32'd0;
            f_req       <= 1'b0;
            p_hold      <= 1'b0;
        end else begin
            case (d_state)
            D_IDLE:
                if (kick_accept) begin
                    busy       <= 1'b1;
                    cur_buf    <= pend_buf;
                    band_open  <= 1'b0;
                    scan_valid <= 1'b0;
                    cyc_frame  <= 32'd0;
                    cyc_wait   <= 32'd0;
                    f_go       <= 1'b1;
                    d_state    <= D_RUN;
                end

            D_RUN:
                if (st_take) begin
                    if (st_op == OP_BAND) begin
                        next_band <= st_w[0][10:6];
                        d_band    <= 1'b1;
                        d_state   <= D_DRAIN;
                    end else if (st_op == OP_END) begin
                        // stop parsing this region; the fetcher drops what
                        // it read past the END
                        p_hold     <= 1'b1;
                        f_req      <= 1'b1;
                        f_req_next <= region != LAST_REGION;
                        d_band     <= 1'b0;
                        if (region == LAST_REGION)
                            d_state <= D_DRAIN;
                    end
                end

            D_DRAIN:                        // let every pixel land first
                if (pipe_empty)
                    d_state <= band_open ? D_FLUSH_WAIT : d_band ? D_CLEAR : D_DONE;

            D_FLUSH_WAIT:
                if (beam_past) begin
                    if (beam_late)
                        late_count <= late_count + 32'd1;
                    if (!scan_valid) begin
                        scan_valid <= 1'b1;
                        frame_scan <= beam_scan;
                    end
                    fl_w     <= 11'd0;
                    fl_prime <= 1'b1;
                    d_state  <= D_FLUSH;
                end

            D_FLUSH: begin
                fl_prime <= 1'b0;
                if (fl_accept) begin
                    fl_w <= fl_w + 11'd1;
                    if (fl_w == 11'd1279) begin
                        band_open <= 1'b0;
                        d_state   <= d_band ? D_CLEAR : D_DONE;
                    end
                end
            end

            D_CLEAR:                        // start the new band
                if (zseq != 6'd63 || clear_a == 14'd10239) begin
                    zseq      <= zseq == 6'd63 ? 6'd1 : zseq + 6'd1;
                    band      <= next_band;
                    band_open <= 1'b1;
                    d_state   <= D_RUN;
                end

            default:                        // D_DONE
                if (f_state == F_IDLE) begin
                    frames_done  <= frames_done + 16'd1;
                    frame_cycles <= cyc_frame;
                    draw_cycles  <= cyc_frame - cyc_wait;
                    if (scan_valid) begin
                        last_valid <= 1'b1;
                        last_scan  <= frame_scan;
                    end
                    busy         <= 1'b0;
                    d_state      <= D_IDLE;
                end
            endcase
        end
    end

    always @(posedge clk) begin
        if (d_state == D_CLEAR && zseq == 6'd63)
            clear_a <= clear_a + 14'd1;
        else
            clear_a <= 14'd0;
    end

    // =========================================================================
    // row engine: a polygon's rows, one per clock, through a 7-stage pipeline
    // =========================================================================
    //
    //   RA  each edge's x at the row (stepped from its first row) and whether it
    //       covers the row; 1/z at pixel 0 of the row; the row's y
    //   RB..RD  min / max over the edges, a tree of pairs
    //   RE  first and last pixel (rounded, clipped)
    //   RF  multiplies: 1/z and view ray at the first pixel
    //   RG  sums -> row FIFO, read by the pixel issuer
    //
    // A new polygon is loaded once the previous one's rows are all through.

    reg  [4:0]  r_rows, r_r;
    reg         r_gen;                      // rows still to start
    reg  [8:0]  r_y0;
    reg  [63:0] r_iz0, r_dizdx, r_dizdy;
    reg  [31:0] r_cx [0:6], r_dx [0:6];     // edge x at row r_r (12.12), step
    reg  [4:0]  r_r0 [0:6], r_r1 [0:6];
    reg  [2:0]  r_n;
    reg  [1:0]  r_axis;
    reg  [2:0]  r_pk;
    reg         r_flag;
    reg  [10:0] r_set;
    reg  [31:0] r_n0a, r_n0b, r_nxa, r_nxb, r_nya, r_nyb;

    // pipeline valid bits and row data
    reg         ra_v, rb_v, rc_v, rd_v, re_v, rf_v;
    reg  signed [31:0] ra_x [0:6];
    reg         ra_act [0:6];
    reg  [8:0]  ra_y, rb_y, rc_y, rd_y, re_y, rf_y;
    reg  [63:0] ra_iz, rb_iz, rc_iz, rd_iz, re_iz, rf_iz;
    reg  signed [31:0] rb_lo [0:3], rb_hi [0:3], rc_lo [0:1], rc_hi [0:1], rd_lo, rd_hi;
    reg         rb_any [0:3], rc_any [0:1], rd_any;
    reg  [9:0]  re_x0, re_x1, rf_x0, rf_x1;
    reg         re_ok, rf_ok;
    reg  [63:0] rf_miz;
    reg  [31:0] rf_mna, rf_mnb, rf_yna, rf_ynb;

    // row FIFO, 4 entries
    reg  [9:0]  q_x0 [0:3], q_x1 [0:3];
    reg  [8:0]  q_y [0:3];
    reg  [63:0] q_iz [0:3];
    reg  [31:0] q_na [0:3], q_nb [0:3];
    reg  [63:0] q_dizdx [0:3], q_dizdy [0:3];
    reg  [2:0]  q_pk [0:3];
    reg         q_flag [0:3];
    reg  [1:0]  q_axis [0:3];
    reg  [10:0] q_set [0:3];
    reg  [1:0]  q_wp, q_rp;
    reg  [2:0]  q_count;
    wire        q_valid = q_count != 3'd0;
    wire        q_take;
    wire        q_push = rf_v && rf_ok;
    wire [2:0]  r_inflight = {2'd0, ra_v} + {2'd0, rb_v} + {2'd0, rc_v} + {2'd0, rd_v} +
                             {2'd0, re_v} + {2'd0, rf_v};
    wire        r_start = r_gen && q_count + r_inflight < 3'd4 && !soft_reset;
    wire        r_pipe_busy = ra_v || rb_v || rc_v || rd_v || re_v || rf_v;
    assign      r_ready = !r_gen && !r_pipe_busy;   // can load a polygon
    assign rows_idle = r_ready && !q_valid;

    function signed [31:0] smin;
        input signed [31:0] p, q;
        smin = p < q ? p : q;
    endfunction
    function signed [31:0] smax;
        input signed [31:0] p, q;
        smax = p > q ? p : q;
    endfunction

    localparam signed [31:0] BIG = 32'sh7FFFFFFF, SMALL = -32'sh7FFFFFFF - 32'sd1;
    wire signed [31:0] rd_x0w = (rd_lo + 32'sd2047) >>> 12;
    wire signed [31:0] rd_x1w = (rd_hi + 32'sd2047) >>> 12;
    wire signed [31:0] rd_x0c = rd_x0w < 0 ? 32'sd0 : rd_x0w;
    wire signed [31:0] rd_x1c = rd_x1w > 640 ? 32'sd640 : rd_x1w;

    always @(posedge clk) begin
        // ---- load a polygon ----
        if (soft_reset) begin
            r_gen <= 1'b0;
        end else if (st_take && st_op == OP_POLY) begin
            r_rows  <= st_w[0][20:16];
            r_r     <= 5'd0;
            r_gen   <= st_w[0][20:16] != 5'd0;
            r_y0    <= st_w[0][34:26];
            r_iz0   <= st_w[1];
            r_dizdx <= st_w[2];
            r_dizdy <= st_w[3];
            r_n     <= st_w[0][7:5];
            r_axis  <= st_w[0][9:8];
            r_pk    <= {1'b1, st_w[0][11:10]};
            r_flag  <= st_w[0][4];
            r_set   <= st_w[0][45:35];
            r_n0a   <= fr_n0[axis_a(st_w[0][9:8])];
            r_n0b   <= fr_n0[axis_b(st_w[0][9:8])];
            r_nxa   <= fr_nx[axis_a(st_w[0][9:8])];
            r_nxb   <= fr_nx[axis_b(st_w[0][9:8])];
            r_nya   <= fr_ny[axis_a(st_w[0][9:8])];
            r_nyb   <= fr_ny[axis_b(st_w[0][9:8])];
            for (i = 0; i < 7; i = i + 1) begin
                r_cx[i] <= {{8{st_w[4+i][23]}}, st_w[4+i][23:0]};
                r_dx[i] <= {{8{st_w[4+i][47]}}, st_w[4+i][47:24]};
                r_r0[i] <= st_w[4+i][52:48];
                r_r1[i] <= st_w[4+i][57:53];
            end
        end else if (r_start) begin
            r_r <= r_r + 5'd1;
            if (r_r + 5'd1 == r_rows)
                r_gen <= 1'b0;
            for (i = 0; i < 7; i = i + 1)       // x at the next row
                if (r_r >= r_r0[i])
                    r_cx[i] <= r_cx[i] + r_dx[i];
        end

        // ---- RA ----
        ra_v <= r_start;
        for (i = 0; i < 7; i = i + 1) begin
            ra_act[i] <= i < r_n && r_r >= r_r0[i] && r_r < r_r1[i];
            ra_x[i]   <= r_cx[i];
        end
        ra_y  <= r_y0 + {4'd0, r_r};
        ra_iz <= r_iz0 + r_dizdy * {59'd0, r_r};

        // ---- RB ----
        rb_v <= ra_v && !soft_reset;
        for (i = 0; i < 4; i = i + 1) begin
            if (2*i+1 < 7) begin
                rb_lo[i]  <= smin(ra_act[2*i] ? ra_x[2*i] : BIG, ra_act[2*i+1] ? ra_x[2*i+1] : BIG);
                rb_hi[i]  <= smax(ra_act[2*i] ? ra_x[2*i] : SMALL, ra_act[2*i+1] ? ra_x[2*i+1] : SMALL);
                rb_any[i] <= ra_act[2*i] || ra_act[2*i+1];
            end else begin
                rb_lo[i]  <= ra_act[2*i] ? ra_x[2*i] : BIG;
                rb_hi[i]  <= ra_act[2*i] ? ra_x[2*i] : SMALL;
                rb_any[i] <= ra_act[2*i];
            end
        end
        rb_y <= ra_y; rb_iz <= ra_iz;

        // ---- RC ----
        rc_v <= rb_v && !soft_reset;
        for (i = 0; i < 2; i = i + 1) begin
            rc_lo[i]  <= smin(rb_lo[2*i], rb_lo[2*i+1]);
            rc_hi[i]  <= smax(rb_hi[2*i], rb_hi[2*i+1]);
            rc_any[i] <= rb_any[2*i] || rb_any[2*i+1];
        end
        rc_y <= rb_y; rc_iz <= rb_iz;

        // ---- RD ----
        rd_v   <= rc_v && !soft_reset;
        rd_lo  <= smin(rc_lo[0], rc_lo[1]);
        rd_hi  <= smax(rc_hi[0], rc_hi[1]);
        rd_any <= rc_any[0] || rc_any[1];
        rd_y <= rc_y; rd_iz <= rc_iz;

        // ---- RE ----
        re_v  <= rd_v && !soft_reset;
        re_x0 <= rd_x0c[9:0];
        re_x1 <= rd_x1c[9:0];
        re_ok <= rd_any && rd_x0c < rd_x1c;
        re_y <= rd_y; re_iz <= rd_iz;

        // ---- RF ----
        rf_v   <= re_v && !soft_reset;
        rf_x0  <= re_x0;
        rf_x1  <= re_x1;
        rf_ok  <= re_ok;
        rf_miz <= r_dizdx * {54'd0, re_x0};
        rf_mna <= r_nxa * {22'd0, re_x0};
        rf_mnb <= r_nxb * {22'd0, re_x0};
        rf_yna <= r_nya * {23'd0, re_y};
        rf_ynb <= r_nyb * {23'd0, re_y};
        rf_y <= re_y; rf_iz <= re_iz;

        // ---- RG: into the FIFO ----
        if (q_push) begin
            q_x0[q_wp] <= rf_x0;
            q_x1[q_wp] <= rf_x1;
            q_y[q_wp]  <= rf_y;
            q_iz[q_wp] <= rf_iz + rf_miz;
            q_na[q_wp] <= r_n0a + rf_yna + rf_mna;
            q_nb[q_wp] <= r_n0b + rf_ynb + rf_mnb;
            q_dizdx[q_wp] <= r_dizdx;
            q_dizdy[q_wp] <= r_dizdy;
            q_pk[q_wp]    <= r_pk;
            q_flag[q_wp]  <= r_flag;
            q_axis[q_wp]  <= r_axis;
            q_set[q_wp]   <= r_set;
        end
        if (soft_reset) begin
            q_wp <= 2'd0;
            q_rp <= 2'd0;
            q_count <= 3'd0;
        end else begin
            if (q_push)
                q_wp <= q_wp + 2'd1;
            if (q_take)
                q_rp <= q_rp + 2'd1;
            q_count <= q_count + (q_push ? 3'd1 : 3'd0) - (q_take ? 3'd1 : 3'd0);
        end
    end

    // =========================================================================
    // pixel issuer
    // =========================================================================

    reg         a_valid;
    reg  [2:0]  a_pk;
    reg         a_flag;
    reg  [1:0]  a_axis;
    reg  [10:0] a_set;
    reg  [9:0]  a_x, a_x1;
    reg  [1:0]  a_y;                        // y & 3, for the dither
    reg  [13:0] a_addr;                     // (y & 15) * 640 + x
    reg  [63:0] a_iz, a_dizdx, a_dizdy;
    reg  [31:0] a_na, a_nb;
    reg  [23:0] a_c24;
    reg  [7:0]  a_alpha;
    reg  [31:0] a_fiz;                      // FILL depth, 1/z * 2^22

    // pipeline context, stages 1..NST
    reg         c_v     [1:NST];
    reg  [2:0]  c_pk    [1:NST];
    reg  [13:0] c_addr  [1:NST];
    reg  [3:0]  c_xy    [1:NST];            // {y & 3, x & 3}
    reg         c_flag  [1:NST];
    reg  [1:0]  c_axis  [1:NST];
    reg  [10:0] c_set   [1:NST];
    reg  [23:0] c_c24   [1:NST];
    reg  [7:0]  c_alpha [1:NST];
    reg         c_pass  [3:NST];
    reg  [7:0]  c_dst   [2:NST];

    // a blend or invert must not read a pixel still on its way to the
    // colour memory
    // Worked out a clock ahead, for the address the issuer will be at.
    reg         hazard;
    reg         hz_issue, hz_hold;
    always @(*) begin
        hz_issue = 1'b0;                    // if this pixel issues: next pixel
        hz_hold  = 1'b0;                    // if it does not: this one again
        for (i = 1; i < NST; i = i + 1) begin
            if (c_v[i] && c_addr[i] == a_addr + 14'd1) hz_issue = 1'b1;
            if (c_v[i] && c_addr[i] == a_addr)         hz_hold  = 1'b1;
        end
    end

    wire        a_reads = a_pk == PK_TINT || a_pk == PK_INVERT || a_pk == PK_GLASS ||
                          a_pk == PK_WATER;
    wire        issue = a_valid && !(a_reads && hazard);
    wire        a_last = a_x + 10'd1 == a_x1;
    wire        a_free = !a_valid || (issue && a_last);

    always @(posedge clk)
        // a newly loaded run waits a clock to be checked; then checked ahead
        hazard <= (q_take || load_cmd) ? 1'b1 : issue ? hz_issue : hz_hold;
    assign      eng_ready = a_free && !q_valid;
    assign      q_take = q_valid && a_free;
    wire        load_cmd = st_take && is_pixel_op;

    reg         any_stage;
    always @(*) begin
        any_stage = 1'b0;
        for (i = 1; i <= NST; i = i + 1)
            any_stage = any_stage | c_v[i];
    end
    assign pipe_empty = !a_valid && !any_stage && rows_idle;

    wire [9:0]  ld_x0 = st_w[0][15:6];
    wire [9:0]  ld_x1r = st_w[0][25:16];
    wire [9:0]  ld_x1 = ld_x1r > 10'd640 ? 10'd640 : ld_x1r;
    wire [3:0]  ld_row = st_w[0][29:26];
    wire [3:0]  q_row = q_y[q_rp][3:0];

    always @(posedge clk) begin
        if (soft_reset) begin
            a_valid <= 1'b0;
        end else if (q_take) begin
            a_valid <= 1'b1;
            a_pk    <= q_pk[q_rp];
            a_flag  <= q_flag[q_rp];
            a_axis  <= q_axis[q_rp];
            a_set   <= q_set[q_rp];
            a_x     <= q_x0[q_rp];
            a_x1    <= q_x1[q_rp];
            a_y     <= q_y[q_rp][1:0];
            a_addr  <= {q_row, 9'd0} + {2'd0, q_row, 7'd0} + {4'd0, q_x0[q_rp]};
            a_iz    <= q_iz[q_rp];
            a_dizdx <= q_dizdx[q_rp];
            a_dizdy <= q_dizdy[q_rp];
            a_na    <= q_na[q_rp];
            a_nb    <= q_nb[q_rp];
        end else if (load_cmd) begin
            a_valid <= ld_x0 < ld_x1;
            a_pk    <= st_op == OP_FILL ? PK_FILL : st_op == OP_TINT ? PK_TINT : PK_INVERT;
            a_flag  <= st_w[0][4];
            a_x     <= ld_x0;
            a_x1    <= ld_x1;
            a_y     <= st_w[0][27:26];
            a_addr  <= {ld_row, 9'd0} + {2'd0, ld_row, 7'd0} + {4'd0, ld_x0};
            a_c24   <= st_w[1][23:0];
            a_alpha <= st_w[1][31:24];
            a_fiz   <= st_w[1][63:32];
            a_iz    <= 64'd0;
            a_dizdx <= 64'd0;
            a_dizdy <= 64'd0;
            a_na    <= 32'd0;
            a_nb    <= 32'd0;
        end else if (issue) begin
            if (a_last)
                a_valid <= 1'b0;
            a_x    <= a_x + 10'd1;
            a_addr <= a_addr + 14'd1;
            a_iz   <= a_iz + a_dizdx;
            a_na   <= a_na + fr_nx[axis_a(a_axis)];
            a_nb   <= a_nb + fr_nx[axis_b(a_axis)];
        end
    end

    // =========================================================================
    // pixel pipeline
    // =========================================================================

    // ---- 0 -> 1: depth; the three sample points; read the band memories ----
    wire        a_poly = a_pk[2];
    wire        a_izok = !a_iz[63] && a_iz >= IZ_MIN;
    wire [23:0] a_zpoly = a_iz[63:45] != 19'd0 ? 24'hFFFFFF : a_iz[44:21];
    wire [23:0] a_zfill = a_fiz[31] ? 24'd0 : (|a_fiz[30:27] ? 24'hFFFFFF : a_fiz[26:3]);
    wire [1:0]  a_ka = axis_a(a_axis), a_kb = axis_b(a_axis);

    reg  [23:0] s1_z;
    reg         s1_ztest, s1_zok, s1_izok;
    reg  [63:0] s1_iz [0:2];
    reg  [31:0] s1_na [0:2], s1_nb [0:2];

    always @(*) begin
        z_ra  = a_addr;
        cb_ra = (d_state == D_FLUSH) ? (fl_accept ? fl_w + 11'd1 : fl_w) : a_addr[13:3];
    end

    // ---- 1 -> 2: the depth memory's output registered; normalise the
    // samples. 2: depth test and write. Pixels one and two clocks ahead
    // have written depth that the memory did not return yet: forward it.
    // The write itself is registered and goes in from stage 3.
    reg  [23:0] s2_z, s3_z, s4_z, s5_z;
    reg  [29:0] s2_zq;
    reg         s2_ztest, s2_zok, s2_izok, s3_zw, s4_zw, s5_zw;
    wire [23:0] stored2 = (s3_zw && c_addr[3] == c_addr[2]) ? s3_z :
                          (s4_zw && c_addr[4] == c_addr[2]) ? s4_z :
                          (s5_zw && c_addr[5] == c_addr[2]) ? s5_z :
                          (s2_zq[29:24] == zseq ? s2_zq[23:0] : 24'd0);
    wire        pass2 = s2_izok && (!s2_ztest || s2_z > stored2);
    wire        zw2 = c_v[2] && s2_zok && pass2;

    always @(*) begin
        if (d_state == D_CLEAR) begin
            z_we = 1'b1;
            z_wa = clear_a;
            z_wd = 30'd0;
        end else begin
            z_we = s3_zw;
            z_wa = c_addr[3];
            z_wd = {zseq, s3_z};
        end
    end

    function [45:0] clamp_iz;
        input [63:0] v;
        if (v[63] || v < IZ_MIN)           clamp_iz = 46'd1 << 20;
        else if (v[62:46] != 17'd0)        clamp_iz = {46{1'b1}};
        else                               clamp_iz = v[45:0];
    endfunction

    function [4:0] clz48;                   // leading zeros of a 48-bit value < 2^46
        input [45:0] v;
        integer k;
        begin
            clz48 = 5'd27;
            for (k = 20; k <= 45; k = k + 1)
                if (v[k]) clz48 = 47 - k;
        end
    endfunction

    reg  [45:0] s2_iz [0:2];
    reg  [4:0]  s2_lz [0:2];
    reg  [31:0] s2_na [0:2], s2_nb [0:2];

    // ---- 2 -> 3: 25-bit mantissas; then 26 divider stages: q = 2^49 / m ----
    localparam DIV0 = 3, DIVN = 26;        // stages 3 .. 28 divide, 29 has q
    reg  [24:0] dv_d  [0:2][DIV0:DIV0+DIVN];
    reg  [25:0] dv_r  [0:2][DIV0:DIV0+DIVN];
    reg  [25:0] dv_q  [0:2][DIV0:DIV0+DIVN];
    reg  [4:0]  dv_lz [0:2][DIV0:DIV0+DIVN];
    reg  [31:0] dv_na [0:2][DIV0:DIV0+DIVN];
    reg  [31:0] dv_nb [0:2][DIV0:DIV0+DIVN];

    // ---- 29 -> 30: n * q ----
    reg  signed [58:0] s30_pa [0:2], s30_pb [0:2];
    reg  [4:0]  s30_lz [0:2];
    reg  [25:0] s30_q;

    // ---- 30 -> 31: shift to 16.16; z ----
    reg  [31:0] s31_ra [0:2], s31_rb [0:2];
    reg  [37:0] s31_z;

    // ---- 31 -> 32: add the eye; z - fog start ----
    reg  [31:0] s32_a [0:2], s32_b [0:2];
    reg  signed [38:0] s32_zd;

    // ---- 32 -> 33: footprint; fog product ----
    reg  [31:0] s33_a, s33_b, s33_fa, s33_fb;
    reg  signed [70:0] s33_fp;

    function [31:0] absdiff;
        input [31:0] p, q;
        reg signed [32:0] d;
        begin
            d = {p[31], p} - {q[31], q};
            if (d[32]) d = -d;
            absdiff = d[32] ? 32'hFFFFFFFF : d[31:0];
        end
    endfunction

    // ---- 33 -> 34: fog, colour set, tone decisions, see-through colour ----
    reg         s34_line, s34_sel, s34_tex, s34_blend;
    reg  [15:0] s34_bu, s34_bv;
    reg  [23:0] s34_mat;
    reg  [7:0]  s34_alpha;
    wire signed [70:0] fog_v = s33_fp >>> 40;
    wire [4:0]  fog33 = fog_v < 0 ? 5'd0 : fog_v > 31 ? 5'd31 : fog_v[4:0];
    wire        tex33 = s33_fa <= FOOT_MAX && s33_fb <= FOOT_MAX;
    wire        thin33 = s33_fa < FOOT_MAX && s33_fb < FOOT_MAX;
    wire        lin33 = {16'd0, s33_a[15:0]} < s33_fa || {16'd0, s33_b[15:0]} < s33_fb;
    wire [1:0]  c33_ka = axis_a(c_axis[33]), c33_kb = axis_b(c_axis[33]);
    wire        sel33 = c_flag[33] && s33_a[31:16] == fr_sel[c33_ka] &&
                        s33_b[31:16] == fr_sel[c33_kb];
    reg  [4:0]  s34_fog;
    wire [47:0] mats34 = mats[s34_fog];

    // ---- 34 .. 37: block shade hash; blend ----
    reg  [31:0] s35_m1, s35_m2;
    reg  [8:0]  s35_dr, s35_dg, s35_db;
    reg  [31:0] s36_h;
    reg  [16:0] s36_pr, s36_pg, s36_pb;
    reg  [31:0] s37_h;
    reg  [23:0] s37_blend;
    wire [31:0] h35 = s35_m1 + s35_m2;
    wire [16:0] mr36 = c_c24[36][23:16] * c_alpha[36];
    wire [16:0] mg36 = c_c24[36][15:8]  * c_alpha[36];
    wire [16:0] mb36 = c_c24[36][7:0]   * c_alpha[36];
    wire [17:0] sr36 = s36_pr + mr36, sg36 = s36_pg + mg36, sb36 = s36_pb + mb36;
    reg         p_line [35:37], p_sel [35:37], p_tex [35:37], p_blend [35:37];

    function [7:0] expand3;
        input [2:0] v;
        case (v)
            3'd0: expand3 = 8'd0;   3'd1: expand3 = 8'd36;
            3'd2: expand3 = 8'd72;  3'd3: expand3 = 8'd109;
            3'd4: expand3 = 8'd145; 3'd5: expand3 = 8'd182;
            3'd6: expand3 = 8'd218; default: expand3 = 8'd255;
        endcase
    endfunction

    function [3:0] bayer;
        input [3:0] k;
        case (k)
            4'd0: bayer = 4'd0;   4'd1: bayer = 4'd8;   4'd2: bayer = 4'd2;   4'd3: bayer = 4'd10;
            4'd4: bayer = 4'd12;  4'd5: bayer = 4'd4;   4'd6: bayer = 4'd14;  4'd7: bayer = 4'd6;
            4'd8: bayer = 4'd3;   4'd9: bayer = 4'd11;  4'd10: bayer = 4'd1;  4'd11: bayer = 4'd9;
            4'd12: bayer = 4'd15; 4'd13: bayer = 4'd7;  4'd14: bayer = 4'd13; default: bayer = 4'd5;
        endcase
    endfunction

    // ---- 37 -> 38: pick the colour ----
    reg  [23:0] s38_rgb;
    reg  [3:0]  s38_d;
    wire [31:0] h37 = s37_h ^ (s37_h >> 12);
    wire [1:0]  hb37 = h37[31:30];
    wire [2:0]  tone37 = p_line[37] ? 3'd3 : p_sel[37] ? 3'd4 : !p_tex[37] ? 3'd0 :
                         (hb37 == 2'd1 ? 3'd1 : hb37 == 2'd2 ? 3'd2 : 3'd0);
    reg  [23:0] tone_rgb;
    always @(*) begin
        case (tone37)
            3'd0:    tone_rgb = cset_q[23:0];
            3'd1:    tone_rgb = cset_q[47:24];
            3'd2:    tone_rgb = cset_q[71:48];
            3'd3:    tone_rgb = cset_q[95:72];
            default: tone_rgb = cset_q[119:96];
        endcase
    end

    // ---- 38 -> 39: dither to RGB332; 39 writes it ----
    reg  [7:0]  s39_pix;
    wire [10:0] q_r = {3'd0, s38_rgb[23:16]} * 11'd7 + {3'd0, s38_d, 4'd8};
    wire [10:0] q_g = {3'd0, s38_rgb[15:8]}  * 11'd7 + {3'd0, s38_d, 4'd8};
    wire [9:0]  q_b = {2'd0, s38_rgb[7:0]}   * 10'd3 + {2'd0, s38_d, 4'd8};

    always @(*) begin
        cset_ra = c_set[36];
        cb_we   = c_v[NST] && c_pass[NST];
        cb_wa   = c_addr[NST];
        cb_wd   = s39_pix;
    end

    integer k, j;
    always @(posedge clk) begin
        // ---- context ----
        c_v[1]     <= issue && !soft_reset;
        c_pk[1]    <= a_pk;
        c_addr[1]  <= a_addr;
        c_xy[1]    <= {a_y, a_x[1:0]};
        c_flag[1]  <= a_flag;
        c_axis[1]  <= a_axis;
        c_set[1]   <= a_set;
        c_c24[1]   <= a_c24;
        c_alpha[1] <= a_alpha;
        for (k = 2; k <= NST; k = k + 1) begin
            c_v[k]     <= c_v[k-1] && !soft_reset;
            c_pk[k]    <= c_pk[k-1];
            c_addr[k]  <= c_addr[k-1];
            c_xy[k]    <= c_xy[k-1];
            c_flag[k]  <= c_flag[k-1];
            c_axis[k]  <= c_axis[k-1];
            c_set[k]   <= c_set[k-1];
            c_c24[k]   <= c_c24[k-1];
            c_alpha[k] <= c_alpha[k-1];
        end
        c_pass[3] <= pass2;
        c_dst[2]  <= cb_qv[{c_addr[1][2:0], 3'b000} +: 8];
        for (k = 3; k <= NST; k = k + 1)
            c_dst[k] <= c_dst[k-1];
        for (k = 4; k <= NST; k = k + 1)
            c_pass[k] <= c_pass[k-1];

        // ---- 0 -> 1 ----
        s1_z     <= a_poly ? a_zpoly : a_zfill;
        s1_ztest <= a_poly || (a_pk == PK_FILL && a_flag);
        s1_zok   <= a_pk == PK_SOLID;
        s1_izok  <= !a_poly || a_izok;
        s1_iz[0] <= a_iz;
        s1_iz[1] <= a_iz + a_dizdy;
        s1_iz[2] <= a_iz + a_dizdx;
        s1_na[0] <= a_na;
        s1_na[1] <= a_na + fr_ny[a_ka];
        s1_na[2] <= a_na + fr_nx[a_ka];
        s1_nb[0] <= a_nb;
        s1_nb[1] <= a_nb + fr_ny[a_kb];
        s1_nb[2] <= a_nb + fr_nx[a_kb];

        // ---- 1 -> 2 ----
        s2_z     <= s1_z;
        s2_zq    <= z_q;
        s2_ztest <= s1_ztest;
        s2_zok   <= s1_zok;
        s2_izok  <= s1_izok;
        s3_z     <= s2_z;
        s3_zw    <= zw2;
        s4_z     <= s3_z;
        s4_zw    <= s3_zw;
        s5_z     <= s4_z;
        s5_zw    <= s4_zw;
        for (k = 0; k < 3; k = k + 1) begin
            s2_iz[k] <= clamp_iz(s1_iz[k]);
            s2_lz[k] <= clz48(clamp_iz(s1_iz[k]));
            s2_na[k] <= s1_na[k];
            s2_nb[k] <= s1_nb[k];
        end

        // ---- 2 -> 3, divider ----
        for (k = 0; k < 3; k = k + 1) begin
            dv_d[k][DIV0]  <= ({2'b00, s2_iz[k]} << s2_lz[k]) >> 23;
            dv_r[k][DIV0]  <= 26'd1 << 23;
            dv_q[k][DIV0]  <= 26'd0;
            dv_lz[k][DIV0] <= s2_lz[k];
            dv_na[k][DIV0] <= s2_na[k];
            dv_nb[k][DIV0] <= s2_nb[k];
            for (j = DIV0; j < DIV0 + DIVN; j = j + 1) begin
                if ({dv_r[k][j], 1'b0} >= {2'b00, dv_d[k][j]}) begin
                    dv_r[k][j+1] <= {dv_r[k][j][24:0], 1'b0} - {1'b0, dv_d[k][j]};
                    dv_q[k][j+1] <= {dv_q[k][j][24:0], 1'b1};
                end else begin
                    dv_r[k][j+1] <= {dv_r[k][j][24:0], 1'b0};
                    dv_q[k][j+1] <= {dv_q[k][j][24:0], 1'b0};
                end
                dv_d[k][j+1]  <= dv_d[k][j];
                dv_lz[k][j+1] <= dv_lz[k][j];
                dv_na[k][j+1] <= dv_na[k][j];
                dv_nb[k][j+1] <= dv_nb[k][j];
            end
        end

        // ---- 29 -> 30 ----
        for (k = 0; k < 3; k = k + 1) begin
            s30_pa[k] <= $signed(dv_na[k][DIV0+DIVN]) * $signed({1'b0, dv_q[k][DIV0+DIVN]});
            s30_pb[k] <= $signed(dv_nb[k][DIV0+DIVN]) * $signed({1'b0, dv_q[k][DIV0+DIVN]});
            s30_lz[k] <= dv_lz[k][DIV0+DIVN];
        end
        s30_q <= dv_q[0][DIV0+DIVN];

        // ---- 30 -> 31 ----
        for (k = 0; k < 3; k = k + 1) begin
            s31_ra[k] <= s30_pa[k] >>> (6'd45 - {1'b0, s30_lz[k]});
            s31_rb[k] <= s30_pb[k] >>> (6'd45 - {1'b0, s30_lz[k]});
        end
        s31_z <= s30_lz[0] >= 5'd16 ? {12'd0, s30_q} << (s30_lz[0] - 5'd16)
                                    : {12'd0, s30_q} >> (5'd16 - s30_lz[0]);

        // ---- 31 -> 32 ----
        for (k = 0; k < 3; k = k + 1) begin
            s32_a[k] <= fr_eye[axis_a(c_axis[31])] + s31_ra[k];
            s32_b[k] <= fr_eye[axis_b(c_axis[31])] + s31_rb[k];
        end
        s32_zd <= $signed({1'b0, s31_z}) - $signed({{7{fr_fog_start[31]}}, fr_fog_start});

        // ---- 32 -> 33 ----
        s33_a  <= s32_a[0];
        s33_b  <= s32_b[0];
        s33_fa <= absdiff(s32_a[2], s32_a[0]) > absdiff(s32_a[1], s32_a[0]) ?
                  absdiff(s32_a[2], s32_a[0]) : absdiff(s32_a[1], s32_a[0]);
        s33_fb <= absdiff(s32_b[2], s32_b[0]) > absdiff(s32_b[1], s32_b[0]) ?
                  absdiff(s32_b[2], s32_b[0]) : absdiff(s32_b[1], s32_b[0]);
        s33_fp <= s32_zd * $signed(fr_fog_scale);

        // ---- 33 -> 34 ----
        s34_fog   <= fog33;
        s34_tex   <= c_pk[33] == PK_SOLID && tex33;
        s34_line  <= c_pk[33] == PK_SOLID ? tex33 && lin33 :
                     c_pk[33] == PK_GLASS ? thin33 && lin33 : 1'b0;
        s34_sel   <= c_pk[33] == PK_SOLID && tex33 && sel33;
        s34_blend <= c_pk[33] == PK_TINT || c_pk[33] == PK_GLASS || c_pk[33] == PK_WATER;
        s34_bu    <= s33_a[31:16];
        s34_bv    <= s33_b[31:16];

        // ---- 34 -> 35: colour set and see-through colour of the fog level ----
        if (c_pk[34][2])
            c_set[35] <= c_set[34] + {6'd0, s34_fog};
        if (c_pk[34] == PK_GLASS) begin
            c_c24[35]   <= mats34[47:24];
            c_alpha[35] <= fr_alpha_glass;
        end else if (c_pk[34] == PK_WATER) begin
            c_c24[35]   <= mats34[23:0];
            c_alpha[35] <= fr_alpha_water;
        end

        // ---- 34 -> 35 ----
        s35_m1 <= {{16{s34_bu[15]}}, s34_bu} * 32'h9E3779B1;
        s35_m2 <= {{16{s34_bv[15]}}, s34_bv} * 32'h85EBCA77;
        s35_dr <= {1'b0, expand3(c_dst[34][7:5])};
        s35_dg <= {1'b0, expand3(c_dst[34][4:2])};
        s35_db <= {1'b0, c_dst[34][1:0], 6'd0} + {3'd0, c_dst[34][1:0], 4'd0} +
                  {5'd0, c_dst[34][1:0], 2'd0} + {7'd0, c_dst[34][1:0]};
        p_line[35] <= s34_line; p_sel[35] <= s34_sel; p_tex[35] <= s34_tex; p_blend[35] <= s34_blend;

        // ---- 35 -> 36 ----
        s36_h  <= h35 ^ (h35 >> 15);
        s36_pr <= s35_dr * (9'd256 - {1'b0, c_alpha[35]});
        s36_pg <= s35_dg * (9'd256 - {1'b0, c_alpha[35]});
        s36_pb <= s35_db * (9'd256 - {1'b0, c_alpha[35]});
        p_line[36] <= p_line[35]; p_sel[36] <= p_sel[35]; p_tex[36] <= p_tex[35]; p_blend[36] <= p_blend[35];

        // ---- 36 -> 37 ----
        s37_h     <= s36_h * 32'h2C1B3C6D;
        s37_blend <= {sr36[15:8], sg36[15:8], sb36[15:8]};
        p_line[37] <= p_line[36]; p_sel[37] <= p_sel[36]; p_tex[37] <= p_tex[36]; p_blend[37] <= p_blend[36];

        // ---- 37 -> 38 ----
        if (c_pk[37] == PK_FILL)
            s38_rgb <= c_c24[37];
        else if (p_blend[37] && !p_line[37])
            s38_rgb <= s37_blend;
        else
            s38_rgb <= tone_rgb;
        s38_d <= (p_blend[37] && !p_line[37]) ? 4'd4 : bayer(c_xy[37]);

        // ---- 38 -> 39 ----
        s39_pix <= c_pk[38] == PK_INVERT ? ~c_dst[38] : {q_r[10:8], q_g[10:8], q_b[9:8]};
    end

endmodule
