// span_gpu.v - draws frames from command lists in DDR (minecraft/gpu.h)
//
// The CPU builds each frame as a list of commands: rows of pixels ("spans")
// with start values and per-pixel steps, colour fills, see-through blends.
// This block fetches the list through the ARM's coherency port and does all
// per-pixel work at one pixel per clock:
//
//   fetch     Avalon bursts from the frame's command buffer (virtual
//             addresses, translated by a page table the CPU loads) into a
//             512-word FIFO; region r of buffer b starts at b*8M + r*256K
//   parse     commands are assembled while the previous one draws
//   draw      a 16-row band at a time, on chip: depth buffer (16 x 640 x
//             24-bit 1/z, invalidated per band by a 6-bit tag), colour
//             buffer (8 banks so 8 pixels can be read at once), and an
//             8-stage pixel pipeline: depth test and write, block grid
//             lines, block shade hash, highlight, blend, ordered dither
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

    localparam OP_NOP = 4'd0, OP_BAND = 4'd1, OP_COLORS = 4'd2, OP_SPAN = 4'd3,
               OP_FILL = 4'd4, OP_BLEND = 4'd5, OP_INVERT = 4'd6, OP_SHADE = 4'd7,
               OP_END = 4'd8;
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

    reg  [63:0] st_w [0:5];
    reg  [2:0]  st_need, st_have;
    reg         st_busy;                    // collecting words
    reg         st_valid;                   // complete command waiting
    wire        st_take;                    // dispatcher takes it
    wire [3:0]  st_op = st_w[0][3:0];

    function [2:0] cmd_words;
        input [3:0] op;
        case (op)
            OP_COLORS: cmd_words = 3'd3;
            OP_SPAN:   cmd_words = 3'd5;
            OP_FILL:   cmd_words = 3'd2;
            OP_BLEND:  cmd_words = 3'd6;
            OP_SHADE:  cmd_words = 3'd2;
            default:   cmd_words = 3'd1;
        endcase
    endfunction

    reg         p_hold;                     // after END: wait for the next region
    assign fifo_pop = fifo_qv && !st_valid && !p_hold && !soft_reset;

    always @(posedge clk) begin
        if (soft_reset) begin
            st_busy  <= 1'b0;
            st_valid <= 1'b0;
        end else if (st_valid) begin
            if (st_take)
                st_valid <= 1'b0;
        end else if (fifo_pop) begin
            if (!st_busy) begin
                st_w[0] <= fifo_q;
                st_have <= 3'd1;
                st_need <= cmd_words(fifo_q[3:0]);
                if (cmd_words(fifo_q[3:0]) == 3'd1)
                    st_valid <= 1'b1;
                else
                    st_busy <= 1'b1;
            end else begin
                st_w[st_have] <= fifo_q;
                st_have <= st_have + 3'd1;
                if (st_have + 3'd1 == st_need) begin
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
    wire        is_pixel_op = st_op == OP_SPAN || st_op == OP_SHADE || st_op == OP_FILL ||
                              st_op == OP_BLEND || st_op == OP_INVERT;

    assign st_take = st_valid && d_state == D_RUN && (!is_pixel_op || eng_ready);
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
    // pixel engine
    // =========================================================================

    // the command being drawn
    reg         a_valid;
    reg  [3:0]  a_op;
    reg         a_flag;
    reg  [9:0]  a_x, a_x1;
    reg  [1:0]  a_y;                        // y & 3, for the dither
    reg  [13:0] a_addr;                     // (y & 15) * 640 + x
    reg  [31:0] a_iz, a_diz, a_ua, a_dua, a_ub, a_dub;
    reg  [15:0] a_la, a_lb, a_sela, a_selb;
    reg  [10:0] a_set;
    reg  [23:0] a_c1, a_c2;
    reg  [7:0]  a_alpha;

    // pipeline stages 1..7
    reg         p_v    [1:7];
    reg  [3:0]  p_op   [1:7];
    reg  [13:0] p_addr [1:7];
    reg  [1:0]  p_x    [1:7];
    reg  [1:0]  p_y    [1:7];
    reg         p_flag [1:7];
    reg         p_line [1:7];
    reg  [10:0] p_set  [1:5];
    reg  [23:0] p_c1   [1:6];
    reg  [23:0] p_c2   [1:6];
    reg  [7:0]  p_alpha[1:4];

    // read-after-write: a blend or invert must not read a pixel that is
    // still on its way to the colour memory
    reg         hazard;
    always @(*) begin
        hazard = 1'b0;
        for (i = 1; i <= 7; i = i + 1)
            if (p_v[i] && p_addr[i] == a_addr)
                hazard = 1'b1;
    end

    wire        a_reads = a_op == OP_BLEND || a_op == OP_INVERT;
    wire        issue = a_valid && !(a_reads && hazard);
    wire        a_last = a_x + 10'd1 == a_x1;
    assign      eng_ready = !a_valid || (issue && a_last);
    wire        load = st_take && is_pixel_op;

    reg         any_stage;
    always @(*) begin
        any_stage = 1'b0;
        for (i = 1; i <= 7; i = i + 1)
            any_stage = any_stage | p_v[i];
    end
    assign pipe_empty = !a_valid && !any_stage;

    // ---- load / step the active command ----
    wire [9:0]  ld_x0 = st_w[0][15:6];
    wire [9:0]  ld_x1r = st_w[0][25:16];
    wire [9:0]  ld_x1 = ld_x1r > 10'd640 ? 10'd640 : ld_x1r;
    wire [3:0]  ld_row = st_w[0][29:26];
    wire [13:0] ld_addr = {ld_row, 9'd0} + {2'd0, ld_row, 7'd0} + {4'd0, ld_x0};
    wire        ld_span  = st_op == OP_SPAN;
    wire        ld_blend = st_op == OP_BLEND;
    wire        ld_tex   = ld_span || ld_blend;

    always @(posedge clk) begin
        if (soft_reset) begin
            a_valid <= 1'b0;
        end else if (load) begin
            a_valid <= ld_x0 < ld_x1;
            a_op    <= st_op;
            a_flag  <= st_w[0][4];
            a_x     <= ld_x0;
            a_x1    <= ld_x1;
            a_y     <= st_w[0][27:26];
            a_addr  <= ld_addr;
            a_set   <= st_w[0][45:35];
            a_la    <= ld_tex ? st_w[0][63:48] : 16'd0;
            a_iz    <= st_op == OP_FILL ? st_w[1][63:32] : st_w[1][31:0];
            a_diz   <= st_op == OP_FILL ? 32'd0 : st_w[1][63:32];
            a_ua    <= ld_tex ? st_w[2][31:0]  : 32'd0;
            a_dua   <= ld_tex ? st_w[2][63:32] : 32'd0;
            a_ub    <= ld_tex ? st_w[3][31:0]  : 32'd0;
            a_dub   <= ld_tex ? st_w[3][63:32] : 32'd0;
            a_lb    <= ld_tex ? st_w[4][15:0]  : 16'd0;
            a_sela  <= st_w[4][31:16];
            a_selb  <= st_w[4][47:32];
            a_c1    <= ld_blend ? st_w[4][39:16] : st_w[1][23:0];
            a_alpha <= st_w[4][47:40];
            a_c2    <= st_w[5][23:0];
        end else if (issue) begin
            if (a_last)
                a_valid <= 1'b0;
            a_x    <= a_x + 10'd1;
            a_addr <= a_addr + 14'd1;
            a_iz   <= a_iz + a_diz;
            a_ua   <= a_ua + a_dua;
            a_ub   <= a_ub + a_dub;
        end
    end

    // ---- stage 0 -> 1: depth, grid line, highlight; read the memories ----
    wire [23:0] z0 = a_iz[31] ? 24'd0 : (|a_iz[30:27] ? 24'hFFFFFF : a_iz[26:3]);
    wire        line0 = a_ua[15:0] < a_la || a_ub[15:0] < a_lb;
    wire        sel0 = a_flag && a_op == OP_SPAN && a_ua[31:16] == a_sela && a_ub[31:16] == a_selb;

    reg  [23:0] p1_z;
    reg         p1_sel;
    reg  [15:0] p1_bu, p1_bv;

    always @(*) begin
        z_ra  = a_addr;
        cb_ra = (d_state == D_FLUSH) ? (fl_accept ? fl_w + 11'd1 : fl_w) : a_addr[13:3];
    end

    // ---- stage 1 -> 2: depth test and write; block hash, first step ----
    // pixels drawn the clock before have not reached zmem yet: forward
    reg  [23:0] p2_z;
    reg         p2_zw;
    wire [23:0] stored1 = (p2_zw && p_addr[2] == p_addr[1]) ? p2_z :
                          (z_q[29:24] == zseq ? z_q[23:0] : 24'd0);
    wire        ztest1 = p_op[1] == OP_SPAN || p_op[1] == OP_SHADE ||
                         ((p_op[1] == OP_FILL || p_op[1] == OP_BLEND) && p_flag[1]);
    wire        pass1 = !ztest1 || p1_z > stored1;
    wire        zw1 = p_v[1] && (p_op[1] == OP_SPAN || p_op[1] == OP_SHADE) && pass1;

    always @(*) begin
        if (d_state == D_CLEAR) begin
            z_we = 1'b1;
            z_wa = clear_a;
            z_wd = 30'd0;
        end else begin
            z_we = zw1;
            z_wa = p_addr[1];
            z_wd = {zseq, p1_z};
        end
    end

    reg         p_pass [2:7];
    reg         p_sel  [2:5];
    reg  [31:0] p2_m1, p2_m2;
    reg  [7:0]  p2_dst;
    wire [31:0] bu32 = {{16{p1_bu[15]}}, p1_bu};
    wire [31:0] bv32 = {{16{p1_bv[15]}}, p1_bv};

    // ---- stage 2 -> 3: hash, second step; expand the pixel behind ----
    reg  [31:0] p3_h;
    reg  [7:0]  p3_dst;
    reg  [8:0]  p3_dr, p3_dg, p3_db;
    wire [31:0] h2 = p2_m1 + p2_m2;

    function [7:0] expand3;
        input [2:0] v;
        case (v)
            3'd0: expand3 = 8'd0;   3'd1: expand3 = 8'd36;
            3'd2: expand3 = 8'd72;  3'd3: expand3 = 8'd109;
            3'd4: expand3 = 8'd145; 3'd5: expand3 = 8'd182;
            3'd6: expand3 = 8'd218; default: expand3 = 8'd255;
        endcase
    endfunction

    // ---- stage 3 -> 4: hash multiply; blend products ----
    reg  [31:0] p4_h;
    reg  [7:0]  p4_dst;
    reg  [16:0] p4_r, p4_g, p4_b;

    // ---- stage 4 -> 5: tone; blend sum ----
    reg  [2:0]  p5_tone;
    reg  [7:0]  p5_dst;
    reg  [23:0] p5_blend;
    wire [31:0] h5 = p4_h ^ (p4_h >> 12);
    wire [1:0]  hbits = h5[31:30];
    wire [2:0]  tone4 = p_line[4] ? 3'd3 : p_sel[4] ? 3'd4 :
                        (hbits == 2'd1 ? 3'd1 : hbits == 2'd2 ? 3'd2 : 3'd0);
    wire [16:0] mr4 = p_c1[4][23:16] * p_alpha[4];
    wire [16:0] mg4 = p_c1[4][15:8]  * p_alpha[4];
    wire [16:0] mb4 = p_c1[4][7:0]   * p_alpha[4];
    wire [17:0] sr4 = p4_r + mr4, sg4 = p4_g + mg4, sb4 = p4_b + mb4;

    // ---- stage 5 -> 6: pick the colour ----
    reg  [23:0] p6_rgb;
    reg  [3:0]  p6_d;
    reg  [7:0]  p6_dst;
    reg  [23:0] tone_rgb;
    always @(*) begin
        case (p5_tone)
            3'd0:    tone_rgb = cset_q[23:0];
            3'd1:    tone_rgb = cset_q[47:24];
            3'd2:    tone_rgb = cset_q[71:48];
            3'd3:    tone_rgb = cset_q[95:72];
            default: tone_rgb = cset_q[119:96];
        endcase
    end

    function [3:0] bayer;
        input [3:0] k;
        case (k)
            4'd0: bayer = 4'd0;   4'd1: bayer = 4'd8;   4'd2: bayer = 4'd2;   4'd3: bayer = 4'd10;
            4'd4: bayer = 4'd12;  4'd5: bayer = 4'd4;   4'd6: bayer = 4'd14;  4'd7: bayer = 4'd6;
            4'd8: bayer = 4'd3;   4'd9: bayer = 4'd11;  4'd10: bayer = 4'd1;  4'd11: bayer = 4'd9;
            4'd12: bayer = 4'd15; 4'd13: bayer = 4'd7;  4'd14: bayer = 4'd13; default: bayer = 4'd5;
        endcase
    endfunction

    // ---- stage 6 -> 7: dither to RGB332 ----
    reg  [7:0]  p7_pix;
    wire [10:0] q_r = {3'd0, p6_rgb[23:16]} * 11'd7 + {3'd0, p6_d, 4'd8};
    wire [10:0] q_g = {3'd0, p6_rgb[15:8]}  * 11'd7 + {3'd0, p6_d, 4'd8};
    wire [9:0]  q_b = {2'd0, p6_rgb[7:0]}   * 10'd3 + {2'd0, p6_d, 4'd8};

    always @(*) begin
        cset_ra = p_set[4];
        cb_we   = p_v[7] && p_pass[7];
        cb_wa   = p_addr[7];
        cb_wd   = p7_pix;
    end

    always @(posedge clk) begin
        // 0 -> 1
        p_v[1]     <= issue && !soft_reset;
        p_op[1]    <= a_op;
        p_addr[1]  <= a_addr;
        p_x[1]     <= a_x[1:0];
        p_y[1]     <= a_y;
        p_flag[1]  <= a_flag;
        p_line[1]  <= line0;
        p_set[1]   <= a_set;
        p_c1[1]    <= a_c1;
        p_c2[1]    <= a_c2;
        p_alpha[1] <= a_alpha;
        p1_z       <= z0;
        p1_sel     <= sel0;
        p1_bu      <= a_ua[31:16];
        p1_bv      <= a_ub[31:16];

        // 1 -> 2
        p2_z   <= p1_z;
        p2_zw  <= zw1;
        p_pass[2] <= pass1;
        p_sel[2]  <= p1_sel;
        p2_m1  <= bu32 * 32'h9E3779B1;
        p2_m2  <= bv32 * 32'h85EBCA77;
        p2_dst <= cb_qv[{p_addr[1][2:0], 3'b000} +: 8];

        // 2 -> 3
        p3_h   <= h2 ^ (h2 >> 15);
        p3_dst <= p2_dst;
        p3_dr  <= {1'b0, expand3(p2_dst[7:5])};
        p3_dg  <= {1'b0, expand3(p2_dst[4:2])};
        p3_db  <= {1'b0, p2_dst[1:0], 6'd0} + {3'd0, p2_dst[1:0], 4'd0} +
                  {5'd0, p2_dst[1:0], 2'd0} + {7'd0, p2_dst[1:0]};      // * 85

        // 3 -> 4
        p4_h   <= p3_h * 32'h2C1B3C6D;
        p4_dst <= p3_dst;
        p4_r   <= p3_dr * (9'd256 - {1'b0, p_alpha[3]});
        p4_g   <= p3_dg * (9'd256 - {1'b0, p_alpha[3]});
        p4_b   <= p3_db * (9'd256 - {1'b0, p_alpha[3]});

        // 4 -> 5
        p5_tone  <= tone4;
        p5_dst   <= p4_dst;
        p5_blend <= {sr4[15:8], sg4[15:8], sb4[15:8]};

        // 5 -> 6
        p6_dst <= p5_dst;
        case (p_op[5])
            OP_SPAN, OP_SHADE: p6_rgb <= tone_rgb;
            OP_FILL:           p6_rgb <= p_c1[5];
            default:           p6_rgb <= p_line[5] ? p_c2[5] : p5_blend;
        endcase
        p6_d <= (p_op[5] == OP_BLEND && !p_line[5]) ? 4'd4 : bayer({p_y[5], p_x[5]});

        // 6 -> 7
        p7_pix <= p_op[6] == OP_INVERT ? ~p6_dst : {q_r[10:8], q_g[10:8], q_b[9:8]};

        for (i = 2; i <= 7; i = i + 1) begin
            p_v[i]    <= p_v[i-1] && !soft_reset;
            p_op[i]   <= p_op[i-1];
            p_addr[i] <= p_addr[i-1];
            p_x[i]    <= p_x[i-1];
            p_y[i]    <= p_y[i-1];
            p_flag[i] <= p_flag[i-1];
            p_line[i] <= p_line[i-1];
        end
        for (i = 3; i <= 7; i = i + 1)
            p_pass[i] <= p_pass[i-1];
        for (i = 3; i <= 5; i = i + 1)
            p_sel[i] <= p_sel[i-1];
        for (i = 2; i <= 5; i = i + 1)
            p_set[i] <= p_set[i-1];
        for (i = 2; i <= 6; i = i + 1) begin
            p_c1[i] <= p_c1[i-1];
            p_c2[i] <= p_c2[i-1];
        end
        for (i = 2; i <= 4; i = i + 1)
            p_alpha[i] <= p_alpha[i-1];
    end

endmodule
