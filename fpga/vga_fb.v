// vga_fb.v - 640x480 @ 60 Hz VGA scanout from a 64-bit wide framebuffer RAM
//
// Framebuffer layout (as seen by the CPU at 0xC0000000):
//   byte (y * 640 + x) = pixel (x, y), RGB332:  [7:5] R  [4:2] G  [1:0] B
// The RAM read port is 64 bits wide, so one word holds 8 adjacent pixels;
// byte 0 of a word (bits [7:0]) is the leftmost pixel.
//
// Drives the ADV7123 video DAC on the DE1-SoC (8 bits per channel).
// Runs entirely in the pixel clock domain (25.175 MHz).

module vga_fb #(
    parameter RAM_LATENCY = 1           // fb_ram slave2Latency
) (
    input  wire        clk,             // pixel clock
    input  wire        reset_n,         // synchronous to clk

    // framebuffer read port (fb_ram.s2)
    output wire [15:0] fb_address,
    input  wire [63:0] fb_readdata,

    // VGA DAC
    output reg  [7:0]  vga_r,
    output reg  [7:0]  vga_g,
    output reg  [7:0]  vga_b,
    output reg         vga_hs,
    output reg         vga_vs,
    output reg         vga_blank_n,
    output wire        vga_sync_n,
    output wire        vga_clk,

    // high while the beam is in the vertical blanking interval
    output reg         vblank
);

    // 640x480@60 timing (pixels / lines), negative sync polarity
    localparam H_VISIBLE = 640, H_FRONT = 16, H_SYNC = 96, H_BACK = 48;
    localparam V_VISIBLE = 480, V_FRONT = 10, V_SYNC = 2,  V_BACK = 33;
    localparam H_TOTAL = H_VISIBLE + H_FRONT + H_SYNC + H_BACK;   // 800
    localparam V_TOTAL = V_VISIBLE + V_FRONT + V_SYNC + V_BACK;   // 525

    // ---- stage 0: beam position ----------------------------------------------

    reg [9:0] hc, vc;

    always @(posedge clk) begin
        if (!reset_n) begin
            hc <= 10'd0;
            vc <= 10'd0;
        end else if (hc == H_TOTAL - 1) begin
            hc <= 10'd0;
            vc <= (vc == V_TOTAL - 1) ? 10'd0 : vc + 10'd1;
        end else begin
            hc <= hc + 10'd1;
        end
    end

    wire visible0 = (hc < H_VISIBLE) && (vc < V_VISIBLE);
    wire hs0 = ~((hc >= H_VISIBLE + H_FRONT) && (hc < H_VISIBLE + H_FRONT + H_SYNC));
    wire vs0 = ~((vc >= V_VISIBLE + V_FRONT) && (vc < V_VISIBLE + V_FRONT + V_SYNC));

    // word address = y * 80 + x / 8   (80 words of 8 pixels per line)
    assign fb_address = {vc[8:0], 6'b0} + {2'b0, vc[8:0], 4'b0} + {9'b0, hc[9:3]};

    // ---- delay the beam signals to line up with the RAM output ---------------

    reg [RAM_LATENCY-1:0] visible_d, hs_d, vs_d;
    reg [2:0]             xsel_d [0:RAM_LATENCY-1];
    integer i;

    always @(posedge clk) begin
        visible_d[0] <= visible0;
        hs_d[0]      <= hs0;
        vs_d[0]      <= vs0;
        xsel_d[0]    <= hc[2:0];
        for (i = 1; i < RAM_LATENCY; i = i + 1) begin
            visible_d[i] <= visible_d[i-1];
            hs_d[i]      <= hs_d[i-1];
            vs_d[i]      <= vs_d[i-1];
            xsel_d[i]    <= xsel_d[i-1];
        end
    end

    // ---- final stage: pick the pixel byte, expand RGB332 to RGB888 -----------

    wire [7:0] pix = fb_readdata[{xsel_d[RAM_LATENCY-1], 3'b000} +: 8];
    wire       vis = visible_d[RAM_LATENCY-1];

    always @(posedge clk) begin
        vga_r       <= vis ? {pix[7:5], pix[7:5], pix[7:6]}   : 8'd0;
        vga_g       <= vis ? {pix[4:2], pix[4:2], pix[4:3]}   : 8'd0;
        vga_b       <= vis ? {pix[1:0], pix[1:0], pix[1:0], pix[1:0]} : 8'd0;
        vga_blank_n <= vis;
        vga_hs      <= hs_d[RAM_LATENCY-1];
        vga_vs      <= vs_d[RAM_LATENCY-1];
        vblank      <= (vc >= V_VISIBLE);
    end

    assign vga_sync_n = 1'b0;           // no sync-on-green
    assign vga_clk    = ~clk;           // DAC samples mid-way through each pixel

endmodule
