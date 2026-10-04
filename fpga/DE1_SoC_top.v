// DE1_SoC_top.v - top level for the DE1-SoC HPS framebuffer + VGA design
//
// Port names follow pin_assignment_DE1_SoC.tcl (sahandKashani/
// Altera-FPGA-top-level-files), so that script assigns every pin used here.
//
//   HPS writes pixels  -> 0xC0000000 .. 0xC004AFFF  (fb_ram, 640x480 RGB332)
//   or the span GPU draws them from command lists in DDR (registers at
//   0xFF200100; its reads go through acp_read_adapter, see there)
//   HPS reads status   <- 0xFF200000                (vga_frame PIO)
//                           [31]   1 while in vertical blanking
//                           [30:0] frames completed (increments on vblank)
//
// LEDR shows the frame counter (slowed down) as a sign that scanout is alive.

module DE1_SoC_top (
    // clock
    input  wire        CLOCK_50,

    // user I/O
    input  wire [3:0]  KEY_N,
    output wire [9:0]  LEDR,
    output wire [6:0]  HEX0_N,
    output wire [6:0]  HEX1_N,
    output wire [6:0]  HEX2_N,
    output wire [6:0]  HEX3_N,
    output wire [6:0]  HEX4_N,
    output wire [6:0]  HEX5_N,

    // VGA (ADV7123)
    output wire [7:0]  VGA_R,
    output wire [7:0]  VGA_G,
    output wire [7:0]  VGA_B,
    output wire        VGA_HS,
    output wire        VGA_VS,
    output wire        VGA_BLANK_N,
    output wire        VGA_SYNC_N,
    output wire        VGA_CLK,

    // HPS DDR3
    output wire [14:0] HPS_DDR3_ADDR,
    output wire [2:0]  HPS_DDR3_BA,
    output wire        HPS_DDR3_CAS_N,
    output wire        HPS_DDR3_CK_N,
    output wire        HPS_DDR3_CK_P,
    output wire        HPS_DDR3_CKE,
    output wire        HPS_DDR3_CS_N,
    output wire [3:0]  HPS_DDR3_DM,
    inout  wire [31:0] HPS_DDR3_DQ,
    inout  wire [3:0]  HPS_DDR3_DQS_N,
    inout  wire [3:0]  HPS_DDR3_DQS_P,
    output wire        HPS_DDR3_ODT,
    output wire        HPS_DDR3_RAS_N,
    output wire        HPS_DDR3_RESET_N,
    input  wire        HPS_DDR3_RZQ,
    output wire        HPS_DDR3_WE_N,

    // HPS Ethernet (EMAC1)
    output wire        HPS_ENET_GTX_CLK,
    output wire        HPS_ENET_MDC,
    inout  wire        HPS_ENET_MDIO,
    input  wire        HPS_ENET_RX_CLK,
    input  wire [3:0]  HPS_ENET_RX_DATA,
    input  wire        HPS_ENET_RX_DV,
    output wire [3:0]  HPS_ENET_TX_DATA,
    output wire        HPS_ENET_TX_EN,

    // HPS QSPI flash
    inout  wire [3:0]  HPS_FLASH_DATA,
    output wire        HPS_FLASH_DCLK,
    output wire        HPS_FLASH_NCSO,

    // HPS I2C (I2C0 -> HPS_I2C1_*, I2C1 -> HPS_I2C2_*)
    inout  wire        HPS_I2C1_SCLK,
    inout  wire        HPS_I2C1_SDAT,
    inout  wire        HPS_I2C2_SCLK,
    inout  wire        HPS_I2C2_SDAT,

    // HPS SD card
    output wire        HPS_SD_CLK,
    inout  wire        HPS_SD_CMD,
    inout  wire [3:0]  HPS_SD_DATA,

    // HPS SPI master (SPIM1)
    output wire        HPS_SPIM_CLK,
    input  wire        HPS_SPIM_MISO,
    output wire        HPS_SPIM_MOSI,
    output wire        HPS_SPIM_SS,

    // HPS UART0 (console)
    input  wire        HPS_UART_RX,
    output wire        HPS_UART_TX,

    // HPS USB1 (ULPI) - keyboard
    input  wire        HPS_USB_CLKOUT,
    inout  wire [7:0]  HPS_USB_DATA,
    input  wire        HPS_USB_DIR,
    input  wire        HPS_USB_NXT,
    output wire        HPS_USB_STP
);

    wire        hps_reset_n;
    wire        vga_clk;
    wire [15:0] fb_address;
    wire [63:0] fb_readdata;
    wire        vblank;
    reg  [31:0] frame_status;

    // span GPU command reads: Avalon from the GPU, coherent AXI to the HPS
    wire        sys_clk;
    wire [31:0] cmd_address;
    wire        cmd_read, cmd_waitrequest, cmd_readdatavalid;
    wire [2:0]  cmd_burstcount;
    wire [63:0] cmd_readdata;

    wire [7:0]  csr_address;
    wire        csr_read, csr_write, csr_readdatavalid;
    wire [31:0] csr_writedata, csr_readdata;

    wire [18:0] gfb_address;
    wire        gfb_write, gfb_waitrequest;
    wire [63:0] gfb_writedata;
    wire [7:0]  gfb_byteenable;

    wire [15:0] beam;

    wire [7:0]  ax_arid, ax_awid, ax_wid, ax_wstrb;
    wire [31:0] ax_araddr, ax_awaddr;
    wire [3:0]  ax_arlen, ax_arcache, ax_awlen, ax_awcache;
    wire [2:0]  ax_arsize, ax_arprot, ax_awsize, ax_awprot;
    wire [1:0]  ax_arburst, ax_arlock, ax_awburst, ax_awlock;
    wire [4:0]  ax_aruser, ax_awuser;
    wire        ax_arvalid, ax_arready, ax_rvalid, ax_rready, ax_rlast;
    wire        ax_awvalid, ax_awready, ax_wlast, ax_wvalid, ax_wready, ax_bready, ax_bvalid;
    wire [63:0] ax_rdata, ax_wdata;
    wire [7:0]  ax_rid, ax_bid;
    wire [1:0]  ax_rresp, ax_bresp;

    soc_system u0 (
        .clk_clk                          (CLOCK_50),
        .reset_reset_n                    (1'b1),
        .h2f_reset_reset_n                (hps_reset_n),

        .vga_clk_clk                      (vga_clk),
        .fb_s2_address                    (fb_address),
        .fb_s2_chipselect                 (1'b1),
        .fb_s2_clken                      (1'b1),
        .fb_s2_write                      (1'b0),
        .fb_s2_readdata                   (fb_readdata),
        .fb_s2_writedata                  (64'd0),
        .fb_s2_byteenable                 (8'hFF),
        .vga_frame_export                 (frame_status),

        .sys_clk_clk                      (sys_clk),

        .gpu_csr_waitrequest              (1'b0),
        .gpu_csr_readdata                 (csr_readdata),
        .gpu_csr_readdatavalid            (csr_readdatavalid),
        .gpu_csr_burstcount               (),
        .gpu_csr_writedata                (csr_writedata),
        .gpu_csr_address                  (csr_address),
        .gpu_csr_write                    (csr_write),
        .gpu_csr_read                     (csr_read),
        .gpu_csr_byteenable               (),
        .gpu_csr_debugaccess              (),

        .gpu_fb_waitrequest               (gfb_waitrequest),
        .gpu_fb_readdata                  (),
        .gpu_fb_readdatavalid             (),
        .gpu_fb_burstcount                (1'b1),
        .gpu_fb_writedata                 (gfb_writedata),
        .gpu_fb_address                   (gfb_address),
        .gpu_fb_write                     (gfb_write),
        .gpu_fb_read                      (1'b0),
        .gpu_fb_byteenable                (gfb_byteenable),
        .gpu_fb_debugaccess               (1'b0),

        .f2h_axi_awid    (ax_awid),    .f2h_axi_awaddr  (ax_awaddr),  .f2h_axi_awlen   (ax_awlen),
        .f2h_axi_awsize  (ax_awsize),  .f2h_axi_awburst (ax_awburst), .f2h_axi_awlock  (ax_awlock),
        .f2h_axi_awcache (ax_awcache), .f2h_axi_awprot  (ax_awprot),  .f2h_axi_awvalid (ax_awvalid),
        .f2h_axi_awready (ax_awready), .f2h_axi_awuser  (ax_awuser),
        .f2h_axi_wid     (ax_wid),     .f2h_axi_wdata   (ax_wdata),   .f2h_axi_wstrb   (ax_wstrb),
        .f2h_axi_wlast   (ax_wlast),   .f2h_axi_wvalid  (ax_wvalid),  .f2h_axi_wready  (ax_wready),
        .f2h_axi_bid     (ax_bid),     .f2h_axi_bresp   (ax_bresp),   .f2h_axi_bvalid  (ax_bvalid),
        .f2h_axi_bready  (ax_bready),
        .f2h_axi_arid    (ax_arid),    .f2h_axi_araddr  (ax_araddr),  .f2h_axi_arlen   (ax_arlen),
        .f2h_axi_arsize  (ax_arsize),  .f2h_axi_arburst (ax_arburst), .f2h_axi_arlock  (ax_arlock),
        .f2h_axi_arcache (ax_arcache), .f2h_axi_arprot  (ax_arprot),  .f2h_axi_arvalid (ax_arvalid),
        .f2h_axi_arready (ax_arready), .f2h_axi_aruser  (ax_aruser),
        .f2h_axi_rid     (ax_rid),     .f2h_axi_rdata   (ax_rdata),   .f2h_axi_rresp   (ax_rresp),
        .f2h_axi_rlast   (ax_rlast),   .f2h_axi_rvalid  (ax_rvalid),  .f2h_axi_rready  (ax_rready),

        .memory_mem_a                     (HPS_DDR3_ADDR),
        .memory_mem_ba                    (HPS_DDR3_BA),
        .memory_mem_ck                    (HPS_DDR3_CK_P),
        .memory_mem_ck_n                  (HPS_DDR3_CK_N),
        .memory_mem_cke                   (HPS_DDR3_CKE),
        .memory_mem_cs_n                  (HPS_DDR3_CS_N),
        .memory_mem_ras_n                 (HPS_DDR3_RAS_N),
        .memory_mem_cas_n                 (HPS_DDR3_CAS_N),
        .memory_mem_we_n                  (HPS_DDR3_WE_N),
        .memory_mem_reset_n               (HPS_DDR3_RESET_N),
        .memory_mem_dq                    (HPS_DDR3_DQ),
        .memory_mem_dqs                   (HPS_DDR3_DQS_P),
        .memory_mem_dqs_n                 (HPS_DDR3_DQS_N),
        .memory_mem_odt                   (HPS_DDR3_ODT),
        .memory_mem_dm                    (HPS_DDR3_DM),
        .memory_oct_rzqin                 (HPS_DDR3_RZQ),

        .hps_io_hps_io_emac1_inst_TX_CLK  (HPS_ENET_GTX_CLK),
        .hps_io_hps_io_emac1_inst_TXD0    (HPS_ENET_TX_DATA[0]),
        .hps_io_hps_io_emac1_inst_TXD1    (HPS_ENET_TX_DATA[1]),
        .hps_io_hps_io_emac1_inst_TXD2    (HPS_ENET_TX_DATA[2]),
        .hps_io_hps_io_emac1_inst_TXD3    (HPS_ENET_TX_DATA[3]),
        .hps_io_hps_io_emac1_inst_RXD0    (HPS_ENET_RX_DATA[0]),
        .hps_io_hps_io_emac1_inst_RXD1    (HPS_ENET_RX_DATA[1]),
        .hps_io_hps_io_emac1_inst_RXD2    (HPS_ENET_RX_DATA[2]),
        .hps_io_hps_io_emac1_inst_RXD3    (HPS_ENET_RX_DATA[3]),
        .hps_io_hps_io_emac1_inst_MDIO    (HPS_ENET_MDIO),
        .hps_io_hps_io_emac1_inst_MDC     (HPS_ENET_MDC),
        .hps_io_hps_io_emac1_inst_RX_CTL  (HPS_ENET_RX_DV),
        .hps_io_hps_io_emac1_inst_TX_CTL  (HPS_ENET_TX_EN),
        .hps_io_hps_io_emac1_inst_RX_CLK  (HPS_ENET_RX_CLK),

        .hps_io_hps_io_qspi_inst_IO0      (HPS_FLASH_DATA[0]),
        .hps_io_hps_io_qspi_inst_IO1      (HPS_FLASH_DATA[1]),
        .hps_io_hps_io_qspi_inst_IO2      (HPS_FLASH_DATA[2]),
        .hps_io_hps_io_qspi_inst_IO3      (HPS_FLASH_DATA[3]),
        .hps_io_hps_io_qspi_inst_SS0      (HPS_FLASH_NCSO),
        .hps_io_hps_io_qspi_inst_CLK      (HPS_FLASH_DCLK),

        .hps_io_hps_io_sdio_inst_CMD      (HPS_SD_CMD),
        .hps_io_hps_io_sdio_inst_D0       (HPS_SD_DATA[0]),
        .hps_io_hps_io_sdio_inst_D1       (HPS_SD_DATA[1]),
        .hps_io_hps_io_sdio_inst_D2       (HPS_SD_DATA[2]),
        .hps_io_hps_io_sdio_inst_D3       (HPS_SD_DATA[3]),
        .hps_io_hps_io_sdio_inst_CLK      (HPS_SD_CLK),

        .hps_io_hps_io_usb1_inst_D0       (HPS_USB_DATA[0]),
        .hps_io_hps_io_usb1_inst_D1       (HPS_USB_DATA[1]),
        .hps_io_hps_io_usb1_inst_D2       (HPS_USB_DATA[2]),
        .hps_io_hps_io_usb1_inst_D3       (HPS_USB_DATA[3]),
        .hps_io_hps_io_usb1_inst_D4       (HPS_USB_DATA[4]),
        .hps_io_hps_io_usb1_inst_D5       (HPS_USB_DATA[5]),
        .hps_io_hps_io_usb1_inst_D6       (HPS_USB_DATA[6]),
        .hps_io_hps_io_usb1_inst_D7       (HPS_USB_DATA[7]),
        .hps_io_hps_io_usb1_inst_CLK      (HPS_USB_CLKOUT),
        .hps_io_hps_io_usb1_inst_STP      (HPS_USB_STP),
        .hps_io_hps_io_usb1_inst_DIR      (HPS_USB_DIR),
        .hps_io_hps_io_usb1_inst_NXT      (HPS_USB_NXT),

        .hps_io_hps_io_spim1_inst_CLK     (HPS_SPIM_CLK),
        .hps_io_hps_io_spim1_inst_MOSI    (HPS_SPIM_MOSI),
        .hps_io_hps_io_spim1_inst_MISO    (HPS_SPIM_MISO),
        .hps_io_hps_io_spim1_inst_SS0     (HPS_SPIM_SS),

        .hps_io_hps_io_uart0_inst_RX      (HPS_UART_RX),
        .hps_io_hps_io_uart0_inst_TX      (HPS_UART_TX),

        .hps_io_hps_io_i2c0_inst_SDA      (HPS_I2C1_SDAT),
        .hps_io_hps_io_i2c0_inst_SCL      (HPS_I2C1_SCLK),
        .hps_io_hps_io_i2c1_inst_SDA      (HPS_I2C2_SDAT),
        .hps_io_hps_io_i2c1_inst_SCL      (HPS_I2C2_SCLK)
    );

    // ---- span GPU ------------------------------------------------------------

    reg [1:0] sys_rst_sync;
    always @(posedge sys_clk or negedge hps_reset_n)
        if (!hps_reset_n) sys_rst_sync <= 2'b00;
        else              sys_rst_sync <= {sys_rst_sync[0], 1'b1};

    // the beam position crosses from the pixel clock; it changes only every
    // 16 lines, so a value seen twice in a row is a whole one
    reg [15:0] beam_s1, beam_s2, beam_s3, beam_sys;
    always @(posedge sys_clk) begin
        beam_s1 <= beam;
        beam_s2 <= beam_s1;
        beam_s3 <= beam_s2;
        if (beam_s2 == beam_s3)
            beam_sys <= beam_s3;
    end

    span_gpu u_gpu (
        .clk               (sys_clk),
        .reset             (!sys_rst_sync[1]),
        .csr_address       (csr_address[7:2]),
        .csr_read          (csr_read),
        .csr_write         (csr_write),
        .csr_writedata     (csr_writedata),
        .csr_readdata      (csr_readdata),
        .csr_readdatavalid (csr_readdatavalid),
        .cmd_address       (cmd_address),
        .cmd_read          (cmd_read),
        .cmd_burstcount    (cmd_burstcount),
        .cmd_readdata      (cmd_readdata),
        .cmd_waitrequest   (cmd_waitrequest),
        .cmd_readdatavalid (cmd_readdatavalid),
        .fb_address        (gfb_address),
        .fb_write          (gfb_write),
        .fb_writedata      (gfb_writedata),
        .fb_byteenable     (gfb_byteenable),
        .fb_waitrequest    (gfb_waitrequest),
        .beam_scan         (beam_sys[15:5]),
        .beam_band         (beam_sys[4:0])
    );

    acp_read_adapter u_acp (
        .clk               (sys_clk),
        .avm_address       (cmd_address),
        .avm_read          (cmd_read),
        .avm_burstcount    (cmd_burstcount),
        .avm_readdata      (cmd_readdata),
        .avm_waitrequest   (cmd_waitrequest),
        .avm_readdatavalid (cmd_readdatavalid),
        .arid (ax_arid), .araddr (ax_araddr), .arlen (ax_arlen), .arsize (ax_arsize),
        .arburst (ax_arburst), .arlock (ax_arlock), .arcache (ax_arcache), .arprot (ax_arprot),
        .aruser (ax_aruser), .arvalid (ax_arvalid), .arready (ax_arready),
        .rdata (ax_rdata), .rvalid (ax_rvalid), .rready (ax_rready),
        .awid (ax_awid), .awaddr (ax_awaddr), .awlen (ax_awlen), .awsize (ax_awsize),
        .awburst (ax_awburst), .awlock (ax_awlock), .awcache (ax_awcache), .awprot (ax_awprot),
        .awuser (ax_awuser), .awvalid (ax_awvalid),
        .wid (ax_wid), .wdata (ax_wdata), .wstrb (ax_wstrb), .wlast (ax_wlast), .wvalid (ax_wvalid),
        .bready (ax_bready)
    );

    // ---- VGA scanout (pixel clock domain) ------------------------------------

    reg [1:0] vga_rst_sync;
    always @(posedge vga_clk or negedge hps_reset_n)
        if (!hps_reset_n) vga_rst_sync <= 2'b00;
        else              vga_rst_sync <= {vga_rst_sync[0], KEY_N[0]};

    vga_fb #(.RAM_LATENCY(1)) u_vga (
        .clk          (vga_clk),
        .reset_n      (vga_rst_sync[1]),
        .fb_address   (fb_address),
        .fb_readdata  (fb_readdata),
        .vga_r        (VGA_R),
        .vga_g        (VGA_G),
        .vga_b        (VGA_B),
        .vga_hs       (VGA_HS),
        .vga_vs       (VGA_VS),
        .vga_blank_n  (VGA_BLANK_N),
        .vga_sync_n   (VGA_SYNC_N),
        .vga_clk      (VGA_CLK),
        .vblank       (vblank),
        .beam         (beam)
    );

    // ---- frame counter (50 MHz domain, read by the HPS) ----------------------

    reg [2:0] vblank_sync;
    always @(posedge CLOCK_50) begin
        vblank_sync <= {vblank_sync[1:0], vblank};
        if (vblank_sync[1] && !vblank_sync[2])
            frame_status[30:0] <= frame_status[30:0] + 31'd1;
        frame_status[31] <= vblank_sync[1];
    end

    // ---- board indicators ----------------------------------------------------

    assign LEDR   = frame_status[14:5];   // ~2 Hz on LEDR[0] at 60 fps
    assign HEX0_N = 7'h7F;
    assign HEX1_N = 7'h7F;
    assign HEX2_N = 7'h7F;
    assign HEX3_N = 7'h7F;
    assign HEX4_N = 7'h7F;
    assign HEX5_N = 7'h7F;

endmodule
