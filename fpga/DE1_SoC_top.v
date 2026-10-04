// DE1_SoC_top.v - top level for the DE1-SoC HPS framebuffer + VGA design
//
// Port names follow pin_assignment_DE1_SoC.tcl (sahandKashani/
// Altera-FPGA-top-level-files), so that script assigns every pin used here.
//
//   HPS writes pixels  -> 0xC0000000 .. 0xC004AFFF  (fb_ram, 640x480 RGB332)
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
        .vblank       (vblank)
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
