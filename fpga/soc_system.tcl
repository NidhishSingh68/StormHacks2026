# soc_system.tcl - Platform Designer system for the DE1-SoC framebuffer
#
#   HPS --h2f_axi_master (64-bit)----> fb_ram.s1     @ 0xC0000000  (300 KB)
#       --h2f_lw_axi_master (32-bit)-> vga_frame.s1  @ 0xFF200000  (frame ctr)
#
#   fb_ram.s2 (read port) and its clock are exported to the VGA scanout logic
#   in the top level, which runs from pll_0's 25.175 MHz pixel clock.
#
# Framebuffer: 640 x 480, 8 bpp RGB332, one byte per pixel, row-major,
# 640 bytes per line. 307200 bytes = ~300 of the 397 M10K blocks in a
# 5CSEMA5, so this is the largest single 640x480 buffer that fits.
#
# Run with:  qsys-script --script=soc_system.tcl
# HPS DDR3 / clock settings are the DE1-SoC GHRD values.

package require -exact qsys 16.1

create_system soc_system
set_project_property DEVICE_FAMILY "Cyclone V"
set_project_property DEVICE 5CSEMA5F31C6

# ---- 50 MHz system clock -----------------------------------------------------

add_instance clk_0 clock_source
set_instance_parameter_value clk_0 clockFrequency 50000000.0
set_instance_parameter_value clk_0 clockFrequencyKnown true
set_instance_parameter_value clk_0 resetSynchronousEdges NONE

# ---- 25.175 MHz VGA pixel clock ---------------------------------------------

add_instance pll_0 altera_pll
set_instance_parameter_value pll_0 gui_reference_clock_frequency 50.0
set_instance_parameter_value pll_0 gui_operation_mode direct
set_instance_parameter_value pll_0 gui_number_of_clocks 1
set_instance_parameter_value pll_0 gui_output_clock_frequency0 25.175
set_instance_parameter_value pll_0 gui_use_locked false

# Clock bridge so the pixel clock can feed both fb_ram.clk2 (inside the
# system) and the VGA scanout logic (outside it).
add_instance vga_clk_bridge altera_clock_bridge
set_instance_parameter_value vga_clk_bridge EXPLICIT_CLOCK_RATE 25175000.0

# ---- HPS ---------------------------------------------------------------------

add_instance hps_0 altera_hps

set hps_params {
    S2F_Width                   2
    F2S_Width                   0
    LWH2F_Enable                true
    F2SINTERRUPT_Enable         false
    F2SDRAM_Type                {}
    F2SDRAM_Width               {}
    MPU_EVENTS_Enable           false
    BOOTFROMFPGA_Enable         false

    EMAC1_PinMuxing             {HPS I/O Set 0}
    EMAC1_Mode                  RGMII
    QSPI_PinMuxing              {HPS I/O Set 0}
    QSPI_Mode                   {1 SS}
    SDIO_PinMuxing              {HPS I/O Set 0}
    SDIO_Mode                   {4-bit Data}
    USB1_PinMuxing              {HPS I/O Set 0}
    USB1_Mode                   SDR
    SPIM1_PinMuxing             {HPS I/O Set 0}
    SPIM1_Mode                  {Single Slave Select}
    UART0_PinMuxing             {HPS I/O Set 0}
    UART0_Mode                  {No Flow Control}
    I2C0_PinMuxing              {HPS I/O Set 0}
    I2C0_Mode                   I2C
    I2C1_PinMuxing              {HPS I/O Set 0}
    I2C1_Mode                   I2C

    desired_mpu_clk_mhz         800.0
    eosc1_clk_mhz               25.0
    eosc2_clk_mhz               25.0

    HPS_PROTOCOL                DDR3
    MEM_VENDOR                  JEDEC
    MEM_CLK_FREQ                400.0
    MEM_CLK_FREQ_MAX            800.0
    REF_CLK_FREQ                25.0
    MEM_DQ_WIDTH                32
    MEM_ROW_ADDR_WIDTH          15
    MEM_COL_ADDR_WIDTH          10
    MEM_BANKADDR_WIDTH          3
    MEM_DQ_PER_DQS              8
    MEM_IF_DM_PINS_EN           true
    MEM_IF_DQSN_EN              true
    MEM_DRV_STR                 RZQ/7
    MEM_RTT_NOM                 RZQ/4
    MEM_RTT_WR                  RZQ/4
    MEM_TCL                     11
    MEM_WTCL                    8
    MEM_ATCL                    Disabled
    MEM_TINIT_US                500
    MEM_TMRD_CK                 4
    MEM_TRAS_NS                 35.0
    MEM_TRCD_NS                 13.75
    MEM_TRP_NS                  13.75
    MEM_TREFI_US                7.8
    MEM_TRFC_NS                 260.0
    MEM_TWR_NS                  15.0
    MEM_TWTR                    4
    MEM_TFAW_NS                 30.0
    MEM_TRRD_NS                 7.5
    MEM_TRTP_NS                 7.5

    TIMING_TIS                  180
    TIMING_TIH                  140
    TIMING_TDS                  30
    TIMING_TDH                  65
    TIMING_TDQSQ                125
    TIMING_TQH                  0.38
    TIMING_TDQSCK               255
    TIMING_TDQSS                0.25
    TIMING_TQSH                 0.4
    TIMING_TDSH                 0.2
    TIMING_TDSS                 0.2

    TIMING_BOARD_MAX_CK_DELAY        0.03
    TIMING_BOARD_MAX_DQS_DELAY       0.02
    TIMING_BOARD_SKEW_CKDQS_DIMM_MIN 0.09
    TIMING_BOARD_SKEW_CKDQS_DIMM_MAX 0.16
    TIMING_BOARD_SKEW_BETWEEN_DIMMS  0.05
    TIMING_BOARD_SKEW_WITHIN_DQS     0.01
    TIMING_BOARD_SKEW_BETWEEN_DQS    0.08
    TIMING_BOARD_DQ_TO_DQS_SKEW      0.0
    TIMING_BOARD_AC_SKEW             0.03
    TIMING_BOARD_AC_TO_CK_SKEW       0.0
}
foreach {name value} $hps_params {
    set_instance_parameter_value hps_0 $name $value
}

# ---- framebuffer: true dual-port on-chip RAM --------------------------------

add_instance fb_ram altera_avalon_onchip_memory2
set_instance_parameter_value fb_ram memorySize 307200.0
set_instance_parameter_value fb_ram dataWidth 64
set_instance_parameter_value fb_ram dualPort true
set_instance_parameter_value fb_ram singleClockOperation false
set_instance_parameter_value fb_ram slave1Latency 1
set_instance_parameter_value fb_ram slave2Latency 1
set_instance_parameter_value fb_ram writable true
set_instance_parameter_value fb_ram initMemContent false
set_instance_parameter_value fb_ram blockType AUTO

# ---- frame counter (input PIO, read by the CPU for vblank sync) -------------

add_instance vga_frame altera_avalon_pio
set_instance_parameter_value vga_frame width 32
set_instance_parameter_value vga_frame direction Input
set_instance_parameter_value vga_frame captureEdge false
set_instance_parameter_value vga_frame generateIRQ false

# ---- clocks & resets ---------------------------------------------------------

add_connection clk_0.clk       pll_0.refclk
add_connection clk_0.clk_reset pll_0.reset
add_connection pll_0.outclk0   vga_clk_bridge.in_clk
add_connection pll_0.outclk0   fb_ram.clk2

add_connection clk_0.clk hps_0.h2f_axi_clock
add_connection clk_0.clk hps_0.h2f_lw_axi_clock
add_connection clk_0.clk fb_ram.clk1
add_connection clk_0.clk vga_frame.clk

add_connection clk_0.clk_reset fb_ram.reset1
add_connection clk_0.clk_reset fb_ram.reset2
add_connection clk_0.clk_reset vga_frame.reset

# ---- address map -------------------------------------------------------------

add_connection hps_0.h2f_axi_master fb_ram.s1
set_connection_parameter_value hps_0.h2f_axi_master/fb_ram.s1 baseAddress 0x00000000

add_connection hps_0.h2f_lw_axi_master vga_frame.s1
set_connection_parameter_value hps_0.h2f_lw_axi_master/vga_frame.s1 baseAddress 0x00000000

# ---- exports -----------------------------------------------------------------

set_interface_property clk        EXPORT_OF clk_0.clk_in
set_interface_property reset      EXPORT_OF clk_0.clk_in_reset
set_interface_property memory     EXPORT_OF hps_0.memory
set_interface_property hps_io     EXPORT_OF hps_0.hps_io
set_interface_property h2f_reset  EXPORT_OF hps_0.h2f_reset
set_interface_property vga_clk    EXPORT_OF vga_clk_bridge.out_clk
set_interface_property fb_s2      EXPORT_OF fb_ram.s2
set_interface_property vga_frame  EXPORT_OF vga_frame.external_connection

save_system soc_system.qsys
