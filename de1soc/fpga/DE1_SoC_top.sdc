# DE1_SoC_top.sdc

create_clock -name CLOCK_50 -period 20.000 [get_ports CLOCK_50]

derive_pll_clocks
derive_clock_uncertainty

# vblank crosses from the pixel clock to CLOCK_50 through a 2-FF synchronizer
set_false_path -to [get_registers {vblank_sync[0]}]

# the beam position crosses to sys_clk through a synchronizer that only
# accepts a value seen on two clocks in a row
set_false_path -to [get_registers {beam_s1[*]}]

# Board-level I/O with no timing relationship we care about
set_false_path -from [get_ports {KEY_N[*]}]
set_false_path -to   [get_ports {LEDR[*] HEX*}]

# The ADV7123 is clocked by the inverted pixel clock forwarded on VGA_CLK,
# giving half a pixel period (~20 ns) of setup/hold margin on the data pins.
set_false_path -to   [get_ports {VGA_*}]
