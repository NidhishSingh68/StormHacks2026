# create_project.tcl - creates the Quartus project for the DE1-SoC framebuffer
# Run with:  quartus_sh -t create_project.tcl

project_new DE1_SoC_top -overwrite

set_global_assignment -name TOP_LEVEL_ENTITY DE1_SoC_top
set_global_assignment -name PROJECT_OUTPUT_DIRECTORY output_files

set_global_assignment -name QIP_FILE        soc_system/synthesis/soc_system.qip
set_global_assignment -name VERILOG_FILE    DE1_SoC_top.v
set_global_assignment -name VERILOG_FILE    vga_fb.v
set_global_assignment -name SDC_FILE        DE1_SoC_top.sdc

# Board pinout (device, locations, I/O standards)
source pin_assignment_DE1_SoC.tcl

# Leave unused board pins (audio, SDRAM, GPIO headers, ...) tri-stated
set_global_assignment -name RESERVE_ALL_UNUSED_PINS_WEAK_PULLUP "AS INPUT TRI-STATED"

project_close
