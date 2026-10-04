#!/bin/sh
# build.sh - regenerate the Platform Designer system and compile the design
#
# Output:  output_files/DE1_SoC_top.sof   (JTAG programming)
#          output_files/DE1_SoC_top.rbf   (loading from the HPS / SD card)
set -e
cd "$(dirname "$0")"

: "${QUARTUS_ROOTDIR:=$HOME/altera_lite/25.1std/quartus}"
export QUARTUS_ROOTDIR
Q="$QUARTUS_ROOTDIR/bin"
QSYS="$QUARTUS_ROOTDIR/sopc_builder/bin"

"$QSYS/qsys-script"   --script=soc_system.tcl
"$QSYS/qsys-generate" soc_system.qsys --synthesis=VERILOG \
                      --output-directory=soc_system --part=5CSEMA5F31C6

"$Q/quartus_sh" -t create_project.tcl

# Analysis & synthesis first, so the HPS DDR3 pin script can find its pins
"$Q/quartus_map" DE1_SoC_top
"$Q/quartus_sh"  -t soc_system/synthesis/submodules/hps_sdram_p0_pin_assignments.tcl DE1_SoC_top

"$Q/quartus_sh"  --flow compile DE1_SoC_top

# Uncompressed raw binary, loaded by U-Boot from the SD card's FAT partition
# (same format as the Terasic image's soc_system.rbf)
"$Q/quartus_cpf" -c output_files/DE1_SoC_top.sof output_files/soc_system.rbf

echo
echo "done: output_files/DE1_SoC_top.sof, output_files/soc_system.rbf"
