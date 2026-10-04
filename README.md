# DE1-SoC voxel world

A small Minecraft-like game on a Terasic DE1-SoC (Cyclone V SoC: dual
Cortex-A9 + FPGA), drawn on VGA by a custom GPU in the FPGA fabric. Worlds
come from real terrain data, generated on a PC (`../worldgen`) and sent to
the board over its USB-UART.

```
fpga/        the FPGA design: VGA scan-out, framebuffer, span GPU
  span_gpu.v       the GPU (command fetch, row engine, 40-stage pixel pipeline)
  vga_fb.v         640x480@60 VGA from the framebuffer, beam position
  acp_read_adapter.v  cache-coherent DDR reads through the ARM's ACP
  DE1_SoC_top.v    top level; soc_system.tcl = Platform Designer system
  build.sh         Quartus build -> output_files/soc_system.rbf
  sim/run.sh       Verilator test: span_gpu.v vs the C model, byte for byte
minecraft/   the game and its board services (C, runs on the ARM)
  main.c           game loop, player, threads
  world.c          chunks, terrain, trees, water, greedy meshing
  render.c         culling, polygon setup, GPU command lists
  gpu.h, gpu_sw.c  GPU command format, and its exact C model
  fpga.c           /dev/mem access: GPU registers, page table, framebuffer
  terrad.c         owns the UART: receives worlds, restarts the game
  tools/           terrain_gen (test worlds), bridgebench (bridge speeds)
demos/       first bring-up programs (toolchain check, framebuffer, keyboard)
```

## Who does what

**ARM (C):** input, physics, block breaking/placing, weather; loading and
meshing chunks (second core); per frame: frustum and back-face culling,
transforming and clipping each visible face, and writing a command list:
one `POLY` per face polygon and 16-row band (its edges and 1/z plane),
fills for the sky, sun, stars, rain/snow and HUD, plus the camera and fog
once per frame. It never touches a pixel.

**FPGA (Verilog, 102 MHz, one pixel per clock):** reads the command lists
straight from DDR through the ACP (coherent with the CPU's caches, via a
page table); walks polygon edges; per pixel: depth test, 1/z by pipelined
dividers, the exact world position on the face, how big a pixel is on it
(block grid lines vs plain), fog, per-block shade, highlight, water and
glass blending, ordered dithering to RGB332; draws each band on chip and
copies it out only once the VGA beam has passed it, so frames never tear.

`gpu_sw.c` executes the same commands with the same integer arithmetic; it
is the PC build's renderer, the `--cpu` fallback, and the reference the
Verilog is checked against (`fpga/sim/run.sh`).

## Build

Arm GNU toolchain 14.2 (`arm-none-linux-gnueabihf`), Quartus Prime Lite
25.1 for Cyclone V, and optionally Verilator for the GPU test.

```
cd fpga && ./build.sh                 # -> output_files/soc_system.rbf
cd minecraft && make                  # -> mc, terrad (static ARM binaries)
make host && ./mc_host --shot 0 95 0 30 -20 shot.ppm   # PC screenshot
make terrain                          # test world (128 MiB terrain.bin)
fpga/sim/run.sh                       # GPU vs C model on six views
```

## Run on the board

The board runs the Terasic Ubuntu image (user `ubuntu`). Copy
`fpga/output_files/soc_system.rbf` to the SD card's FAT partition (U-Boot
loads it at boot), and `mc`, `terrad`, `terrad.service`, `install_terrad.sh`
to `/home/ubuntu`. Once: `sudo sh install_terrad.sh`. terrad then starts the
game on boot, takes worlds from `../worldgen/run.sh` over the UART, and
logs the game's output (fps, CPU and GPU time per frame) to `mc.log`.

Manual start: `sudo ./mc [--terrain FILE] [--weather sunny|cloudy|night|snow|rain]`.
Keys: WASD move, Q/E turn, mouse look, Space jump, F fly, left/right click
break/place, 1-7 or wheel pick a block, Y cycle weather, Esc quit.
