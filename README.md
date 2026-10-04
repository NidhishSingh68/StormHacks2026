# StormHacks 2026: real places as voxel worlds on an FPGA

Pick a place on the map; its real terrain (elevation, land cover, water,
climate) becomes a Minecraft-like world that is sent to a DE1-SoC board and
played there, rendered by a GPU we built in the board's FPGA.

```
worldgen/    PC side (Python + web UI)
  worldgen.py      fetches elevation and land-cover data, classifies it
                   (water, beach, desert, forest, snow line, weather),
                   builds terrain.bin, serves the UI, uploads to the board
  indexnew.html    the UI: pick a place, Generate, Upload
  run.sh           starts everything: ./worldgen/run.sh [serial port]
de1soc/      board side (C on the ARM, Verilog in the FPGA), see de1soc/README.md
  minecraft/       the game, and terrad (receives worlds over the UART)
  fpga/            VGA, framebuffer, and the span GPU
```

## How it fits together

```
 browser UI ──HTTP──> worldgen.py ──USB-UART (115200)──> terrad (board)
                         │                                  │ restarts
                    terrain data                            v
                  (DEM, land cover)              mc: game logic, culling,
                                                 GPU command lists in DDR
                                                            │ ACP reads
                                                            v
                                             FPGA span GPU: per-pixel
                                             rendering -> framebuffer -> VGA
```

1. `worldgen/run.sh` starts the server and opens the UI. Generate builds a
   world for the selected place; Upload sends it (compressed, CRC-checked)
   to the board.
2. On the board, `terrad` owns the serial port: it stores the world, shows a
   loading screen, and restarts the game on it.
3. The game (`de1soc/minecraft`) runs on the dual Cortex-A9: input, physics,
   chunks and meshing, culling, and a per-frame list of drawing commands.
4. The span GPU (`de1soc/fpga/span_gpu.v`) reads that list from DDR and
   draws every pixel (depth, perspective, block grid lines, fog, shading,
   water and glass, dithering) at one pixel per clock, tear-free.

Building and flashing the board side: `de1soc/README.md`.
