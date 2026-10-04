#!/bin/sh
# run.sh - check span_gpu.v against the game's C model on real frames
#
#   sim/run.sh            build the simulator and run the standard views
#
# Needs Verilator and the PC build of the game (minecraft/mc_host).
set -e
cd "$(dirname "$0")"
GAME=../../minecraft
OUT=out
mkdir -p "$OUT"

verilator --cc --exe --build -O3 -Wno-fatal -Wno-WIDTH -Wno-UNUSED \
    --top-module span_gpu -Mdir obj ../span_gpu.v tb.cpp >/dev/null

make -C "$GAME" mc_host >/dev/null
shot() {
    name=$1; shift
    (cd "$GAME" && ./mc_host "$@" --dump "$OLDPWD/$OUT/$name.cmd" \
        --shot $SHOT "$OLDPWD/$OUT/$name.ppm" >/dev/null)
    printf '%-10s ' "$name"
    ./obj/Vspan_gpu "$OUT/$name.cmd" "$OUT/$name.ppm" 2 | tr '\n' ' '
    echo
}
SHOT="0 95 0 30 -20"            shot underwater
SHOT="40 75 60 200 -25"         shot build --build
SHOT="40 75 60 200 -10"         shot rain --weather rain
SHOT="-50 90 30 120 5"          shot night --weather night
SHOT="40 110 60 200 -30"        shot far --dist 15
SHOT="300 70 -200 45 -15"       shot snow --weather snow
