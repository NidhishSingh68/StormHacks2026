# Makefile - DE1-SoC voxel demo
#
#   make          ARM binary for the board (mc)
#   make host     PC build for screenshots/testing (mc_host)
#   make terrain  generate terrain.bin (4096 x 4096 columns, 128 MiB)
#   make terrad   world receiver / game supervisor for the board (terrad)

CROSS   ?= $(HOME)/toolchains/arm-gnu-toolchain-14.2.rel1-x86_64-arm-none-linux-gnueabihf/bin/arm-none-linux-gnueabihf-
CC_ARM  := $(CROSS)gcc
CC_HOST ?= gcc

SRCS    := main.c world.c render.c fpga.c input.c terrain.c
HDRS    := mc.h world.h render.h fpga.h input.h terrain.h
WARN    := -Wall -Wextra

ARM_FLAGS  := -O3 -mcpu=cortex-a9 -mfpu=neon -mfloat-abi=hard -ffast-math \
              -fno-strict-aliasing $(WARN) -static -pthread
HOST_FLAGS := -O2 -ffast-math -fno-strict-aliasing $(WARN) -DHOST -pthread

all: mc terrad

mc: $(SRCS) $(HDRS)
	$(CC_ARM) $(ARM_FLAGS) $(SRCS) -o $@ -lm

host: mc_host terrad_host

terrad: terrad.c fpga.c fpga.h font.h mc.h
	$(CC_ARM) $(ARM_FLAGS) terrad.c fpga.c -o $@

terrad_host: terrad.c fpga.c fpga.h font.h mc.h
	$(CC_HOST) $(HOST_FLAGS) terrad.c fpga.c -o $@

mc_host: $(SRCS) $(HDRS)
	$(CC_HOST) $(HOST_FLAGS) $(SRCS) -o $@ -lm

terrain_gen: tools/terrain_gen.c mc.h terrain.h
	$(CC_HOST) -O2 $(WARN) tools/terrain_gen.c -o $@ -lm

terrain: terrain.bin

terrain.bin: terrain_gen
	./terrain_gen -o $@ -p terrain_preview.ppm

clean:
	rm -f mc mc_host terrad terrad_host terrain_gen

.PHONY: all host terrain clean
