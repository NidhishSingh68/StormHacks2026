/*
 * mc.h - shared definitions for the DE1-SoC voxel demo
 */
#ifndef MC_H
#define MC_H

#include <stdint.h>

/* ---- display (the FPGA framebuffer) -------------------------------------- */

#define SCREEN_W        640
#define SCREEN_H        480
#define SCREEN_SIZE     (SCREEN_W * SCREEN_H)

/* The frame is rendered in horizontal bands so both cores can share it. */
#define BAND_H          16
#define NUM_BANDS       (SCREEN_H / BAND_H)

/* ---- world --------------------------------------------------------------- */

#define CHUNK_SIZE      16      /* blocks along x and z */
#define CHUNK_H         128     /* blocks along y (world height) */

#define RENDER_DIST_DEFAULT 8   /* radius, in chunks, loaded and drawn */
#define RENDER_DIST_MAX     15

/*
 * Chunks live in a CHUNK_GRID x CHUNK_GRID ring buffer indexed by chunk
 * coordinate modulo CHUNK_GRID. As long as the loaded square
 * (2 * render distance + 1 chunks wide) fits, every loaded chunk has its own
 * slot and a slot holding some other chunk is out of range and reusable.
 */
#define CHUNK_GRID      32

enum {
    BLOCK_AIR,
    BLOCK_GRASS,
    BLOCK_DIRT,
    BLOCK_STONE,
    BLOCK_SAND,
    BLOCK_SNOW,
    BLOCK_BEDROCK,
    BLOCK_WATER,
    BLOCK_LOG,
    BLOCK_LEAVES,
    NUM_BLOCK_TYPES
};

/* Can the player walk through it? */
static inline int block_solid(int b)
{
    return b != BLOCK_AIR && b != BLOCK_WATER;
}

static inline int floor_div(int a, int b)
{
    return (a >= 0) ? a / b : -((-a + b - 1) / b);
}

static inline int floor_mod(int a, int b)
{
    int m = a % b;

    return m < 0 ? m + b : m;
}

#endif
