/*
 * terrain.h - terrain data file: format and reader
 *
 * The world is built from a terrain file describing a rectangle of block
 * columns. Beyond its edges the same terrain repeats (wraps). Anything that
 * writes this format can supply the world: tools/terrain_gen.c makes a
 * procedural one; a real-world importer (elevation + land cover) can write
 * the same structure later.
 *
 * Layout (little-endian):
 *
 *   offset 0            struct terrain_header, zero-padded to header_size
 *   offset header_size  tiles of TERRAIN_TILE x TERRAIN_TILE columns,
 *                       tile rows along z, tiles along x within a row:
 *                         tile (tx, tz) at header_size +
 *                             (tz * tiles_x + tx) * TILE_BYTES
 *                       within a tile, columns row-major: (x, z) at
 *                         z * TERRAIN_TILE + x
 *
 * Tiles match the game's 16 x 16 chunks, so loading a chunk reads one
 * contiguous 2 KB block (plus its neighbours' for trees that overhang).
 *
 * A column record says what the column is made of, not every block in it;
 * the game fills in the layers (surface block, a few blocks of dirt or sand
 * under it, stone, bedrock at y = 0), water, and trees.
 */
#ifndef TERRAIN_H
#define TERRAIN_H

#include <stdint.h>

#define TERRAIN_MAGIC       "MCTERR1"       /* 8 bytes with the NUL */
#define TERRAIN_VERSION     1
#define TERRAIN_HEADER_SIZE 4096
#define TERRAIN_TILE        16
#define TERRAIN_TILE_COLS   (TERRAIN_TILE * TERRAIN_TILE)

enum { FEATURE_NONE = 0, FEATURE_TREE = 1 };

struct terrain_header {
    char     magic[8];          /* TERRAIN_MAGIC */
    uint32_t version;           /* TERRAIN_VERSION */
    uint32_t header_size;       /* bytes before the first tile */
    uint32_t width;             /* columns along x, multiple of TERRAIN_TILE */
    uint32_t depth;             /* columns along z, multiple of TERRAIN_TILE */
    uint32_t tile_size;         /* TERRAIN_TILE */
    uint32_t record_size;       /* sizeof(struct terrain_column) */
    uint32_t height_limit;      /* world height the data was made for */
    uint32_t sea_level;         /* informational */
    /* optional real-world anchor of column (0, 0); zero if synthetic */
    double   origin_lat;
    double   origin_lon;
    double   meters_per_block;  /* horizontal scale */
    char     source[64];        /* free text: generator / data source */
};

struct terrain_column {
    uint16_t height;            /* blocks y < height are ground */
    uint16_t water;             /* blocks height <= y < water are water */
    uint8_t  surface;           /* block id of the top ground block */
    uint8_t  feature;           /* FEATURE_* standing on this column */
    uint8_t  reserved[2];
};

#define TERRAIN_TILE_BYTES  (TERRAIN_TILE_COLS * sizeof(struct terrain_column))

/*
 * Open a terrain file. If it can't be used, the reader falls back to a flat
 * grass world and returns -1 (the game still runs).
 */
int  terrain_open(const char *path);
void terrain_close(void);

/* Columns of the tile containing chunk (cx, cz), wrapping. Thread-safe. */
void terrain_tile(int cx, int cz, struct terrain_column out[TERRAIN_TILE_COLS]);

/* The record for world column (x, z), wrapping. */
void terrain_column_at(int x, int z, struct terrain_column *out);

#endif
