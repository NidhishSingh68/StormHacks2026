/*
 * terrain.c - terrain file reader (format in terrain.h)
 */

#include <stdio.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include "mc.h"
#include "terrain.h"

#define FLAT_HEIGHT     40

static int fd = -1;
static struct terrain_header hdr;
static uint32_t tiles_x, tiles_z;

int terrain_open(const char *path)
{
    off_t size, need;

    fd = open(path, O_RDONLY);
    if (fd < 0) {
        perror(path);
        goto flat;
    }
    if (pread(fd, &hdr, sizeof(hdr), 0) != (ssize_t)sizeof(hdr) ||
        memcmp(hdr.magic, TERRAIN_MAGIC, 8) != 0 ||
        hdr.version != TERRAIN_VERSION ||
        hdr.tile_size != TERRAIN_TILE ||
        hdr.record_size != sizeof(struct terrain_column) ||
        hdr.width == 0 || hdr.depth == 0 ||
        hdr.width % TERRAIN_TILE || hdr.depth % TERRAIN_TILE) {
        fprintf(stderr, "%s: not a usable terrain file\n", path);
        goto flat;
    }
    tiles_x = hdr.width / TERRAIN_TILE;
    tiles_z = hdr.depth / TERRAIN_TILE;
    size = lseek(fd, 0, SEEK_END);
    need = (off_t)hdr.header_size + (off_t)tiles_x * tiles_z * TERRAIN_TILE_BYTES;
    if (size < need) {
        fprintf(stderr, "%s: truncated (%lld of %lld bytes)\n", path,
                (long long)size, (long long)need);
        goto flat;
    }
    printf("terrain: %s, %u x %u columns, sea level %u (%.48s)\n", path,
           hdr.width, hdr.depth, hdr.sea_level, hdr.source);
    return 0;

flat:
    if (fd >= 0)
        close(fd);
    fd = -1;
    printf("terrain: using a flat world\n");
    return -1;
}

void terrain_close(void)
{
    if (fd >= 0)
        close(fd);
    fd = -1;
}

void terrain_tile(int cx, int cz, struct terrain_column out[TERRAIN_TILE_COLS])
{
    int i;

    if (fd >= 0) {
        uint32_t tx = (uint32_t)floor_mod(cx, (int)tiles_x);
        uint32_t tz = (uint32_t)floor_mod(cz, (int)tiles_z);
        off_t off = (off_t)hdr.header_size +
                    ((off_t)tz * tiles_x + tx) * TERRAIN_TILE_BYTES;

        if (pread(fd, out, TERRAIN_TILE_BYTES, off) == (ssize_t)TERRAIN_TILE_BYTES) {
            /* clamp to what this world can hold */
            for (i = 0; i < TERRAIN_TILE_COLS; i++) {
                if (out[i].height < 1) out[i].height = 1;
                if (out[i].height > CHUNK_H - 10) out[i].height = CHUNK_H - 10;
                if (out[i].water > CHUNK_H - 10) out[i].water = CHUNK_H - 10;
                if (out[i].surface >= NUM_BLOCK_TYPES || out[i].surface == BLOCK_AIR)
                    out[i].surface = BLOCK_GRASS;
                if (out[i].weather >= NUM_WEATHERS)
                    out[i].weather = WEATHER_SUNNY;
            }
            return;
        }
    }

    for (i = 0; i < TERRAIN_TILE_COLS; i++) {
        out[i].height = FLAT_HEIGHT;
        out[i].water = 0;
        out[i].surface = BLOCK_GRASS;
        out[i].feature = FEATURE_NONE;
        out[i].weather = WEATHER_SUNNY;
        out[i].reserved = 0;
    }
}

void terrain_column_at(int x, int z, struct terrain_column *out)
{
    struct terrain_column t[TERRAIN_TILE_COLS];

    terrain_tile(floor_div(x, TERRAIN_TILE), floor_div(z, TERRAIN_TILE), t);
    *out = t[floor_mod(z, TERRAIN_TILE) * TERRAIN_TILE + floor_mod(x, TERRAIN_TILE)];
}
