/*
 * terrain_gen.c - write a procedural terrain file for the voxel demo
 *
 * The output follows terrain.h, so the game can't tell it from any other
 * source (e.g. a future real-world importer). All noise is periodic with
 * the map size, so the terrain wraps seamlessly when the game tiles it.
 *
 *   terrain_gen [-n SIZE] [-s SEED] [-o terrain.bin] [-p preview.ppm]
 *
 * SIZE is the map width/depth in blocks (multiple of 1024, default 4096:
 * 4096 x 4096 columns, 128 MiB).
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <math.h>
#include "../mc.h"
#include "../terrain.h"

#define SEA_LEVEL       48

static int N = 4096;
static uint32_t seed = 1;

static uint32_t hash3(uint32_t x, uint32_t z, uint32_t s)
{
    uint32_t h = x * 0x9E3779B1u ^ z * 0x85EBCA77u ^ s * 0xC2B2AE3Du;

    h ^= h >> 15;
    h *= 0x2C1B3C6Du;
    h ^= h >> 12;
    h *= 0x297A2D39u;
    h ^= h >> 15;
    return h;
}

static float rand01(uint32_t x, uint32_t z, uint32_t s)
{
    return (hash3(x, z, s) >> 8) * (1.0f / 16777216.0f);
}

/* value noise in [-1, 1] on a lattice that repeats every N blocks */
static float vnoise(float x, float z, int wavelength, uint32_t s)
{
    int period = N / wavelength;
    float fx = x / wavelength, fz = z / wavelength;
    int ix = (int)floorf(fx), iz = (int)floorf(fz);
    float tx = fx - ix, tz = fz - iz;
    int x0 = ((ix % period) + period) % period, x1 = (x0 + 1) % period;
    int z0 = ((iz % period) + period) % period, z1 = (z0 + 1) % period;
    float a, b, c, d;

    /* quintic fade for smooth slopes */
    tx = tx * tx * tx * (tx * (tx * 6 - 15) + 10);
    tz = tz * tz * tz * (tz * (tz * 6 - 15) + 10);
    a = rand01(x0, z0, s); b = rand01(x1, z0, s);
    c = rand01(x0, z1, s); d = rand01(x1, z1, s);
    return 2.0f * ((a + (b - a) * tx) + ((c + (d - c) * tx) - (a + (b - a) * tx)) * tz) - 1.0f;
}

static float fbm(float x, float z, int wavelength, int octaves, uint32_t s)
{
    float sum = 0.0f, amp = 1.0f, norm = 0.0f;
    int o;

    for (o = 0; o < octaves && wavelength >= 2; o++, wavelength /= 2, amp *= 0.5f) {
        sum += amp * vnoise(x, z, wavelength, s + o * 101u);
        norm += amp;
    }
    return sum / norm;
}

static float smoothstep(float a, float b, float x)
{
    float t = (x - a) / (b - a);

    t = t < 0 ? 0 : t > 1 ? 1 : t;
    return t * t * (3 - 2 * t);
}

static int ground_height(int x, int z, int *rock)
{
    /* land vs water, changing every few hundred blocks */
    float continent = fbm(x, z, 512, 4, seed * 7 + 1);
    float hills = fbm(x, z, 128, 4, seed * 7 + 2);
    /* sharp ridged mountains over about half of the land */
    float ridge = 1.0f - fabsf(fbm(x, z, 256, 4, seed * 7 + 3));
    float mask = smoothstep(-0.25f, 0.25f, fbm(x, z, 512, 3, seed * 7 + 4));
    float mountain = mask * ridge * ridge * ridge * 62;
    float h = SEA_LEVEL - 1 + continent * 24 + hills * 6 + mountain;

    *rock = (int)mountain;

    if (h < 4) h = 4;
    if (h > CHUNK_H - 12) h = CHUNK_H - 12;
    return (int)h;
}

#define WRAP(v)  ((((v) % N) + N) % N)
#define AT(x, z) ((size_t)WRAP(z) * N + WRAP(x))

/*
 * The game spawns near column (0, 0) facing -z. Pick the point of the map
 * that should become (0, 0): flat open ground right behind a sandy beach,
 * with the water just past it, and ideally mountains somewhere in view.
 * The map is then written shifted by that offset (still seamless, since
 * all noise wraps).
 */
static void choose_origin(const uint16_t *height, const uint8_t *surface,
                          const uint8_t *feature, const uint8_t *weather,
                          int *ox, int *oz)
{
    float best = -1.0f, best_w = 0, best_m = 0, sn[11], cs[11];
    int cx, cz, i, best_sand = 0;

    for (i = 0; i <= 10; i++) {
        float a = (-50.0f + 10.0f * i) * (float)M_PI / 180.0f;

        sn[i] = sinf(a);
        cs[i] = cosf(a);
    }
    *ox = *oz = 0;

    for (cz = 0; cz < N; cz += 8) {
        for (cx = 0; cx < N; cx += 8) {
            int dx, dz, d, sand = 0, water = 0, wtotal = 0, peaks = 0, ptotal = 0, ok = 1;
            int h0 = height[AT(cx, cz)];
            float wf, mf, score;

            /* flat open grass or sand to stand on, above the water */
            for (dz = -2; dz <= 2 && ok; dz++)
                for (dx = -2; dx <= 2 && ok; dx++) {
                    size_t k = AT(cx + dx, cz + dz);

                    if ((surface[k] != BLOCK_GRASS && surface[k] != BLOCK_SAND) ||
                        feature[k] || abs(height[k] - h0) > 1 || height[k] < SEA_LEVEL + 1 ||
                        weather[k] != WEATHER_SUNNY)
                        ok = 0;
                }
            if (!ok)
                continue;

            /* the beach: sand straight ahead, 3..14 blocks out */
            for (d = 3; d <= 14; d++)
                if (surface[AT(cx, cz - d)] == BLOCK_SAND && height[AT(cx, cz - d)] >= SEA_LEVEL)
                    sand++;
            if (sand < 3)
                continue;

            /* the water past it, 12..60 blocks ahead */
            for (d = 12; d <= 60; d += 4)
                for (i = 2; i <= 8; i++, wtotal++)
                    if (height[AT(cx + (int)(d * sn[i]), cz - (int)(d * cs[i]))] < SEA_LEVEL)
                        water++;
            wf = (float)water / wtotal;
            if (wf < 0.3f)
                continue;

            /* mountains anywhere around within view distance: a bonus */
            for (d = 30; d <= 100; d += 10)
                for (i = 0; i < 16; i++, ptotal++) {
                    float a = i * (float)M_PI / 8.0f;

                    if (height[AT(cx + (int)(d * sinf(a)), cz + (int)(d * cosf(a)))] >= 88)
                        peaks++;
                }
            mf = (float)peaks / ptotal;

            score = fminf(wf, 0.7f) + fminf(mf, 0.15f) / 0.15f * 0.6f;
            if (score > best) {
                best = score;
                best_w = wf;
                best_m = mf;
                best_sand = sand;
                *ox = cx;
                *oz = cz;
            }
        }
    }
    fprintf(stderr, "origin moved to generated column (%d, %d): beach %d blocks, "
            "%.0f%% water ahead, %.0f%% peaks around\n", *ox, *oz, best_sand,
            best_w * 100, best_m * 100);
}

int main(int argc, char **argv)
{
    const char *out_path = "terrain.bin", *preview_path = NULL;
    struct terrain_header hdr;
    uint16_t *height;
    struct terrain_column *tile;
    uint8_t *surface, *feature, *rock, *weather;
    FILE *f;
    int i, x, z, tx, tz, ox, oz;
    long trees = 0, water = 0;

    for (i = 1; i < argc; i++) {
        if (!strcmp(argv[i], "-n") && i + 1 < argc)      N = atoi(argv[++i]);
        else if (!strcmp(argv[i], "-s") && i + 1 < argc) seed = (uint32_t)atoi(argv[++i]);
        else if (!strcmp(argv[i], "-o") && i + 1 < argc) out_path = argv[++i];
        else if (!strcmp(argv[i], "-p") && i + 1 < argc) preview_path = argv[++i];
        else {
            fprintf(stderr, "usage: %s [-n SIZE] [-s SEED] [-o out.bin] [-p preview.ppm]\n", argv[0]);
            return 2;
        }
    }
    if (N < 1024 || N % 1024) {
        fprintf(stderr, "size must be a multiple of 1024\n");
        return 2;
    }

    height = malloc((size_t)N * N * sizeof(*height));
    surface = malloc((size_t)N * N);
    feature = calloc((size_t)N * N, 1);
    rock = malloc((size_t)N * N);
    weather = malloc((size_t)N * N);
    tile = malloc(TERRAIN_TILE_BYTES);
    if (!height || !surface || !feature || !rock || !weather || !tile) {
        fprintf(stderr, "out of memory\n");
        return 1;
    }

    fprintf(stderr, "heights...\n");
    for (z = 0; z < N; z++) {
        for (x = 0; x < N; x++) {
            int r;

            height[(size_t)z * N + x] = (uint16_t)ground_height(x, z, &r);
            rock[(size_t)z * N + x] = (uint8_t)(r > 255 ? 255 : r);
        }
    }

    fprintf(stderr, "surfaces...\n");
    for (z = 0; z < N; z++) {
        for (x = 0; x < N; x++) {
            int h = height[(size_t)z * N + x], slope = 0, k;
            static const int nb[4][2] = { { 1, 0 }, { -1, 0 }, { 0, 1 }, { 0, -1 } };
            uint8_t s;

            for (k = 0; k < 4; k++) {
                int nx = (x + nb[k][0] + N) % N, nz = (z + nb[k][1] + N) % N;
                int d = abs(h - height[(size_t)nz * N + nx]);

                if (d > slope)
                    slope = d;
            }
            if (h < SEA_LEVEL)              s = (h >= SEA_LEVEL - 5) ? BLOCK_SAND : BLOCK_DIRT;
            else if (h <= SEA_LEVEL + 2)    s = BLOCK_SAND;
            else if (h > 98)                s = BLOCK_SNOW;
            else if (h > 74 || slope >= 2 || rock[(size_t)z * N + x] > 10)
                                            s = BLOCK_STONE;
            else                            s = BLOCK_GRASS;
            surface[(size_t)z * N + x] = s;
        }
    }

    /*
     * Weather: zones a few hundred blocks across from slow noise, and snow
     * over every mountain area, whose grass turns to snowy ground.
     */
    fprintf(stderr, "weather...\n");
    for (z = 0; z < N; z++) {
        for (x = 0; x < N; x++) {
            size_t idx = (size_t)z * N + x;
            float v = fbm(x, z, 1024, 2, seed * 7 + 6);
            float night = fbm(x, z, 1024, 2, seed * 7 + 7);
            uint8_t w;

            if (night > 0.45f)          w = WEATHER_NIGHT;
            else if (v < -0.35f)        w = WEATHER_RAIN;
            else if (v < -0.1f)         w = WEATHER_CLOUDY;
            else                        w = WEATHER_SUNNY;
            if (rock[idx] > 10 || height[idx] > 84) {
                w = WEATHER_SNOW;
                if (surface[idx] == BLOCK_GRASS)
                    surface[idx] = BLOCK_SNOW;
            }
            weather[idx] = w;
        }
    }

    /* trees: one candidate per 5 x 5 cell, kept by local forest density */
    fprintf(stderr, "trees...\n");
    for (z = 0; z < N; z += 5) {
        for (x = 0; x < N; x += 5) {
            int px = (x + (int)(hash3(x, z, seed + 11) % 5)) % N;
            int pz = (z + (int)(hash3(x, z, seed + 12) % 5)) % N;
            size_t idx = (size_t)pz * N + px;
            /* mostly scattered trees, with the odd grove */
            float density = 0.04f + 0.5f * smoothstep(0.25f, 0.6f, fbm(px, pz, 256, 3, seed * 7 + 5));

            if (surface[idx] != BLOCK_GRASS || height[idx] <= SEA_LEVEL + 2)
                continue;
            if (rand01(px, pz, seed + 13) < density * 0.6f) {
                feature[idx] = FEATURE_TREE;
                trees++;
            }
        }
    }

    choose_origin(height, surface, feature, weather, &ox, &oz);

    f = fopen(out_path, "wb");
    if (!f) {
        perror(out_path);
        return 1;
    }
    memset(&hdr, 0, sizeof(hdr));
    memcpy(hdr.magic, TERRAIN_MAGIC, 8);
    hdr.version = TERRAIN_VERSION;
    hdr.header_size = TERRAIN_HEADER_SIZE;
    hdr.width = (uint32_t)N;
    hdr.depth = (uint32_t)N;
    hdr.tile_size = TERRAIN_TILE;
    hdr.record_size = sizeof(struct terrain_column);
    hdr.height_limit = CHUNK_H;
    hdr.sea_level = SEA_LEVEL;
    hdr.meters_per_block = 1.0;
    snprintf(hdr.source, sizeof(hdr.source), "terrain_gen seed %u", seed);
    fwrite(&hdr, sizeof(hdr), 1, f);
    {
        static const char pad[TERRAIN_HEADER_SIZE];

        fwrite(pad, TERRAIN_HEADER_SIZE - sizeof(hdr), 1, f);
    }

    fprintf(stderr, "writing %s...\n", out_path);
    for (tz = 0; tz < N / TERRAIN_TILE; tz++) {
        for (tx = 0; tx < N / TERRAIN_TILE; tx++) {
            for (z = 0; z < TERRAIN_TILE; z++) {
                for (x = 0; x < TERRAIN_TILE; x++) {
                    size_t idx = AT(tx * TERRAIN_TILE + x + ox, tz * TERRAIN_TILE + z + oz);
                    struct terrain_column *c = &tile[z * TERRAIN_TILE + x];

                    c->height = height[idx];
                    c->water = height[idx] < SEA_LEVEL ? SEA_LEVEL : 0;
                    c->surface = surface[idx];
                    c->feature = feature[idx];
                    c->weather = weather[idx];
                    c->reserved = 0;
                    if (c->water)
                        water++;
                }
            }
            fwrite(tile, TERRAIN_TILE_BYTES, 1, f);
        }
    }
    if (fclose(f) != 0) {
        perror(out_path);
        return 1;
    }
    fprintf(stderr, "%d x %d columns, %.1f%% water, %ld trees\n", N, N,
            100.0 * water / ((double)N * N), trees);

    /* top-down preview, one pixel per 4 x 4 columns, with hill shading */
    if (preview_path) {
        int w = N / 4;
        FILE *p = fopen(preview_path, "wb");

        if (!p) {
            perror(preview_path);
            return 1;
        }
        fprintf(p, "P6\n%d %d\n255\n", w, w);
        for (z = 0; z < N; z += 4) {
            for (x = 0; x < N; x += 4) {
                size_t idx = AT(x + ox, z + oz);
                int h = height[idx], hw = height[AT(x + ox - 4, z + oz)];
                float shade = 1.0f + 0.08f * (h - hw);
                float r, g, b;
                uint8_t px[3];

                switch (h < SEA_LEVEL ? BLOCK_WATER : surface[idx]) {
                case BLOCK_WATER: r = 52;  g = 96;  b = 220; shade = 1.0f; break;
                case BLOCK_SAND:  r = 218; g = 206; b = 160; break;
                case BLOCK_STONE: r = 128; g = 128; b = 128; break;
                case BLOCK_SNOW:  r = 242; g = 246; b = 250; break;
                default:          r = 95;  g = 159; b = 53;  break;
                }
                if (feature[idx] || feature[AT(x + ox + 1, z + oz)] ||
                    feature[AT(x + ox + 2, z + oz)] || feature[AT(x + ox + 3, z + oz)]) {
                    r = 40; g = 100; b = 30;
                }
                switch (weather[idx]) {     /* tint the map by weather */
                case WEATHER_NIGHT:  r *= 0.45f; g *= 0.45f; b *= 0.6f; break;
                case WEATHER_RAIN:   r *= 0.7f;  g *= 0.75f; b *= 0.9f; break;
                case WEATHER_CLOUDY: r = r * 0.8f + 25; g = g * 0.8f + 25; b = b * 0.8f + 25; break;
                default: break;
                }
                px[0] = (uint8_t)fminf(255, r * shade);
                px[1] = (uint8_t)fminf(255, g * shade);
                px[2] = (uint8_t)fminf(255, b * shade);
                fwrite(px, 3, 1, p);
            }
        }
        fclose(p);
    }
    return 0;
}
