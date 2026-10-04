/*
 * render.c - software rasterizer for the voxel world
 *
 * Per frame, render_setup() turns the greedy-meshed face rectangles of every
 * visible chunk into screen-space convex polygons and sorts them into the
 * horizontal bands they cover. Culling happens at three levels: chunks
 * outside the view frustum (using each chunk's real height range), whole
 * groups of faces pointing away from the eye, and single back faces.
 * Chunks are visited nearest first, so near geometry fills the z-buffer
 * early and hidden pixels behind it are rejected before shading.
 *
 * render_band() then draws one band: sky gradient, sun, polygons with a
 * z-buffer, crosshair. Bands are independent, so both cores draw them.
 *
 * Shading has no textures: each block gets one of three tones of its colour
 * (from a hash of its position) and a darker ~1 pixel line along its edges,
 * which fades out where blocks get too small on screen. Faces are lit by
 * direction, fogged towards the horizon colour with distance, and reduced to
 * RGB332 with a 4x4 ordered dither through precomputed lookup tables.
 *
 * Depth: 1/z is linear in screen space, so the z-buffer stores 1/z and needs
 * no division per pixel. On horizontal faces 1/z is constant along a
 * scanline, so such a span costs one division and world coordinates step by
 * a constant per pixel; vertical faces are perspective-corrected every 8
 * pixels.
 */

#include <math.h>
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include "render.h"
#include "world.h"
#include "font.h"

#define CX              (SCREEN_W * 0.5f)
#define CY              (SCREEN_H * 0.5f)
#define FOV_X_DEG       70.0f
#define NEAR_Z          0.05f
#define HALF            (1.0f / QUAD_SCALE)     /* quad units -> blocks */
#define FOG_LEVELS      32

enum { FACE_TOP, FACE_BOTTOM, FACE_X, FACE_Z, NUM_FACES };
enum { TONE_A, TONE_B, TONE_C, TONE_LINE, TONE_SEL, NUM_TONES };
#define TONE_STRIDE     (FOG_LEVELS * 16)

#define SKY_LEVELS      64
#define SUN_CORE        18      /* half-size in pixels */
#define SUN_HALO        28

static float focal, fog_start, fog_scale;

static uint8_t pal[NUM_BLOCK_TYPES][NUM_FACES][NUM_TONES][FOG_LEVELS][16];
static uint8_t sky_lut[SKY_LEVELS][16];
static uint8_t sun_lut[2][16];
static uint8_t water_tint[256];

static const uint8_t bayer[16] = {
     0,  8,  2, 10,
    12,  4, 14,  6,
     3, 11,  1,  9,
    15,  7, 13,  5,
};

/* block colours: top, bottom, sides (before directional light) */
static const float block_rgb[NUM_BLOCK_TYPES][3][3] = {
    [BLOCK_GRASS]   = { {  95, 159,  53 }, { 134,  96,  67 }, { 134,  96,  67 } },
    [BLOCK_DIRT]    = { { 134,  96,  67 }, { 134,  96,  67 }, { 134,  96,  67 } },
    [BLOCK_STONE]   = { { 128, 128, 128 }, { 128, 128, 128 }, { 128, 128, 128 } },
    [BLOCK_SAND]    = { { 218, 206, 160 }, { 218, 206, 160 }, { 218, 206, 160 } },
    [BLOCK_SNOW]    = { { 242, 246, 250 }, { 242, 246, 250 }, { 242, 246, 250 } },
    [BLOCK_BEDROCK] = { {  70,  70,  70 }, {  70,  70,  70 }, {  70,  70,  70 } },
    [BLOCK_WATER]   = { {  52,  96, 220 }, {  52,  96, 220 }, {  52,  96, 220 } },
    [BLOCK_LOG]     = { { 160, 130,  80 }, { 160, 130,  80 }, { 104,  82,  50 } },
    [BLOCK_LEAVES]  = { {  60, 128,  42 }, {  60, 128,  42 }, {  60, 128,  42 } },
    [BLOCK_COBBLE]  = { { 118, 118, 118 }, { 118, 118, 118 }, { 118, 118, 118 } },
    [BLOCK_GLASS]   = { { 225, 238, 250 }, { 225, 238, 250 }, { 225, 238, 250 } },
    [BLOCK_PLANKS]  = { { 182, 146,  92 }, { 182, 146,  92 }, { 182, 146,  92 } },
    [BLOCK_STAIRS_XN] = { { 182, 146,  92 }, { 182, 146,  92 }, { 182, 146,  92 } },
    [BLOCK_STAIRS_XP] = { { 182, 146,  92 }, { 182, 146,  92 }, { 182, 146,  92 } },
    [BLOCK_STAIRS_ZN] = { { 182, 146,  92 }, { 182, 146,  92 }, { 182, 146,  92 } },
    [BLOCK_STAIRS_ZP] = { { 182, 146,  92 }, { 182, 146,  92 }, { 182, 146,  92 } },
};

/*
 * See-through blocks: the colour behind them is blended towards this colour
 * by alpha. Glass also gets a light frame along its block edges.
 */
enum { MAT_WATER, MAT_GLASS, NUM_MATS };
static const float mat_rgb[NUM_MATS][3] = { { 36, 84, 205 }, { 220, 240, 255 } };
static const float mat_alpha[NUM_MATS]  = { 0.6f, 0.22f };
static uint8_t blend_lut[NUM_MATS][FOG_LEVELS][256];

/* hotbar contents and names (main.c maps stairs to a facing) */
const uint8_t hotbar_blocks[HOTBAR_SLOTS] = {
    BLOCK_COBBLE, BLOCK_DIRT, BLOCK_GRASS, BLOCK_GLASS,
    BLOCK_LOG, BLOCK_PLANKS, BLOCK_STAIRS_ZN,
};
static const char *const hotbar_names[HOTBAR_SLOTS] = {
    "COBBLESTONE", "DIRT", "GRASS", "GLASS", "LOG", "PLANKS", "STAIRS",
};
#define ICON            32
static uint8_t icons[HOTBAR_SLOTS][ICON * ICON];
static uint8_t icon_on[HOTBAR_SLOTS][ICON * ICON];      /* pixel is part of it */

/* water is a flat surface: no grid lines, no per-block tones */
static const uint8_t block_plain[NUM_BLOCK_TYPES] = { [BLOCK_WATER] = 1 };

static const float face_light[NUM_FACES] = { 1.0f, 0.5f, 0.8f, 0.65f };

static const float sun_rgb[2][3]  = { { 236, 236, 196 }, { 255, 252, 210 } };
static const float moon_rgb[2][3] = { {  90,  96, 130 }, { 226, 226, 214 } };
static const float star_rgb[3]    = { 210, 214, 235 };
static const float rain_rgb[3]    = { 165, 180, 212 };
static const float snow_rgb[3]    = { 248, 248, 252 };
static const float water_fog[3]   = {  30,  60, 150 };

/* sun direction (towards the sun), normalised in render_init; the moon
 * sits opposite it */
static float sun_dir[3] = { 0.25f, 0.42f, -0.87f };
static float moon_dir[3];
static float star_dir[NUM_STARS][3];
static uint8_t moon_lut[2][16], star_lut[16], rain_lut[16], snow_lut[16];

/* hash bits -> block tone */
static const uint8_t tone_map[4] = { TONE_A, TONE_B, TONE_C, TONE_A };

static uint8_t quantize(const float c[3], int k)
{
    float t = (bayer[k] + 0.5f) * (1.0f / 16.0f);
    int r = (int)(c[0] * (7.0f / 255.0f) + t);
    int g = (int)(c[1] * (7.0f / 255.0f) + t);
    int b = (int)(c[2] * (3.0f / 255.0f) + t);

    if (r > 7) r = 7;
    if (g > 7) g = 7;
    if (b > 3) b = 3;
    if (r < 0) r = 0;
    if (g < 0) g = 0;
    if (b < 0) b = 0;
    return (uint8_t)((r << 5) | (g << 2) | b);
}

static void mix(float out[3], const float a[3], const float b[3], float t)
{
    int i;

    for (i = 0; i < 3; i++)
        out[i] = a[i] + (b[i] - a[i]) * t;
}

static float clamp255(float v)
{
    return v < 0.0f ? 0.0f : v > 255.0f ? 255.0f : v;
}

/*
 * Hotbar icons: a small isometric cube (top, left and right faces), or a
 * stair profile, dithered to RGB332.
 */
static void make_icons(void)
{
    int i, x, y, k;

    for (i = 0; i < HOTBAR_SLOTS; i++) {
        int b = hotbar_blocks[i];

        for (y = 0; y < ICON; y++) {
            for (x = 0; x < ICON; x++) {
                float c[3], shade = 0.0f, fx = x + 0.5f, fy = y + 0.5f;
                const float *base = NULL;
                int bk = (y & 3) * 4 + (x & 3);

                if (block_is_stairs(b)) {
                    /* side view of a step: full bottom half, right top half */
                    int in = fy >= 4 && fy < 28 && fx >= 4 && fx < 28 &&
                             (fy >= 16 || fx >= 16);
                    int edge = in && (fy < 5 || fy >= 27 || fx < 5 || fx >= 27 ||
                                      (fy < 17 && fx < 17));

                    if (in) {
                        base = block_rgb[b][2];
                        shade = edge ? 0.55f : 0.85f;
                    }
                } else if (fabsf(fx - 16.0f) / 16.0f + fabsf(fy - 8.0f) / 8.0f <= 1.0f) {
                    base = block_rgb[b][0];
                    shade = 1.0f;
                } else if (fx < 16.0f && fy >= 8.0f + fx * 0.5f && fy < 24.0f + fx * 0.5f) {
                    base = block_rgb[b][2];
                    shade = 0.8f;
                } else if (fx >= 16.0f && fy >= 16.0f - (fx - 16.0f) * 0.5f &&
                           fy < 32.0f - (fx - 16.0f) * 0.5f) {
                    base = block_rgb[b][2];
                    shade = 0.62f;
                }

                icon_on[i][y * ICON + x] = base != NULL;
                if (!base)
                    continue;
                for (k = 0; k < 3; k++)
                    c[k] = clamp255(base[k] * shade);
                if (b == BLOCK_GLASS) {
                    /* glass: pale frame, see-through middle */
                    int frame = fx < 2 || fx > 30 || fabsf(fx - 16.0f) < 1.0f ||
                                fy < 2 || fy > 30;

                    for (k = 0; k < 3; k++)
                        c[k] = frame ? 245.0f : c[k] * 0.55f + 60.0f;
                }
                icons[i][y * ICON + x] = quantize(c, bk);
            }
        }
    }
}

/* All 16 dithered RGB332 versions of one colour. */
static void quantize16(const float c[3], uint8_t out[16])
{
    float r = c[0] * (7.0f / 255.0f), g = c[1] * (7.0f / 255.0f), b = c[2] * (3.0f / 255.0f);
    int k;

    for (k = 0; k < 16; k++) {
        float t = (bayer[k] + 0.5f) * (1.0f / 16.0f);
        int ri = (int)(r + t), gi = (int)(g + t), bi = (int)(b + t);

        if (ri > 7) ri = 7;
        if (gi > 7) gi = 7;
        if (bi > 3) bi = 3;
        if (ri < 0) ri = 0;
        if (gi < 0) gi = 0;
        if (bi < 0) bi = 0;
        out[k] = (uint8_t)((ri << 5) | (gi << 2) | bi);
    }
}

/* ---- weather ------------------------------------------------------------- */

static const char *const weather_names[NUM_WEATHERS] = {
    "SUNNY", "CLOUDY", "NIGHT", "SNOW", "RAIN",
};

static const struct env weather_env[NUM_WEATHERS] = {
    [WEATHER_SUNNY]  = { { 178, 212, 255 }, {  92, 146, 242 }, 1.00f, 0.35f, 0.80f, 1, 0, 0, 0, 0 },
    /* greys picked close to exact RGB332 colours, so dithering stays neutral */
    [WEATHER_CLOUDY] = { { 146, 146, 170 }, { 118, 118, 155 }, 0.78f, 0.25f, 0.65f, 0, 0, 0, 0, 0 },
    [WEATHER_NIGHT]  = { {  28,  36,  70 }, {   6,  10,  30 }, 0.32f, 0.30f, 0.75f, 0, 1, 1, 0, 0 },
    [WEATHER_SNOW]   = { { 182, 182, 180 }, { 146, 146, 170 }, 0.85f, 0.12f, 0.50f, 0, 0, 0, 0, 1 },
    [WEATHER_RAIN]   = { { 109, 109, 128 }, {  73,  73,  96 }, 0.60f, 0.15f, 0.55f, 0, 0, 0, 1, 0 },
};

static struct env cur_env;          /* what the tables were built for */
static float render_range;          /* render distance in blocks */

const char *weather_name(int w)
{
    return (w >= 0 && w < NUM_WEATHERS) ? weather_names[w] : "?";
}

void env_for_weather(int w, struct env *out)
{
    *out = weather_env[(w >= 0 && w < NUM_WEATHERS) ? w : WEATHER_SUNNY];
}

void env_mix(struct env *out, const struct env *a, const struct env *b, float t)
{
    const float *pa = (const float *)a, *pb = (const float *)b;
    float *po = (float *)out;
    unsigned i;

    /* struct env is all floats */
    for (i = 0; i < sizeof(*out) / sizeof(float); i++)
        po[i] = pa[i] + (pb[i] - pa[i]) * t;
}

/*
 * Rebuild the colour tables for a sky / light / fog setting. The band
 * renderer reads them, so call this only between frames.
 */
void render_set_env(const struct env *e)
{
    static const float tone_mul[NUM_TONES] = { 1.0f, 0.92f, 1.07f, 0.62f, 1.0f };
    const float *fogc = e->sky_horizon;
    int b, f, t, l, k;

    cur_env = *e;
    fog_start = render_range * e->fog_near;
    fog_scale = (FOG_LEVELS - 1) / (render_range * e->fog_far - fog_start);

    for (b = 1; b < NUM_BLOCK_TYPES; b++) {
        for (f = 0; f < NUM_FACES; f++) {
            const float *base = block_rgb[b][f == FACE_TOP ? 0 : f == FACE_BOTTOM ? 1 : 2];

            for (t = 0; t < NUM_TONES; t++) {
                float lit[3];
                int tt = block_plain[b] ? TONE_A : t;

                for (k = 0; k < 3; k++) {
                    lit[k] = base[k] * face_light[f] * tone_mul[tt];
                    if (tt == TONE_SEL)         /* halfway to white */
                        lit[k] = lit[k] * 0.5f + 128.0f;
                    if (b == BLOCK_GLASS)       /* glass frame: light */
                        lit[k] = base[k];
                    lit[k] = clamp255(lit[k] * (tt == TONE_SEL ? fmaxf(e->light, 0.6f) : e->light));
                }
                for (l = 0; l < FOG_LEVELS; l++) {
                    float c[3];

                    mix(c, lit, fogc, (float)l / (FOG_LEVELS - 1));
                    quantize16(c, pal[b][f][t][l]);
                }
            }
        }
    }

    for (l = 0; l < SKY_LEVELS; l++) {
        float c[3];

        mix(c, e->sky_horizon, e->sky_zenith, (float)l / (SKY_LEVELS - 1));
        quantize16(c, sky_lut[l]);
    }

    /* see-through blocks: background colour -> colour seen through them */
    for (b = 0; b < NUM_MATS; b++) {
        for (l = 0; l < FOG_LEVELS; l++) {
            float m[3], lit[3];

            for (k = 0; k < 3; k++)
                lit[k] = mat_rgb[b][k] * e->light;
            mix(m, lit, fogc, (float)l / (FOG_LEVELS - 1));
            for (k = 0; k < 256; k++) {
                float c[3] = { (float)((k >> 5) * 255 / 7),
                               (float)(((k >> 2) & 7) * 255 / 7),
                               (float)((k & 3) * 85) };

                mix(c, c, m, mat_alpha[b]);
                blend_lut[b][l][k] = quantize(c, 5);
            }
        }
    }
}

void render_init(int render_dist)
{
    struct env e;
    float len;
    int l, k;

    focal = CX / tanf(FOV_X_DEG * 0.5f * (float)M_PI / 180.0f);
    render_range = (float)(render_dist * CHUNK_SIZE);

    for (l = 0; l < 2; l++)
        quantize16(sun_rgb[l], sun_lut[l]);
    for (l = 0; l < 2; l++)
        quantize16(moon_rgb[l], moon_lut[l]);
    quantize16(star_rgb, star_lut);
    quantize16(rain_rgb, rain_lut);
    quantize16(snow_rgb, snow_lut);

    /* RGB332 -> same colour seen through water */
    for (k = 0; k < 256; k++) {
        float c[3] = { (float)((k >> 5) * 255 / 7), (float)(((k >> 2) & 7) * 255 / 7),
                       (float)((k & 3) * 85) };

        mix(c, c, water_fog, 0.7f);
        water_tint[k] = quantize(c, 5);
    }

    len = sqrtf(sun_dir[0] * sun_dir[0] + sun_dir[1] * sun_dir[1] +
                sun_dir[2] * sun_dir[2]);
    for (k = 0; k < 3; k++) {
        sun_dir[k] /= len;
        moon_dir[k] = k == 1 ? sun_dir[k] : -sun_dir[k];
    }

    /* stars: fixed random directions over the upper sky */
    for (l = 0; l < NUM_STARS; l++) {
        uint32_t h = (uint32_t)l * 0x9E3779B1u;
        float az, el;

        h ^= h >> 15; h *= 0x2C1B3C6Du; h ^= h >> 12;
        az = (h & 0xFFFF) * (2.0f * (float)M_PI / 65536.0f);
        el = 0.05f + ((h >> 16) & 0xFFFF) * (1.45f / 65536.0f);
        star_dir[l][0] = cosf(el) * sinf(az);
        star_dir[l][1] = sinf(el);
        star_dir[l][2] = cosf(el) * cosf(az);
    }

    make_icons();
    env_for_weather(WEATHER_SUNNY, &e);
    render_set_env(&e);
}

/* ---- frame setup --------------------------------------------------------- */

static inline float dot3(const float a[3], const float b[3])
{
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

/* Conservative frustum test of a box. */
static int box_visible(const struct frame *fr, const float lo[3], const float hi[3])
{
    const float tx = (CX / focal) * 1.02f, ty = (CY / focal) * 1.02f;
    int behind = 1, left = 1, right = 1, below = 1, above = 1;
    int i;

    for (i = 0; i < 8; i++) {
        float q[3] = { ((i & 1) ? hi[0] : lo[0]) - fr->eye[0],
                       ((i & 2) ? hi[1] : lo[1]) - fr->eye[1],
                       ((i & 4) ? hi[2] : lo[2]) - fr->eye[2] };
        float cx = dot3(q, fr->right), cy = dot3(q, fr->up), cz = dot3(q, fr->fwd);

        if (cz > NEAR_Z)        behind = 0;
        if (cx <  cz * tx)      right = 0;
        if (cx > -cz * tx)      left = 0;
        if (cy <  cz * ty)      above = 0;
        if (cy > -cz * ty)      below = 0;
    }
    return !(behind || left || right || below || above);
}

static void add_quad(struct frame *fr, const struct quad *q, const int *sel)
{
    int a = q->axis, u = (a + 1) % 3, v = (a + 2) % 3;
    float cam[4][3], clip[MAX_POLY_EDGES][3], sx[MAX_POLY_EDGES], sy[MAX_POLY_EDGES];
    float minx = 1e30f, maxx = -1e30f, miny = 1e30f, maxy = -1e30f, k;
    float base[3], du[3], dv[3];
    struct poly *p;
    int i, n = 0, ytop, ybot, b0, b1, idx;

    if (fr->npolys >= MAX_POLYS)
        return;

    /* camera-space corners: base + s * du + t * dv */
    {
        float w[3], eu, ev;

        w[a] = (float)q->d * HALF - fr->eye[a];
        w[u] = (float)q->u0 * HALF - fr->eye[u];
        w[v] = (float)q->v0 * HALF - fr->eye[v];
        base[0] = dot3(w, fr->right);
        base[1] = dot3(w, fr->up);
        base[2] = dot3(w, fr->fwd);
        eu = (float)(q->u1 - q->u0) * HALF;
        ev = (float)(q->v1 - q->v0) * HALF;
        du[0] = fr->right[u] * eu; du[1] = fr->up[u] * eu; du[2] = fr->fwd[u] * eu;
        dv[0] = fr->right[v] * ev; dv[1] = fr->up[v] * ev; dv[2] = fr->fwd[v] * ev;
        for (i = 0; i < 3; i++) {
            cam[0][i] = base[i];
            cam[1][i] = base[i] + du[i];
            cam[2][i] = base[i] + du[i] + dv[i];
            cam[3][i] = base[i] + dv[i];
        }
    }

    /* clip against the near plane (Sutherland-Hodgman) */
    for (i = 0; i < 4; i++) {
        const float *A = cam[i], *B = cam[(i + 1) & 3];
        int ain = A[2] >= NEAR_Z, bin = B[2] >= NEAR_Z;

        if (ain) {
            memcpy(clip[n], A, sizeof(clip[n]));
            n++;
        }
        if (ain != bin) {
            float t = (NEAR_Z - A[2]) / (B[2] - A[2]);

            clip[n][0] = A[0] + (B[0] - A[0]) * t;
            clip[n][1] = A[1] + (B[1] - A[1]) * t;
            clip[n][2] = NEAR_Z;
            n++;
        }
    }
    if (n < 3)
        return;

    for (i = 0; i < n; i++) {
        float iz = 1.0f / clip[i][2];

        sx[i] = CX + focal * clip[i][0] * iz;
        sy[i] = CY - focal * clip[i][1] * iz;
        if (sx[i] < minx) minx = sx[i];
        if (sx[i] > maxx) maxx = sx[i];
        if (sy[i] < miny) miny = sy[i];
        if (sy[i] > maxy) maxy = sy[i];
    }
    if (maxx < 0.0f || minx > SCREEN_W || maxy < 0.0f || miny > SCREEN_H)
        return;

    ytop = miny < 0.0f ? 0 : (int)ceilf(miny - 0.5f);
    ybot = maxy > SCREEN_H ? SCREEN_H : (int)ceilf(maxy - 0.5f);
    if (ytop >= ybot)
        return;
    /* too thin to cover a pixel centre horizontally */
    if ((int)ceilf(maxx - 0.5f) <= (int)ceilf(minx - 0.5f) && maxx - minx < 1.0f)
        return;

    idx = fr->npolys;
    p = &fr->polys[idx];
    p->ytop = (int16_t)ytop;
    p->ybot = (int16_t)ybot;
    p->nedges = 0;
    for (i = 0; i < n; i++) {
        int j = (i + 1) % n;
        struct edge *e;

        if (sy[i] == sy[j])
            continue;
        e = &p->e[p->nedges++];
        if (sy[i] < sy[j]) {
            e->y0 = sy[i]; e->y1 = sy[j]; e->x0 = sx[i];
        } else {
            e->y0 = sy[j]; e->y1 = sy[i]; e->x0 = sx[j];
        }
        e->dxdy = (sx[j] - sx[i]) / (sy[j] - sy[i]);
    }

    /* 1/depth on the face plane: d[a] / (plane - eye[a]) along view ray d */
    k = 1.0f / ((float)q->d * HALF - fr->eye[a]);
    p->ia = fr->dk0[a] * k;
    p->ib = fr->dkx[a] * k;
    p->ic = fr->dky[a] * k;
    p->axis = (uint8_t)a;
    p->face = (a == 1) ? (q->side ? FACE_TOP : FACE_BOTTOM) : (a == 0 ? FACE_X : FACE_Z);
    p->block = q->block;
    p->trans = (uint8_t)block_translucent(q->block);
    /* does this face belong to the selected block? (half-block units) */
    p->sel = sel && sel[a] == floor_div(q->side ? q->d - 1 : q->d, QUAD_SCALE) &&
             sel[u] * QUAD_SCALE < q->u1 && (sel[u] + 1) * QUAD_SCALE > q->u0 &&
             sel[v] * QUAD_SCALE < q->v1 && (sel[v] + 1) * QUAD_SCALE > q->v0;

    /* file it under every band it touches */
    b0 = ytop / BAND_H;
    b1 = (ybot - 1) / BAND_H;
    for (i = b0; i <= b1; i++)
        if (fr->band_count[i] < MAX_BAND_POLYS)
            fr->band_polys[i][fr->band_count[i]++] = (uint16_t)idx;
    fr->npolys++;
}

/* Project a direction at infinity; returns 0 if it is behind the eye. */
static int project_dir(const struct frame *fr, const float d[3], float *x, float *y)
{
    float cx = dot3(d, fr->right), cy = dot3(d, fr->up), cz = dot3(d, fr->fwd);

    if (cz < 0.01f)
        return 0;
    *x = CX + focal * cx / cz;
    *y = CY - focal * cy / cz;
    return 1;
}

static float wrapf(float v, float m)
{
    v = fmodf(v, m);
    return v < 0.0f ? v + m : v;
}

/*
 * Rain and snow: particles fixed in the world in a box that wraps around
 * the camera, so they keep their place as the view turns and moves.
 * Positions are a function of time, so nothing needs to be stored.
 */
static void make_particles(struct frame *fr, float t)
{
    const float bx = 28.0f, by = 20.0f;
    int nrain = (int)(cur_env.rain * 520), nsnow = (int)(cur_env.snow * 760), i;

    fr->nparticles = 0;
    for (i = 0; i < nrain + nsnow && fr->nparticles < MAX_PARTICLES; i++) {
        uint32_t h = (uint32_t)(i + 1) * 0x9E3779B1u;
        float p[3], q[3], c[3], e[3], sway;
        int rain = i < nrain;
        struct particle *pt = &fr->particles[fr->nparticles];

        h ^= h >> 15; h *= 0x2C1B3C6Du; h ^= h >> 12;
        p[0] = (h & 0x3FF) * (bx / 1024.0f);
        p[1] = ((h >> 10) & 0x3FF) * (by / 1024.0f);
        p[2] = ((h >> 20) & 0x3FF) * (bx / 1024.0f);
        sway = (float)(h >> 29);

        if (rain) {                         /* fast, slightly slanted */
            p[0] += 1.5f * t;
            p[1] -= 15.0f * t;
        } else {                            /* slow, drifting */
            p[0] += 0.4f * t + 0.6f * sinf(t * 1.3f + sway);
            p[1] -= 2.2f * t;
            p[2] += 0.3f * t + 0.6f * cosf(t * 1.1f + sway);
        }
        /* wrap into the box centred on the eye */
        q[0] = wrapf(p[0] - fr->eye[0], bx) - bx * 0.5f;
        q[1] = wrapf(p[1] - fr->eye[1], by) - by * 0.5f;
        q[2] = wrapf(p[2] - fr->eye[2], bx) - bx * 0.5f;

        c[2] = dot3(q, fr->fwd);
        if (c[2] < 0.3f)
            continue;
        c[0] = dot3(q, fr->right);
        c[1] = dot3(q, fr->up);
        pt->x0 = (int16_t)(CX + focal * c[0] / c[2]);
        pt->y0 = (int16_t)(CY - focal * c[1] / c[2]);
        if (pt->x0 < -2 || pt->x0 >= SCREEN_W + 2 || pt->y0 < -40 || pt->y0 >= SCREEN_H)
            continue;
        pt->iz = 1.0f / c[2];
        pt->rain = (uint8_t)rain;
        pt->size = c[2] < 5.0f ? 2 : 1;
        if (rain) {
            /* streak: where the drop was 1/30 s ago */
            e[0] = q[0] - 1.5f / 30.0f;
            e[1] = q[1] + 15.0f / 30.0f;
            e[2] = q[2];
            c[0] = dot3(e, fr->right);
            c[1] = dot3(e, fr->up);
            c[2] = dot3(e, fr->fwd);
            if (c[2] < 0.3f)
                continue;
            pt->x1 = (int16_t)(CX + focal * c[0] / c[2]);
            pt->y1 = (int16_t)(CY - focal * c[1] / c[2]);
        } else {
            pt->x1 = pt->x0;
            pt->y1 = pt->y0;
        }
        fr->nparticles++;
    }
}

void render_setup(struct frame *fr, const struct camera *cam, const struct view *vw)
{
    float cyw = cosf(cam->yaw), syw = sinf(cam->yaw);
    float cp = cosf(cam->pitch), sp = sinf(cam->pitch);
    const int (*ring)[2];
    const int *sel = vw->sel;
    int nring, i, k, pcx, pcz;
    float x, y;

    memcpy(fr->eye, cam->pos, sizeof(fr->eye));
    fr->fwd[0] = cp * syw;   fr->fwd[1] = sp;  fr->fwd[2] = -cp * cyw;
    fr->right[0] = cyw;      fr->right[1] = 0; fr->right[2] = syw;
    fr->up[0] = -syw * sp;   fr->up[1] = cp;   fr->up[2] = cyw * sp;

    for (k = 0; k < 3; k++) {
        fr->dk0[k] = fr->fwd[k] - (CX / focal) * fr->right[k] + (CY / focal) * fr->up[k];
        fr->dkx[k] = fr->right[k] / focal;
        fr->dky[k] = -fr->up[k] / focal;
    }

    fr->underwater = vw->underwater;
    fr->hotbar = vw->hotbar;
    fr->weather = vw->weather;
    fr->fps = vw->fps;
    fr->sel_valid = sel != NULL;
    if (sel)
        memcpy(fr->sel, sel, sizeof(fr->sel));

    /* sun and moon sit at infinity in fixed directions */
    fr->sun_visible = cur_env.sun > 0.5f && project_dir(fr, sun_dir, &x, &y) &&
                      x > -SUN_HALO && x < SCREEN_W + SUN_HALO &&
                      y > -SUN_HALO && y < SCREEN_H + SUN_HALO;
    if (fr->sun_visible) {
        fr->sun_x = (int)x;
        fr->sun_y = (int)y;
    }
    fr->moon_visible = cur_env.moon > 0.5f && project_dir(fr, moon_dir, &x, &y) &&
                       x > -SUN_HALO && x < SCREEN_W + SUN_HALO &&
                       y > -SUN_HALO && y < SCREEN_H + SUN_HALO;
    if (fr->moon_visible) {
        fr->moon_x = (int)x;
        fr->moon_y = (int)y;
    }

    fr->nstars = 0;
    for (i = 0; i < (int)(cur_env.stars * NUM_STARS); i++) {
        if (project_dir(fr, star_dir[i], &x, &y) &&
            x >= 0 && x < SCREEN_W && y >= 0 && y < SCREEN_H) {
            fr->stars[fr->nstars][0] = (int16_t)x;
            fr->stars[fr->nstars][1] = (int16_t)y;
            fr->nstars++;
        }
    }

    make_particles(fr, vw->time);

    fr->npolys = 0;
    fr->nchunks = 0;
    memset(fr->band_count, 0, sizeof(fr->band_count));
    pcx = (int)floorf(cam->pos[0] / CHUNK_SIZE);
    pcz = (int)floorf(cam->pos[2] / CHUNK_SIZE);
    nring = world_ring(&ring);

    for (i = 0; i < nring; i++) {
        struct chunk *c = world_chunk(pcx + ring[i][0], pcz + ring[i][1]);
        const struct mesh *m;
        float lo[3], hi[3];
        int g, j;

        if (!c || c->mesh.nquads == 0)
            continue;
        m = &c->mesh;
        lo[0] = (float)(c->cx * CHUNK_SIZE); hi[0] = lo[0] + CHUNK_SIZE;
        lo[1] = (float)m->ymin;              hi[1] = (float)m->ymax;
        lo[2] = (float)(c->cz * CHUNK_SIZE); hi[2] = lo[2] + CHUNK_SIZE;
        if (!box_visible(fr, lo, hi))
            continue;
        fr->nchunks++;

        for (g = 0; g < NUM_GROUPS; g++) {
            int a = g >> 1, side = g & 1;

            if (m->group_start[g] == m->group_start[g + 1])
                continue;
            /* every face in the group points away from the eye */
            if (side ? fr->eye[a] <= (float)m->group_dmin[g] * HALF
                     : fr->eye[a] >= (float)m->group_dmax[g] * HALF)
                continue;
            for (j = m->group_start[g]; j < m->group_start[g + 1]; j++) {
                const struct quad *q = &m->quads[j];
                float rel = fr->eye[a] - (float)q->d * HALF;

                if (side ? rel > 0.0f : rel < 0.0f)
                    add_quad(fr, q, sel);
            }
        }
    }
}

/* ---- band rendering ------------------------------------------------------ */

static inline int fog_index(float z)
{
    float f = (z - fog_start) * fog_scale;

    if (f <= 0.0f)
        return 0;
    if (f >= FOG_LEVELS - 1)
        return FOG_LEVELS - 1;
    return (int)f;
}

static inline int32_t float_bits(float f)
{
    union { float f; int32_t i; } u;

    u.f = f;
    return u.i;
}

static inline unsigned block_tone(int32_t bu, int32_t bv)
{
    uint32_t h = (uint32_t)bu * 0x9E3779B1u + (uint32_t)bv * 0x85EBCA77u;

    /* mix so the top bits depend on every input bit */
    h ^= h >> 15;
    h *= 0x2C1B3C6Du;
    h ^= h >> 12;
    return tone_map[h >> 30];
}

/*
 * Shade pixels x0..x1 of a span given 16.16 fixed-point world coordinates
 * (ua, ub) along the face's two in-plane axes and their per-pixel steps.
 * ZEXPR gives this pixel's 1/z as float bits (positive floats compare like
 * integers); ZSTEP advances it to the next pixel.
 */
#define SHADE_SPAN(ZEXPR, ZSTEP)                                            \
    {                                                                       \
        int32_t cba = INT32_MIN, cbb = 0;   /* block of the cached tone */  \
        unsigned ct = TONE_A;                                               \
                                                                            \
        for (x = x0; x < x1; x++, ua += dua, ub += dub, ZSTEP) {            \
            int32_t zb = (ZEXPR), ba, bb;                                   \
            unsigned t;                                                     \
                                                                            \
            if (zrow[x] >= zb)                                              \
                continue;                                                   \
            zrow[x] = zb;                                                   \
            ba = ua >> 16;                                                  \
            bb = ub >> 16;                                                  \
            if (((uint32_t)ua & 0xFFFF) < la || ((uint32_t)ub & 0xFFFF) < lb) { \
                t = TONE_LINE;                                              \
            } else {                                                        \
                /* neighbouring pixels are nearly always the same block */  \
                if (ba != cba || bb != cbb) {                               \
                    cba = ba;                                               \
                    cbb = bb;                                               \
                    ct = (sel && ba == sela && bb == selb) ? TONE_SEL       \
                                                           : block_tone(ba, bb); \
                }                                                           \
                t = ct;                                                     \
            }                                                               \
            row[x] = lut[t * TONE_STRIDE + (x & 3)];                        \
        }                                                                   \
    }

/* A span of a horizontal face: 1/z is constant across it. */
static void span_flat(const struct frame *fr, const struct poly *p,
                      uint8_t *row, int32_t *zrow, int y, int x0, int x1)
{
    float pyc = y + 0.5f, px = x0 + 0.5f;
    float invz = p->ia + p->ic * pyc;
    float z, z2, bx, bz, wx, wz, sx, sz, fx, fz;
    const uint8_t *lut;
    int32_t zbits;
    int x;

    if (invz <= 1e-6f)
        return;
    z = 1.0f / invz;
    zbits = float_bits(invz);

    /* world x and z of the first pixel, and their change per pixel */
    bx = fr->dk0[0] + fr->dky[0] * pyc;
    bz = fr->dk0[2] + fr->dky[2] * pyc;
    wx = fr->eye[0] + z * (bx + fr->dkx[0] * px);
    wz = fr->eye[2] + z * (bz + fr->dkx[2] * px);
    sx = z * fr->dkx[0];
    sz = z * fr->dkx[2];

    /* pixel footprint along x and z, including towards the next row */
    z2 = (invz + p->ic > 1e-6f) ? 1.0f / (invz + p->ic) : z * 4.0f;
    fx = fabsf(fr->eye[0] + z2 * (bx + fr->dky[0] + fr->dkx[0] * px) - wx);
    fz = fabsf(fr->eye[2] + z2 * (bz + fr->dky[2] + fr->dkx[2] * px) - wz);
    if (fabsf(sx) > fx) fx = fabsf(sx);
    if (fabsf(sz) > fz) fz = fabsf(sz);

    lut = &pal[p->block][p->face][0][fog_index(z)][(y & 3) * 4];

    if (block_plain[p->block] || fx > 0.3f || fz > 0.3f) {
        /* water, or blocks only a few pixels wide: plain colour */
        for (x = x0; x < x1; x++) {
            if (zrow[x] < zbits) {
                zrow[x] = zbits;
                row[x] = lut[x & 3];
            }
        }
        return;
    }

    {
        /* a = world x, b = world z */
        int32_t ua = (int32_t)(wx * 65536.0f), ub = (int32_t)(wz * 65536.0f);
        int32_t dua = (int32_t)(sx * 65536.0f), dub = (int32_t)(sz * 65536.0f);
        uint32_t la = (uint32_t)(fx * 65536.0f), lb = (uint32_t)(fz * 65536.0f);
        int sel = p->sel, sela = fr->sel[0], selb = fr->sel[2];

        SHADE_SPAN(zbits, (void)0)
    }
}

/* A span of a vertical face: perspective-correct every 8 pixels. */
static void span_wall(const struct frame *fr, const struct poly *p,
                      uint8_t *row, int32_t *zrow, int y, int sx0, int sx1)
{
    int ka = (p->axis + 1) % 3, kb = (p->axis + 2) % 3;
    float pyc = y + 0.5f;
    float iz_row = p->ia + p->ic * pyc;
    float ba = fr->dk0[ka] + fr->dky[ka] * pyc;
    float bb = fr->dk0[kb] + fr->dky[kb] * pyc;
    int sel = p->sel, sela = fr->sel[ka], selb = fr->sel[kb];
    int xs;

    for (xs = sx0; xs < sx1; xs += 8) {
        int xe = (xs + 8 < sx1) ? xs + 8 : sx1, n = xe - xs, x, x0 = xs, x1 = xe;
        float pa = xs + 0.5f, pb = xe + 0.5f;
        float izs = iz_row + p->ib * pa, ize = iz_row + p->ib * pb;
        float zs, ze, was, wae, wbs, wbe, sa, sb, fa, fb, iz;
        const uint8_t *lut;

        if (izs < 1e-6f) izs = 1e-6f;
        if (ize < 1e-6f) ize = 1e-6f;
        zs = 1.0f / izs;
        ze = 1.0f / ize;
        was = fr->eye[ka] + zs * (ba + fr->dkx[ka] * pa);
        wae = fr->eye[ka] + ze * (ba + fr->dkx[ka] * pb);
        wbs = fr->eye[kb] + zs * (bb + fr->dkx[kb] * pa);
        wbe = fr->eye[kb] + ze * (bb + fr->dkx[kb] * pb);
        sa = (wae - was) / n;
        sb = (wbe - wbs) / n;

        /* footprint: along the span, or one pixel's worth at this depth */
        fa = fabsf(sa); if (zs / focal > fa) fa = zs / focal;
        fb = fabsf(sb); if (zs / focal > fb) fb = zs / focal;

        lut = &pal[p->block][p->face][0][fog_index(zs)][(y & 3) * 4];
        iz = izs;

        if (block_plain[p->block] || fa > 0.3f || fb > 0.3f) {
            for (x = xs; x < xe; x++, iz += p->ib) {
                int32_t zb = float_bits(iz);

                if (zrow[x] < zb) {
                    zrow[x] = zb;
                    row[x] = lut[x & 3];
                }
            }
            continue;
        }

        {
            int32_t ua = (int32_t)(was * 65536.0f), ub = (int32_t)(wbs * 65536.0f);
            int32_t dua = (int32_t)(sa * 65536.0f), dub = (int32_t)(sb * 65536.0f);
            uint32_t la = (uint32_t)(fa * 65536.0f), lb = (uint32_t)(fb * 65536.0f);

            SHADE_SPAN(float_bits(iz), iz += p->ib)
        }
    }
}

/*
 * A span of a see-through face (water, glass): depth-tested against what is
 * already drawn but not written, and blended over it. Works for any face
 * orientation, perspective-correct every 16 pixels.
 */
static void span_trans(const struct frame *fr, const struct poly *p,
                       uint8_t *row, const int32_t *zrow, int y, int sx0, int sx1)
{
    int ka = (p->axis + 1) % 3, kb = (p->axis + 2) % 3;
    int mat = (p->block == BLOCK_GLASS) ? MAT_GLASS : MAT_WATER;
    float pyc = y + 0.5f;
    float iz_row = p->ia + p->ic * pyc;
    float ba = fr->dk0[ka] + fr->dky[ka] * pyc;
    float bb = fr->dk0[kb] + fr->dky[kb] * pyc;
    int xs;

    for (xs = sx0; xs < sx1; xs += 16) {
        int xe = (xs + 16 < sx1) ? xs + 16 : sx1, n = xe - xs, x, fog;
        float pa = xs + 0.5f, pb = xe + 0.5f;
        float izs = iz_row + p->ib * pa, ize = iz_row + p->ib * pb, iz = izs;
        float zs, ze, was, wbs, sa, sb, fa, fb;
        const uint8_t *blend, *line;
        int32_t ua, ub, dua, dub;
        uint32_t la = 0, lb = 0;

        if (izs < 1e-6f) izs = 1e-6f;
        if (ize < 1e-6f) ize = 1e-6f;
        zs = 1.0f / izs;
        ze = 1.0f / ize;
        was = fr->eye[ka] + zs * (ba + fr->dkx[ka] * pa);
        wbs = fr->eye[kb] + zs * (bb + fr->dkx[kb] * pa);
        sa = (fr->eye[ka] + ze * (ba + fr->dkx[ka] * pb) - was) / n;
        sb = (fr->eye[kb] + ze * (bb + fr->dkx[kb] * pb) - wbs) / n;
        fog = fog_index(zs);
        blend = blend_lut[mat][fog];
        line = &pal[p->block][p->face][TONE_LINE][fog][(y & 3) * 4];

        /* glass frame lines, while blocks are big enough to show them */
        fa = fabsf(sa); if (zs / focal > fa) fa = zs / focal;
        fb = fabsf(sb); if (zs / focal > fb) fb = zs / focal;
        if (mat == MAT_GLASS && fa < 0.3f && fb < 0.3f) {
            la = (uint32_t)(fa * 65536.0f);
            lb = (uint32_t)(fb * 65536.0f);
        }
        ua = (int32_t)(was * 65536.0f);
        ub = (int32_t)(wbs * 65536.0f);
        dua = (int32_t)(sa * 65536.0f);
        dub = (int32_t)(sb * 65536.0f);

        for (x = xs; x < xe; x++, iz += p->ib, ua += dua, ub += dub) {
            if (zrow[x] >= float_bits(iz))
                continue;
            if (((uint32_t)ua & 0xFFFF) < la || ((uint32_t)ub & 0xFFFF) < lb)
                row[x] = line[x & 3];
            else
                row[x] = blend[row[x]];
        }
    }
}

/* ---- HUD ------------------------------------------------------------------ */

static void put_px(uint8_t *pixels, int y0, int y1, int x, int y, uint8_t c)
{
    if (y >= y0 && y < y1 && x >= 0 && x < SCREEN_W)
        pixels[y * SCREEN_W + x] = c;
}

/* Text at (x, y), each font pixel scale x scale, with a 1-pixel shadow. */
static void draw_text(uint8_t *pixels, int y0, int y1, int x, int y,
                      const char *s, int scale, uint8_t color)
{
    for (; *s; s++, x += 6 * scale) {
        const uint8_t *g = font_glyph(*s);
        int r, c, i, j;

        if (!g)
            continue;
        for (r = 0; r < 7; r++)
            for (c = 0; c < 5; c++)
                if (g[r] & (0x10 >> c))
                    for (j = 0; j < scale; j++)
                        for (i = 0; i < scale; i++) {
                            put_px(pixels, y0, y1, x + c * scale + i + 1,
                                   y + r * scale + j + 1, 0x00);
                            put_px(pixels, y0, y1, x + c * scale + i,
                                   y + r * scale + j, color);
                        }
    }
}

#define SLOT            44
#define SLOT_GAP        4
#define HOTBAR_W        (HOTBAR_SLOTS * SLOT + (HOTBAR_SLOTS - 1) * SLOT_GAP)
#define HOTBAR_X        ((SCREEN_W - HOTBAR_W) / 2)
#define HOTBAR_Y        (SCREEN_H - SLOT - 8)

static void draw_hud(const struct frame *fr, uint8_t *pixels, int y0, int y1)
{
    int i, x, y;

    if (y0 < 24) {
        draw_text(pixels, y0, y1, 8, 8, weather_name(fr->weather), 2, 0xFF);
        if (fr->fps >= 0) {
            char buf[16];
            int w;

            snprintf(buf, sizeof(buf), "%d FPS", fr->fps);
            w = (int)strlen(buf) * 12 - 2;
            draw_text(pixels, y0, y1, SCREEN_W - 8 - w, 8, buf, 2, 0xFF);
        }
    }
    if (y1 <= HOTBAR_Y - 24)
        return;

    for (i = 0; i < HOTBAR_SLOTS; i++) {
        int sx = HOTBAR_X + i * (SLOT + SLOT_GAP), sel = (i == fr->hotbar);
        char num[2] = { (char)('1' + i), 0 };

        for (y = HOTBAR_Y; y < HOTBAR_Y + SLOT; y++) {
            if (y < y0 || y >= y1)
                continue;
            for (x = sx; x < sx + SLOT; x++) {
                int border = sel ? 3 : 1;
                int edge = x < sx + border || x >= sx + SLOT - border ||
                           y < HOTBAR_Y + border || y >= HOTBAR_Y + SLOT - border;
                int ix = x - sx - (SLOT - ICON) / 2, iy = y - HOTBAR_Y - (SLOT - ICON) / 2;
                uint8_t *px = &pixels[y * SCREEN_W + x];

                if (edge)
                    *px = sel ? 0xFF : 0x00;
                else if (ix >= 0 && ix < ICON && iy >= 0 && iy < ICON &&
                         icon_on[i][iy * ICON + ix])
                    *px = icons[i][iy * ICON + ix];
                else
                    *px = water_tint[*px] & 0x6D;   /* darkened backdrop */
            }
        }
        draw_text(pixels, y0, y1, sx + 3, HOTBAR_Y + 3, num, 1, 0xFF);
    }

    /* name of the selected block above the bar */
    {
        const char *name = hotbar_names[fr->hotbar];
        int w = (int)strlen(name) * 12 - 2;

        draw_text(pixels, y0, y1, (SCREEN_W - w) / 2, HOTBAR_Y - 20, name, 2, 0xFF);
    }
}

static void fill_row(uint8_t *row, const uint8_t pat[4])
{
    uint32_t v = (uint32_t)pat[0] | (uint32_t)pat[1] << 8 |
                 (uint32_t)pat[2] << 16 | (uint32_t)pat[3] << 24;
    uint32_t *w = (uint32_t *)row;
    int i;

    for (i = 0; i < SCREEN_W / 4; i++)
        w[i] = v;
}

static void draw_sky(const struct frame *fr, uint8_t *pixels, int y0, int y1)
{
    int y;

    for (y = y0; y < y1; y++) {
        float pyc = y + 0.5f;
        float d0 = fr->dk0[0] + fr->dkx[0] * CX + fr->dky[0] * pyc;
        float d1 = fr->dk0[1] + fr->dkx[1] * CX + fr->dky[1] * pyc;
        float d2 = fr->dk0[2] + fr->dkx[2] * CX + fr->dky[2] * pyc;
        float e = d1 / sqrtf(d0 * d0 + d1 * d1 + d2 * d2);
        int l = (e <= 0.0f) ? 0 : (int)(sqrtf(e) * (SKY_LEVELS - 1));

        if (l > SKY_LEVELS - 1)
            l = SKY_LEVELS - 1;
        fill_row(pixels + y * SCREEN_W, &sky_lut[l][(y & 3) * 4]);
    }
}

static void draw_square(uint8_t *pixels, int y0, int y1, int cx, int cy,
                        int half, const uint8_t *lut)
{
    int ya = cy - half, yb = cy + half, xa = cx - half, xb = cx + half, x, y;

    if (ya < y0) ya = y0;
    if (yb > y1) yb = y1;
    if (xa < 0) xa = 0;
    if (xb > SCREEN_W) xb = SCREEN_W;
    for (y = ya; y < yb; y++)
        for (x = xa; x < xb; x++)
            pixels[y * SCREEN_W + x] = lut[(y & 3) * 4 + (x & 3)];
}

static void draw_crosshair(uint8_t *pixels, int y0, int y1)
{
    const int cx = SCREEN_W / 2, cy = SCREEN_H / 2, len = 7;
    int x, y;

    for (y = cy - len; y <= cy + len; y++)
        if (y >= y0 && y < y1)
            pixels[y * SCREEN_W + cx] ^= 0xFF;
    if (cy >= y0 && cy < y1)
        for (x = cx - len; x <= cx + len; x++)
            if (x != cx)
                pixels[cy * SCREEN_W + x] ^= 0xFF;
}

static void draw_stars(const struct frame *fr, uint8_t *pixels, int y0, int y1)
{
    int i;

    for (i = 0; i < fr->nstars; i++) {
        int x = fr->stars[i][0], y = fr->stars[i][1];

        if (y >= y0 && y < y1)
            pixels[y * SCREEN_W + x] = star_lut[(y & 3) * 4 + (x & 3)];
    }
}

/* Rain streaks and snowflakes, hidden behind anything nearer. */
static void draw_particles(const struct frame *fr, uint8_t *pixels, int y0, int y1,
                           const int32_t *zb)
{
    int i;

    for (i = 0; i < fr->nparticles; i++) {
        const struct particle *pt = &fr->particles[i];
        int32_t iz = float_bits(pt->iz);

        if (pt->rain) {
            int ya = pt->y1 < pt->y0 ? pt->y1 : pt->y0;
            int yb = pt->y1 < pt->y0 ? pt->y0 : pt->y1;
            int y;

            if (yb < y0 || ya >= y1)
                continue;
            for (y = ya > y0 ? ya : y0; y <= yb && y < y1; y++) {
                int x = (yb == ya) ? pt->x0
                      : pt->x1 + (pt->x0 - pt->x1) * (y - pt->y1) / (pt->y0 - pt->y1);

                if (x < 0 || x >= SCREEN_W || zb[(y - y0) * SCREEN_W + x] >= iz)
                    continue;
                pixels[y * SCREEN_W + x] = rain_lut[(y & 3) * 4 + (x & 3)];
            }
        } else {
            int dx, dy;

            for (dy = 0; dy < pt->size; dy++) {
                int y = pt->y0 + dy;

                if (y < y0 || y >= y1)
                    continue;
                for (dx = 0; dx < pt->size; dx++) {
                    int x = pt->x0 + dx;

                    if (x < 0 || x >= SCREEN_W || zb[(y - y0) * SCREEN_W + x] >= iz)
                        continue;
                    pixels[y * SCREEN_W + x] = snow_lut[(y & 3) * 4 + (x & 3)];
                }
            }
        }
    }
}

void render_band(const struct frame *fr, uint8_t *pixels, int band, float *zbuf)
{
    int y0 = band * BAND_H, y1 = y0 + BAND_H;
    int32_t *zb = (int32_t *)zbuf;
    int i, y, pass;

    draw_sky(fr, pixels, y0, y1);
    draw_stars(fr, pixels, y0, y1);
    if (fr->sun_visible) {
        draw_square(pixels, y0, y1, fr->sun_x, fr->sun_y, SUN_HALO, sun_lut[0]);
        draw_square(pixels, y0, y1, fr->sun_x, fr->sun_y, SUN_CORE, sun_lut[1]);
    }
    if (fr->moon_visible) {
        draw_square(pixels, y0, y1, fr->moon_x, fr->moon_y, SUN_CORE + 4, moon_lut[0]);
        draw_square(pixels, y0, y1, fr->moon_x, fr->moon_y, SUN_CORE - 4, moon_lut[1]);
    }

    /* 1/z of 0 means "infinitely far": anything drawn is in front */
    memset(zb, 0, BAND_H * SCREEN_W * sizeof(*zb));

    /* pass 0: solid faces (writing depth), pass 1: see-through faces */
    for (pass = 0; pass < 2; pass++)
    for (i = 0; i < fr->band_count[band]; i++) {
        const struct poly *p = &fr->polys[fr->band_polys[band][i]];
        int ya = p->ytop > y0 ? p->ytop : y0;
        int yb = p->ybot < y1 ? p->ybot : y1;

        if (p->trans != pass)
            continue;

        for (y = ya; y < yb; y++) {
            float yc = y + 0.5f, xl = 1e30f, xr = -1e30f;
            int e, x0, x1;

            for (e = 0; e < p->nedges; e++) {
                const struct edge *ed = &p->e[e];

                if (yc >= ed->y0 && yc < ed->y1) {
                    float x = ed->x0 + (yc - ed->y0) * ed->dxdy;

                    if (x < xl) xl = x;
                    if (x > xr) xr = x;
                }
            }
            if (xl > xr)
                continue;
            x0 = (xl < 0.0f) ? 0 : (int)ceilf(xl - 0.5f);
            x1 = (xr > SCREEN_W) ? SCREEN_W : (int)ceilf(xr - 0.5f);
            if (x0 >= x1)
                continue;

            if (p->trans)
                span_trans(fr, p, pixels + y * SCREEN_W, zb + (y - y0) * SCREEN_W, y, x0, x1);
            else if (p->axis == 1)
                span_flat(fr, p, pixels + y * SCREEN_W, zb + (y - y0) * SCREEN_W, y, x0, x1);
            else
                span_wall(fr, p, pixels + y * SCREEN_W, zb + (y - y0) * SCREEN_W, y, x0, x1);
        }
    }

    if (fr->nparticles && !fr->underwater)
        draw_particles(fr, pixels, y0, y1, zb);

    if (fr->underwater) {
        uint8_t *px = pixels + y0 * SCREEN_W;

        for (i = 0; i < BAND_H * SCREEN_W; i++)
            px[i] = water_tint[px[i]];
    }
    draw_crosshair(pixels, y0, y1);
    draw_hud(fr, pixels, y0, y1);
}
