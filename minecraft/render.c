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
#include "gpu.h"

#define CX              (SCREEN_W * 0.5f)
#define CY              (SCREEN_H * 0.5f)
#define FOV_X_DEG       70.0f
#define NEAR_Z          0.05f
#define HALF            (1.0f / QUAD_SCALE)     /* quad units -> blocks */
#define FOG_LEVELS      32

enum { FACE_TOP, FACE_BOTTOM, FACE_X, FACE_Z, NUM_FACES };
enum { TONE_A, TONE_B, TONE_C, TONE_LINE, TONE_SEL, NUM_TONES };

#define SKY_LEVELS      64
#define SUN_CORE        18      /* half-size in pixels */
#define SUN_HALO        28

/*
 * GPU colour sets: the five tones of each block face at each fog level.
 * Block 0 (air) is never drawn, so 16 blocks x 4 faces x 32 fog levels
 * fill all 2048 sets exactly.
 */
#define SET_INDEX(b, f, fog)    ((((b) - 1) * NUM_FACES + (f)) * FOG_LEVELS + (fog))
_Static_assert((NUM_BLOCK_TYPES - 1) * NUM_FACES * FOG_LEVELS == GPU_COLOR_SETS,
               "colour sets must cover every block, face and fog level");
_Static_assert(NUM_TONES == GPU_TONES, "tones");

static float focal, fog_start, fog_scale;

static uint32_t set_rgb[GPU_COLOR_SETS][NUM_TONES];
static int sets_dirty;                  /* not yet sent to the GPU */
static uint32_t sky_rgb[SKY_LEVELS];
static uint32_t sun_c[2], moon_c[2], star_c, rain_c, snow_c, water_fog_rgb;
int render_overflows;                   /* bands that ran out of command space */

/* underwater: everything seen through murky water */
#define WATER_FOG_ALPHA 179
/* hotbar slots: darkened backdrop */
#define HUD_BACK_RGB    0x0A1430u
#define HUD_BACK_ALPHA  150

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
static const uint8_t mat_alpha8[NUM_MATS] = { 154, 56 };   /* 0.6, 0.22 */
static uint32_t mat_fog_rgb[NUM_MATS][FOG_LEVELS];

/* hotbar contents and names (main.c maps stairs to a facing) */
const uint8_t hotbar_blocks[HOTBAR_SLOTS] = {
    BLOCK_COBBLE, BLOCK_DIRT, BLOCK_GRASS, BLOCK_GLASS,
    BLOCK_LOG, BLOCK_PLANKS, BLOCK_STAIRS_ZN,
};
static const char *const hotbar_names[HOTBAR_SLOTS] = {
    "COBBLESTONE", "DIRT", "GRASS", "GLASS", "LOG", "PLANKS", "STAIRS",
};
#define ICON            32
#define ICON_OFF        0xFFFFFFFFu                     /* not part of the icon */
static uint32_t icons[HOTBAR_SLOTS][ICON * ICON];

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

static uint32_t rgbf(const float c[3])
{
    return gpu_rgb(c[0], c[1], c[2]);
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

                icons[i][y * ICON + x] = ICON_OFF;
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
                icons[i][y * ICON + x] = rgbf(c);
            }
        }
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
                    set_rgb[SET_INDEX(b, f, l)][t] = rgbf(c);
                }
            }
        }
    }
    sets_dirty = 1;

    for (l = 0; l < SKY_LEVELS; l++) {
        float c[3];

        mix(c, e->sky_horizon, e->sky_zenith, (float)l / (SKY_LEVELS - 1));
        sky_rgb[l] = rgbf(c);
    }

    /* see-through blocks: the colour things behind them are blended towards */
    for (b = 0; b < NUM_MATS; b++) {
        for (l = 0; l < FOG_LEVELS; l++) {
            float m[3], lit[3];

            for (k = 0; k < 3; k++)
                lit[k] = mat_rgb[b][k] * e->light;
            mix(m, lit, fogc, (float)l / (FOG_LEVELS - 1));
            mat_fog_rgb[b][l] = rgbf(m);
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

    for (l = 0; l < 2; l++) {
        sun_c[l] = rgbf(sun_rgb[l]);
        moon_c[l] = rgbf(moon_rgb[l]);
    }
    star_c = rgbf(star_rgb);
    rain_c = rgbf(rain_rgb);
    snow_c = rgbf(snow_rgb);
    water_fog_rgb = rgbf(water_fog);

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


/* ---- band command lists ---------------------------------------------------- */

/* Where a band's commands go; cap leaves room for the final END. */
struct cmds {
    uint64_t *p;
    int n, cap;
    int full;                   /* something did not fit */
};

static inline uint64_t *put(struct cmds *c, int words)
{
    uint64_t *w;

    if (c->n + words > c->cap) {
        c->full = 1;
        return NULL;
    }
    w = c->p + c->n;
    c->n += words;
    return w;
}

static inline int fog_index(float z)
{
    float f = (z - fog_start) * fog_scale;

    if (f <= 0.0f)
        return 0;
    if (f >= FOG_LEVELS - 1)
        return FOG_LEVELS - 1;
    return (int)f;
}

static inline int32_t fixed16(float v)
{
    return (int32_t)(v * 65536.0f);
}

static void fill(struct cmds *c, int y, int x0, int x1, uint32_t rgb)
{
    uint64_t *w;

    if (x0 < 0) x0 = 0;
    if (x1 > SCREEN_W) x1 = SCREEN_W;
    if (x0 >= x1 || !(w = put(c, 2)))
        return;
    w[0] = gpu_hdr(GPU_FILL, x0, x1, y);
    w[1] = rgb;
}

/* fill behind anything nearer than 1/z = iz */
static void fill_z(struct cmds *c, int y, int x0, int x1, uint32_t rgb, float iz)
{
    uint64_t *w;

    if (x0 < 0) x0 = 0;
    if (x1 > SCREEN_W) x1 = SCREEN_W;
    if (x0 >= x1 || !(w = put(c, 2)))
        return;
    w[0] = gpu_hdr(GPU_FILL, x0, x1, y) | GPU_FLAG;
    w[1] = gpu_pair(rgb, gpu_iz(iz));
}

/* see-through overlay of a whole run, no grid lines, no depth test */
static void tint(struct cmds *c, int y, int x0, int x1, uint32_t rgb, int alpha)
{
    uint64_t *w;

    if (x0 >= x1 || !(w = put(c, 6)))
        return;
    w[0] = gpu_hdr(GPU_BLEND, x0, x1, y);
    w[1] = w[2] = w[3] = 0;
    w[4] = (uint64_t)rgb << 16 | (uint64_t)alpha << 40;
    w[5] = 0;
}

/*
 * A span of a horizontal face: 1/z is constant across it, and the world
 * coordinates step by a constant per pixel.
 */
static void span_flat(struct cmds *c, const struct frame *fr, const struct poly *p,
                      int y, int x0, int x1)
{
    float pyc = y + 0.5f, px = x0 + 0.5f;
    float invz = p->ia + p->ic * pyc;
    float z, z2, bx, bz, wx, wz, sx, sz, fx, fz;
    uint64_t *w, hdr, set;

    if (invz <= 1e-6f)
        return;
    z = 1.0f / invz;

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

    set = (uint64_t)SET_INDEX(p->block, p->face, fog_index(z)) << 35;
    if (block_plain[p->block] || fx > 0.3f || fz > 0.3f) {
        /* water, or blocks only a few pixels wide: plain colour */
        if ((w = put(c, 2))) {
            w[0] = gpu_hdr(GPU_SHADE, x0, x1, y) | set;
            w[1] = gpu_pair(gpu_iz(invz), 0);
        }
        return;
    }
    if (!(w = put(c, 5)))
        return;
    hdr = gpu_hdr(GPU_SPAN, x0, x1, y) | set;
    w[1] = gpu_pair(gpu_iz(invz), 0);
    /* a = world x, b = world z */
    w[0] = hdr | (p->sel ? GPU_FLAG : 0) | (uint64_t)(uint16_t)fixed16(fx) << 48;
    w[2] = gpu_pair((uint32_t)fixed16(wx), (uint32_t)fixed16(sx));
    w[3] = gpu_pair((uint32_t)fixed16(wz), (uint32_t)fixed16(sz));
    w[4] = (uint64_t)(uint16_t)fixed16(fz) | (uint64_t)(uint16_t)fr->sel[0] << 16 |
           (uint64_t)(uint16_t)fr->sel[2] << 32;
}

/*
 * A span of a vertical or see-through face, cut into pieces of SUBSPAN
 * pixels: exact 1/z, world coordinates perspective-correct at the ends of
 * each piece and linear in between.
 */
#define SUBSPAN         16

static void span_wall(struct cmds *c, const struct frame *fr, const struct poly *p,
                      int y, int sx0, int sx1)
{
    int ka = (p->axis + 1) % 3, kb = (p->axis + 2) % 3;
    int trans = p->trans, mat = (p->block == BLOCK_GLASS) ? MAT_GLASS : MAT_WATER;
    float pyc = y + 0.5f;
    float iz_row = p->ia + p->ic * pyc;
    float ba = fr->dk0[ka] + fr->dky[ka] * pyc;
    float bb = fr->dk0[kb] + fr->dky[kb] * pyc;
    uint32_t diz = gpu_iz(p->ib);
    int xs;

    for (xs = sx0; xs < sx1; xs += SUBSPAN) {
        int xe = (xs + SUBSPAN < sx1) ? xs + SUBSPAN : sx1, n = xe - xs, set;
        float pa = xs + 0.5f, pb = xe + 0.5f;
        float izs = iz_row + p->ib * pa, ize = iz_row + p->ib * pb;
        float zs, ze, was, wbs, sa, sb, fa, fb;
        uint64_t *w, hdr;

        if (izs < 1e-6f) izs = 1e-6f;
        if (ize < 1e-6f) ize = 1e-6f;
        zs = 1.0f / izs;
        ze = 1.0f / ize;
        was = fr->eye[ka] + zs * (ba + fr->dkx[ka] * pa);
        wbs = fr->eye[kb] + zs * (bb + fr->dkx[kb] * pa);
        sa = (fr->eye[ka] + ze * (ba + fr->dkx[ka] * pb) - was) / n;
        sb = (fr->eye[kb] + ze * (bb + fr->dkx[kb] * pb) - wbs) / n;

        /* footprint: along the span, or one pixel's worth at this depth */
        fa = fabsf(sa); if (zs / focal > fa) fa = zs / focal;
        fb = fabsf(sb); if (zs / focal > fb) fb = zs / focal;
        set = SET_INDEX(p->block, p->face, fog_index(zs));

        if (trans) {
            int fog = fog_index(zs);

            if (!(w = put(c, 6)))
                return;
            w[0] = gpu_hdr(GPU_BLEND, xs, xe, y) | GPU_FLAG;
            w[4] = (uint64_t)mat_fog_rgb[mat][fog] << 16 | (uint64_t)mat_alpha8[mat] << 40;
            w[5] = set_rgb[set][TONE_LINE];
            /* glass frame lines, while blocks are big enough to show them */
            if (mat == MAT_GLASS && fa < 0.3f && fb < 0.3f) {
                w[0] |= (uint64_t)(uint16_t)fixed16(fa) << 48;
                w[4] |= (uint16_t)fixed16(fb);
            }
        } else {
            if (block_plain[p->block] || fa > 0.3f || fb > 0.3f) {
                if (!(w = put(c, 2)))
                    return;
                w[0] = gpu_hdr(GPU_SHADE, xs, xe, y) | (uint64_t)set << 35;
                w[1] = gpu_pair(gpu_iz(izs), diz);
                continue;
            }
            if (!(w = put(c, 5)))
                return;
            hdr = gpu_hdr(GPU_SPAN, xs, xe, y) | (uint64_t)set << 35;
            w[0] = hdr | (p->sel ? GPU_FLAG : 0) | (uint64_t)(uint16_t)fixed16(fa) << 48;
            w[4] = (uint64_t)(uint16_t)fixed16(fb) | (uint64_t)(uint16_t)fr->sel[ka] << 16 |
                   (uint64_t)(uint16_t)fr->sel[kb] << 32;
        }
        w[1] = gpu_pair(gpu_iz(izs), diz);
        w[2] = gpu_pair((uint32_t)fixed16(was), (uint32_t)fixed16(sa));
        w[3] = gpu_pair((uint32_t)fixed16(wbs), (uint32_t)fixed16(sb));
    }
}

/* ---- HUD ------------------------------------------------------------------ */

/*
 * Row y of text at (x, ty), each font pixel scale x scale, with a 1-pixel
 * shadow: the shadow runs, then the text runs over them.
 */
static void text_row(struct cmds *c, int y, int x, int ty, const char *s, int scale,
                     uint32_t rgb)
{
    int pass;

    for (pass = 0; pass < 2; pass++) {
        int d = pass ? 0 : 1, gy = y - ty - d, cx;
        const char *q;

        if (gy < 0 || gy >= 7 * scale)
            continue;
        for (q = s, cx = x + d; *q; q++, cx += 6 * scale) {
            const uint8_t *g = font_glyph(*q);
            int col = 0;

            if (!g)
                continue;
            while (col < 5) {
                int start;

                if (!(g[gy / scale] & (0x10 >> col))) {
                    col++;
                    continue;
                }
                for (start = col; col < 5 && (g[gy / scale] & (0x10 >> col)); col++)
                    ;
                fill(c, y, cx + start * scale, cx + col * scale, pass ? rgb : 0);
            }
        }
    }
}

#define SLOT            44
#define SLOT_GAP        4
#define HOTBAR_W        (HOTBAR_SLOTS * SLOT + (HOTBAR_SLOTS - 1) * SLOT_GAP)
#define HOTBAR_X        ((SCREEN_W - HOTBAR_W) / 2)
#define HOTBAR_Y        (SCREEN_H - SLOT - 8)
#define WHITE           0xFFFFFFu

static void hud_row(struct cmds *c, const struct frame *fr, int y)
{
    int i;

    if (y < 24) {
        text_row(c, y, 8, 8, weather_name(fr->weather), 2, WHITE);
        if (fr->fps >= 0) {
            char buf[16];
            int w;

            snprintf(buf, sizeof(buf), "%d FPS", fr->fps);
            w = (int)strlen(buf) * 12 - 2;
            text_row(c, y, SCREEN_W - 8 - w, 8, buf, 2, WHITE);
        }
        return;
    }
    if (y < HOTBAR_Y - 24)
        return;

    for (i = 0; i < HOTBAR_SLOTS && y >= HOTBAR_Y && y < HOTBAR_Y + SLOT; i++) {
        int sx = HOTBAR_X + i * (SLOT + SLOT_GAP), sel = (i == fr->hotbar);
        int border = sel ? 3 : 1, iy = y - HOTBAR_Y - (SLOT - ICON) / 2;
        uint32_t edge = sel ? WHITE : 0;

        if (y < HOTBAR_Y + border || y >= HOTBAR_Y + SLOT - border) {
            fill(c, y, sx, sx + SLOT, edge);
            continue;
        }
        fill(c, y, sx, sx + border, edge);
        fill(c, y, sx + SLOT - border, sx + SLOT, edge);
        tint(c, y, sx + border, sx + SLOT - border, HUD_BACK_RGB, HUD_BACK_ALPHA);
        if (iy >= 0 && iy < ICON) {
            const uint32_t *row = icons[i] + iy * ICON;
            int ix = 0, x0 = sx + (SLOT - ICON) / 2;

            while (ix < ICON) {
                int start = ix;

                while (ix < ICON && row[ix] == row[start])
                    ix++;
                if (row[start] != ICON_OFF)
                    fill(c, y, x0 + start, x0 + ix, row[start]);
            }
        }
    }
    for (i = 0; i < HOTBAR_SLOTS; i++) {
        char num[2] = { (char)('1' + i), 0 };

        text_row(c, y, HOTBAR_X + i * (SLOT + SLOT_GAP) + 3, HOTBAR_Y + 3, num, 1, WHITE);
    }

    /* name of the selected block above the bar */
    {
        const char *name = hotbar_names[fr->hotbar];
        int w = (int)strlen(name) * 12 - 2;

        text_row(c, y, (SCREEN_W - w) / 2, HOTBAR_Y - 20, name, 2, WHITE);
    }
}

/* ---- sky, sun, weather ------------------------------------------------------ */

static void sky_row(struct cmds *c, const struct frame *fr, int y)
{
    float pyc = y + 0.5f;
    float d0 = fr->dk0[0] + fr->dkx[0] * CX + fr->dky[0] * pyc;
    float d1 = fr->dk0[1] + fr->dkx[1] * CX + fr->dky[1] * pyc;
    float d2 = fr->dk0[2] + fr->dkx[2] * CX + fr->dky[2] * pyc;
    float e = d1 / sqrtf(d0 * d0 + d1 * d1 + d2 * d2);
    int l = (e <= 0.0f) ? 0 : (int)(sqrtf(e) * (SKY_LEVELS - 1));

    if (l > SKY_LEVELS - 1)
        l = SKY_LEVELS - 1;
    fill(c, y, 0, SCREEN_W, sky_rgb[l]);
}

static void square_row(struct cmds *c, int y, int cx, int cy, int half, uint32_t rgb)
{
    if (y >= cy - half && y < cy + half)
        fill(c, y, cx - half, cx + half, rgb);
}

/* Rain streaks and snowflakes in row y, hidden behind anything nearer. */
static void particles_row(struct cmds *c, const struct frame *fr, int y)
{
    int i;

    for (i = 0; i < fr->nparticles; i++) {
        const struct particle *pt = &fr->particles[i];

        if (pt->rain) {
            int ya = pt->y1 < pt->y0 ? pt->y1 : pt->y0;
            int yb = pt->y1 < pt->y0 ? pt->y0 : pt->y1;
            int x;

            if (y < ya || y > yb)
                continue;
            x = (yb == ya) ? pt->x0
              : pt->x1 + (pt->x0 - pt->x1) * (y - pt->y1) / (pt->y0 - pt->y1);
            fill_z(c, y, x, x + 1, rain_c, pt->iz);
        } else if (y >= pt->y0 && y < pt->y0 + pt->size) {
            fill_z(c, y, pt->x0, pt->x0 + pt->size, snow_c, pt->iz);
        }
    }
}

/* ---- band ------------------------------------------------------------------ */

void render_resend_colors(void)
{
    sets_dirty = 1;
}

int render_prologue(uint64_t *cmd, int cap)
{
    int n = 0, s;

    if (sets_dirty && cap > GPU_COLOR_SETS * 3) {
        for (s = 0; s < GPU_COLOR_SETS; s++) {
            const uint32_t *k = set_rgb[s];
            uint64_t lo = (uint64_t)k[0] | (uint64_t)k[1] << 24 | (uint64_t)k[2] << 48;
            uint64_t hi = (uint64_t)(k[2] >> 16) | (uint64_t)k[3] << 8 | (uint64_t)k[4] << 32;

            cmd[n++] = (uint64_t)GPU_COLORS | (uint64_t)s << 35;
            cmd[n++] = lo;
            cmd[n++] = hi;
        }
        sets_dirty = 0;
    }
    cmd[n++] = GPU_END;
    return n;
}

int render_band(const struct frame *fr, int band, uint64_t *cmd, int cap)
{
    struct cmds c = { cmd, 0, cap - 1, 0 };
    int y0 = band * BAND_H, y1 = y0 + BAND_H;
    int i, y, pass;

    cmd[c.n++] = gpu_hdr(GPU_BAND, band, 0, y0);

    for (y = y0; y < y1; y++) {
        sky_row(&c, fr, y);
        for (i = 0; i < fr->nstars; i++)
            if (fr->stars[i][1] == y)
                fill(&c, y, fr->stars[i][0], fr->stars[i][0] + 1, star_c);
        if (fr->sun_visible) {
            square_row(&c, y, fr->sun_x, fr->sun_y, SUN_HALO, sun_c[0]);
            square_row(&c, y, fr->sun_x, fr->sun_y, SUN_CORE, sun_c[1]);
        }
        if (fr->moon_visible) {
            square_row(&c, y, fr->moon_x, fr->moon_y, SUN_CORE + 4, moon_c[0]);
            square_row(&c, y, fr->moon_x, fr->moon_y, SUN_CORE - 4, moon_c[1]);
        }
    }

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

            if (p->axis == 1 && !p->trans)
                span_flat(&c, fr, p, y, x0, x1);
            else
                span_wall(&c, fr, p, y, x0, x1);
        }
    }

    for (y = y0; y < y1; y++) {
        if (fr->nparticles && !fr->underwater)
            particles_row(&c, fr, y);
        if (fr->underwater)
            tint(&c, y, 0, SCREEN_W, water_fog_rgb, WATER_FOG_ALPHA);

        /* crosshair: inverted pixels */
        if (y >= SCREEN_H / 2 - 7 && y <= SCREEN_H / 2 + 7) {
            int x0 = SCREEN_W / 2, x1 = x0 + 1;
            uint64_t *w;

            if (y == SCREEN_H / 2) {
                x0 -= 7;
                x1 += 7;
            }
            if ((w = put(&c, 1)))
                w[0] = gpu_hdr(GPU_INVERT, x0, x1, y);
        }
        hud_row(&c, fr, y);
    }

    if (c.full)
        __atomic_fetch_add(&render_overflows, 1, __ATOMIC_RELAXED);
    cmd[c.n++] = GPU_END;
    return c.n;
}
