/*
 * render.h - polygon setup and GPU command lists for the voxel world
 */
#ifndef RENDER_H
#define RENDER_H

#include "mc.h"
#include "terrain.h"            /* WEATHER_* */

#define MAX_POLYS       16384   /* indices must fit in uint16_t */
#define MAX_BAND_POLYS  8192
#define MAX_POLY_EDGES  6       /* a quad clipped by the near plane has <= 5 */
#define NUM_STARS       320
#define MAX_PARTICLES   900

/*
 * The look of a weather: sky, light, fog and what is in the sky or falling
 * from it. Weathers are blended by mixing these (all floats).
 */
struct env {
    float sky_horizon[3];       /* also the fog colour */
    float sky_zenith[3];
    float light;                /* multiplies all block colours */
    float fog_near, fog_far;    /* fog range, as fractions of render distance */
    float sun, moon, stars;     /* 0..1: how visible */
    float rain, snow;           /* 0..1: how much is falling */
};

const char *weather_name(int weather);
void env_for_weather(int weather, struct env *out);
void env_mix(struct env *out, const struct env *a, const struct env *b, float t);

/* Rebuild the colour tables for e. Main thread, between frames only. */
void render_set_env(const struct env *e);

/* Per-frame inputs besides the camera. */
struct view {
    const int *sel;             /* block under the crosshair, or NULL */
    int underwater;
    int hotbar;                 /* selected hotbar slot */
    int weather;                /* WEATHER_* shown in the corner */
    float time;                 /* seconds, animates rain and snow */
    int fps;                    /* frames per second shown top right, <0 hides */
};

/* A raindrop streak (x0,y0)-(x1,y1) or a snowflake at (x0,y0), on screen. */
struct particle {
    int16_t x0, y0, x1, y1;
    float   iz;                 /* 1/depth, for the depth test */
    uint8_t rain;
    uint8_t size;
};

/* The hotbar: blocks the player can place, picked with keys 1..7. */
#define HOTBAR_SLOTS    7
extern const uint8_t hotbar_blocks[HOTBAR_SLOTS];

struct camera {
    float pos[3];               /* eye position */
    float yaw, pitch;           /* radians; yaw 0 looks along -z */
};

/* A screen-space convex polygon, one per visible block-face rectangle. */
struct edge {
    int16_t r0, r1;             /* covers rows r0 .. r1 - 1 */
    float x, dx;                /* x at the centre of row r0, step per row */
};

struct poly {
    int16_t ytop, ybot;         /* rows [ytop, ybot) */
    uint8_t nedges;
    uint8_t axis;               /* face normal axis */
    uint8_t face;               /* shading index: top, bottom, x side, z side */
    uint8_t block;
    uint8_t sel;                /* contains a face of the selected block */
    uint8_t trans;              /* see-through: drawn blended, after the rest */
    float   ia, ib, ic;         /* 1/depth = ia + ib * px + ic * py */
    struct edge e[MAX_POLY_EDGES];
};

/* Everything the band renderer needs for one frame. */
struct frame {
    float    eye[3];
    float    fwd[3], right[3], up[3];
    float    dk0[3], dkx[3], dky[3];    /* view ray = dk0 + dkx * px + dky * py */
    int      sun_visible, sun_x, sun_y;
    int      moon_visible, moon_x, moon_y;
    int      nstars;
    int16_t  stars[NUM_STARS][2];
    int      nparticles;
    struct particle particles[MAX_PARTICLES];
    int      underwater;
    int      sel_valid, sel[3];         /* block under the crosshair */
    int      hotbar;                    /* selected hotbar slot */
    int      weather;
    int      fps;
    int      npolys, nchunks;
    struct poly polys[MAX_POLYS];
    int      band_count[NUM_BANDS];
    uint16_t band_polys[NUM_BANDS][MAX_BAND_POLYS];
};

void render_init(int render_dist);

/* Build the polygon list (and sky, rain, snow) for this frame. */
void render_setup(struct frame *fr, const struct camera *cam, const struct view *v);

/*
 * The frame as GPU commands (gpu.h): render_prologue() writes region 0
 * (camera; colours, when they changed since the last call), render_band() the
 * commands drawing rows [band * BAND_H, (band + 1) * BAND_H). Both return
 * the number of 64-bit words written (at most cap, ending with END).
 * Different bands may be built by different threads at the same time; the
 * prologue only by the thread that calls render_set_env().
 */
int render_prologue(const struct frame *fr, uint64_t *cmd, int cap);
int render_band(const struct frame *fr, int band, uint64_t *cmd, int cap);

/* Put the colour sets into the next prologue again (e.g. after a GPU reset). */
void render_resend_colors(void);

extern int render_overflows;    /* bands whose commands did not all fit */

#endif
