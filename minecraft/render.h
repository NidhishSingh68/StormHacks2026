/*
 * render.h - software rasterizer for the voxel world
 */
#ifndef RENDER_H
#define RENDER_H

#include "mc.h"

#define MAX_POLYS       16384   /* indices must fit in uint16_t */
#define MAX_BAND_POLYS  8192
#define MAX_POLY_EDGES  6       /* a quad clipped by the near plane has <= 5 */

struct camera {
    float pos[3];               /* eye position */
    float yaw, pitch;           /* radians; yaw 0 looks along -z */
};

/* A screen-space convex polygon, one per visible block-face rectangle. */
struct edge {
    float y0, y1;               /* covers rows whose centre is in [y0, y1) */
    float x0, dxdy;             /* x at y0, and slope */
};

struct poly {
    int16_t ytop, ybot;         /* rows [ytop, ybot) */
    uint8_t nedges;
    uint8_t axis;               /* face normal axis */
    uint8_t face;               /* shading index: top, bottom, x side, z side */
    uint8_t block;
    uint8_t sel;                /* contains a face of the selected block */
    float   ia, ib, ic;         /* 1/depth = ia + ib * px + ic * py */
    struct edge e[MAX_POLY_EDGES];
};

/* Everything the band renderer needs for one frame. */
struct frame {
    float    eye[3];
    float    fwd[3], right[3], up[3];
    float    dk0[3], dkx[3], dky[3];    /* view ray = dk0 + dkx * px + dky * py */
    int      sun_visible, sun_x, sun_y;
    int      underwater;
    int      sel_valid, sel[3];         /* block under the crosshair */
    int      npolys, nchunks;
    struct poly polys[MAX_POLYS];
    int      band_count[NUM_BANDS];
    uint16_t band_polys[NUM_BANDS][MAX_BAND_POLYS];
};

void render_init(int render_dist);

/*
 * Build the polygon list for this frame from the loaded chunks. sel is the
 * block to highlight (NULL for none).
 */
void render_setup(struct frame *fr, const struct camera *cam, const int *sel,
                  int underwater);

/*
 * Draw rows [band * BAND_H, (band + 1) * BAND_H) of the frame into pixels
 * (a full SCREEN_W x SCREEN_H RGB332 buffer). zbuf is BAND_H * SCREEN_W
 * floats of scratch space private to the calling thread.
 */
void render_band(const struct frame *fr, uint8_t *pixels, int band, float *zbuf);

#endif
