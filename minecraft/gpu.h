/*
 * gpu.h - command stream for the span GPU in the FPGA fabric
 *
 * The CPU turns each frame into a list of commands: one POLY per block-face
 * polygon and band it touches, plus colour fills and see-through tints for
 * the sky, weather and HUD. The GPU in the fabric (fpga/span_gpu.v) reads
 * the list straight from DDR and does all the rest: it walks the polygon's
 * edges row by row, and for every pixel computes 1/z, the depth test, the
 * exact (perspective-correct) world position on the face, how big a block
 * is on screen there (block grid lines fade out, then the face goes plain),
 * fog, the block's shade, the highlight, blending, ordered dithering to
 * RGB332, and writes the framebuffer. gpu_sw.c executes the same commands
 * in C with exactly the same integer rules; it is the reference the Verilog
 * is tested against, the PC build's renderer, and the fallback on
 * bitstreams without the GPU.
 *
 * A frame is GPU_REGIONS regions of GPU_REGION bytes each: region 0 is the
 * prologue (frame constants, colours), region 1 + b draws band b (rows
 * 16b..16b+15). Each region is a list of commands ending with END, so the
 * two cores can fill different regions at the same time.
 *
 * Every command is one or more little-endian 64-bit words. Word 0:
 *
 *   [3:0]   op
 *   [4]     flag: POLY = highlight the selected block, FILL = depth test
 *   [15:6]  x0 (first pixel; BAND: band index)
 *   [25:16] x1 (one past the last pixel)
 *   [34:26] y (screen row; must be inside the current band)
 *   [45:35] COLORS: set to load; POLY: colour set of fog level 0
 *
 * Commands:
 *   NOP     1 word
 *   BAND    1 word   start band b: fresh depth buffer. The GPU draws the band
 *                    on chip and copies it to the framebuffer once the
 *                    display has scanned past it, so frames never tear.
 *   COLORS  3 words  w1..w2 = 5 RGB888 tones of a colour set, 24 bits each,
 *                    tone k at bits [24k+23:24k] of the 120-bit w1 | w2 << 64
 *                    (tones: 0..2 block shades, 3 grid line, 4 highlight)
 *   FRAME   9 words  camera and fog (see struct gpu_frame below)
 *   FILL    2 words  w1 = rgb888 | iz << 32: dithered fill (flag: depth test,
 *                    iz = 1/z * 2^22)
 *   TINT    2 words  w1 = rgb888 | alpha << 24: pixel blended towards rgb
 *   INVERT  1 word   pixel = ~pixel
 *   MATS    33 words w1 + f: see-through colours at fog level f,
 *                    water rgb888 | glass rgb888 << 24
 *   END     1 word   end of region
 *   POLY    4 + n words, a convex polygon's rows y..y+rows-1 (in one band)
 *                    w0 [7:5] n edges, [9:8] axis of the face normal,
 *                       [11:10] kind (0 solid, 2 glass, 3 water),
 *                       [20:16] rows, y, colour set of fog level 0, flag
 *                    w1..w3: 1/z at pixel (0, y), its step per pixel in x
 *                       and per row, all * 2^40 (signed)
 *                    w4 + i: edge i: [23:0] x at row r0 (12.12 signed),
 *                       [47:24] x step per row (12.12 signed), [52:48] r0,
 *                       [57:53] r1: covers rows y+r0 .. y+r1-1
 *
 * Per-pixel rules (gpu_sw.c is the definition):
 *   depth   FILL: z = clamp(iz >> 3, 0, 2^24 - 1), 0 if iz is negative;
 *           POLY: z = clamp(iz >> 21, ...) with iz * 2^40; drawn if
 *           z > stored (stored = 0 at the start of each band); solid POLY
 *           pixels store z, glass and water do not; POLY pixels with
 *           iz < 2^20 are not drawn
 *   edges   row x range: xl = min, xr = max of x over the edges covering
 *           the row; pixels ceil(xl - 0.5) .. ceil(xr - 0.5) - 1, in 0..639
 *   world   r = 1/iz from 2^49 / (25-bit mantissa of iz); the face's two
 *           in-plane world coordinates a, b (16.16) = eye + N * r, with N the
 *           view ray's components; the same at (x + 1, y) and (x, y + 1)
 *           gives the size of a pixel on the face: fa, fb (16.16)
 *   fog     f = clamp(((z - fog_start) * fog_scale) >> 40, 0, 31), z = r
 *   solid   textured if fa <= 0.3 and fb <= 0.3, else plain (tone 0);
 *           tone 3 if (a & 0xFFFF) < fa or (b & 0xFFFF) < fb (grid line),
 *           4 if flag and a >> 16 == sel_a and b >> 16 == sel_b,
 *           else tone_map[hash(a >> 16, b >> 16) >> 30], tone_map {0,1,2,0}
 *   glass   grid lines (tone 3 of the set) if fa < 0.3 and fb < 0.3;
 *   water   other pixels blended towards MATS[f] with the kind's alpha
 *   dither  d = BAYER[(y & 3) * 4 + (x & 3)]
 *           r3 = min(7, (r * 7 + 16d + 8) >> 8), likewise g3
 *           b2 = min(3, (b * 3 + 16d + 8) >> 8)
 *   blend   dst expanded (EXPAND3[r3], EXPAND3[g3], b2 * 85), then per
 *           channel c = (dst * (256 - alpha) + mat * alpha) >> 8, dithered
 *           with d = BAYER[5]
 */
#ifndef GPU_H
#define GPU_H

#include <stdint.h>

enum {
    GPU_NOP, GPU_BAND, GPU_COLORS, GPU_FRAME, GPU_FILL,
    GPU_TINT, GPU_INVERT, GPU_MATS, GPU_END, GPU_POLY
};

enum { GPU_SOLID = 0, GPU_GLASS = 2, GPU_WATER = 3 };

#define GPU_FLAG            (1ull << 4)
#define GPU_COLOR_SETS      2048
#define GPU_TONES           5
#define GPU_FOG_LEVELS      32
#define GPU_MAX_EDGES       6
#define GPU_Z_SCALE         4194304.0f  /* FILL: iz = 1/z * 2^22 */
#define GPU_IZ_SCALE        1099511627776.0     /* POLY: iz = 1/z * 2^40 */
#define GPU_IZ_MIN          (1ll << 20)
#define GPU_N_SCALE         536870912.0         /* view ray: 2.29 */
#define GPU_FOOT_MAX        19661u              /* 0.3 in 16.16 */
#define GPU_BLEND_DITHER    4                   /* BAYER[5]: dither of blends */

#define GPU_REGION          (256u * 1024u)
#define GPU_REGION_WORDS    (GPU_REGION / 8)
#define GPU_REGIONS         31          /* 0 = prologue, 1..30 = bands 0..29 */
#define GPU_BUF_SIZE        (GPU_REGION * GPU_REGIONS)
#define GPU_BUF_STRIDE      (8u << 20)  /* buffer b at GPU virtual b * 8 MiB */

/*
 * FRAME: the view ray through pixel (x, y) is N = n0 + nx * x + ny * y per
 * world axis (2.29 fixed point); eye and fog_start in 16.16, fog_scale in
 * 8.24; the selected block; the see-through alphas. Words:
 *   w1 n0[0] | nx[0] << 32    w2 ny[0] | n0[1] << 32   w3 nx[1] | ny[1] << 32
 *   w4 n0[2] | nx[2] << 32    w5 ny[2] | eye[0] << 32  w6 eye[1] | eye[2] << 32
 *   w7 fog_start | fog_scale << 32
 *   w8 sel[0] | sel[1] << 16 | sel[2] << 32 (16 bits each)
 *      | water alpha << 48 | glass alpha << 56
 */
struct gpu_frame {
    int32_t n0[3], nx[3], ny[3];
    int32_t eye[3];
    int32_t fog_start, fog_scale;
    uint16_t sel[3];
    uint8_t alpha_water, alpha_glass;
};

/* The two in-plane world axes of a face with normal along axis: horizontal
 * faces use (x, z), x faces (y, z), z faces (x, y). */
static inline int gpu_axis_a(int axis) { return axis == 0 ? 1 : 0; }
static inline int gpu_axis_b(int axis) { return axis == 2 ? 1 : 2; }

static const uint8_t GPU_BAYER[16] = {
     0,  8,  2, 10,
    12,  4, 14,  6,
     3, 11,  1,  9,
    15,  7, 13,  5,
};

static const uint8_t GPU_EXPAND3[8] = { 0, 36, 72, 109, 145, 182, 218, 255 };

static inline uint64_t gpu_hdr(unsigned op, int x0, int x1, int y)
{
    return (uint64_t)op | (uint64_t)(x0 & 0x3FF) << 6 | (uint64_t)(x1 & 0x3FF) << 16 |
           (uint64_t)(y & 0x1FF) << 26;
}

static inline uint64_t gpu_pair(uint32_t lo, uint32_t hi)
{
    return (uint64_t)lo | (uint64_t)hi << 32;
}

/* RGB888 -> RGB332 with dither value d (0..15) */
static inline uint8_t gpu_quantize(uint32_t rgb, unsigned d)
{
    unsigned t = 16 * d + 8;
    unsigned r3 = (((rgb >> 16) & 0xFF) * 7 + t) >> 8;
    unsigned g3 = (((rgb >> 8) & 0xFF) * 7 + t) >> 8;
    unsigned b2 = ((rgb & 0xFF) * 3 + t) >> 8;

    if (r3 > 7) r3 = 7;
    if (g3 > 7) g3 = 7;
    if (b2 > 3) b2 = 3;
    return (uint8_t)(r3 << 5 | g3 << 2 | b2);
}

static inline uint32_t gpu_rgb(float r, float g, float b)
{
    int ri = (int)(r + 0.5f), gi = (int)(g + 0.5f), bi = (int)(b + 0.5f);

    ri = ri < 0 ? 0 : ri > 255 ? 255 : ri;
    gi = gi < 0 ? 0 : gi > 255 ? 255 : gi;
    bi = bi < 0 ? 0 : bi > 255 ? 255 : bi;
    return (uint32_t)ri << 16 | (uint32_t)gi << 8 | (uint32_t)bi;
}

static inline uint32_t gpu_hash(uint32_t bu, uint32_t bv)
{
    uint32_t h = bu * 0x9E3779B1u + bv * 0x85EBCA77u;

    h ^= h >> 15;
    h *= 0x2C1B3C6Du;
    h ^= h >> 12;
    return h;
}

/* FILL depth: 1/z * 2^22 (negative: never drawn) */
static inline uint32_t gpu_iz(float iz)
{
    float v = iz * GPU_Z_SCALE;

    if (v >= 2147483520.0f)
        return 0x7FFFFF80u;
    return (uint32_t)(int32_t)v;
}

/* a value as signed fixed point with the given scale, saturated */
static inline int64_t gpu_fix64(double v, double scale)
{
    v *= scale;
    if (v > 9.2e18) return INT64_MAX;
    if (v < -9.2e18) return INT64_MIN;
    return (int64_t)(v < 0 ? v - 0.5 : v + 0.5);
}

static inline int32_t gpu_fix32(double v, double scale)
{
    v *= scale;
    if (v > 2147483647.0) return INT32_MAX;
    if (v < -2147483648.0) return INT32_MIN;
    return (int32_t)(v < 0 ? v - 0.5 : v + 0.5);
}

/* edge word of POLY: x at its first row and step per row, 12.12 */
static inline uint64_t gpu_edge(double x, double dx, int r0, int r1)
{
    int32_t xi = gpu_fix32(x, 4096.0), di = gpu_fix32(dx, 4096.0);

    xi = xi > 0x7FFFFF ? 0x7FFFFF : xi < -0x800000 ? -0x800000 : xi;
    di = di > 0x7FFFFF ? 0x7FFFFF : di < -0x800000 ? -0x800000 : di;
    return (uint64_t)(xi & 0xFFFFFF) | (uint64_t)(di & 0xFFFFFF) << 24 |
           (uint64_t)(r0 & 31) << 48 | (uint64_t)(r1 & 31) << 53;
}

/*
 * Software GPU (gpu_sw.c). Colour sets, MATS and FRAME persist across
 * calls, like the GPU's registers. gpu_sw_region() runs one region (until
 * END or maxwords) into a 640x480 RGB332 framebuffer; zbuf is 16 x 640
 * words of scratch for the calling thread. Regions of different bands may
 * run in parallel, after the prologue.
 */
void gpu_sw_region(const uint64_t *cmds, uint32_t maxwords, uint8_t *fb, uint32_t *zbuf);

#endif
