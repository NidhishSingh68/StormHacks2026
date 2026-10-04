/*
 * gpu.h - command stream for the span GPU in the FPGA fabric
 *
 * The CPU turns each frame into a list of commands: rows of pixels ("spans")
 * with start values and per-pixel steps, colour fills and see-through
 * blends. The span GPU in the fabric (fpga/span_gpu.v) reads the list
 * straight from DDR and does all per-pixel work: depth test, block grid
 * lines, per-block shade hash, highlight, blending, ordered dithering to
 * RGB332, and writing the framebuffer. gpu_sw.c executes the same commands
 * in C with exactly the same integer rules; it is the reference the Verilog
 * is tested against, the PC build's renderer, and the fallback on
 * bitstreams without the GPU.
 *
 * A frame is GPU_REGIONS regions of GPU_REGION bytes each: region 0 is the
 * prologue (colour sets), region 1 + b draws band b (rows 16b..16b+15). Each
 * region is a list of commands ending with END, so the two cores can fill
 * different regions at the same time.
 *
 * Every command is one or more little-endian 64-bit words. Word 0:
 *
 *   [3:0]   op
 *   [4]     flag: SPAN = highlight the selected block, FILL/BLEND = depth test
 *   [15:6]  x0 (first pixel; BAND: band index)
 *   [25:16] x1 (one past the last pixel)
 *   [34:26] y (screen row; must be inside the current band)
 *   [45:35] SPAN/SHADE: colour set; COLORS: set to load
 *   [63:48] SPAN/BLEND: la (grid-line threshold along the first axis)
 *
 * Commands:
 *   NOP     1 word
 *   BAND    1 word   start band b: fresh depth buffer. The GPU draws the band
 *                    on chip and copies it to the framebuffer once the
 *                    display has scanned past it, so frames never tear.
 *   COLORS  3 words  w1..w2 = 5 RGB888 tones of a colour set, 24 bits each,
 *                    tone k at bits [24k+23:24k] of the 120-bit w1 | w2 << 64
 *   SPAN    5 words  opaque span, depth tested and written
 *                    w1 = iz0 | diz << 32    1/z * 2^22 and its step
 *                    w2 = ua0 | dua << 32    world coordinate (16.16), step
 *                    w3 = ub0 | dub << 32
 *                    w4 = lb | sela << 16 | selb << 32   (16 bits each)
 *   FILL    2 words  w1 = rgb888 | iz << 32: dithered fill (flag: depth test)
 *   BLEND   6 words  see-through span, not written to the depth buffer
 *                    w1..w3 as SPAN, w4 = lb | mat_rgb << 16 | alpha << 40,
 *                    w5 = line_rgb
 *   SHADE   2 words  SPAN in tone 0 only (ua = ub = la = lb = 0): plain
 *                    faces and far-away blocks. w1 = iz0 | diz << 32
 *   INVERT  1 word   pixel = ~pixel
 *   END     1 word   end of region
 *
 * Per-pixel rules (gpu_sw.c is the definition):
 *   depth   z = clamp(iz >> 3, 0, 2^24 - 1), 0 if iz is negative; drawn if
 *           z > stored (stored = 0 at the start of each band); SPAN stores z
 *   tone    3 if (ua & 0xFFFF) < la or (ub & 0xFFFF) < lb   (grid line)
 *           4 if selected: flag and ua >> 16 == sela and ub >> 16 == selb
 *           else tone_map[hash(ua >> 16, ub >> 16) >> 30], tone_map = {0,1,2,0}
 *           (ua >> 16 and ub >> 16 sign-extended from 16 bits)
 *   dither  d = BAYER[(y & 3) * 4 + (x & 3)]
 *           r3 = min(7, (r * 7 + 16d + 8) >> 8), likewise g3
 *           b2 = min(3, (b * 3 + 16d + 8) >> 8)
 *   blend   dst expanded (EXPAND3[r3], EXPAND3[g3], b2 * 85), then per
 *           channel c = (dst * (256 - alpha) + mat * alpha) >> 8, dithered
 *           with d = BAYER[5]; grid-line pixels get line_rgb, dithered
 *           normally
 */
#ifndef GPU_H
#define GPU_H

#include <stdint.h>

enum {
    GPU_NOP, GPU_BAND, GPU_COLORS, GPU_SPAN, GPU_FILL,
    GPU_BLEND, GPU_INVERT, GPU_SHADE, GPU_END
};

#define GPU_FLAG            (1ull << 4)
#define GPU_COLOR_SETS      2048
#define GPU_TONES           5
#define GPU_Z_SCALE         4194304.0f  /* iz = 1/z * 2^22 */
#define GPU_BLEND_DITHER    4           /* BAYER[5]: dither used for blends */

#define GPU_REGION          (256u * 1024u)
#define GPU_REGION_WORDS    (GPU_REGION / 8)
#define GPU_REGIONS         31          /* 0 = prologue, 1..30 = bands 0..29 */
#define GPU_BUF_SIZE        (GPU_REGION * GPU_REGIONS)
#define GPU_BUF_STRIDE      (8u << 20)  /* buffer b at GPU virtual b * 8 MiB */

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

/* 1/z as the GPU's fixed point (negative: never drawn) */
static inline uint32_t gpu_iz(float iz)
{
    float v = iz * GPU_Z_SCALE;

    if (v >= 2147483520.0f)
        return 0x7FFFFF80u;
    return (uint32_t)(int32_t)v;
}

/*
 * Software GPU (gpu_sw.c). Colour sets persist across calls, like the
 * GPU's. gpu_sw_region() runs one region (until END or maxwords) into a
 * 640x480 RGB332 framebuffer; zbuf is 16 x 640 words of scratch for the
 * calling thread. Regions of different bands may run in parallel, after the
 * prologue.
 */
void gpu_sw_region(const uint64_t *cmds, uint32_t maxwords, uint8_t *fb, uint32_t *zbuf);

#endif
