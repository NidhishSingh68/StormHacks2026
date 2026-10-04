/*
 * gpu_sw.c - the span GPU in C (reference model)
 *
 * Executes the command stream of gpu.h with exactly the integer rules the
 * FPGA uses (fpga/span_gpu.v). It is the definition the hardware is tested
 * against (byte-identical output), the renderer of the PC build, and the
 * fallback on bitstreams without the GPU.
 */

#include <string.h>
#include "mc.h"
#include "gpu.h"

static uint32_t colors[GPU_COLOR_SETS][GPU_TONES];     /* persist across frames */

static const uint8_t tone_map[4] = { 0, 1, 2, 0 };

static inline uint32_t depth(uint32_t iz)
{
    if (iz & 0x80000000u)               /* negative: behind, never drawn */
        return 0;
    iz >>= 3;
    return iz > 0xFFFFFFu ? 0xFFFFFFu : iz;
}

static inline unsigned dither_at(int x, int y)
{
    return GPU_BAYER[(y & 3) * 4 + (x & 3)];
}

static inline uint8_t blend(uint8_t dst, uint32_t mat, unsigned alpha)
{
    unsigned dr = GPU_EXPAND3[dst >> 5], dg = GPU_EXPAND3[(dst >> 2) & 7], db = (dst & 3) * 85u;
    unsigned mr = (mat >> 16) & 0xFF, mg = (mat >> 8) & 0xFF, mb = mat & 0xFF;
    unsigned r = (dr * (256 - alpha) + mr * alpha) >> 8;
    unsigned g = (dg * (256 - alpha) + mg * alpha) >> 8;
    unsigned b = (db * (256 - alpha) + mb * alpha) >> 8;

    return gpu_quantize(r << 16 | g << 8 | b, GPU_BLEND_DITHER);
}

static inline unsigned tone_at(uint32_t ua, uint32_t ub, unsigned la, unsigned lb,
                               int sel, unsigned sela, unsigned selb)
{
    unsigned bu = ua >> 16, bv = ub >> 16;

    if ((ua & 0xFFFF) < la || (ub & 0xFFFF) < lb)
        return 3;
    if (sel && bu == sela && bv == selb)
        return 4;
    return tone_map[gpu_hash((uint32_t)(int32_t)(int16_t)bu,
                             (uint32_t)(int32_t)(int16_t)bv) >> 30];
}

void gpu_sw_region(const uint64_t *p, uint32_t n, uint8_t *fb, uint32_t *zbuf)
{
    uint32_t i = 0;

    while (i < n) {
        uint64_t w0 = p[i];
        unsigned op = w0 & 15;
        int flag = (w0 >> 4) & 1;
        int x0 = (w0 >> 6) & 0x3FF, x1 = (w0 >> 16) & 0x3FF, y = (w0 >> 26) & 0x1FF;
        uint8_t *row = fb + (y < SCREEN_H ? y : 0) * SCREEN_W;
        uint32_t *zrow = zbuf + (y & 15) * SCREEN_W;
        int x;

        if (x1 > SCREEN_W)
            x1 = SCREEN_W;

        switch (op) {
        case GPU_BAND:
            memset(zbuf, 0, 16 * SCREEN_W * sizeof(*zbuf));
            i += 1;
            break;

        case GPU_COLORS: {
            unsigned set = (w0 >> 35) & 0x7FF, k;
            uint64_t lo = p[i + 1], hi = p[i + 2];

            for (k = 0; k < GPU_TONES; k++) {
                unsigned bit = 24 * k;
                uint32_t c;

                if (bit + 24 <= 64)
                    c = (uint32_t)(lo >> bit);
                else if (bit < 64)
                    c = (uint32_t)(lo >> bit) | (uint32_t)(hi << (64 - bit));
                else
                    c = (uint32_t)(hi >> (bit - 64));
                colors[set][k] = c & 0xFFFFFF;
            }
            i += 3;
            break;
        }

        case GPU_SHADE: {
            unsigned set = (w0 >> 35) & 0x7FF;
            uint32_t iz = (uint32_t)p[i + 1], diz = (uint32_t)(p[i + 1] >> 32);

            for (x = x0; x < x1; x++, iz += diz) {
                uint32_t z = depth(iz);

                if (z > zrow[x]) {
                    zrow[x] = z;
                    row[x] = gpu_quantize(colors[set][0], dither_at(x, y));
                }
            }
            i += 2;
            break;
        }

        case GPU_SPAN: {
            unsigned set = (w0 >> 35) & 0x7FF, la = (w0 >> 48) & 0xFFFF;
            uint32_t iz = (uint32_t)p[i + 1], diz = (uint32_t)(p[i + 1] >> 32);
            uint32_t ua = (uint32_t)p[i + 2], dua = (uint32_t)(p[i + 2] >> 32);
            uint32_t ub = (uint32_t)p[i + 3], dub = (uint32_t)(p[i + 3] >> 32);
            unsigned lb = p[i + 4] & 0xFFFF;
            unsigned sela = (p[i + 4] >> 16) & 0xFFFF, selb = (p[i + 4] >> 32) & 0xFFFF;

            for (x = x0; x < x1; x++, iz += diz, ua += dua, ub += dub) {
                uint32_t z = depth(iz);

                if (z > zrow[x]) {
                    zrow[x] = z;
                    row[x] = gpu_quantize(colors[set][tone_at(ua, ub, la, lb, flag,
                                                              sela, selb)],
                                          dither_at(x, y));
                }
            }
            i += 5;
            break;
        }

        case GPU_FILL: {
            uint32_t rgb = (uint32_t)p[i + 1] & 0xFFFFFF;
            uint32_t z = depth((uint32_t)(p[i + 1] >> 32));

            for (x = x0; x < x1; x++)
                if (!flag || z > zrow[x])
                    row[x] = gpu_quantize(rgb, dither_at(x, y));
            i += 2;
            break;
        }

        case GPU_BLEND: {
            unsigned la = (w0 >> 48) & 0xFFFF;
            uint32_t iz = (uint32_t)p[i + 1], diz = (uint32_t)(p[i + 1] >> 32);
            uint32_t ua = (uint32_t)p[i + 2], dua = (uint32_t)(p[i + 2] >> 32);
            uint32_t ub = (uint32_t)p[i + 3], dub = (uint32_t)(p[i + 3] >> 32);
            unsigned lb = p[i + 4] & 0xFFFF;
            uint32_t mat = (p[i + 4] >> 16) & 0xFFFFFF;
            unsigned alpha = (p[i + 4] >> 40) & 0xFF;
            uint32_t line = (uint32_t)p[i + 5] & 0xFFFFFF;

            for (x = x0; x < x1; x++, iz += diz, ua += dua, ub += dub) {
                if (flag && !(depth(iz) > zrow[x]))
                    continue;
                if ((ua & 0xFFFF) < la || (ub & 0xFFFF) < lb)
                    row[x] = gpu_quantize(line, dither_at(x, y));
                else
                    row[x] = blend(row[x], mat, alpha);
            }
            i += 6;
            break;
        }

        case GPU_INVERT:
            for (x = x0; x < x1; x++)
                row[x] ^= 0xFF;
            i += 1;
            break;

        case GPU_END:
            return;

        default:                        /* NOP */
            i += 1;
            break;
        }
    }
}
