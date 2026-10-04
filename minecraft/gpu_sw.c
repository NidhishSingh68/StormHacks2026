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

/* GPU registers: loaded by the prologue, persist across frames */
static uint32_t colors[GPU_COLOR_SETS][GPU_TONES];
static uint32_t mats[GPU_FOG_LEVELS][2];                /* water, glass */
static struct gpu_frame frm;

static const uint8_t tone_map[4] = { 0, 1, 2, 0 };

static inline uint32_t depth22(uint32_t iz)             /* FILL */
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

static inline int32_t sext(uint64_t v, int bits)
{
    return (int32_t)((int64_t)(v << (64 - bits)) >> (64 - bits));
}

static inline int32_t add32(int32_t a, int32_t b)       /* wrapping */
{
    return (int32_t)((uint32_t)a + (uint32_t)b);
}

static inline int32_t mul32(int32_t a, int32_t b)       /* wrapping */
{
    return (int32_t)((uint32_t)a * (uint32_t)b);
}

/*
 * The world position of a view ray (na, nb: its components along the
 * face's axes) where 1/z is iz: eye + n / iz, 16.16. Also z itself, 16.16.
 */
static inline void sample(int64_t iz, int32_t na, int32_t nb, int32_t ea, int32_t eb,
                          int32_t *a, int32_t *b, int64_t *z)
{
    uint32_t mant, q;
    int lz, s;

    if (iz < GPU_IZ_MIN)
        iz = GPU_IZ_MIN;
    if (iz >= (1ll << 46))
        iz = (1ll << 46) - 1;
    lz = __builtin_clzll((uint64_t)iz) - 16;            /* leading zeros in 48 bits */
    mant = (uint32_t)(((uint64_t)iz << lz) >> 23);     /* 2^24 .. 2^25 - 1 */
    q = (uint32_t)((1ull << 49) / mant);               /* 1/iz = q * 2^(lz - 32) */
    s = 45 - lz;
    *a = add32(ea, (int32_t)(((int64_t)na * q) >> s));
    *b = add32(eb, (int32_t)(((int64_t)nb * q) >> s));
    if (z)
        *z = lz >= 16 ? (int64_t)q << (lz - 16) : (int64_t)(q >> (16 - lz));
}

static inline uint32_t absdiff(int32_t p, int32_t q)
{
    int64_t d = (int64_t)p - q;

    d = d < 0 ? -d : d;
    return d > 0xFFFFFFFFll ? 0xFFFFFFFFu : (uint32_t)d;
}

static void poly(const uint64_t *p, uint8_t *fb, uint32_t *zbuf)
{
    uint64_t w0 = p[0];
    int flag = (w0 >> 4) & 1, nedges = (w0 >> 5) & 7, axis = (w0 >> 8) & 3;
    int kind = (w0 >> 10) & 3, rows = (w0 >> 16) & 31, y0 = (w0 >> 26) & 0x1FF;
    unsigned setbase = (w0 >> 35) & 0x7FF;
    int64_t iz0 = (int64_t)p[1], dizdx = (int64_t)p[2], dizdy = (int64_t)p[3];
    int ka = gpu_axis_a(axis), kb = gpu_axis_b(axis);
    int32_t ea = frm.eye[ka], eb = frm.eye[kb];
    unsigned alpha = kind == GPU_GLASS ? frm.alpha_glass : frm.alpha_water;
    int r, e;

    for (r = 0; r < rows; r++) {
        int y = y0 + r, x0, x1, x, any = 0;
        int32_t xl = INT32_MAX, xr = INT32_MIN, na, nb;
        int64_t izr, iz;
        uint8_t *row = fb + y * SCREEN_W;
        uint32_t *zrow = zbuf + (y & 15) * SCREEN_W;

        for (e = 0; e < nedges; e++) {
            uint64_t ew = p[4 + e];
            int er0 = (ew >> 48) & 31, er1 = (ew >> 53) & 31;
            int32_t ex;

            if (r < er0 || r >= er1)
                continue;
            ex = sext(ew, 24) + sext(ew >> 24, 24) * (r - er0);
            if (ex < xl) xl = ex;
            if (ex > xr) xr = ex;
            any = 1;
        }
        if (!any)
            continue;
        x0 = (xl + 2047) >> 12;
        x1 = (xr + 2047) >> 12;
        if (x0 < 0) x0 = 0;
        if (x1 > SCREEN_W) x1 = SCREEN_W;
        if (x0 >= x1)
            continue;

        izr = iz0 + dizdy * r;
        iz = izr + dizdx * x0;
        na = add32(add32(frm.n0[ka], mul32(frm.ny[ka], y)), mul32(frm.nx[ka], x0));
        nb = add32(add32(frm.n0[kb], mul32(frm.ny[kb], y)), mul32(frm.nx[kb], x0));

        for (x = x0; x < x1; x++, iz += dizdx, na = add32(na, frm.nx[ka]),
                                               nb = add32(nb, frm.nx[kb])) {
            int32_t a1, b1, a2, b2, a3, b3;
            uint32_t z, fa, fb2;
            int64_t zq, f;
            unsigned set, fog;

            if (iz < GPU_IZ_MIN)
                continue;
            z = (uint32_t)(iz >> 21);
            if (z > 0xFFFFFFu)
                z = 0xFFFFFFu;
            if (!(z > zrow[x]))
                continue;
            if (kind == GPU_SOLID)
                zrow[x] = z;

            sample(iz, na, nb, ea, eb, &a1, &b1, &zq);
            sample(iz + dizdy, add32(na, frm.ny[ka]), add32(nb, frm.ny[kb]), ea, eb,
                   &a2, &b2, NULL);
            sample(iz + dizdx, add32(na, frm.nx[ka]), add32(nb, frm.nx[kb]), ea, eb,
                   &a3, &b3, NULL);
            fa = absdiff(a3, a1);
            if (absdiff(a2, a1) > fa) fa = absdiff(a2, a1);
            fb2 = absdiff(b3, b1);
            if (absdiff(b2, b1) > fb2) fb2 = absdiff(b2, b1);

            f = ((zq - frm.fog_start) * (int64_t)frm.fog_scale) >> 40;
            fog = f < 0 ? 0 : f > GPU_FOG_LEVELS - 1 ? GPU_FOG_LEVELS - 1 : (unsigned)f;
            set = (setbase + fog) & 0x7FF;

            if (kind == GPU_SOLID) {
                unsigned tone = 0;

                if (fa <= GPU_FOOT_MAX && fb2 <= GPU_FOOT_MAX) {
                    unsigned bu = ((uint32_t)a1 >> 16), bv = ((uint32_t)b1 >> 16);

                    if (((uint32_t)a1 & 0xFFFF) < fa || ((uint32_t)b1 & 0xFFFF) < fb2)
                        tone = 3;
                    else if (flag && bu == frm.sel[ka] && bv == frm.sel[kb])
                        tone = 4;
                    else
                        tone = tone_map[gpu_hash((uint32_t)(int32_t)(int16_t)bu,
                                                 (uint32_t)(int32_t)(int16_t)bv) >> 30];
                }
                row[x] = gpu_quantize(colors[set][tone], dither_at(x, y));
            } else if (kind == GPU_GLASS && fa < GPU_FOOT_MAX && fb2 < GPU_FOOT_MAX &&
                       (((uint32_t)a1 & 0xFFFF) < fa || ((uint32_t)b1 & 0xFFFF) < fb2)) {
                row[x] = gpu_quantize(colors[set][3], dither_at(x, y));
            } else {
                row[x] = blend(row[x], mats[fog][kind == GPU_GLASS], alpha);
            }
        }
    }
}

static void load_frame(const uint64_t *w)
{
    frm.n0[0] = (int32_t)w[1];  frm.nx[0] = (int32_t)(w[1] >> 32);
    frm.ny[0] = (int32_t)w[2];  frm.n0[1] = (int32_t)(w[2] >> 32);
    frm.nx[1] = (int32_t)w[3];  frm.ny[1] = (int32_t)(w[3] >> 32);
    frm.n0[2] = (int32_t)w[4];  frm.nx[2] = (int32_t)(w[4] >> 32);
    frm.ny[2] = (int32_t)w[5];  frm.eye[0] = (int32_t)(w[5] >> 32);
    frm.eye[1] = (int32_t)w[6]; frm.eye[2] = (int32_t)(w[6] >> 32);
    frm.fog_start = (int32_t)w[7];
    frm.fog_scale = (int32_t)(w[7] >> 32);
    frm.sel[0] = (uint16_t)w[8];
    frm.sel[1] = (uint16_t)(w[8] >> 16);
    frm.sel[2] = (uint16_t)(w[8] >> 32);
    frm.alpha_water = (uint8_t)(w[8] >> 48);
    frm.alpha_glass = (uint8_t)(w[8] >> 56);
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

        case GPU_FRAME:
            load_frame(p + i);
            i += 9;
            break;

        case GPU_MATS: {
            int f;

            for (f = 0; f < GPU_FOG_LEVELS; f++) {
                mats[f][0] = (uint32_t)p[i + 1 + f] & 0xFFFFFF;
                mats[f][1] = (uint32_t)(p[i + 1 + f] >> 24) & 0xFFFFFF;
            }
            i += 1 + GPU_FOG_LEVELS;
            break;
        }

        case GPU_FILL: {
            uint32_t rgb = (uint32_t)p[i + 1] & 0xFFFFFF;
            uint32_t z = depth22((uint32_t)(p[i + 1] >> 32));

            for (x = x0; x < x1; x++)
                if (!flag || z > zrow[x])
                    row[x] = gpu_quantize(rgb, dither_at(x, y));
            i += 2;
            break;
        }

        case GPU_TINT: {
            uint32_t rgb = (uint32_t)p[i + 1] & 0xFFFFFF;
            unsigned alpha = (p[i + 1] >> 24) & 0xFF;

            for (x = x0; x < x1; x++)
                row[x] = blend(row[x], rgb, alpha);
            i += 2;
            break;
        }

        case GPU_INVERT:
            for (x = x0; x < x1; x++)
                row[x] ^= 0xFF;
            i += 1;
            break;

        case GPU_POLY:
            poly(p + i, fb, zbuf);
            i += 4 + ((w0 >> 5) & 7);
            break;

        case GPU_END:
            return;

        default:                        /* NOP */
            i += 1;
            break;
        }
    }
}
