/*
 * fpga.c - FPGA framebuffer access (see fpga.h)
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <time.h>
#include <unistd.h>
#include <sys/mman.h>
#include "mc.h"
#include "fpga.h"

#define FB_BASE         0xC0000000u
#define STATUS_BASE     0xFF200000u

static double now_ms(void)
{
    struct timespec ts;

    clock_gettime(CLOCK_MONOTONIC, &ts);
    return ts.tv_sec * 1e3 + ts.tv_nsec / 1e6;
}

#ifdef HOST

static uint8_t host_fb[SCREEN_SIZE] __attribute__((aligned(64)));
static double host_t0;

int fpga_open(void)
{
    host_t0 = now_ms();
    return 0;
}

void fpga_close(void)
{
}

uint32_t fpga_status(void)
{
    double frames = (now_ms() - host_t0) / (1000.0 / 60.0);
    uint32_t n = (uint32_t)frames;
    int vblank = (frames - n) < (45.0 / 525.0);

    return (n & FPGA_FRAMES) | (vblank ? FPGA_VBLANK : 0);
}

void fpga_present(const uint8_t *frame)
{
    memcpy(host_fb, frame, SCREEN_SIZE);
}

void fpga_pick_copy(const uint8_t *frame)
{
    fpga_present(frame);
}

int fpga_setup_dma(uint8_t *const bufs[], int n)
{
    (void)bufs;
    (void)n;
    return -1;
}

#else

#include "fbdma.h"

static struct fbdma dma;
static int use_dma;

static int mem_fd = -1;
static void *fb_map, *status_map;
static volatile uint32_t *status_reg;
static long page_size;

/* 0: 64-bit stores, 1: NEON 4 x 64-bit stores */
static int copy_method;

int fpga_open(void)
{
    page_size = sysconf(_SC_PAGESIZE);
    mem_fd = open("/dev/mem", O_RDWR | O_SYNC);
    if (mem_fd < 0) {
        perror("open /dev/mem (are you root?)");
        return -1;
    }
    fb_map = mmap(NULL, SCREEN_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED,
                  mem_fd, FB_BASE);
    if (fb_map == MAP_FAILED) {
        perror("mmap framebuffer");
        close(mem_fd);
        return -1;
    }
    /* writable: the DMA's registers share this page */
    status_map = mmap(NULL, page_size, PROT_READ | PROT_WRITE, MAP_SHARED, mem_fd, STATUS_BASE);
    if (status_map == MAP_FAILED) {
        perror("mmap status");
        munmap(fb_map, SCREEN_SIZE);
        close(mem_fd);
        return -1;
    }
    status_reg = status_map;
    return 0;
}

void fpga_close(void)
{
    munmap(status_map, page_size);
    munmap(fb_map, SCREEN_SIZE);
    close(mem_fd);
}

uint32_t fpga_status(void)
{
    return *status_reg;
}

/*
 * The bridge is mapped uncached (strongly ordered), so every store is its
 * own bus transaction and the copy is bound by transaction count, not by
 * bytes. memcpy() must not be used: it may issue unaligned accesses, which
 * fault on device memory.
 */
static void copy_u64(const uint8_t *frame)
{
    const uint64_t *src = (const uint64_t *)frame;
    volatile uint64_t *dst = fb_map;
    int i;

    for (i = 0; i < SCREEN_SIZE / 8; i += 4) {
        dst[i + 0] = src[i + 0];
        dst[i + 1] = src[i + 1];
        dst[i + 2] = src[i + 2];
        dst[i + 3] = src[i + 3];
    }
}

static void copy_neon(const uint8_t *frame)
{
#ifdef __ARM_NEON
    const uint8_t *s = frame;
    volatile uint8_t *d = fb_map;
    int n = SCREEN_SIZE;

    /* 32-byte aligned loads/stores of four doubleword registers */
    __asm__ volatile(
        "1:                                 \n"
        "   vld1.64 {d0-d3}, [%[s] :128]!   \n"
        "   vst1.64 {d0-d3}, [%[d] :128]!   \n"
        "   subs    %[n], %[n], #32         \n"
        "   bgt     1b                      \n"
        : [s] "+r" (s), [d] "+r" (d), [n] "+r" (n)
        :
        : "d0", "d1", "d2", "d3", "cc", "memory");
#else
    copy_u64(frame);
#endif
}

int fpga_setup_dma(uint8_t *const bufs[], int n)
{
    int i;

    if (fbdma_probe(&dma, (volatile uint32_t *)status_map) < 0) {
        printf("frame DMA: not in this FPGA bitstream, the CPU copies frames\n");
        return -1;
    }
    for (i = 0; i < n; i++) {
        if (fbdma_register(&dma, bufs[i], SCREEN_SIZE) < 0) {
            printf("frame DMA: cannot use buffer %d, the CPU copies frames\n", i);
            return -1;
        }
    }
    use_dma = 1;
    return 0;
}

void fpga_present(const uint8_t *frame)
{
    if (use_dma) {
        int i = fbdma_index(&dma, frame);

        if (i >= 0 && fbdma_copy(&dma, i) == 0)
            return;
        fprintf(stderr, "frame DMA failed; the CPU copies frames from now on\n");
        use_dma = 0;
    }
    if (copy_method == 1)
        copy_neon(frame);
    else
        copy_u64(frame);
}

void fpga_pick_copy(const uint8_t *frame)
{
    static const char *const name[2] = { "64-bit stores", "NEON 256-bit" };
    double best[2] = { 1e9, 1e9 };
    int m, r;

    for (m = 0; m < 2; m++) {
        for (r = 0; r < 3; r++) {
            double t0 = now_ms(), dt;

            if (m == 1)
                copy_neon(frame);
            else
                copy_u64(frame);
            dt = now_ms() - t0;
            if (dt < best[m])
                best[m] = dt;
        }
    }
    copy_method = best[1] < best[0] ? 1 : 0;
    if (use_dma) {
        double dbest = 1e9;
        int i = fbdma_index(&dma, frame);

        for (r = 0; r < 3 && i >= 0; r++) {
            double t0 = now_ms(), dt;

            if (fbdma_copy(&dma, i) < 0) {
                use_dma = 0;
                break;
            }
            dt = now_ms() - t0;
            if (dt < dbest)
                dbest = dt;
        }
        if (use_dma) {
            printf("frame copy: %s %.2f ms, %s %.2f ms, FPGA DMA %.2f ms -> using FPGA DMA\n",
                   name[0], best[0], name[1], best[1], dbest);
            return;
        }
    }
    printf("frame copy: %s %.2f ms, %s %.2f ms -> using %s\n",
           name[0], best[0], name[1], best[1], name[copy_method]);
}

#endif

int write_ppm(const char *path, const uint8_t *frame)
{
    FILE *f = fopen(path, "wb");
    int i;

    if (!f) {
        perror(path);
        return -1;
    }
    fprintf(f, "P6\n%d %d\n255\n", SCREEN_W, SCREEN_H);
    for (i = 0; i < SCREEN_SIZE; i++) {
        uint8_t p = frame[i];
        uint8_t r = p >> 5, g = (p >> 2) & 7, b = p & 3;
        uint8_t rgb[3] = {
            (uint8_t)((r << 5) | (r << 2) | (r >> 1)),
            (uint8_t)((g << 5) | (g << 2) | (g >> 1)),
            (uint8_t)(b * 0x55),
        };

        fwrite(rgb, 1, 3, f);
    }
    fclose(f);
    return 0;
}
