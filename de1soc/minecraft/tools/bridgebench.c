/*
 * bridgebench.c - how fast can the ARM move a frame to the FPGA?
 *
 * Times full 640x480 (307200-byte) copies from a normal (cached) buffer in
 * DDR to the framebuffer behind the HPS-to-FPGA bridge, written in every
 * way the CPU can, plus reads back over the bridge, register latency on the
 * lightweight bridge, and two cores copying halves at the same time.
 *
 * Build: see the Makefile (make bridgebench). Run as root on the board.
 */

#define _GNU_SOURCE
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <fcntl.h>
#include <setjmp.h>
#include <signal.h>
#include <time.h>
#include <unistd.h>
#include <pthread.h>
#include <sched.h>
#include <sys/mman.h>
#include "../fbdma.h"

#define FB_BASE     0xC0000000u
#define FB_SIZE     307200u
#define LW_BASE     0xFF200000u
#define REPEATS     8

static volatile uint8_t *fb;
static volatile uint32_t *lw;
static uint8_t *src;

static double now_us(void)
{
    struct timespec ts;

    clock_gettime(CLOCK_MONOTONIC, &ts);
    return ts.tv_sec * 1e6 + ts.tv_nsec / 1e3;
}

/* ---- copy methods: dst/src 64-byte aligned, n a multiple of 64 ----------- */

static void copy_str32(volatile void *d, const void *s, size_t n)
{
    volatile uint32_t *dp = d;
    const uint32_t *sp = s;
    size_t i;

    for (i = 0; i < n / 4; i += 4) {
        dp[i] = sp[i]; dp[i + 1] = sp[i + 1]; dp[i + 2] = sp[i + 2]; dp[i + 3] = sp[i + 3];
    }
}

static void copy_strd64(volatile void *d, const void *s, size_t n)
{
    volatile uint64_t *dp = d;
    const uint64_t *sp = s;
    size_t i;

    for (i = 0; i < n / 8; i += 4) {
        dp[i] = sp[i]; dp[i + 1] = sp[i + 1]; dp[i + 2] = sp[i + 2]; dp[i + 3] = sp[i + 3];
    }
}

static void copy_stm16(volatile void *d, const void *s, size_t n)
{
    __asm__ volatile(
        "1: ldmia %[s]!, {r3, r4, r5, r6}  \n"
        "   stmia %[d]!, {r3, r4, r5, r6}  \n"
        "   subs  %[n], %[n], #16          \n"
        "   bgt   1b                       \n"
        : [s] "+r" (s), [d] "+r" (d), [n] "+r" (n)
        :
        : "r3", "r4", "r5", "r6", "cc", "memory");
}

static void copy_stm32(volatile void *d, const void *s, size_t n)
{
    __asm__ volatile(
        "1: ldmia %[s]!, {r3, r4, r5, r6, r8, r9, r10, r12} \n"
        "   stmia %[d]!, {r3, r4, r5, r6, r8, r9, r10, r12} \n"
        "   subs  %[n], %[n], #32                           \n"
        "   bgt   1b                                        \n"
        : [s] "+r" (s), [d] "+r" (d), [n] "+r" (n)
        :
        : "r3", "r4", "r5", "r6", "r8", "r9", "r10", "r12", "cc", "memory");
}

static void copy_neon16(volatile void *d, const void *s, size_t n)
{
    __asm__ volatile(
        "1: vld1.64 {d0-d1}, [%[s] :128]!  \n"
        "   vst1.64 {d0-d1}, [%[d] :128]!  \n"
        "   subs    %[n], %[n], #16        \n"
        "   bgt     1b                     \n"
        : [s] "+r" (s), [d] "+r" (d), [n] "+r" (n)
        :
        : "d0", "d1", "cc", "memory");
}

static void copy_neon32(volatile void *d, const void *s, size_t n)
{
    __asm__ volatile(
        "1: vld1.64 {d0-d3}, [%[s] :128]!  \n"
        "   vst1.64 {d0-d3}, [%[d] :128]!  \n"
        "   subs    %[n], %[n], #32        \n"
        "   bgt     1b                     \n"
        : [s] "+r" (s), [d] "+r" (d), [n] "+r" (n)
        :
        : "d0", "d1", "d2", "d3", "cc", "memory");
}

static void copy_neon64(volatile void *d, const void *s, size_t n)
{
    __asm__ volatile(
        "1: vld1.64 {d0-d3}, [%[s] :128]!  \n"
        "   vld1.64 {d4-d7}, [%[s] :128]!  \n"
        "   vst1.64 {d0-d3}, [%[d] :128]!  \n"
        "   vst1.64 {d4-d7}, [%[d] :128]!  \n"
        "   subs    %[n], %[n], #64        \n"
        "   bgt     1b                     \n"
        : [s] "+r" (s), [d] "+r" (d), [n] "+r" (n)
        :
        : "d0", "d1", "d2", "d3", "d4", "d5", "d6", "d7", "cc", "memory");
}

static void copy_memcpy(volatile void *d, const void *s, size_t n)
{
    memcpy((void *)d, s, n);
}

static void read_ldrd64(volatile void *d, const void *s, size_t n)
{
    volatile uint64_t *fp = d;
    uint64_t acc = 0;
    size_t i;

    (void)s;
    for (i = 0; i < n / 8; i++)
        acc ^= fp[i];
    __asm__ volatile("" :: "r" ((uint32_t)acc));
}

/* ---- running a method safely --------------------------------------------- */

static sigjmp_buf bus_jump;

static void on_bus(int sig)
{
    siglongjmp(bus_jump, sig);
}

struct method {
    const char *name;
    void (*fn)(volatile void *, const void *, size_t);
};

/* Best and average time for a full frame, or a negative signal number. */
static int run(const struct method *m, double *best, double *avg)
{
    int r, sig;

    *best = 1e12;
    *avg = 0;
    if ((sig = sigsetjmp(bus_jump, 1)) != 0)
        return -sig;
    for (r = 0; r < REPEATS; r++) {
        double t0 = now_us(), dt;

        m->fn(fb, src, FB_SIZE);
        dt = now_us() - t0;
        if (dt < *best)
            *best = dt;
        *avg += dt / REPEATS;
    }
    return 0;
}

/* ---- two cores at once --------------------------------------------------- */

struct half {
    int cpu;
    size_t off;
    pthread_barrier_t *go;
    double us;
};

static void *half_copy(void *arg)
{
    struct half *h = arg;
    cpu_set_t set;
    double t0;

    CPU_ZERO(&set);
    CPU_SET(h->cpu, &set);
    pthread_setaffinity_np(pthread_self(), sizeof(set), &set);
    pthread_barrier_wait(h->go);
    t0 = now_us();
    copy_strd64(fb + h->off, src + h->off, FB_SIZE / 2);
    h->us = now_us() - t0;
    return NULL;
}

static double two_cores(void)
{
    pthread_barrier_t go;
    pthread_t t[2];
    struct half h[2];
    double best = 1e12, t0, dt;
    int r, i;

    for (r = 0; r < REPEATS; r++) {
        pthread_barrier_init(&go, NULL, 3);
        for (i = 0; i < 2; i++) {
            h[i].cpu = i;
            h[i].off = i * (FB_SIZE / 2);
            h[i].go = &go;
            pthread_create(&t[i], NULL, half_copy, &h[i]);
        }
        t0 = now_us();
        pthread_barrier_wait(&go);
        for (i = 0; i < 2; i++)
            pthread_join(t[i], NULL);
        dt = now_us() - t0;
        if (dt < best)
            best = dt;
        pthread_barrier_destroy(&go);
    }
    return best;
}

int main(void)
{
    static const struct method methods[] = {
        { "32-bit STR (x4 unrolled)",        copy_str32  },
        { "64-bit STRD (x4 unrolled) [game]", copy_strd64 },
        { "LDM/STM 4 regs (16 B)",           copy_stm16  },
        { "LDM/STM 8 regs (32 B)",           copy_stm32  },
        { "NEON VST1 2 regs (16 B)",         copy_neon16 },
        { "NEON VST1 4 regs (32 B)",         copy_neon32 },
        { "NEON VST1 2x4 regs (64 B)",       copy_neon64 },
        { "glibc memcpy",                    copy_memcpy },
        { "READ: 64-bit LDRD from bridge",   read_ldrd64 },
    };
    struct sigaction sa;
    int fd, i, rc;
    double best, avg, t0;
    uint32_t sink = 0;

    fd = open("/dev/mem", O_RDWR | O_SYNC);
    if (fd < 0) {
        perror("/dev/mem (run as root)");
        return 1;
    }
    fb = mmap(NULL, FB_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, fd, FB_BASE);
    lw = mmap(NULL, 4096, PROT_READ | PROT_WRITE, MAP_SHARED, fd, LW_BASE);
    if (fb == MAP_FAILED || lw == MAP_FAILED) {
        perror("mmap");
        return 1;
    }
    if (posix_memalign((void **)&src, 4096, FB_SIZE))
        return 1;
    for (i = 0; i < (int)FB_SIZE; i++)
        src[i] = (uint8_t)((i / 640 + i % 640) >> 2);   /* a gradient, visible on screen */

    memset(&sa, 0, sizeof(sa));
    sa.sa_handler = on_bus;
    sigaction(SIGBUS, &sa, NULL);
    sigaction(SIGSEGV, &sa, NULL);

    printf("bridge benchmark: %u-byte frame, best/avg of %d\n", FB_SIZE, REPEATS);
    printf("%-36s %9s %9s %9s\n", "method", "best ms", "avg ms", "MB/s");
    for (i = 0; i < (int)(sizeof(methods) / sizeof(methods[0])); i++) {
        rc = run(&methods[i], &best, &avg);
        if (rc < 0)
            printf("%-36s   faulted (signal %d)\n", methods[i].name, -rc);
        else
            printf("%-36s %9.2f %9.2f %9.1f\n", methods[i].name, best / 1e3, avg / 1e3,
                   FB_SIZE / best);
        fflush(stdout);
    }

    best = two_cores();
    printf("%-36s %9.2f %9s %9.1f\n", "2 cores, 64-bit STRD, half each", best / 1e3, "-",
           FB_SIZE / best);

    t0 = now_us();
    for (i = 0; i < 10000; i++)
        sink ^= lw[0];
    printf("lightweight bridge register read: %.0f ns each\n", (now_us() - t0) * 1e3 / 10000);
    t0 = now_us();
    for (i = 0; i < 10000; i++)
        lw[1] = (uint32_t)i;        /* PIO direction register, harmless */
    printf("lightweight bridge register write: %.0f ns each\n", (now_us() - t0) * 1e3 / 10000);
    (void)sink;

    /* ---- the FPGA's own DMA ---------------------------------------------- */
    {
        static struct fbdma d;
        int idx, bad = 0, r, k;
        double dbest = 1e12, davg = 0;

        if (fbdma_probe(&d, lw) < 0) {
            printf("frame DMA: not in this bitstream\n");
            return 0;
        }
        idx = fbdma_register(&d, src, FB_SIZE);
        if (idx < 0) {
            printf("frame DMA: cannot map the buffer\n");
            return 0;
        }
        printf("frame DMA: %d descriptor(s) for the frame\n", d.nruns[idx]);
        for (r = 0; r < REPEATS; r++) {
            double t = now_us(), dt;

            if (fbdma_copy(&d, idx) < 0) {
                printf("frame DMA: timed out\n");
                return 1;
            }
            dt = now_us() - t;
            if (dt < dbest) dbest = dt;
            davg += dt / REPEATS;
        }
        printf("%-36s %9.2f %9.2f %9.1f\n", "FPGA DMA (DDR -> framebuffer)", dbest / 1e3,
               davg / 1e3, FB_SIZE / dbest);

        /* correct data, and coherent: change the frame in the CPU's cache and
         * copy it at once, without any cache flush */
        for (k = 0; k < 3; k++) {
            int j;

            for (j = 0; j < (int)FB_SIZE; j++)
                src[j] = (uint8_t)(j * 7 + k * 31);
            if (fbdma_copy(&d, idx) < 0)
                return 1;
            for (j = 0; j < (int)FB_SIZE; j += 4)
                if (*(volatile uint32_t *)(fb + j) != *(uint32_t *)(src + j))
                    bad++;
        }
        printf("frame DMA check: %s (%d mismatched words over 3 fresh frames)\n",
               bad ? "FAILED - stale or wrong data" : "OK, coherent", bad);
    }
    return 0;
}
