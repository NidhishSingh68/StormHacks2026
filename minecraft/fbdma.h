/*
 * fbdma.h - copy frames to the FPGA with its DMA engine (fb_dma, an mSGDMA)
 *
 * Instead of the CPU storing every pixel across the bridge (each store is a
 * separate ~230 ns bus transaction), the FPGA reads the frame straight from
 * DDR in bursts. The CPU only queues one descriptor per physically
 * contiguous run of the frame buffer and waits.
 *
 * Register map (lightweight bridge, 0xFF200000 + ...):
 *   0x100  CSR:        +0 status, +4 control
 *   0x200  descriptor: +0 read address (DDR physical), +4 write address
 *                      (framebuffer offset), +8 length, +12 control (GO)
 *
 * The read side goes through the ARM's coherency port (see
 * fpga/acp_read_adapter.v), so frames drawn into normal cached memory are
 * read correctly without flushing caches.
 *
 * Header-only so the game and the benchmark share it.
 */
#ifndef FBDMA_H
#define FBDMA_H

#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <fcntl.h>
#include <time.h>
#include <unistd.h>
#include <sys/mman.h>

#define FBDMA_CSR       (0x100 / 4)
#define FBDMA_DESC      (0x200 / 4)
#define FBDMA_MAX_BUFS  4
#define FBDMA_MAX_RUNS  128
#define FBDMA_RUN_MAX   32768u          /* bytes per descriptor */

#define DMA_ST_BUSY         (1u << 0)
#define DMA_ST_DESC_EMPTY   (1u << 1)
#define DMA_ST_RESETTING    (1u << 6)
#define DMA_CTL_RESET       (1u << 1)
#define DMA_CTL_IRQ_MASK    (1u << 4)   /* harmless: the IRQ is not wired */
#define DMA_DESC_GO         (1u << 31)

struct fbdma_run {
    uint32_t phys, off, len;
};

struct fbdma {
    volatile uint32_t *lw;              /* lightweight bridge page */
    int nbufs;
    const uint8_t *buf[FBDMA_MAX_BUFS];
    int nruns[FBDMA_MAX_BUFS];
    struct fbdma_run runs[FBDMA_MAX_BUFS][FBDMA_MAX_RUNS];
};

static inline double fbdma_now_ms(void)
{
    struct timespec ts;

    clock_gettime(CLOCK_MONOTONIC, &ts);
    return ts.tv_sec * 1e3 + ts.tv_nsec / 1e6;
}

/*
 * Is the DMA in this bitstream? Its control register reads back what was
 * written; on a bitstream without it the address aliases the read-only
 * frame counter. Also resets the dispatcher to a clean state.
 */
static inline int fbdma_probe(struct fbdma *d, volatile uint32_t *lw)
{
    volatile uint32_t *csr = lw + FBDMA_CSR;
    double t0;

    memset(d, 0, sizeof(*d));
    d->lw = lw;
    csr[1] = DMA_CTL_IRQ_MASK;
    if (csr[1] != DMA_CTL_IRQ_MASK)
        return -1;
    csr[1] = 0;
    if (csr[1] != 0)
        return -1;

    csr[1] = DMA_CTL_RESET;
    for (t0 = fbdma_now_ms(); csr[0] & DMA_ST_RESETTING; )
        if (fbdma_now_ms() - t0 > 100)
            return -1;
    csr[1] = 0;
    return 0;
}

/*
 * Make buf (size bytes, page aligned) copyable by the DMA: lock it in RAM
 * and record its physical pages (needs root for /proc/self/pagemap).
 * Returns its index for fbdma_copy, or -1.
 */
static inline int fbdma_register(struct fbdma *d, const uint8_t *buf, size_t size)
{
    long page = sysconf(_SC_PAGESIZE);
    int fd, i = d->nbufs, n = 0;
    size_t off;

    if (i >= FBDMA_MAX_BUFS || ((uintptr_t)buf % page) || (size % page))
        return -1;
    if (mlock(buf, size) < 0) {
        perror("fbdma: mlock");
        return -1;
    }
    fd = open("/proc/self/pagemap", O_RDONLY);
    if (fd < 0) {
        perror("fbdma: pagemap");
        return -1;
    }
    for (off = 0; off < size; off += page) {
        uint64_t e;
        uint32_t phys;
        uintptr_t va = (uintptr_t)buf + off;

        if (pread(fd, &e, 8, (off_t)(va / page) * 8) != 8 || !(e >> 63) ||
            !(e & ((1ull << 55) - 1))) {
            fprintf(stderr, "fbdma: no physical address for the frame buffer\n");
            close(fd);
            return -1;
        }
        phys = (uint32_t)((e & ((1ull << 55) - 1)) * page);
        if (phys >= 0x40000000u) {              /* outside the ACP window */
            close(fd);
            return -1;
        }
        /* extend the previous run if this page follows it physically */
        if (n && d->runs[i][n - 1].phys + d->runs[i][n - 1].len == phys &&
            d->runs[i][n - 1].len + page <= FBDMA_RUN_MAX) {
            d->runs[i][n - 1].len += page;
        } else {
            if (n == FBDMA_MAX_RUNS) {
                close(fd);
                return -1;
            }
            d->runs[i][n].phys = phys;
            d->runs[i][n].off = (uint32_t)off;
            d->runs[i][n].len = (uint32_t)page;
            n++;
        }
    }
    close(fd);
    d->buf[i] = buf;
    d->nruns[i] = n;
    d->nbufs++;
    return i;
}

static inline int fbdma_index(const struct fbdma *d, const uint8_t *buf)
{
    int i;

    for (i = 0; i < d->nbufs; i++)
        if (d->buf[i] == buf)
            return i;
    return -1;
}

/* Copy registered buffer i to the framebuffer; 0 when done, -1 on a hang. */
static inline int fbdma_copy(struct fbdma *d, int i)
{
    volatile uint32_t *csr = d->lw + FBDMA_CSR, *desc = d->lw + FBDMA_DESC;
    double t0;
    int k;

    __sync_synchronize();               /* the frame is fully written first */
    for (k = 0; k < d->nruns[i]; k++) {
        const struct fbdma_run *r = &d->runs[i][k];

        desc[0] = r->phys;
        desc[1] = r->off;
        desc[2] = r->len;
        desc[3] = DMA_DESC_GO;          /* writing the control word queues it */
    }
    for (t0 = fbdma_now_ms(); ; ) {
        uint32_t st = csr[0];

        if (!(st & DMA_ST_BUSY) && (st & DMA_ST_DESC_EMPTY))
            return 0;
        if (fbdma_now_ms() - t0 > 50) {
            csr[1] = DMA_CTL_RESET;     /* stuck: clear it and give up */
            return -1;
        }
    }
}

#endif
