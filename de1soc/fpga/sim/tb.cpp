// tb.cpp - span_gpu.v against the game's C model (gpu_sw.c), byte for byte
//
//   ./obj/Vspan_gpu FRAME.cmd FRAME.ppm [frames]
//
// FRAME.cmd is a command buffer saved by `mc_host --dump`, FRAME.ppm the
// frame the C model drew from it. The buffer is scattered over "physical"
// pages through a shuffled page table; command reads and framebuffer writes
// see random stalls and latencies; the display beam moves at the real
// 640x480@60 rate of the 102.38 MHz system clock.
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cstdint>
#include <deque>
#include <vector>
#include "Vspan_gpu.h"
#include "verilated.h"
#include "Vspan_gpu___024root.h"

static const int BUF_SIZE = 31 * 256 * 1024, PAGES = 4096;
static const double CLK_HZ = 102.38e6, LINE_HZ = 25.175e6 / 800;

static Vspan_gpu *top;
static uint64_t cycle;
static std::vector<uint8_t> phys(PAGES * 4096), fb(640 * 480), want(640 * 480);
static uint32_t page_of[PAGES];                 // virtual page -> physical page
static uint32_t rnd = 12345;
static unsigned urand(unsigned n) { rnd = rnd * 1103515245u + 12345u; return (rnd >> 8) % n; }

struct beat { uint64_t at; uint64_t data; };
static std::deque<beat> rq;                     // read data on its way back
static uint64_t rq_free;                        // when the return path is next free

static void tick()
{
    // ---- inputs, decided from the outputs before the edge ----
    // beam: lines since start, bands of 16 lines, 525 lines a scan
    uint64_t line = (uint64_t)(cycle * LINE_HZ / CLK_HZ);
    unsigned in_scan = line % 525, scan = (unsigned)(line / 525);
    top->beam_scan = scan & 0x7FF;
    top->beam_band = in_scan >= 480 ? 30 : in_scan / 16;

    top->cmd_waitrequest = urand(100) < 30;
    if (top->cmd_read && !top->cmd_waitrequest) {
        uint64_t t = cycle + 12 + urand(40);
        uint32_t a = top->cmd_address;
        if (t < rq_free) t = rq_free;
        for (int k = 0; k < 4; k++) {
            uint64_t d;
            memcpy(&d, &phys[a + 8 * k], 8);
            rq.push_back({t, d});
            t += 1 + (urand(100) < 20);
        }
        rq_free = t;
        if (a & 31) { printf("unaligned burst %08x\n", a); exit(1); }
    }
    top->cmd_readdatavalid = 0;
    if (!rq.empty() && rq.front().at <= cycle) {
        top->cmd_readdatavalid = 1;
        top->cmd_readdata = rq.front().data;
        rq.pop_front();
    }

    top->fb_waitrequest = urand(100) < 20;
    if (top->fb_write && !top->fb_waitrequest) {
        uint32_t a = top->fb_address;
        if (a + 8 > fb.size() || (a & 7)) { printf("bad fb write %x\n", a); exit(1); }
        uint64_t d = top->fb_writedata;
        memcpy(&fb[a], &d, 8);
    }

    top->clk = 0; top->eval();
    top->clk = 1; top->eval();
    cycle++;
    top->csr_write = 0;
    top->csr_read = 0;
}

static void csr_wr(int a, uint32_t v)
{
    top->csr_address = a;
    top->csr_writedata = v;
    top->csr_write = 1;
    tick();
}

static uint32_t csr_rd(int a)
{
    top->csr_address = a;
    top->csr_read = 1;
    tick();
    while (!top->csr_readdatavalid) tick();
    return top->csr_readdata;
}

static int load_ppm(const char *path)
{
    FILE *f = fopen(path, "rb");
    int w, h, m;
    if (!f || fscanf(f, "P6 %d %d %d", &w, &h, &m) != 3 || w != 640 || h != 480) return -1;
    fgetc(f);
    for (int i = 0; i < 640 * 480; i++) {
        int r = fgetc(f), g = fgetc(f), b = fgetc(f);
        want[i] = (uint8_t)((r >> 5) << 5 | (g >> 5) << 2 | (b >> 6));
    }
    fclose(f);
    return 0;
}

int main(int argc, char **argv)
{
    Verilated::commandArgs(argc, argv);
    if (argc < 3) { fprintf(stderr, "usage: %s FRAME.cmd FRAME.ppm [frames]\n", argv[0]); return 2; }
    int frames = argc > 3 ? atoi(argv[3]) : 2;

    std::vector<uint8_t> buf(BUF_SIZE);
    FILE *f = fopen(argv[1], "rb");
    if (!f || fread(buf.data(), 1, BUF_SIZE, f) != (size_t)BUF_SIZE) { perror(argv[1]); return 1; }
    fclose(f);
    if (load_ppm(argv[2])) { fprintf(stderr, "%s: not a 640x480 PPM\n", argv[2]); return 1; }

    // scatter buffer 0's pages (virtual pages 0..2047) over physical memory
    for (int v = 0; v < PAGES; v++) page_of[v] = v;
    for (int v = PAGES - 1; v > 0; v--) { int j = urand(v + 1); std::swap(page_of[v], page_of[j]); }
    for (int v = 0; v < BUF_SIZE / 4096; v++)
        memcpy(&phys[page_of[v] * 4096], &buf[v * 4096], 4096);
    memset(fb.data(), 0x55, fb.size());

    top = new Vspan_gpu;
    top->reset = 1;
    for (int i = 0; i < 5; i++) tick();
    top->reset = 0;
    for (int i = 0; i < 5; i++) tick();

    if (csr_rd(0) != 0x47505531) { printf("bad ID\n"); return 1; }
    csr_wr(3, 0);
    for (int v = 0; v < PAGES; v++) csr_wr(4, page_of[v]);
    csr_wr(1, 1u << 31);
    for (int i = 0; i < 5; i++) tick();

    int fails = 0;
    for (int n = 1; n <= frames; n++) {
        uint64_t t0 = cycle;
        csr_wr(1, 1);                           // frame in buffer 0
        while ((csr_rd(2) & 0xFFFF) != (uint32_t)n) {
            for (int i = 0; i < 1000; i++) tick();
            if (cycle - t0 > 40000000ull) {
                Vspan_gpu___024root *r = top->rootp;
                printf("frame %d: timeout: d_state %d f_state %d region %d band %d fifo %d inflight %d "
                       "st_valid %d p_hold %d a_valid %d off %x\n", n,
                       r->span_gpu__DOT__d_state, r->span_gpu__DOT__f_state, r->span_gpu__DOT__region,
                       r->span_gpu__DOT__band, r->span_gpu__DOT__fifo_count, r->span_gpu__DOT__inflight,
                       r->span_gpu__DOT__st_valid, r->span_gpu__DOT__p_hold, r->span_gpu__DOT__a_valid,
                       r->span_gpu__DOT__f_off);
                return 1;
            }
        }
        int bad = 0, first = -1;
        for (int i = 0; i < 640 * 480; i++)
            if (fb[i] != want[i]) { if (first < 0) first = i; bad++; }
        printf("frame %d: %s, %d pixels differ", n, bad ? "MISMATCH" : "identical", bad);
        if (bad) printf(" (first at x=%d y=%d: got %02x want %02x)", first % 640, first / 640,
                        fb[first], want[first]);
        printf(" | frame %.2f ms, drawing %.2f ms, late bands %u\n",
               csr_rd(6) / (CLK_HZ / 1e3), csr_rd(7) / (CLK_HZ / 1e3), csr_rd(8));
        fails += bad != 0;
        memset(fb.data(), 0x55, fb.size());
    }
    if (argc > 4) {
        FILE *o = fopen(argv[4], "wb");
        fprintf(o, "P5\n640 480\n255\n");
        fwrite(fb.data(), 1, fb.size(), o);
        fclose(o);
    }
    delete top;
    return fails ? 1 : 0;
}
