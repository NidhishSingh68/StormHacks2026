/*
 * fpga.h - access to the FPGA framebuffer and status register
 *
 *   0xC0000000  framebuffer, 640 x 480 RGB332 (HPS-to-FPGA bridge)
 *   0xFF200000  status: [31] in vblank, [30:0] frame counter (LW bridge)
 *
 * Built with -DHOST, this is replaced by an in-memory framebuffer and a
 * simulated 60 Hz frame counter, so the game can run and take screenshots
 * on a PC.
 */
#ifndef FPGA_H
#define FPGA_H

#include <stdint.h>

#define FPGA_VBLANK     0x80000000u
#define FPGA_FRAMES     0x7FFFFFFFu

int      fpga_open(void);
void     fpga_close(void);
uint32_t fpga_status(void);

/*
 * Let the FPGA's DMA engine copy these frame buffers (page aligned,
 * SCREEN_SIZE bytes each), if the bitstream has one. Returns 0 if it will.
 */
int      fpga_setup_dma(uint8_t *const bufs[], int n);

/*
 * The span GPU (fpga/span_gpu.v), if the bitstream has one: it draws frames
 * from command lists (gpu.h) straight into the framebuffer.
 *
 * fpga_gpu_open() locks the n (<= 2) command buffers, each GPU_BUF_SIZE
 * bytes and page aligned, and loads their physical pages into the GPU's
 * page table. Returns 0 if the GPU will draw. fpga_gpu_kick() queues the
 * frame in buffer b, first waiting while another frame is still queued;
 * fpga_gpu_done() counts finished frames (mod 2^16). The buffer of a
 * queued frame must not be touched until it is done.
 */
int      fpga_gpu_open(uint64_t *const bufs[], int n);
void     fpga_gpu_kick(int b);
uint32_t fpga_gpu_done(void);
void     fpga_gpu_close(void);

/* Last frame: ms from start to end, ms spent drawing (not waiting for the
 * display), bands copied out late (torn) since the last call. */
void     fpga_gpu_stats(double *frame_ms, double *draw_ms, unsigned *late);

/* Copy a full frame (SCREEN_SIZE bytes, 64-byte aligned) to the FPGA. */
void     fpga_present(const uint8_t *frame);

/* Time the available copy methods and keep the fastest. */
void     fpga_pick_copy(const uint8_t *frame);

/* Expand an RGB332 frame the way the VGA DAC does and save it as PPM. */
int      write_ppm(const char *path, const uint8_t *frame);

#endif
