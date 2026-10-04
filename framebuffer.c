/*
 * framebuffer.c - write frames from the DE1-SoC HPS into the FPGA framebuffer
 *
 * The FPGA design (fpga/) holds a 640x480, 8-bit-per-pixel framebuffer in
 * on-chip RAM behind the HPS-to-FPGA (heavy) bridge, and scans it out to the
 * VGA connector continuously:
 *
 *   0xC0000000  framebuffer, 307200 bytes, row-major, 640 bytes per line
 *               pixel format RGB332:  [7:5] red  [4:2] green  [1:0] blue
 *   0xFF200000  status (lightweight bridge), read-only
 *               [31]   1 while the display is in vertical blanking
 *               [30:0] frame counter, +1 at the start of every vblank
 *
 * The game renders into an ordinary buffer in DDR ("back buffer") and calls
 * fb_present() once per frame. fb_present() waits for vblank and then copies
 * the whole frame across the bridge, top line first. The copy only has to
 * stay ahead of the beam (~20 MB/s) to avoid tearing; the time it takes is
 * printed by the demo so you can check that on the board.
 *
 * The demo below draws colour bars and an orange square at 60 fps, which
 * you move with the keyboard:
 *   arrows / WASD   move while held (hold Shift to move faster)
 *   Q or Esc        quit           (Ctrl-C works too)
 *
 * Keys are read from a USB keyboard plugged into the board (the first
 * /dev/input/event* with letter and arrow keys; plugging one in while the
 * demo runs is fine) and, when started from a terminal, from that terminal
 * as well (e.g. the serial console). A terminal has no key-release events,
 * so there each key press moves the square one step.
 *
 * Build:
 *   arm-linux-gnueabihf-gcc -O2 -mcpu=cortex-a9 -mfpu=neon -mfloat-abi=hard \
 *       -Wall -static framebuffer.c -o framebuffer
 *
 * Run on the board, after the FPGA has been configured with this design
 * (fpga/output_files/soc_system.rbf as the SD card's soc_system.rbf, or
 * DE1_SoC_top.sof over JTAG):
 *   sudo ./framebuffer            interactive demo
 *   sudo ./framebuffer -p         write one static test pattern and exit
 *
 * The bridges must be out of reset; accessing one that isn't hangs the CPU.
 * On the Terasic Ubuntu image U-Boot enables them at boot
 * ("run bridge_enable_handoff" in u-boot.scr).
 */

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <errno.h>
#include <fcntl.h>
#include <signal.h>
#include <time.h>
#include <unistd.h>
#include <dirent.h>
#include <termios.h>
#include <sys/mman.h>
#include <sys/ioctl.h>
#include <linux/input.h>

/* ---- memory map ---------------------------------------------------------- */

#define FB_BASE         0xC0000000u     /* HPS-to-FPGA bridge */
#define STATUS_BASE     0xFF200000u     /* lightweight HPS-to-FPGA bridge */

#define FB_WIDTH        640
#define FB_HEIGHT       480
#define FB_SIZE         (FB_WIDTH * FB_HEIGHT)

#define STATUS_VBLANK   0x80000000u
#define STATUS_FRAMES   0x7FFFFFFFu

/* Pack 8-bit-per-channel colour into RGB332. */
#define RGB332(r, g, b) ((uint8_t)(((r) & 0xE0) | (((g) >> 3) & 0x1C) | ((b) >> 6)))

/* ---- framebuffer device -------------------------------------------------- */

struct fb {
    int                      mem_fd;
    volatile uint64_t       *pixels;    /* FPGA framebuffer, 64-bit words */
    volatile uint32_t       *status;
    void                    *status_map;
    long                     page;
};

static int fb_open(struct fb *fb)
{
    fb->page = sysconf(_SC_PAGESIZE);
    fb->mem_fd = open("/dev/mem", O_RDWR | O_SYNC);
    if (fb->mem_fd < 0) {
        perror("open /dev/mem (are you root?)");
        return -1;
    }

    fb->pixels = mmap(NULL, FB_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED,
                      fb->mem_fd, FB_BASE);
    if (fb->pixels == MAP_FAILED) {
        perror("mmap framebuffer");
        close(fb->mem_fd);
        return -1;
    }

    fb->status_map = mmap(NULL, fb->page, PROT_READ, MAP_SHARED,
                          fb->mem_fd, STATUS_BASE);
    if (fb->status_map == MAP_FAILED) {
        perror("mmap status");
        munmap((void *)fb->pixels, FB_SIZE);
        close(fb->mem_fd);
        return -1;
    }
    fb->status = fb->status_map;

    return 0;
}

static void fb_close(struct fb *fb)
{
    munmap(fb->status_map, fb->page);
    munmap((void *)fb->pixels, FB_SIZE);
    close(fb->mem_fd);
}

static uint32_t fb_frame_count(const struct fb *fb)
{
    return *fb->status & STATUS_FRAMES;
}

/* Block until the next vertical blanking interval begins. */
static void fb_wait_vblank(const struct fb *fb)
{
    uint32_t start = fb_frame_count(fb);

    while (fb_frame_count(fb) == start)
        ;
}

/*
 * Copy a full frame to the FPGA. Uses aligned 64-bit stores to match the
 * bridge width; memcpy() is avoided because it may issue unaligned or NEON
 * accesses, which fault on device memory.
 */
static void fb_write(struct fb *fb, const uint8_t *frame)
{
    const uint64_t *src = (const uint64_t *)frame;
    volatile uint64_t *dst = fb->pixels;
    int i;

    for (i = 0; i < FB_SIZE / 8; i += 4) {
        dst[i + 0] = src[i + 0];
        dst[i + 1] = src[i + 1];
        dst[i + 2] = src[i + 2];
        dst[i + 3] = src[i + 3];
    }
}

/* Show a frame: wait for vblank, then copy it across. */
static void fb_present(struct fb *fb, const uint8_t *frame)
{
    fb_wait_vblank(fb);
    fb_write(fb, frame);
}

/* Allocate a back buffer suitably aligned for fb_write(). */
static uint8_t *fb_alloc_frame(void)
{
    void *p = NULL;

    if (posix_memalign(&p, 64, FB_SIZE) != 0)
        return NULL;
    memset(p, 0, FB_SIZE);
    return p;
}

/* ---- drawing helpers ----------------------------------------------------- */

static void fill_rect(uint8_t *frame, int x, int y, int w, int h, uint8_t c)
{
    int row;

    if (x < 0) { w += x; x = 0; }
    if (y < 0) { h += y; y = 0; }
    if (x + w > FB_WIDTH)  w = FB_WIDTH - x;
    if (y + h > FB_HEIGHT) h = FB_HEIGHT - y;
    if (w <= 0 || h <= 0)
        return;

    for (row = y; row < y + h; row++)
        memset(frame + row * FB_WIDTH + x, c, w);
}

/* Eight vertical colour bars over a top-to-bottom grey ramp. */
static void draw_test_pattern(uint8_t *frame)
{
    static const uint8_t bars[8] = {
        RGB332(255, 255, 255), RGB332(255, 255, 0), RGB332(0, 255, 255),
        RGB332(0, 255, 0),     RGB332(255, 0, 255), RGB332(255, 0, 0),
        RGB332(0, 0, 255),     RGB332(0, 0, 0),
    };
    int i, y;

    for (i = 0; i < 8; i++)
        fill_rect(frame, i * FB_WIDTH / 8, 0, FB_WIDTH / 8, FB_HEIGHT * 3 / 4,
                  bars[i]);

    for (y = FB_HEIGHT * 3 / 4; y < FB_HEIGHT; y++) {
        for (i = 0; i < FB_WIDTH; i++) {
            uint8_t v = (uint8_t)(i * 255 / (FB_WIDTH - 1));
            frame[y * FB_WIDTH + i] = RGB332(v, v, v);
        }
    }

    /* 1-pixel white border to check nothing is cropped or shifted */
    fill_rect(frame, 0, 0, FB_WIDTH, 1, 0xFF);
    fill_rect(frame, 0, FB_HEIGHT - 1, FB_WIDTH, 1, 0xFF);
    fill_rect(frame, 0, 0, 1, FB_HEIGHT, 0xFF);
    fill_rect(frame, FB_WIDTH - 1, 0, 1, FB_HEIGHT, 0xFF);
}

/* ---- keyboard input ------------------------------------------------------ */

#define BITS_PER_LONG   (8 * sizeof(long))
#define NLONGS(n)       (((n) + BITS_PER_LONG - 1) / BITS_PER_LONG)
#define TEST_BIT(b, a)  (((a)[(b) / BITS_PER_LONG] >> ((b) % BITS_PER_LONG)) & 1)

enum { HELD_UP, HELD_DOWN, HELD_LEFT, HELD_RIGHT, HELD_SHIFT, HELD_COUNT };

struct input {
    int             kbd_fd;             /* evdev keyboard, -1 if none */
    time_t          last_scan;
    int             held[HELD_COUNT];   /* keys currently down (evdev) */

    int             tty;                /* stdin is a terminal in raw mode */
    struct termios  saved_tio;
    int             saved_flags;
    int             step_x, step_y;     /* key presses from the terminal */

    int             quit;
};

static int is_keyboard(int fd)
{
    unsigned long evbits[NLONGS(EV_MAX + 1)];
    unsigned long keybits[NLONGS(KEY_MAX + 1)];

    memset(evbits, 0, sizeof(evbits));
    memset(keybits, 0, sizeof(keybits));

    if (ioctl(fd, EVIOCGBIT(0, sizeof(evbits)), evbits) < 0 ||
        !TEST_BIT(EV_KEY, evbits))
        return 0;
    if (ioctl(fd, EVIOCGBIT(EV_KEY, sizeof(keybits)), keybits) < 0)
        return 0;

    /* A real keyboard has letters and arrows; mice and buttons don't. */
    return TEST_BIT(KEY_A, keybits) && TEST_BIT(KEY_UP, keybits);
}

static int find_keyboard(char *path, size_t len)
{
    DIR *dir = opendir("/dev/input");
    struct dirent *de;
    int fd = -1;

    if (!dir)
        return -1;

    while ((de = readdir(dir)) != NULL) {
        if (strncmp(de->d_name, "event", 5) != 0)
            continue;
        snprintf(path, len, "/dev/input/%s", de->d_name);
        fd = open(path, O_RDONLY | O_NONBLOCK);
        if (fd < 0)
            continue;
        if (is_keyboard(fd))
            break;
        close(fd);
        fd = -1;
    }

    closedir(dir);
    return fd;
}

static void input_open(struct input *in)
{
    memset(in, 0, sizeof(*in));
    in->kbd_fd = -1;

    if (isatty(STDIN_FILENO) && tcgetattr(STDIN_FILENO, &in->saved_tio) == 0) {
        struct termios tio = in->saved_tio;

        /* No line buffering or echo; keep ISIG so Ctrl-C still works. */
        tio.c_lflag &= ~(ICANON | ECHO);
        tio.c_cc[VMIN] = 0;
        tio.c_cc[VTIME] = 0;
        if (tcsetattr(STDIN_FILENO, TCSANOW, &tio) == 0) {
            in->saved_flags = fcntl(STDIN_FILENO, F_GETFL);
            fcntl(STDIN_FILENO, F_SETFL, in->saved_flags | O_NONBLOCK);
            in->tty = 1;
        }
    }
}

static void input_close(struct input *in)
{
    if (in->kbd_fd >= 0) {
        ioctl(in->kbd_fd, EVIOCGRAB, 0);
        close(in->kbd_fd);
    }
    if (in->tty) {
        fcntl(STDIN_FILENO, F_SETFL, in->saved_flags);
        tcsetattr(STDIN_FILENO, TCSANOW, &in->saved_tio);
    }
}

static void poll_keyboard(struct input *in)
{
    struct input_event ev[32];
    ssize_t n;
    int i;

    /* No keyboard yet (or it was unplugged): look again once a second. */
    if (in->kbd_fd < 0) {
        char path[300];
        time_t now = time(NULL);

        if (now == in->last_scan)
            return;
        in->last_scan = now;

        in->kbd_fd = find_keyboard(path, sizeof(path));
        if (in->kbd_fd < 0)
            return;
        /* Grab so key presses don't also land on the console. */
        ioctl(in->kbd_fd, EVIOCGRAB, 1);
        printf("keyboard: %s\n", path);
        fflush(stdout);
    }

    while ((n = read(in->kbd_fd, ev, sizeof(ev))) > 0) {
        for (i = 0; i < n / (ssize_t)sizeof(ev[0]); i++) {
            /* value: 1 = press, 2 = autorepeat, 0 = release */
            int down = (ev[i].value != 0);

            if (ev[i].type != EV_KEY)
                continue;

            switch (ev[i].code) {
            case KEY_UP:    case KEY_W: in->held[HELD_UP]    = down; break;
            case KEY_DOWN:  case KEY_S: in->held[HELD_DOWN]  = down; break;
            case KEY_LEFT:  case KEY_A: in->held[HELD_LEFT]  = down; break;
            case KEY_RIGHT: case KEY_D: in->held[HELD_RIGHT] = down; break;
            case KEY_LEFTSHIFT: case KEY_RIGHTSHIFT:
                in->held[HELD_SHIFT] = down;
                break;
            case KEY_ESC: case KEY_Q:
                if (down)
                    in->quit = 1;
                break;
            default:
                break;
            }
        }
    }

    if (n < 0 && errno != EAGAIN && errno != EWOULDBLOCK) {
        printf("keyboard disconnected\n");
        fflush(stdout);
        close(in->kbd_fd);
        in->kbd_fd = -1;
        memset(in->held, 0, sizeof(in->held));
    }
}

static void poll_terminal(struct input *in)
{
    unsigned char buf[64];
    ssize_t n, i;

    if (!in->tty)
        return;

    while ((n = read(STDIN_FILENO, buf, sizeof(buf))) > 0) {
        for (i = 0; i < n; i++) {
            unsigned char c = buf[i];

            /* Arrow keys arrive as ESC [ A..D (or ESC O A..D). */
            if (c == 0x1B && i + 2 < n && (buf[i + 1] == '[' || buf[i + 1] == 'O')) {
                switch (buf[i + 2]) {
                case 'A': in->step_y--; break;
                case 'B': in->step_y++; break;
                case 'C': in->step_x++; break;
                case 'D': in->step_x--; break;
                default: break;
                }
                i += 2;
                continue;
            }

            switch (c) {
            case 'w': in->step_y -= 1; break;
            case 's': in->step_y += 1; break;
            case 'a': in->step_x -= 1; break;
            case 'd': in->step_x += 1; break;
            case 'W': in->step_y -= 4; break;
            case 'S': in->step_y += 4; break;
            case 'A': in->step_x -= 4; break;
            case 'D': in->step_x += 4; break;
            case 'q': case 'Q': case 0x1B: in->quit = 1; break;
            default: break;
            }
        }
    }
}

static void input_poll(struct input *in)
{
    poll_keyboard(in);
    poll_terminal(in);
}

/* -------------------------------------------------------------------------- */

#define SQUARE_SIZE     64
#define SPEED           4       /* pixels per frame while a key is held */
#define SPEED_FAST      12      /* ... with Shift */
#define TTY_STEP        16      /* pixels per key press from a terminal */

static volatile sig_atomic_t running = 1;

static void on_signal(int sig)
{
    (void)sig;
    running = 0;
}

static double now_us(void)
{
    struct timespec ts;

    clock_gettime(CLOCK_MONOTONIC, &ts);
    return ts.tv_sec * 1e6 + ts.tv_nsec / 1e3;
}

int main(int argc, char **argv)
{
    struct fb fb;
    struct input in;
    uint8_t *background, *frame;
    int pattern_only = (argc > 1 && strcmp(argv[1], "-p") == 0);
    int x = (FB_WIDTH - SQUARE_SIZE) / 2;
    int y = (FB_HEIGHT - SQUARE_SIZE) / 2;
    double copy_us = 0.0, worst_us = 0.0;
    unsigned long frames = 0;
    uint32_t first_count, missed;

    if (fb_open(&fb) < 0)
        return 1;

    background = fb_alloc_frame();
    frame = fb_alloc_frame();
    if (!background || !frame) {
        fprintf(stderr, "out of memory\n");
        fb_close(&fb);
        return 1;
    }

    draw_test_pattern(background);

    if (pattern_only) {
        fb_present(&fb, background);
        printf("test pattern written (frame %u)\n", fb_frame_count(&fb));
        goto out;
    }

    signal(SIGINT, on_signal);
    signal(SIGTERM, on_signal);

    input_open(&in);
    printf("arrows/WASD move (Shift = faster), Q or Esc quits\n");
    if (!in.tty)
        printf("waiting for a USB keyboard...\n");
    fflush(stdout);

    first_count = fb_frame_count(&fb);

    while (running && !in.quit) {
        double t0, dt;
        int speed;

        /* move the square */
        input_poll(&in);
        speed = in.held[HELD_SHIFT] ? SPEED_FAST : SPEED;
        x += speed * (in.held[HELD_RIGHT] - in.held[HELD_LEFT]);
        y += speed * (in.held[HELD_DOWN]  - in.held[HELD_UP]);
        x += TTY_STEP * in.step_x;
        y += TTY_STEP * in.step_y;
        in.step_x = in.step_y = 0;

        if (x < 0) x = 0;
        if (y < 0) y = 0;
        if (x > FB_WIDTH - SQUARE_SIZE)  x = FB_WIDTH - SQUARE_SIZE;
        if (y > FB_HEIGHT - SQUARE_SIZE) y = FB_HEIGHT - SQUARE_SIZE;

        /* render into the back buffer */
        memcpy(frame, background, FB_SIZE);
        fill_rect(frame, x, y, SQUARE_SIZE, SQUARE_SIZE, RGB332(255, 128, 0));
        fill_rect(frame, x + 8, y + 8, SQUARE_SIZE - 16, SQUARE_SIZE - 16,
                  RGB332(0, 0, 0));

        /* show it */
        fb_wait_vblank(&fb);
        t0 = now_us();
        fb_write(&fb, frame);
        dt = now_us() - t0;

        copy_us += dt;
        if (dt > worst_us)
            worst_us = dt;
        frames++;
    }

    input_close(&in);

    if (frames == 0)
        goto out;
    missed = fb_frame_count(&fb) - first_count - frames;
    printf("\n%lu frames, copy avg %.0f us / worst %.0f us, %u vblanks missed\n",
           frames, copy_us / frames, worst_us, missed);
    printf("(a frame lasts 16683 us; the beam needs ~15250 us to scan it)\n");

out:
    free(frame);
    free(background);
    fb_close(&fb);
    return 0;
}
