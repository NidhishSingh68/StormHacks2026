/*
 * keypos.c - keyboard-driven position writer for the DE1-SoC (Cyclone V HPS)
 *
 * Reads a keyboard attached to the board and writes an (X, Y) position into
 * two registers in the FPGA fabric, reached through the lightweight
 * HPS-to-FPGA bridge (mapped via /dev/mem).
 *
 * Keys:
 *   arrows / WASD   move by STEP_SMALL (hold Shift for STEP_LARGE)
 *   R or Home       recentre
 *   Q or Esc        quit         (Esc only in evdev mode; use Q over a tty)
 *
 * Input sources (picked in this order):
 *   1. evdev device given on the command line:  ./keypos /dev/input/event0
 *   2. first /dev/input/event* that looks like a keyboard (USB keyboard
 *      plugged into the board's USB host port)
 *   3. the controlling terminal in raw mode (keys typed over SSH / UART)
 *
 * Fabric side (Platform Designer), attached to h2f_lw_axi_master:
 *   pos_x : 32-bit output PIO at LW offset POS_X_OFFSET
 *   pos_y : 32-bit output PIO at LW offset POS_Y_OFFSET
 * Adjust the offsets below to match your .qsys / .sopcinfo.
 *
 * Build:
 *   arm-linux-gnueabihf-gcc -O2 -mcpu=cortex-a9 -mfpu=neon -mfloat-abi=hard \
 *       -Wall -static keypos.c -o keypos
 *
 * Run on the board (needs root for /dev/mem and /dev/input):
 *   sudo ./keypos
 */

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <errno.h>
#include <fcntl.h>
#include <signal.h>
#include <unistd.h>
#include <dirent.h>
#include <termios.h>
#include <sys/mman.h>
#include <sys/ioctl.h>
#include <linux/input.h>

/* ---- memory map ---------------------------------------------------------- */

#define LW_BRIDGE_BASE  0xFF200000u     /* HPS-to-FPGA lightweight bridge */
#define LW_BRIDGE_SPAN  0x00200000u     /* 2 MB window */

#define POS_X_OFFSET    0x0000u         /* PIO base offsets inside the bridge */
#define POS_Y_OFFSET    0x0010u

/* ---- position limits (640x480 VGA by default) ---------------------------- */

#define POS_X_MAX       639
#define POS_Y_MAX       479
#define STEP_SMALL      1
#define STEP_LARGE      10

/* -------------------------------------------------------------------------- */

#define BITS_PER_LONG   (8 * sizeof(long))
#define NLONGS(n)       (((n) + BITS_PER_LONG - 1) / BITS_PER_LONG)
#define TEST_BIT(b, a)  (((a)[(b) / BITS_PER_LONG] >> ((b) % BITS_PER_LONG)) & 1)

enum action { ACT_NONE, ACT_UP, ACT_DOWN, ACT_LEFT, ACT_RIGHT,
              ACT_CENTRE, ACT_QUIT };

static volatile sig_atomic_t running = 1;
static struct termios saved_tio;
static int tty_raw;

static void on_signal(int sig)
{
    (void)sig;
    running = 0;
}

/* ---- FPGA access --------------------------------------------------------- */

static volatile uint32_t *pos_x_reg;
static volatile uint32_t *pos_y_reg;

static void *map_lw_bridge(int *fd_out)
{
    void *base;
    int fd = open("/dev/mem", O_RDWR | O_SYNC);

    if (fd < 0) {
        perror("open /dev/mem (are you root?)");
        return NULL;
    }

    base = mmap(NULL, LW_BRIDGE_SPAN, PROT_READ | PROT_WRITE, MAP_SHARED,
                fd, LW_BRIDGE_BASE);
    if (base == MAP_FAILED) {
        perror("mmap lightweight bridge");
        close(fd);
        return NULL;
    }

    *fd_out = fd;
    return base;
}

static void write_position(int x, int y)
{
    *pos_x_reg = (uint32_t)x;
    *pos_y_reg = (uint32_t)y;
}

/* ---- evdev keyboard ------------------------------------------------------ */

static int is_keyboard(int fd)
{
    unsigned long evbits[NLONGS(EV_MAX + 1)];
    unsigned long keybits[NLONGS(KEY_MAX + 1)];

    memset(evbits, 0, sizeof(evbits));
    memset(keybits, 0, sizeof(keybits));

    if (ioctl(fd, EVIOCGBIT(0, sizeof(evbits)), evbits) < 0)
        return 0;
    if (!TEST_BIT(EV_KEY, evbits))
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
        fd = open(path, O_RDONLY);
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

static enum action evdev_next(int fd, int *shift)
{
    struct input_event ev;

    while (running) {
        ssize_t n = read(fd, &ev, sizeof(ev));

        if (n < 0) {
            if (errno == EINTR)
                continue;
            perror("read keyboard");
            return ACT_QUIT;
        }
        if (n != sizeof(ev) || ev.type != EV_KEY)
            continue;

        if (ev.code == KEY_LEFTSHIFT || ev.code == KEY_RIGHTSHIFT) {
            *shift = (ev.value != 0);
            continue;
        }

        /* 1 = press, 2 = autorepeat while held, 0 = release */
        if (ev.value == 0)
            continue;

        switch (ev.code) {
        case KEY_UP:    case KEY_W: return ACT_UP;
        case KEY_DOWN:  case KEY_S: return ACT_DOWN;
        case KEY_LEFT:  case KEY_A: return ACT_LEFT;
        case KEY_RIGHT: case KEY_D: return ACT_RIGHT;
        case KEY_HOME:  case KEY_R: return ACT_CENTRE;
        case KEY_ESC:   case KEY_Q: return ACT_QUIT;
        default: break;
        }
    }
    return ACT_QUIT;
}

/* ---- terminal fallback --------------------------------------------------- */

static void tty_restore(void)
{
    if (tty_raw)
        tcsetattr(STDIN_FILENO, TCSANOW, &saved_tio);
    tty_raw = 0;
}

static int tty_setup(void)
{
    struct termios tio;

    if (!isatty(STDIN_FILENO) || tcgetattr(STDIN_FILENO, &saved_tio) < 0)
        return -1;

    tio = saved_tio;
    tio.c_lflag &= ~(ICANON | ECHO);
    tio.c_cc[VMIN] = 1;
    tio.c_cc[VTIME] = 0;
    if (tcsetattr(STDIN_FILENO, TCSANOW, &tio) < 0)
        return -1;

    tty_raw = 1;
    return 0;
}

static int tty_getc(void)
{
    unsigned char c;
    ssize_t n;

    do {
        n = read(STDIN_FILENO, &c, 1);
    } while (n < 0 && errno == EINTR && running);

    return (n == 1) ? c : -1;
}

static enum action tty_next(int *shift)
{
    while (running) {
        int c = tty_getc();

        if (c < 0)
            return ACT_QUIT;

        *shift = (c >= 'A' && c <= 'Z');

        if (c == 0x1b) {
            /* Arrow keys arrive as ESC [ A..D (or ESC O A..D). */
            int c1 = tty_getc();
            int c2 = tty_getc();

            *shift = 0;
            if (c1 != '[' && c1 != 'O')
                continue;
            switch (c2) {
            case 'A': return ACT_UP;
            case 'B': return ACT_DOWN;
            case 'C': return ACT_RIGHT;
            case 'D': return ACT_LEFT;
            case 'H': return ACT_CENTRE;
            default:  continue;
            }
        }

        switch (c) {
        case 'w': case 'W': return ACT_UP;
        case 's': case 'S': return ACT_DOWN;
        case 'a': case 'A': return ACT_LEFT;
        case 'd': case 'D': return ACT_RIGHT;
        case 'r': case 'R': return ACT_CENTRE;
        case 'q': case 'Q': case 0x03: return ACT_QUIT;
        default: break;
        }
    }
    return ACT_QUIT;
}

/* -------------------------------------------------------------------------- */

static int clamp(int v, int lo, int hi)
{
    return v < lo ? lo : (v > hi ? hi : v);
}

int main(int argc, char **argv)
{
    struct sigaction sa;
    char kbd_path[300];
    void *lw_base;
    int mem_fd = -1;
    int kbd_fd = -1;
    int x = POS_X_MAX / 2;
    int y = POS_Y_MAX / 2;
    int shift = 0;

    /* No SA_RESTART: Ctrl-C must break out of a blocking read(). */
    memset(&sa, 0, sizeof(sa));
    sa.sa_handler = on_signal;
    sigaction(SIGINT, &sa, NULL);
    sigaction(SIGTERM, &sa, NULL);

    lw_base = map_lw_bridge(&mem_fd);
    if (!lw_base)
        return 1;
    pos_x_reg = (volatile uint32_t *)((uint8_t *)lw_base + POS_X_OFFSET);
    pos_y_reg = (volatile uint32_t *)((uint8_t *)lw_base + POS_Y_OFFSET);

    if (argc > 1) {
        snprintf(kbd_path, sizeof(kbd_path), "%s", argv[1]);
        kbd_fd = open(kbd_path, O_RDONLY);
        if (kbd_fd < 0) {
            perror(kbd_path);
            goto out;
        }
    } else {
        kbd_fd = find_keyboard(kbd_path, sizeof(kbd_path));
    }

    if (kbd_fd >= 0) {
        /* Grab so key presses don't also land on the console. */
        ioctl(kbd_fd, EVIOCGRAB, 1);
        printf("keyboard : %s\n", kbd_path);
    } else if (tty_setup() == 0) {
        printf("keyboard : terminal (no evdev keyboard found)\n");
    } else {
        fprintf(stderr, "no keyboard found and stdin is not a terminal\n");
        goto out;
    }

    printf("bridge   : 0x%08X  X @ +0x%04X  Y @ +0x%04X\n",
           LW_BRIDGE_BASE, POS_X_OFFSET, POS_Y_OFFSET);
    printf("keys     : arrows/WASD move, Shift = x%d, R recentre, Q quit\n\n",
           STEP_LARGE);

    write_position(x, y);
    printf("\rpos = (%3d, %3d)  ", x, y);
    fflush(stdout);

    while (running) {
        enum action act = (kbd_fd >= 0) ? evdev_next(kbd_fd, &shift)
                                        : tty_next(&shift);
        int step = shift ? STEP_LARGE : STEP_SMALL;

        switch (act) {
        case ACT_UP:     y -= step; break;
        case ACT_DOWN:   y += step; break;
        case ACT_LEFT:   x -= step; break;
        case ACT_RIGHT:  x += step; break;
        case ACT_CENTRE: x = POS_X_MAX / 2; y = POS_Y_MAX / 2; break;
        case ACT_QUIT:   running = 0; continue;
        default:         continue;
        }

        x = clamp(x, 0, POS_X_MAX);
        y = clamp(y, 0, POS_Y_MAX);
        write_position(x, y);

        printf("\rpos = (%3d, %3d)  ", x, y);
        fflush(stdout);
    }
    printf("\n");

out:
    tty_restore();
    if (kbd_fd >= 0) {
        ioctl(kbd_fd, EVIOCGRAB, 0);
        close(kbd_fd);
    }
    munmap(lw_base, LW_BRIDGE_SPAN);
    close(mem_fd);
    return 0;
}
