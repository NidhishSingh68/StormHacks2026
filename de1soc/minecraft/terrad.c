/*
 * terrad.c - world receiver and game supervisor for the DE1-SoC
 *
 * The board's only USB serial port is the Linux console UART. terrad takes
 * it over (systemd stops the login shell on it), runs the game, and listens
 * for new worlds from the PC (worldgen.py --uart / --serve --uart):
 *
 *   1. a world frame starts arriving  -> the game is stopped and a loading
 *                                        screen with a progress bar is shown
 *   2. the transfer completes         -> the blob is checked (CRC-32),
 *                                        unpacked into terrain.bin (written
 *                                        to a temporary file, then renamed)
 *   3. the game is started again on the new world. A failed transfer keeps
 *      the old terrain.bin and restarts the game on that.
 *
 * The game is also restarted if it exits or crashes (but not in a tight loop).
 *
 * Wire format, PC -> board (all little-endian):
 *
 *   "TRRA" | u8 type | u8 seq | u16 0 | u32 length | u32 CRC-32 of payload
 *          | u32 CRC-32 of the 16 bytes before it | payload
 *
 *   type 1 WORLD    payload is a TRZ1 terrain blob (see decode_trz1)
 *   type 2 PING     no payload; answered with PONG
 *   type 3 CONSOLE  no payload; hand the UART back to a login shell
 *
 * Replies, board -> PC, one text line each (kernel messages may be mixed in
 * on the same UART, so the PC looks for the "@TERRAD" prefix):
 *
 *   @TERRAD READY | RECV <seq> <bytes> | OK <seq> | ERR <seq> <reason>
 *   @TERRAD PONG <seq> | CONSOLE
 *
 * Typing "console" and Enter in a terminal on the UART (e.g. minicom) also
 * gives the port back to a login shell. "sudo systemctl start terrad" from
 * that shell hands it back to terrad.
 *
 * Usage: terrad [--tty /dev/ttyS0] [--baud 115200] [--dir /home/ubuntu]
 *               [--game ./mc] [--no-game]
 */

#define _GNU_SOURCE
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdarg.h>
#include <string.h>
#include <strings.h>
#include <ctype.h>
#include <errno.h>
#include <fcntl.h>
#include <signal.h>
#include <termios.h>
#include <time.h>
#include <unistd.h>
#include <sys/select.h>
#include <sys/stat.h>
#include <sys/wait.h>
#include "mc.h"
#include "fpga.h"
#include "font.h"

#define FRAME_HDR       20
#define TYPE_WORLD      1
#define TYPE_PING       2
#define TYPE_CONSOLE    3
#define MAX_PAYLOAD     (64u << 20)
#define BYTE_TIMEOUT_MS 5000        /* give up on a transfer that stalls */

#define TERRAIN_HEADER  4096
#define TILE            16
#define RECORD          8
#define MAX_COLUMNS     (8192u * 8192u)

#define RESPAWN_DELAY_MS   2000
#define RESPAWN_WINDOW_MS  30000
#define RESPAWN_MAX        5        /* exits within the window before giving up */

static const char *tty_path = "/dev/ttyS0";
static const char *dir = "/home/ubuntu";
static const char *game = "./mc";
static int baud = 115200, run_game = 1;
static int tty = -1;
static volatile sig_atomic_t stop;

static pid_t game_pid = -1;
static double respawn_at = -1, exit_times[RESPAWN_MAX];
static int exit_count;

static uint8_t screen[SCREEN_SIZE] __attribute__((aligned(64)));
static int have_fpga;

static double now_ms(void)
{
    struct timespec ts;

    clock_gettime(CLOCK_MONOTONIC, &ts);
    return ts.tv_sec * 1e3 + ts.tv_nsec / 1e6;
}

static void on_signal(int sig)
{
    (void)sig;
    stop = 1;
}

/* A status line to the PC over the UART, and to our own log. */
static void reply(const char *fmt, ...)
{
    char line[256];
    va_list ap;
    int n;

    n = snprintf(line, sizeof(line), "\r\n@TERRAD ");
    va_start(ap, fmt);
    n += vsnprintf(line + n, sizeof(line) - n, fmt, ap);
    va_end(ap);
    if (n > (int)sizeof(line) - 3)
        n = sizeof(line) - 3;
    line[n++] = '\r';
    line[n++] = '\n';
    if (tty >= 0 && write(tty, line, n) < 0)
        perror("write tty");
    fprintf(stderr, "%.*s\n", n - 4, line + 2);
}

/* ---- CRC-32 (zlib / IEEE) ------------------------------------------------ */

static uint32_t crc_table[256];

static void crc_init(void)
{
    uint32_t i, k, c;

    for (i = 0; i < 256; i++) {
        for (c = i, k = 0; k < 8; k++)
            c = (c & 1) ? 0xEDB88320u ^ (c >> 1) : c >> 1;
        crc_table[i] = c;
    }
}

static uint32_t crc32_buf(const uint8_t *p, size_t n)
{
    uint32_t c = 0xFFFFFFFFu;

    while (n--)
        c = crc_table[(c ^ *p++) & 0xFF] ^ (c >> 8);
    return c ^ 0xFFFFFFFFu;
}

static uint32_t rd32(const uint8_t *p)
{
    return (uint32_t)p[0] | (uint32_t)p[1] << 8 | (uint32_t)p[2] << 16 | (uint32_t)p[3] << 24;
}

/* ---- loading screen ------------------------------------------------------ */

static void fill(int x0, int y0, int w, int h, uint8_t c)
{
    int x, y;

    for (y = y0; y < y0 + h && y < SCREEN_H; y++)
        for (x = x0; x < x0 + w && x < SCREEN_W; x++)
            if (x >= 0 && y >= 0)
                screen[y * SCREEN_W + x] = c;
}

static void text(int y, const char *s, int scale, uint8_t c)
{
    int len = (int)strlen(s), x = (SCREEN_W - (len * 6 - 1) * scale) / 2;

    for (; *s; s++, x += 6 * scale) {
        const uint8_t *g = font_glyph((char)toupper((unsigned char)*s));
        int r, col;

        if (!g)
            continue;
        for (r = 0; r < 7; r++)
            for (col = 0; col < 5; col++)
                if (g[r] & (0x10 >> col))
                    fill(x + col * scale, y + r * scale, scale, scale, c);
    }
}

/* pct < 0: no bar. */
static void show(const char *title, const char *place, int pct, const char *status)
{
    static double last;
    char p[64];
    double t = now_ms();

    if (!have_fpga)
        return;
    if (pct >= 0 && pct < 100 && t - last < 100)    /* at most 10 updates/s */
        return;
    last = t;

    memset(screen, 0x05, sizeof(screen));           /* dark blue */
    text(140, title, 3, 0xFF);
    if (place && *place) {
        snprintf(p, sizeof(p), "%.48s", place);
        text(190, p, 2, 0xDB);
    }
    if (pct >= 0) {
        fill(118, 248, 404, 24, 0xFF);
        fill(120, 250, 400, 20, 0x00);
        fill(120, 250, 400 * (pct > 100 ? 100 : pct) / 100, 20, 0x1C);
    }
    if (status)
        text(300, status, 2, 0xFF);
    fpga_present(screen);
}

/* ---- the game ------------------------------------------------------------ */

static void game_start(void)
{
    char log_path[512], terrain[512];
    pid_t pid;

    if (!run_game || game_pid > 0)
        return;
    snprintf(log_path, sizeof(log_path), "%s/mc.log", dir);
    snprintf(terrain, sizeof(terrain), "%s/terrain.bin", dir);

    pid = fork();
    if (pid < 0) {
        perror("fork");
        return;
    }
    if (pid == 0) {
        int in = open("/dev/null", O_RDONLY);
        int out = open(log_path, O_WRONLY | O_CREAT | O_TRUNC, 0644);

        if (in >= 0) dup2(in, 0);
        if (out >= 0) { dup2(out, 1); dup2(out, 2); }
        if (tty > 2)
            close(tty);
        if (chdir(dir) < 0)
            _exit(126);
        setsid();
        execl(game, game, "--terrain", terrain, (char *)NULL);
        _exit(127);
    }
    game_pid = pid;
    respawn_at = -1;
    fprintf(stderr, "terrad: started %s (pid %d)\n", game, (int)pid);
}

/* Note an exit; returns 0 if the game keeps dying and should stay down. */
static int game_exited(int status)
{
    double t = now_ms();
    int i, recent = 0;

    if (WIFEXITED(status))
        fprintf(stderr, "terrad: game exited with %d\n", WEXITSTATUS(status));
    else if (WIFSIGNALED(status))
        fprintf(stderr, "terrad: game killed by signal %d\n", WTERMSIG(status));
    game_pid = -1;

    exit_times[exit_count++ % RESPAWN_MAX] = t;
    for (i = 0; i < RESPAWN_MAX && i < exit_count; i++)
        if (t - exit_times[i] < RESPAWN_WINDOW_MS)
            recent++;
    if (recent >= RESPAWN_MAX) {
        reply("ERR - game keeps exiting; not restarting it until the next world");
        return 0;
    }
    return 1;
}

static void game_stop(void)
{
    double t0;
    int status;

    if (game_pid <= 0)
        return;
    kill(game_pid, SIGTERM);
    for (t0 = now_ms(); now_ms() - t0 < 3000; ) {
        if (waitpid(game_pid, &status, WNOHANG) == game_pid) {
            game_pid = -1;
            return;
        }
        usleep(20000);
    }
    kill(game_pid, SIGKILL);
    waitpid(game_pid, &status, 0);
    game_pid = -1;
}

static void game_poll(void)
{
    int status;

    if (game_pid > 0 && waitpid(game_pid, &status, WNOHANG) == game_pid) {
        if (game_exited(status))
            respawn_at = now_ms() + RESPAWN_DELAY_MS;
    }
    if (game_pid <= 0 && respawn_at > 0 && now_ms() >= respawn_at)
        game_start();
}

/* ---- TRZ1 -> terrain.bin --------------------------------------------------- */

/* PackBits into exactly n bytes; returns 0 on success. */
static int unpackbits(const uint8_t *in, size_t inlen, uint8_t *out, size_t n)
{
    size_t i = 0, o = 0;

    while (o < n) {
        unsigned c;

        if (i >= inlen)
            return -1;
        c = in[i++];
        if (c < 128) {
            size_t k = c + 1;

            if (i + k > inlen || o + k > n)
                return -1;
            memcpy(out + o, in + i, k);
            i += k;
            o += k;
        } else if (c > 128) {
            size_t k = 257 - c;

            if (i >= inlen || o + k > n)
                return -1;
            memset(out + o, in[i++], k);
            o += k;
        }
    }
    return 0;
}

/*
 * TRZ1 blob:
 *   "TRZ1" | u32 width | u32 depth | 128-byte terrain.bin header
 *   8 x ( u32 length | PackBits(delta(plane k)) )      plane k = byte k of
 *                                                      every column record,
 *                                                      north->south, west->east
 *   u32 CRC-32 of the rebuilt terrain.bin
 */
static int decode_trz1(const uint8_t *p, size_t n, uint8_t **out, size_t *outlen,
                       const char **err)
{
    uint32_t w, d, k, crc;
    size_t cols, off = 140, i, size;
    uint8_t *rec = NULL, *plane = NULL, *file = NULL;

    *err = "bad blob";
    if (n < 140 + 4 || memcmp(p, "TRZ1", 4) != 0)
        return -1;
    w = rd32(p + 4);
    d = rd32(p + 8);
    if (!w || !d || w % TILE || d % TILE || (uint64_t)w * d > MAX_COLUMNS) {
        *err = "bad size";
        return -1;
    }
    cols = (size_t)w * d;
    size = TERRAIN_HEADER + cols * RECORD;
    rec = malloc(cols * RECORD);
    plane = malloc(cols);
    file = calloc(1, size);
    if (!rec || !plane || !file) {
        *err = "out of memory";
        goto fail;
    }

    for (k = 0; k < RECORD; k++) {
        uint32_t len;
        uint8_t acc = 0;

        if (off + 4 > n)
            goto fail;
        len = rd32(p + off);
        off += 4;
        if (off + len > n || unpackbits(p + off, len, plane, cols) < 0) {
            *err = "bad plane";
            goto fail;
        }
        off += len;
        for (i = 0; i < cols; i++) {
            acc = (uint8_t)(acc + plane[i]);        /* undo the delta coding */
            rec[i * RECORD + k] = acc;
        }
    }
    if (off + 4 > n)
        goto fail;
    crc = rd32(p + off);

    /* header, then 16x16 tiles: each tile row is 16 consecutive records */
    memcpy(file, p + 12, 128);
    {
        uint8_t *dst = file + TERRAIN_HEADER;
        uint32_t tz, tx, lz;

        for (tz = 0; tz < d / TILE; tz++)
            for (tx = 0; tx < w / TILE; tx++)
                for (lz = 0; lz < TILE; lz++) {
                    memcpy(dst, rec + (((size_t)tz * TILE + lz) * w + tx * TILE) * RECORD,
                           TILE * RECORD);
                    dst += TILE * RECORD;
                }
    }
    if (crc32_buf(file, size) != crc) {
        *err = "terrain CRC mismatch";
        goto fail;
    }
    free(rec);
    free(plane);
    *out = file;
    *outlen = size;
    *err = NULL;
    return 0;

fail:
    free(rec);
    free(plane);
    free(file);
    return -1;
}

static int write_terrain(const uint8_t *data, size_t n)
{
    char path[512], tmp[520];
    int fd;
    size_t off = 0;

    snprintf(path, sizeof(path), "%s/terrain.bin", dir);
    snprintf(tmp, sizeof(tmp), "%s.tmp", path);
    fd = open(tmp, O_WRONLY | O_CREAT | O_TRUNC, 0644);
    if (fd < 0)
        return -1;
    while (off < n) {
        ssize_t w = write(fd, data + off, n - off);

        if (w <= 0) {
            close(fd);
            unlink(tmp);
            return -1;
        }
        off += (size_t)w;
    }
    fsync(fd);
    close(fd);
    return rename(tmp, path);           /* the game never sees half a file */
}

/* ---- frames from the PC -------------------------------------------------- */

enum { S_SYNC, S_HEADER, S_PAYLOAD };

static struct {
    int state;
    uint8_t hdr[FRAME_HDR];
    int hlen, matched;
    uint8_t *payload;
    uint32_t len, got, crc;
    uint8_t type, seq;
    double last_byte;
    char place[64];
    char line[32];
    int linelen;
} rx;

static void release_console(void)
{
    reply("CONSOLE");
    fprintf(stderr, "terrad: handing the UART to a login shell\n");
#ifndef HOST                            /* never touch the development PC */
    if (fork() == 0) {
        /* stops terrad too (Conflicts=), and with it the game */
        execlp("systemctl", "systemctl", "--no-block", "start",
               "serial-getty@ttyS0.service", (char *)NULL);
        _exit(1);
    }
#endif
    stop = 1;
}

static void finish_world(void)
{
    const char *err = NULL;
    uint8_t *file = NULL;
    size_t size = 0;

    show("BUILDING WORLD", rx.place, 100, "CHECKING");
    if (crc32_buf(rx.payload, rx.len) != rx.crc)
        err = "payload CRC mismatch";
    else if (decode_trz1(rx.payload, rx.len, &file, &size, &err) < 0)
        ;
    else if (write_terrain(file, size) < 0)
        err = "cannot write terrain.bin";

    if (err) {
        reply("ERR %u %s", rx.seq, err);
        show("TRANSFER FAILED", err, -1, "KEEPING THE OLD WORLD");
        sleep(2);
    } else {
        reply("OK %u", rx.seq);
        show("STARTING", rx.place, -1, NULL);
    }
    free(file);
    exit_count = 0;
    game_start();
}

static void abort_transfer(const char *why)
{
    reply("ERR %u %s", rx.seq, why);
    free(rx.payload);
    rx.payload = NULL;
    rx.state = S_SYNC;
    show("TRANSFER FAILED", why, -1, "KEEPING THE OLD WORLD");
    exit_count = 0;
    game_start();
}

static void header_done(void)
{
    const uint8_t *h = rx.hdr;

    if (crc32_buf(h, 16) != rd32(h + 16)) {         /* noise, not a frame */
        rx.state = S_SYNC;
        return;
    }
    rx.type = h[4];
    rx.seq = h[5];
    rx.len = rd32(h + 8);
    rx.crc = rd32(h + 12);

    switch (rx.type) {
    case TYPE_PING:
        reply("PONG %u", rx.seq);
        rx.state = S_SYNC;
        return;
    case TYPE_CONSOLE:
        rx.state = S_SYNC;
        release_console();
        return;
    case TYPE_WORLD:
        if (rx.len < 144 || rx.len > MAX_PAYLOAD) {
            reply("ERR %u bad length", rx.seq);
            rx.state = S_SYNC;
            return;
        }
        rx.payload = malloc(rx.len);
        if (!rx.payload) {
            reply("ERR %u out of memory", rx.seq);
            rx.state = S_SYNC;
            return;
        }
        game_stop();                                /* new world: game off */
        respawn_at = -1;
        rx.got = 0;
        rx.place[0] = 0;
        rx.state = S_PAYLOAD;
        reply("RECV %u %u", rx.seq, rx.len);
        show("LOADING NEW WORLD", NULL, 0, "0%");
        return;
    default:
        reply("ERR %u unknown frame type %u", rx.seq, rx.type);
        rx.state = S_SYNC;
    }
}

/* Typed text: "console" + Enter gives the UART back to a login shell. */
static void typed(uint8_t c)
{
    if (c == '\r' || c == '\n') {
        rx.line[rx.linelen] = 0;
        if (strcasecmp(rx.line, "console") == 0)
            release_console();
        rx.linelen = 0;
    } else if (isprint(c) && rx.linelen < (int)sizeof(rx.line) - 1) {
        rx.line[rx.linelen++] = (char)c;
    } else if (!isprint(c)) {
        rx.linelen = 0;
    }
}

static void feed(const uint8_t *b, size_t n)
{
    static const uint8_t magic[4] = { 'T', 'R', 'R', 'A' };
    size_t i = 0;

    rx.last_byte = now_ms();
    while (i < n && !stop) {
        switch (rx.state) {
        case S_SYNC:
            typed(b[i]);
            if (b[i] == magic[rx.matched]) {
                if (++rx.matched == 4) {
                    memcpy(rx.hdr, magic, 4);
                    rx.hlen = 4;
                    rx.matched = 0;
                    rx.linelen = 0;
                    rx.state = S_HEADER;
                }
            } else {
                rx.matched = (b[i] == magic[0]) ? 1 : 0;
            }
            i++;
            break;

        case S_HEADER:
            rx.hdr[rx.hlen++] = b[i++];
            if (rx.hlen == FRAME_HDR)
                header_done();
            break;

        case S_PAYLOAD: {
            size_t k = n - i;

            if (k > rx.len - rx.got)
                k = rx.len - rx.got;
            memcpy(rx.payload + rx.got, b + i, k);
            rx.got += (uint32_t)k;
            i += k;

            /* the place name is in the terrain header near the start */
            if (!rx.place[0] && rx.got >= 140 && memcmp(rx.payload, "TRZ1", 4) == 0) {
                const char *src = (const char *)rx.payload + 12 + 64;

                snprintf(rx.place, sizeof(rx.place), "%.63s",
                         strncmp(src, "worldgen: ", 10) == 0 ? src + 10 : src);
            }
            if (rx.got == rx.len) {
                rx.state = S_SYNC;
                finish_world();
                free(rx.payload);
                rx.payload = NULL;
            } else {
                char pct[16];
                int p = (int)((uint64_t)rx.got * 100 / rx.len);

                snprintf(pct, sizeof(pct), "%d%%", p);
                show("LOADING NEW WORLD", rx.place, p, pct);
            }
            break;
        }
        }
    }
}

/* ---- setup --------------------------------------------------------------- */

static speed_t baud_const(int b)
{
    switch (b) {
    case 9600:   return B9600;
    case 19200:  return B19200;
    case 38400:  return B38400;
    case 57600:  return B57600;
    case 230400: return B230400;
    case 460800: return B460800;
    case 921600: return B921600;
    default:     return B115200;
    }
}

static int open_tty(void)
{
    struct termios t;
    int fd = open(tty_path, O_RDWR | O_NOCTTY);

    if (fd < 0) {
        perror(tty_path);
        return -1;
    }
    if (tcgetattr(fd, &t) == 0) {
        cfmakeraw(&t);
        t.c_cflag |= CLOCAL | CREAD;
        t.c_cflag &= ~CRTSCTS;
        t.c_cc[VMIN] = 0;
        t.c_cc[VTIME] = 0;
        cfsetispeed(&t, baud_const(baud));
        cfsetospeed(&t, baud_const(baud));
        tcsetattr(fd, TCSANOW, &t);     /* a pty in tests may refuse speeds */
    }
    tcflush(fd, TCIFLUSH);
    return fd;
}

static void usage(void)
{
    fprintf(stderr, "usage: terrad [--tty DEV] [--baud N] [--dir DIR] [--game PATH] [--no-game]\n");
}

int main(int argc, char **argv)
{
    int i;

    for (i = 1; i < argc; i++) {
        if (!strcmp(argv[i], "--tty") && i + 1 < argc)        tty_path = argv[++i];
        else if (!strcmp(argv[i], "--baud") && i + 1 < argc)  baud = atoi(argv[++i]);
        else if (!strcmp(argv[i], "--dir") && i + 1 < argc)   dir = argv[++i];
        else if (!strcmp(argv[i], "--game") && i + 1 < argc)  game = argv[++i];
        else if (!strcmp(argv[i], "--no-game"))               run_game = 0;
        else { usage(); return 2; }
    }

    crc_init();
    signal(SIGINT, on_signal);
    signal(SIGTERM, on_signal);
    signal(SIGPIPE, SIG_IGN);

    tty = open_tty();
    if (tty < 0)
        return 1;
    have_fpga = fpga_open() == 0;
    if (!have_fpga)
        fprintf(stderr, "terrad: no FPGA access, loading screen disabled\n");

    reply("READY");
    if (tty >= 0) {
        static const char hint[] =
            "terrad owns this serial port (world transfers from the PC).\r\n"
            "Type 'console' and Enter for a login shell.\r\n";

        if (write(tty, hint, sizeof(hint) - 1) < 0)
            perror("write tty");
    }
    game_start();

    while (!stop) {
        fd_set rd;
        struct timeval tv = { 0, 100000 };
        uint8_t buf[4096];

        FD_ZERO(&rd);
        FD_SET(tty, &rd);
        if (select(tty + 1, &rd, NULL, NULL, &tv) > 0) {
            ssize_t n = read(tty, buf, sizeof(buf));

            if (n > 0)
                feed(buf, (size_t)n);
        }
        if (rx.state != S_SYNC && now_ms() - rx.last_byte > BYTE_TIMEOUT_MS)
            abort_transfer("timeout");
        game_poll();
    }

    game_stop();
    if (have_fpga)
        fpga_close();
    close(tty);
    return 0;
}
