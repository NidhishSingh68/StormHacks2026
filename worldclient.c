/*
 * world_client.c  --  loads a TERRA world on the board (plain C, no libraries)
 *
 * Build on the board (or any Linux box):
 *     gcc -O2 -Wall -o world_client world_client.c
 *
 * Two ways to get the world:
 *
 *   A) Poll the PC's worldgen server over the network (easiest):
 *        ./world_client --server 192.168.1.20:8080
 *      The PC must run:  python worldgen.py --serve --host 0.0.0.0
 *      Every time someone presses "Enter this world" in the UI, the version
 *      number on the server goes up, we notice within 2 seconds, download the
 *      16 KB frame, verify its CRC and hand it to your game.
 *
 *   B) Read a frame file that was copied to the board (scp / SD card):
 *        ./world_client --file /home/root/world/current.frame
 *      Start the PC server with  --board root@<board-ip>:/home/root/world/current.frame
 *      and it copies the file for you on every Enter. Here we watch the file's
 *      modification time and reload when it changes.
 *
 * Wire format (all little endian), same as pack_frame() in worldgen.py:
 *     0xA5 0x5A | 32 byte header | 128*128 terrain bytes | fire cells (2 B each) | CRC16
 *   terrain byte = (height << 3) | block_type      row 0 = north, column 0 = west
 *   CRC16 = CCITT-FALSE (poly 0x1021, init 0xFFFF) over header + terrain + fires
 */
#include <errno.h>
#include <netdb.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <time.h>
#include <unistd.h>

#define GRID        128
#define HEADER_SIZE 32

/* Block ids. Same numbers as the Python side and the FPGA colour table. */
enum { B_WATER, B_GRASS, B_SAND, B_STONE, B_SNOW, B_FOREST, B_CITY, B_DIRT };

typedef struct {
    uint8_t height[GRID][GRID];   /* 0..31 */
    uint8_t block[GRID][GRID];    /* 0..7  */
    int     spawn_x, spawn_y, spawn_z, spawn_yaw;   /* yaw 0..255 = 0..360 deg */
    float   temperature_c, rain_mm_h, snow_cm_h;
    int     cloud_pct, wind_kmh, wind_dir_deg, is_day, weather_code;
    int     water_level, flood_level;
    int     meters_per_block, meters_per_level;
    float   lat, lon, local_hour;
    int     flags;                /* bit0 sentinel, bit1 landcover, bit2 flood, bit3 fires */
    int     fire_count;
    uint8_t fire_x[255], fire_y[255];
} world_t;

static uint16_t crc16_ccitt(const uint8_t *d, size_t n)
{
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < n; i++) {
        crc ^= (uint16_t)d[i] << 8;
        for (int b = 0; b < 8; b++)
            crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ 0x1021) : (uint16_t)(crc << 1);
    }
    return crc;
}

static int16_t rd_i16(const uint8_t *p) { return (int16_t)(p[0] | (p[1] << 8)); }

/* Returns 0 on success, negative on a bad frame. Never writes to *w unless the CRC is good. */
int world_parse_frame(const uint8_t *buf, size_t n, world_t *w)
{
    if (n < 2 + HEADER_SIZE + GRID * GRID + 2) return -1;       /* too short */
    if (buf[0] != 0xA5 || buf[1] != 0x5A)      return -2;       /* bad sync */
    const uint8_t *h = buf + 2;
    if (memcmp(h, "WGEN", 4) != 0)             return -3;       /* bad magic */
    if (h[4] != 1)                             return -4;       /* unknown format version */
    if (h[5] != GRID)                          return -5;       /* grid size mismatch */

    int fires = h[21];
    size_t body_len = HEADER_SIZE + (size_t)GRID * GRID + (size_t)fires * 2;
    if (n < 2 + body_len + 2)                  return -1;
    uint16_t sent = (uint16_t)(buf[2 + body_len] | (buf[2 + body_len + 1] << 8));
    if (crc16_ccitt(h, body_len) != sent)      return -6;       /* corrupted in transit */

    w->spawn_x = h[6];  w->spawn_y = h[7];  w->spawn_z = h[8];  w->spawn_yaw = h[9];
    w->temperature_c = rd_i16(h + 10) / 10.0f;
    w->rain_mm_h     = h[12] / 10.0f;
    w->snow_cm_h     = h[13] / 10.0f;
    w->cloud_pct     = h[14];
    w->wind_kmh      = h[15];
    w->wind_dir_deg  = h[16] * 2;
    w->is_day        = h[17];
    w->weather_code  = h[18];
    w->water_level   = h[19];
    w->flood_level   = h[20];
    w->fire_count    = fires;
    w->meters_per_block = h[22];
    w->meters_per_level = h[23];
    w->lat = rd_i16(h + 24) / 100.0f;
    w->lon = rd_i16(h + 26) / 100.0f;
    w->local_hour = h[28] / 255.0f * 24.0f;
    w->flags = h[29];

    const uint8_t *t = h + HEADER_SIZE;
    for (int y = 0; y < GRID; y++)
        for (int x = 0; x < GRID; x++) {
            uint8_t v = t[y * GRID + x];
            w->height[y][x] = v >> 3;
            w->block[y][x]  = v & 7;
        }
    const uint8_t *f = t + GRID * GRID;
    for (int i = 0; i < fires; i++) { w->fire_x[i] = f[i * 2]; w->fire_y[i] = f[i * 2 + 1]; }
    return 0;
}

/* ------------------------------------------------------------------------- */
/*  YOUR HOOK: the game takes over here. Called once per new world.          */
/* ------------------------------------------------------------------------- */
static void on_new_world(const world_t *w)
{
    /* Replace this with: copy w->height / w->block into the voxel engine,
       place the player at (spawn_x, spawn_y, spawn_z) facing spawn_yaw, set the
       sky from is_day / local_hour, rain from rain_mm_h, and so on. */
    int counts[8] = {0};
    for (int y = 0; y < GRID; y++)
        for (int x = 0; x < GRID; x++) counts[w->block[y][x]]++;
    printf("new world: %.2f, %.2f  %d m/block  spawn (%d,%d,%d) yaw %d\n",
           w->lat, w->lon, w->meters_per_block, w->spawn_x, w->spawn_y, w->spawn_z, w->spawn_yaw);
    printf("  weather: %.1f C  rain %.1f  snow %.1f  cloud %d%%  wind %d km/h  %s\n",
           w->temperature_c, w->rain_mm_h, w->snow_cm_h, w->cloud_pct, w->wind_kmh,
           w->is_day ? "day" : "night");
    printf("  blocks: water %d grass %d sand %d stone %d snow %d forest %d city %d dirt %d\n",
           counts[0], counts[1], counts[2], counts[3], counts[4], counts[5], counts[6], counts[7]);
    fflush(stdout);
}

/* ------------------------------------------------------------------------- */
/*  Tiny HTTP/1.0 GET (the Python server closes the connection after replying) */
/* ------------------------------------------------------------------------- */
static int http_get(const char *host, const char *port, const char *path,
                    uint8_t **out, size_t *out_len)
{
    struct addrinfo hints, *res = NULL;
    memset(&hints, 0, sizeof hints);
    hints.ai_socktype = SOCK_STREAM;
    if (getaddrinfo(host, port, &hints, &res) != 0) return -1;

    int fd = -1;
    for (struct addrinfo *a = res; a; a = a->ai_next) {
        fd = socket(a->ai_family, a->ai_socktype, a->ai_protocol);
        if (fd < 0) continue;
        struct timeval tv = {5, 0};
        setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof tv);
        setsockopt(fd, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof tv);
        if (connect(fd, a->ai_addr, a->ai_addrlen) == 0) break;
        close(fd); fd = -1;
    }
    freeaddrinfo(res);
    if (fd < 0) return -2;

    char req[256];
    int rl = snprintf(req, sizeof req, "GET %s HTTP/1.0\r\nHost: %s\r\nConnection: close\r\n\r\n", path, host);
    if (write(fd, req, (size_t)rl) != rl) { close(fd); return -3; }

    size_t cap = 32768, len = 0;
    uint8_t *buf = malloc(cap);
    if (!buf) { close(fd); return -4; }
    for (;;) {
        if (len == cap) { cap *= 2; uint8_t *nb = realloc(buf, cap); if (!nb) { free(buf); close(fd); return -4; } buf = nb; }
        ssize_t r = read(fd, buf + len, cap - len);
        if (r < 0) { if (errno == EINTR) continue; free(buf); close(fd); return -5; }
        if (r == 0) break;
        len += (size_t)r;
    }
    close(fd);

    if (len < 12 || memcmp(buf, "HTTP/", 5) != 0 || !strstr((char *)buf, " 200")) { free(buf); return -6; }
    uint8_t *body = NULL;
    for (size_t i = 0; i + 3 < len; i++)
        if (buf[i] == '\r' && buf[i+1] == '\n' && buf[i+2] == '\r' && buf[i+3] == '\n') { body = buf + i + 4; break; }
    if (!body) { free(buf); return -7; }
    size_t blen = len - (size_t)(body - buf);
    memmove(buf, body, blen);
    *out = buf; *out_len = blen;
    return 0;
}

static int save_atomic(const char *path, const uint8_t *d, size_t n)
{
    char tmp[512];
    snprintf(tmp, sizeof tmp, "%s.tmp", path);
    FILE *f = fopen(tmp, "wb");
    if (!f) return -1;
    if (fwrite(d, 1, n, f) != n) { fclose(f); return -1; }
    fclose(f);
    return rename(tmp, path);            /* atomic: the game never sees half a file */
}

static int load_file(const char *path, world_t *w)
{
    FILE *f = fopen(path, "rb");
    if (!f) return -10;
    uint8_t *buf = malloc(2 + HEADER_SIZE + GRID * GRID + 2 + 510);
    if (!buf) { fclose(f); return -4; }
    size_t n = fread(buf, 1, 2 + HEADER_SIZE + GRID * GRID + 2 + 510, f);
    fclose(f);
    int rc = world_parse_frame(buf, n, w);
    free(buf);
    return rc;
}

int main(int argc, char **argv)
{
    static world_t w;
    if (argc == 3 && strcmp(argv[1], "--file") == 0) {
        time_t last = 0;
        for (;;) {
            struct stat st;
            if (stat(argv[2], &st) == 0 && st.st_mtime != last) {
                usleep(200000);                         /* let a copy in progress finish */
                int rc = load_file(argv[2], &w);
                if (rc == 0) { last = st.st_mtime; on_new_world(&w); }
                else fprintf(stderr, "bad frame (%d), will retry\n", rc);
            }
            sleep(1);
        }
    }
    if (argc >= 3 && strcmp(argv[1], "--server") == 0) {
        const char *save_to = (argc == 5 && strcmp(argv[3], "--save") == 0) ? argv[4] : NULL;
        char host[128], port[16] = "8080";
        strncpy(host, argv[2], sizeof host - 1); host[sizeof host - 1] = 0;
        char *colon = strchr(host, ':');
        if (colon) { *colon = 0; strncpy(port, colon + 1, sizeof port - 1); }
        long seen = 0;
        for (;;) {
            uint8_t *v = NULL; size_t vl = 0;
            if (http_get(host, port, "/current/version", &v, &vl) == 0) {
                long ver = strtol((char *)v, NULL, 10);
                free(v);
                if (ver > 0 && ver != seen) {
                    uint8_t *fr = NULL; size_t fl = 0;
                    if (http_get(host, port, "/current", &fr, &fl) == 0) {
                        int rc = world_parse_frame(fr, fl, &w);
                        if (rc == 0 && save_to && save_atomic(save_to, fr, fl) != 0)
                            fprintf(stderr, "could not write %s\n", save_to);
                        free(fr);
                        if (rc == 0) { seen = ver; on_new_world(&w); }
                        else fprintf(stderr, "bad frame (%d), will retry\n", rc);
                    }
                }
            } else {
                fprintf(stderr, "cannot reach %s:%s, retrying\n", host, port);
            }
            sleep(2);
        }
    }
    fprintf(stderr, "usage:\n  %s --server <pc-ip>[:8080] [--save /path/current.frame]\n  %s --file <path-to-current.frame>\n", argv[0], argv[0]);
    return 1;
}