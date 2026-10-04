/* ===========================================================================
 * render.c  --  the game, written in plain C
 * ===========================================================================
 * Reads world.bin (16384 bytes from worldgen.py) and draws a first-person
 * 3D view of it, exactly the way the FPGA will.
 *
 * This is the REFERENCE version. Get the picture right here, where you can
 * see it and change it in seconds, then translate the drawing loop into
 * Verilog. If the FPGA picture does not match this one, the Verilog is wrong.
 *
 * ---------------------------------------------------------------------------
 * BUILD
 *     gcc -O2 -o render render.c -lm
 *
 * RUN
 *     ./render world.bin                 -> writes frame.ppm
 *     ./render world.bin -x 64 -y 64 -a 37
 *     ./render world.bin --fly 6         -> 6 frames walking forward
 *
 * A .ppm opens in GIMP, IrfanView, or most image viewers. To convert:
 *     ffmpeg -i frame.ppm frame.png
 *
 * ---------------------------------------------------------------------------
 * HOW THE DRAWING WORKS  (this is the part to translate to Verilog)
 *
 * For each column of pixels on the screen:
 *   1. Work out which direction that column looks in.
 *   2. Walk outward from the player along that direction, one step at a time.
 *   3. At each step, look up the ground height at that spot in the world.
 *   4. Work out how high on the screen that ground would appear.
 *   5. If it is higher than anything drawn so far in this column, paint the
 *      gap with that cell's colour.
 *   6. Keep walking until the far distance.
 * Walking far-to-near would need a depth test; walking near-to-far and only
 * painting upward gives correct occlusion for free. That is the classic
 * "voxel space" trick, and it is cheap enough for hardware.
 * =========================================================================== */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <stdint.h>

/* For the interactive mode: raw keyboard input on Linux / macOS.
 * On Windows use WSL, or build without --play. */
#if !defined(_WIN32)
  #include <termios.h>
  #include <unistd.h>
  #include <fcntl.h>
  #include <sys/ioctl.h>
  #define HAVE_RAW_INPUT 1
#endif

/* ---- world format: must match worldgen.py and the Verilog ---------------- */
#define GRID        128        /* world is 128 x 128 cells                    */
#define MAX_HEIGHT  31         /* heights are 0..31 (5 bits)                  */

#define BLOCK_WATER  0
#define BLOCK_GRASS  1
#define BLOCK_SAND   2
#define BLOCK_STONE  3
#define BLOCK_SNOW   4
#define BLOCK_FOREST 5
#define BLOCK_CITY   6
#define BLOCK_DIRT   7

/* Colour of each block type. Hand these same numbers to the FPGA colour
 * lookup table so both versions produce the same picture. */
static const uint8_t PALETTE[8][3] = {
    { 38,  92, 160},   /* water  */
    { 96, 150,  62},   /* grass  */
    {214, 196, 134},   /* sand   */
    {124, 124, 128},   /* stone  */
    {238, 243, 248},   /* snow   */
    { 42,  94,  54},   /* forest */
    {156, 134, 128},   /* city   */
    {124,  92,  62},   /* dirt   */
};

static const uint8_t SKY[3]     = {135, 178, 222};
static const uint8_t SIDE_DIM   = 70;   /* % brightness of block sides       */

/* ---- screen -------------------------------------------------------------- */
#define SCREEN_W 320
#define SCREEN_H 240

/* ---- camera -------------------------------------------------------------- */
static float eye_above_ground = 2.0f;   /* how far above the ground the eye sits */
#define VIEW_DISTANCE     90     /* how many cells ahead we look              */
#define HORIZON          (SCREEN_H / 2)
#define SCALE             90.0f  /* bigger = taller-looking terrain           */
#define FOV_SCALE        0.55f   /* field of view; bigger = wider             */

static uint8_t world[GRID * GRID];        /* raw bytes straight from the file */
static uint8_t frame[SCREEN_H][SCREEN_W][3];

/* ---- reading the world --------------------------------------------------- */
/* On the FPGA this becomes a memory read at address (y * 128 + x). */
static inline uint8_t cell_at(int x, int y)
{
    if (x < 0)     x = 0;
    if (x >= GRID) x = GRID - 1;
    if (y < 0)     y = 0;
    if (y >= GRID) y = GRID - 1;
    return world[y * GRID + x];
}
static inline int cell_height(int x, int y) { return cell_at(x, y) >> 3; }
static inline int cell_block (int x, int y) { return cell_at(x, y) & 0x07; }

/* ---- drawing ------------------------------------------------------------- */
/* Paint one pixel.
 *   bright  = 100 for the lit top face, less for the shaded side face
 *   haze    = 0 for close up, up to 100 far away: blends toward the sky
 *             colour so distant terrain fades out instead of ending in a
 *             hard line. On the FPGA this is a small multiply-add. */
static void put_pixel(int sx, int sy, const uint8_t rgb[3],
                      int bright, int haze)
{
    if (sx < 0 || sx >= SCREEN_W || sy < 0 || sy >= SCREEN_H) return;
    for (int c = 0; c < 3; c++) {
        int v = (int)rgb[c] * bright / 100;
        v = (v * (100 - haze) + (int)SKY[c] * haze) / 100;
        frame[sy][sx][c] = (uint8_t)v;
    }
}

static void render(float cam_x, float cam_y, float cam_angle_deg);
static void render(float cam_x, float cam_y, float cam_angle_deg)
{
    /* 1. Fill the whole screen with sky. Anything the terrain does not
     *    cover stays sky. */
    for (int sy = 0; sy < SCREEN_H; sy++)
        for (int sx = 0; sx < SCREEN_W; sx++)
            for (int c = 0; c < 3; c++)
                frame[sy][sx][c] = SKY[c];

    /* The eye sits a little above the ground under the player. */
    float cam_z = cell_height((int)cam_x, (int)cam_y) + eye_above_ground;

    float a = cam_angle_deg * (float)M_PI / 180.0f;
    float fwd_x = cosf(a),  fwd_y = -sinf(a);     /* forward direction       */
    float rgt_x = -fwd_y,   rgt_y =  fwd_x;       /* 90 degrees to the right */

    /* 2. One column of pixels at a time. */
    for (int sx = 0; sx < SCREEN_W; sx++) {

        /* How far left or right of straight ahead this column looks. */
        float off = ((float)sx / SCREEN_W - 0.5f) * 2.0f * FOV_SCALE;
        float dir_x = fwd_x + rgt_x * off;
        float dir_y = fwd_y + rgt_y * off;

        /* The highest point painted so far in this column. We only ever
         * paint ABOVE this, which is what hides things behind hills. */
        int highest_painted = SCREEN_H;

        /* 3. Walk outward, near to far. */
        for (int step = 1; step <= VIEW_DISTANCE; step++) {

            float wx = cam_x + dir_x * step;
            float wy = cam_y + dir_y * step;

            int cx = (int)floorf(wx);
            int cy = (int)floorf(wy);
            if (cx < 0 || cx >= GRID || cy < 0 || cy >= GRID) break;

            int h  = cell_height(cx, cy);
            int bt = cell_block (cx, cy);

            /* 4. Where on the screen does the TOP of this column land?
             *    Things far away shrink toward the horizon, so divide by
             *    the distance. */
            int top_y = (int)(HORIZON + (cam_z - h) * SCALE / (float)step);

            /* 5. Paint from there up to whatever we painted before. */
            if (top_y < highest_painted) {

                /* How much this cell fades into the distance. */
                int haze = (step * 65) / VIEW_DISTANCE;

                /* The flat top of the block keeps its full brightness.
                 * The side facing us is darker, and that cheap shading is
                 * what makes the blocks read as blocks. */
                put_pixel(sx, top_y, PALETTE[bt], 100, haze);

                for (int sy = top_y + 1; sy < highest_painted; sy++)
                    put_pixel(sx, sy, PALETTE[bt], SIDE_DIM, haze);

                highest_painted = top_y;
            }

            if (highest_painted <= 0) break;   /* column full, stop early */
        }
    }
}

/* ---- saving the picture -------------------------------------------------- */
static int save_ppm(const char *path)
{
    FILE *f = fopen(path, "wb");
    if (!f) { perror(path); return 0; }
    fprintf(f, "P6\n%d %d\n255\n", SCREEN_W, SCREEN_H);
    fwrite(frame, 1, sizeof(frame), f);
    fclose(f);
    return 1;
}

/* ---- finding somewhere sensible to stand --------------------------------- */
/* Only used if you do not pass -x and -y. Picks the cell nearest the middle
 * that is on land and not on a cliff edge. */
static void default_spawn(int *out_x, int *out_y)
{
    int best_x = GRID / 2, best_y = GRID / 2;
    float best_d = 1e9f;
    for (int y = 4; y < GRID - 4; y++) {
        for (int x = 4; x < GRID - 4; x++) {
            if (cell_block(x, y) == BLOCK_WATER) continue;
            int h = cell_height(x, y);
            int slope = 0, d;
            d = abs(cell_height(x+1, y) - h); if (d > slope) slope = d;
            d = abs(cell_height(x-1, y) - h); if (d > slope) slope = d;
            d = abs(cell_height(x, y+1) - h); if (d > slope) slope = d;
            d = abs(cell_height(x, y-1) - h); if (d > slope) slope = d;
            if (slope > 2) continue;
            float dx = x - GRID/2.0f, dy = y - GRID/2.0f;
            float dist = dx*dx + dy*dy;
            if (dist < best_d) { best_d = dist; best_x = x; best_y = y; }
        }
    }
    *out_x = best_x; *out_y = best_y;
}

/* Pick a direction worth looking at: of 16 compass directions, choose the one
 * where the ground straight ahead is lowest on average. That points the camera
 * down a valley or out over water instead of straight into a hillside. */
static float default_angle(float cx, float cy)
{
    float best_angle = 0.0f;
    float best_score = 1e9f;
    for (int i = 0; i < 16; i++) {
        float deg = i * 22.5f;
        float a = deg * (float)M_PI / 180.0f;
        float dx = cosf(a), dy = -sinf(a);
        float sum = 0.0f;
        int   n   = 0;
        for (int step = 1; step <= 12; step++) {
            int x = (int)(cx + dx * step);
            int y = (int)(cy + dy * step);
            if (x < 0 || x >= GRID || y < 0 || y >= GRID) break;
            sum += cell_height(x, y);
            n++;
        }
        if (n == 0) continue;
        float score = sum / n;
        if (score < best_score) { best_score = score; best_angle = deg; }
    }
    return best_angle;
}

/* ===========================================================================
 * THE GAME LOOP  --  input, move, draw, repeat
 * ===========================================================================
 * This is the part the FPGA also has to do. On the board the keyboard is
 * replaced by the push buttons or a PS/2 keyboard, and the terminal drawing
 * is replaced by writing pixels to the VGA frame buffer. The middle part,
 * where the player's position and angle get updated, is identical.
 * =========================================================================== */

/* Everything that describes the player. On the FPGA these become registers. */
typedef struct {
    float x, y;        /* where they stand, in world cells */
    float angle;       /* which way they face, in degrees. 0 = east */
    float eye;         /* how far above the ground the eye sits */
} Player;

#define MOVE_STEP   1.0f     /* cells moved per key press  */
#define TURN_STEP   8.0f     /* degrees turned per key press */

/* Can the player stand on this cell? Keeps them on the map and out of
 * the water, and stops them climbing cliffs. */
static int can_stand(float nx, float ny, int from_height)
{
    int cx = (int)nx, cy = (int)ny;
    if (cx < 1 || cx >= GRID - 1 || cy < 1 || cy >= GRID - 1) return 0;
    if (cell_block(cx, cy) == BLOCK_WATER)                     return 0;
    if (abs(cell_height(cx, cy) - from_height) > 3)            return 0;
    return 1;
}

/* Apply one key press to the player. Returns 0 if the player asked to quit. */
static int handle_key(Player *p, int key)
{
    float a   = p->angle * (float)M_PI / 180.0f;
    float fx  = cosf(a), fy = -sinf(a);
    int   now = cell_height((int)p->x, (int)p->y);
    float nx  = p->x, ny = p->y;

    switch (key) {
        case 'w': nx += fx * MOVE_STEP; ny += fy * MOVE_STEP; break;
        case 's': nx -= fx * MOVE_STEP; ny -= fy * MOVE_STEP; break;
        case 'd': nx -= fy * MOVE_STEP; ny += fx * MOVE_STEP; break;  /* strafe */
        case 'a': nx += fy * MOVE_STEP; ny -= fx * MOVE_STEP; break;
        case 'q': p->angle += TURN_STEP; return 1;
        case 'e': p->angle -= TURN_STEP; return 1;
        case 'r': p->eye   += 1.0f;      return 1;
        case 'f': if (p->eye > 1.0f) p->eye -= 1.0f; return 1;
        case 27:                                              /* Esc */
        case 'x': return 0;
        default:  return 1;
    }

    if (can_stand(nx, ny, now)) { p->x = nx; p->y = ny; }

    while (p->angle >= 360.0f) p->angle -= 360.0f;
    while (p->angle <    0.0f) p->angle += 360.0f;
    return 1;
}

#ifdef HAVE_RAW_INPUT
/* Draw the current frame into the terminal using coloured blocks.
 * Two spaces per pixel keeps it roughly square. */
static void draw_to_terminal(const Player *p, int cols, int rows, int term_cols)
{
    static char buf[1 << 20];
    int n = 0;

    n += snprintf(buf + n, sizeof(buf) - n, "\033[H");   /* cursor home */

    for (int r = 0; r < rows; r++) {
        int sy = r * SCREEN_H / rows;
        for (int c = 0; c < cols; c++) {
            int sx = c * SCREEN_W / cols;
            n += snprintf(buf + n, sizeof(buf) - n,
                          "\033[48;2;%d;%d;%dm  ",
                          frame[sy][sx][0], frame[sy][sx][1], frame[sy][sx][2]);
            if (n > (int)sizeof(buf) - 64) goto flush;
        }
        /* \033[K wipes anything left over from a bigger previous frame. */
        n += snprintf(buf + n, sizeof(buf) - n, "\033[0m\033[K\n");
    }
flush:
    /* Status line, trimmed to the window so it never wraps onto a second
     * line and pushes the picture up the screen. */
    {
        char status[256];
        snprintf(status, sizeof(status),
            " x=%.0f y=%.0f facing=%.0f  on %s  "
            "[wasd] move  [qe] turn  [rf] eye  [x] quit",
            p->x, p->y, p->angle,
            (const char *[]){"water","grass","sand","stone",
                             "snow","forest","city","dirt"}
                [cell_block((int)p->x, (int)p->y)]);
        if (term_cols > 1 && (int)strlen(status) > term_cols - 1)
            status[term_cols - 1] = '\0';
        n += snprintf(buf + n, sizeof(buf) - n,
                      "\033[0m\033[K%s\033[J\n", status);
    }

    ssize_t ignored = write(1, buf, n);
    (void)ignored;
}

static struct termios saved_term;

static void restore_terminal(void)
{
    tcsetattr(0, TCSANOW, &saved_term);
    printf("\033[?25h\033[0m\n");        /* show the cursor again */
}

/* How big is the terminal right now? Each picture pixel is drawn as TWO
 * characters (so it looks square), so the usable width is half the columns.
 * If we draw wider than the terminal, every row wraps and you get black
 * stripes through the picture. */
static void terminal_size(int *cols, int *rows, int *term_cols)
{
    struct winsize ws;
    int tc = 80, tr = 24;

    if (ioctl(1, TIOCGWINSZ, &ws) == 0 && ws.ws_col > 0 && ws.ws_row > 0) {
        tc = ws.ws_col;
        tr = ws.ws_row;
    }
    *term_cols = tc;

    *cols = (tc - 1) / 2;             /* two characters per picture pixel  */
    *rows = tr - 2;                   /* leave a line for the status bar   */

    if (*cols > 160) *cols = 160;     /* no point drawing finer than this  */
    if (*rows > 60)  *rows = 60;
    if (*cols < 20)  *cols = 20;
    if (*rows < 10)  *rows = 10;
}

/* Play in the terminal. One key press = one move. */
static void play(Player *p)
{
    struct termios raw;
    tcgetattr(0, &saved_term);
    raw = saved_term;
    raw.c_lflag &= ~(ICANON | ECHO);      /* keys arrive immediately */
    tcsetattr(0, TCSANOW, &raw);
    atexit(restore_terminal);

    printf("\033[?25l\033[2J");          /* hide cursor, clear screen */

    int running = 1;
    while (running) {
        /* Measured every frame, so resizing the window just works. */
        int cols, rows, term_cols;
        terminal_size(&cols, &rows, &term_cols);

        render(p->x, p->y, p->angle);
        draw_to_terminal(p, cols, rows, term_cols);

        int ch = getchar();
        if (ch == EOF) break;
        running = handle_key(p, ch);
    }
    restore_terminal();
    printf("bye\n");
}
#endif /* HAVE_RAW_INPUT */

/* ---- main ---------------------------------------------------------------- */
int main(int argc, char **argv)
{
    const char *world_path = NULL;
    float cam_x = -1, cam_y = -1, cam_angle = -1.0f;
    int fly_frames = 0;
    int play_mode  = 0;

    for (int i = 1; i < argc; i++) {
        if      (!strcmp(argv[i], "-x")    && i+1 < argc) cam_x      = atof(argv[++i]);
        else if (!strcmp(argv[i], "-y")    && i+1 < argc) cam_y      = atof(argv[++i]);
        else if (!strcmp(argv[i], "-a")    && i+1 < argc) cam_angle  = atof(argv[++i]);
        else if (!strcmp(argv[i], "--fly") && i+1 < argc) fly_frames = atoi(argv[++i]);
        else if (!strcmp(argv[i], "--eye") && i+1 < argc) eye_above_ground = atof(argv[++i]);
        else if (!strcmp(argv[i], "--play"))              play_mode = 1;
        else if (argv[i][0] != '-')                       world_path = argv[i];
    }

    if (!world_path) {
        fprintf(stderr,
            "usage: %s world.bin [-x COL] [-y ROW] [-a ANGLE_DEG] [--fly N]\n"
            "  -x -y   where the player stands (0..127). default: near centre\n"
            "  -a      which way they face, in degrees. 0 = east\n"
            "  --fly   write N frames walking forward, frame000.ppm ...\n"
            "  --eye   how far above the ground the eye sits (default 2)\n"
            "  --play  walk around with the keyboard, drawn in the terminal\n",
            argv[0]);
        return 1;
    }

    /* Load the world. */
    FILE *f = fopen(world_path, "rb");
    if (!f) { perror(world_path); return 1; }
    size_t got = fread(world, 1, sizeof(world), f);
    fclose(f);
    if (got != sizeof(world)) {
        fprintf(stderr, "error: expected %zu bytes, got %zu. "
                        "Is this a world.bin from worldgen.py?\n",
                        sizeof(world), got);
        return 1;
    }

    if (cam_x < 0 || cam_y < 0) {
        int sx, sy;
        default_spawn(&sx, &sy);
        if (cam_x < 0) cam_x = sx + 0.5f;
        if (cam_y < 0) cam_y = sy + 0.5f;
        printf("spawn: x=%.0f y=%.0f  (pass -x -y to choose your own)\n",
               cam_x, cam_y);
    }
    if (cam_angle < 0) {
        cam_angle = default_angle(cam_x, cam_y);
        printf("facing: %.0f degrees  (pass -a to choose your own)\n", cam_angle);
    }

    if (play_mode) {
#ifdef HAVE_RAW_INPUT
        Player p = { cam_x, cam_y, cam_angle, eye_above_ground };
        play(&p);
        return 0;
#else
        fprintf(stderr, "--play needs Linux or macOS (try WSL on Windows)\n");
        return 1;
#endif
    }

    if (fly_frames > 0) {
        float a = cam_angle * (float)M_PI / 180.0f;
        for (int i = 0; i < fly_frames; i++) {
            render(cam_x, cam_y, cam_angle);
            char name[64];
            snprintf(name, sizeof(name), "frame%03d.ppm", i);
            if (!save_ppm(name)) return 1;
            printf("wrote %s  (x=%.1f y=%.1f angle=%.0f)\n",
                   name, cam_x, cam_y, cam_angle);
            cam_x += cosf(a) * 3.0f;      /* step forward 3 cells */
            cam_y += -sinf(a) * 3.0f;
        }
    } else {
        render(cam_x, cam_y, cam_angle);
        if (!save_ppm("frame.ppm")) return 1;
        printf("wrote frame.ppm  (%dx%d)  x=%.1f y=%.1f angle=%.0f\n",
               SCREEN_W, SCREEN_H, cam_x, cam_y, cam_angle);
    }
    return 0;
}