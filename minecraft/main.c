/*
 * main.c - DE1-SoC voxel demo: terrain from a file, trees, water, breaking
 *
 * Threads (one per Cortex-A9 core):
 *
 *   main (core 0)    input, movement, block breaking, chunk scheduling,
 *                    polygon setup, and rendering bands of the frame.
 *   worker (core 1)  presenting finished frames to the FPGA (started at the
 *                    first vblank after a frame is ready, racing the beam),
 *                    rendering bands of the next frame, and building and
 *                    meshing chunks when there is time.
 *
 * Two back buffers in DDR: while the worker copies frame N to the FPGA, the
 * main thread already renders frame N + 1 into the other buffer, and the
 * worker joins in once its copy is done.
 *
 * Controls (USB keyboard / mouse on the board):
 *   W A S D    move                Space   jump / swim up (fly: up)
 *   Q E        turn left / right   Shift   fly: down
 *   arrows     look around         Ctrl    sprint
 *   mouse      look around         F       toggle flying
 *   B, Enter, left mouse button    break the highlighted block
 *   P, right mouse button          place the selected block
 *   1..7, mouse wheel              pick the block to place (hotbar)
 *   Y          weather: follow the terrain, or force sunny / cloudy /
 *              night / snow / rain
 *   Esc        quit
 *
 * Usage:
 *   sudo ./mc [--terrain FILE] [--dist N] [--weather NAME] [--auto]
 *       --terrain   terrain file (default terrain.bin; flat world if missing)
 *       --dist      render distance in chunks (default 8, max 15)
 *       --weather   force sunny, cloudy, night, snow or rain (default: the
 *                   weather stored in the terrain where you stand)
 *       --auto      walk forward and turn slowly on its own
 *   ./mc [--terrain FILE] [--dist N] [--break N] [--build] --shot X Y Z YAW PITCH out.ppm
 *       render one frame from that eye position (degrees) and save it,
 *       after breaking the block under the crosshair N times, or placing
 *       one of each hotbar block in front of the camera (--build)
 */

#define _GNU_SOURCE
#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <string.h>
#include <math.h>
#include <signal.h>
#include <time.h>
#include <unistd.h>
#include <pthread.h>
#include <sched.h>
#include "mc.h"
#include "world.h"
#include "terrain.h"
#include "render.h"
#include "fpga.h"
#include "input.h"

#define EYE_HEIGHT      1.62f
#define PLAYER_H        1.8f
#define PLAYER_R        0.3f        /* half width */
#define WALK_SPEED      4.3f
#define SPRINT_SPEED    5.6f
#define FLY_SPEED       10.9f
#define SWIM_FACTOR     0.6f
#define GRAVITY         28.0f
#define WATER_GRAVITY   12.0f
#define WATER_SINK      4.0f        /* max sinking speed, blocks/s */
#define JUMP_SPEED      8.4f
#define SWIM_UP_SPEED   3.0f
#define TURN_SPEED      2.2f        /* rad/s with Q/E or the arrow keys */
#define MOUSE_SENS      0.0025f     /* rad per mouse count */
#define TTY_STEP        1.0f        /* blocks per terminal key press */
#define TTY_TURN        0.26f       /* rad per terminal turn press */
#define REACH           5.0f        /* blocks */
#define BREAK_REPEAT    0.25f       /* s between breaks/places while held */

#define WEATHER_FADE    3.0f        /* s to blend into a new weather */
#define WEATHER_STEPS   20          /* colour table rebuilds per blend */
#define WEATHER_SETTLE  0.75f       /* s a new area's weather must persist */

#define JOB_QUEUE       512
#define JOBS_PER_FRAME  8

/* ---- shared state -------------------------------------------------------- */

enum { JOB_GENERATE, JOB_MESH };

struct job {
    int type;
    struct chunk *chunk;
};

static pthread_mutex_t job_lock = PTHREAD_MUTEX_INITIALIZER;
static struct job jobs[JOB_QUEUE];
static int job_head, job_tail;                  /* under job_lock */
static struct mesh_result results[JOB_QUEUE];
static int res_head, res_tail;                  /* under job_lock */

static struct frame frame;                      /* polygon list being drawn */
static uint8_t *back[2];                        /* back buffers in DDR */
static uint8_t *band_pixels;
static int band_next = NUM_BANDS;               /* next band to take */
static int band_done;                           /* bands finished */
static int present_buf = -1;                    /* buffer waiting for vblank */
static int buf_busy[2];                         /* buffer still being shown */
static int quit;                                /* atomic */

/* stats, written by the worker */
static int stat_presents, stat_copy_us;
static int fps_presents;                        /* presents, for the HUD counter */

static double now_ms(void)
{
    struct timespec ts;

    clock_gettime(CLOCK_MONOTONIC, &ts);
    return ts.tv_sec * 1e3 + ts.tv_nsec / 1e6;
}

static void pin_to_cpu(int cpu)
{
    cpu_set_t set;

    CPU_ZERO(&set);
    CPU_SET(cpu, &set);
    pthread_setaffinity_np(pthread_self(), sizeof(set), &set);
}

static void on_signal(int sig)
{
    int saved = errno;

    (void)sig;
    __atomic_store_n(&quit, 1, __ATOMIC_RELAXED);
    errno = saved;
}

static int quitting(void)
{
    return __atomic_load_n(&quit, __ATOMIC_RELAXED);
}

/* ---- jobs ---------------------------------------------------------------- */

/* main thread: queue a job (at the front if urgent), if there is room */
static int job_push(int type, struct chunk *c, int urgent)
{
    int ok = 0;

    pthread_mutex_lock(&job_lock);
    if ((job_tail + 1) % JOB_QUEUE != job_head) {
        if (urgent) {
            job_head = (job_head + JOB_QUEUE - 1) % JOB_QUEUE;
            jobs[job_head].type = type;
            jobs[job_head].chunk = c;
        } else {
            jobs[job_tail].type = type;
            jobs[job_tail].chunk = c;
            job_tail = (job_tail + 1) % JOB_QUEUE;
        }
        ok = 1;
    }
    pthread_mutex_unlock(&job_lock);
    if (ok)
        c->jobs++;
    return ok;
}

static int jobs_pending(void)
{
    int n;

    pthread_mutex_lock(&job_lock);
    n = (job_tail - job_head + JOB_QUEUE) % JOB_QUEUE;
    pthread_mutex_unlock(&job_lock);
    return n;
}

/* any thread: run one queued job; returns 0 if there was none */
static int job_run_one(void)
{
    struct job j;
    struct mesh_result r;

    pthread_mutex_lock(&job_lock);
    if (job_head == job_tail) {
        pthread_mutex_unlock(&job_lock);
        return 0;
    }
    j = jobs[job_head];
    job_head = (job_head + 1) % JOB_QUEUE;
    pthread_mutex_unlock(&job_lock);

    if (j.type == JOB_GENERATE)
        chunk_generate(j.chunk);
    chunk_mesh(j.chunk, &r);

    /* results never outnumber jobs, so this queue can't overflow */
    pthread_mutex_lock(&job_lock);
    results[res_tail] = r;
    res_tail = (res_tail + 1) % JOB_QUEUE;
    pthread_mutex_unlock(&job_lock);
    return 1;
}

/* main thread: get c a fresh mesh, now or as soon as its current job ends */
static void request_mesh(struct chunk *c, int urgent)
{
    if (!c)
        return;
    if (c->jobs > 0 || !job_push(JOB_MESH, c, urgent))
        c->dirty = 1;
}

/* Remesh c if a neighbour has appeared since its mesh was built. */
static void maybe_remesh(struct chunk *c)
{
    if (c && world_neighbours(c) != c->nbmask)
        request_mesh(c, 0);
}

/* main thread: install finished meshes */
static int integrate_results(void)
{
    int n = 0;

    for (;;) {
        struct mesh_result r;
        struct chunk *c;

        pthread_mutex_lock(&job_lock);
        if (res_head == res_tail) {
            pthread_mutex_unlock(&job_lock);
            break;
        }
        r = results[res_head];
        res_head = (res_head + 1) % JOB_QUEUE;
        pthread_mutex_unlock(&job_lock);

        c = r.chunk;
        mesh_free(&c->mesh);
        c->mesh = r.mesh;
        c->nbmask = r.nbmask;
        c->jobs--;
        n++;

        if (c->state == CH_LOADING) {
            __atomic_store_n(&c->state, CH_READY, __ATOMIC_RELEASE);
            maybe_remesh(world_chunk(c->cx - 1, c->cz));
            maybe_remesh(world_chunk(c->cx + 1, c->cz));
            maybe_remesh(world_chunk(c->cx, c->cz - 1));
            maybe_remesh(world_chunk(c->cx, c->cz + 1));
        }
        if (c->jobs == 0 && c->dirty) {
            c->dirty = 0;
            request_mesh(c, 1);
        } else {
            maybe_remesh(c);
        }
    }
    return n;
}

/* main thread: start loading the nearest missing chunks */
static void schedule_chunks(const float pos[3])
{
    int pcx = (int)floorf(pos[0] / CHUNK_SIZE), pcz = (int)floorf(pos[2] / CHUNK_SIZE);
    const int (*ring)[2];
    int nring = world_ring(&ring), i, queued = 0;

    for (i = 0; i < nring && queued < JOBS_PER_FRAME; i++) {
        int cx = pcx + ring[i][0], cz = pcz + ring[i][1];
        struct chunk *c = world_slot(cx, cz);

        if (c->state != CH_EMPTY && c->cx == cx && c->cz == cz)
            continue;               /* loaded or loading */
        if (c->jobs > 0)
            continue;               /* old occupant still has work queued */

        /* recycle the slot: whatever it held is out of range */
        __atomic_store_n(&c->state, CH_LOADING, __ATOMIC_RELEASE);
        c->cx = cx;
        c->cz = cz;
        c->nbmask = 0;
        c->dirty = 0;
        mesh_free(&c->mesh);
        if (!job_push(JOB_GENERATE, c, 0)) {
            c->state = CH_EMPTY;
            break;
        }
        queued++;
    }
}

/* main thread: remesh what a block change at b touched */
static void remesh_after_edit(const int b[3], int m)
{
    int cx = floor_div(b[0], CHUNK_SIZE), cz = floor_div(b[2], CHUNK_SIZE);

    if (m & 1)              request_mesh(world_chunk(cx, cz), 1);
    if (m & (NB_XN << 1))   request_mesh(world_chunk(cx - 1, cz), 1);
    if (m & (NB_XP << 1))   request_mesh(world_chunk(cx + 1, cz), 1);
    if (m & (NB_ZN << 1))   request_mesh(world_chunk(cx, cz - 1), 1);
    if (m & (NB_ZP << 1))   request_mesh(world_chunk(cx, cz + 1), 1);
}

/* main thread: break a block */
static void break_block(const int b[3])
{
    int blk = world_block(b[0], b[1], b[2]);

    if (blk <= 0 || blk == BLOCK_BEDROCK || blk == BLOCK_WATER)
        return;
    remesh_after_edit(b, world_set_block(b[0], b[1], b[2], BLOCK_AIR));
}

/*
 * main thread: put hotbar block slot into cell c (air or water), facing
 * stairs away from a player looking along yaw. feet is the player's
 * position, so nobody gets walled in; NULL to skip that check.
 */
static int place_block(const int c[3], int slot, float yaw, const float *feet)
{
    int cur = world_block(c[0], c[1], c[2]), b = hotbar_blocks[slot];

    if (cur != BLOCK_AIR && cur != BLOCK_WATER)
        return 0;
    if (feet && c[0] + 1 > feet[0] - 0.3f && c[0] < feet[0] + 0.3f &&
        c[1] + 1 > feet[1] && c[1] < feet[1] + 1.8f &&
        c[2] + 1 > feet[2] - 0.3f && c[2] < feet[2] + 0.3f)
        return 0;

    if (block_is_stairs(b)) {
        float fx = sinf(yaw), fz = -cosf(yaw);

        if (fabsf(fx) > fabsf(fz))
            b = fx > 0 ? BLOCK_STAIRS_XP : BLOCK_STAIRS_XN;
        else
            b = fz > 0 ? BLOCK_STAIRS_ZP : BLOCK_STAIRS_ZN;
    }
    remesh_after_edit(c, world_set_block(c[0], c[1], c[2], b));
    return 1;
}

/* ---- frame pipeline ------------------------------------------------------ */

/* any thread: render one band of the current frame, if any is left */
static int render_one_band(float *zbuf)
{
    int b;

    if (__atomic_load_n(&band_next, __ATOMIC_ACQUIRE) >= NUM_BANDS)
        return 0;
    b = __atomic_fetch_add(&band_next, 1, __ATOMIC_ACQ_REL);
    if (b >= NUM_BANDS)
        return 0;
    render_band(&frame, band_pixels, b, zbuf);
    __atomic_fetch_add(&band_done, 1, __ATOMIC_RELEASE);
    return 1;
}

static void *worker_main(void *arg)
{
    float *zbuf = aligned_alloc(64, BAND_H * SCREEN_W * sizeof(float));
    uint32_t armed_count = 0, seen_count = fpga_status() & FPGA_FRAMES;
    double vblank_t = now_ms();
    int armed = 0;

    (void)arg;
    pin_to_cpu(1);

    while (!quitting()) {
        int pb = __atomic_load_n(&present_buf, __ATOMIC_ACQUIRE);
        uint32_t count = fpga_status() & FPGA_FRAMES;
        double t = now_ms();

        if (count != seen_count) {
            seen_count = count;
            vblank_t = t;
        }

        /* present at the first vblank after the frame became ready */
        if (pb >= 0) {
            if (!armed) {
                armed = 1;
                armed_count = count;
            } else if (count != armed_count) {
                double t0 = now_ms();

                fpga_present(back[pb]);
                __atomic_fetch_add(&stat_copy_us, (int)((now_ms() - t0) * 1000), __ATOMIC_RELAXED);
                __atomic_fetch_add(&stat_presents, 1, __ATOMIC_RELAXED);
                __atomic_fetch_add(&fps_presents, 1, __ATOMIC_RELAXED);
                __atomic_store_n(&buf_busy[pb], 0, __ATOMIC_RELEASE);
                __atomic_store_n(&present_buf, -1, __ATOMIC_RELEASE);
                armed = 0;
                continue;
            }
        }

        if (render_one_band(zbuf))
            continue;

        /* chunk work, unless a present is due within the next few ms */
        if (pb < 0 || t - vblank_t < 16.68 - 4.0) {
            if (job_run_one())
                continue;
        }
        sched_yield();
    }
    free(zbuf);
    return NULL;
}

/* main thread: render the frame into buffer b with the worker's help */
static void render_frame(int b, float *zbuf)
{
    band_pixels = back[b];
    __atomic_store_n(&band_done, 0, __ATOMIC_RELAXED);
    __atomic_store_n(&band_next, 0, __ATOMIC_RELEASE);

    while (render_one_band(zbuf))
        ;
    while (__atomic_load_n(&band_done, __ATOMIC_ACQUIRE) < NUM_BANDS)
        ;
}

/* ---- player -------------------------------------------------------------- */

struct player {
    struct camera cam;          /* cam.pos is the eye */
    float feet[3];
    float vy;
    int on_ground, flying, in_water;
};

/*
 * Does the player's box with feet at p overlap anything solid? Unloaded
 * chunks count as solid, so nobody falls through the world while it loads.
 */
static int box_blocked(const float p[3])
{
    const float e = 1e-4f;
    int x0 = (int)floorf(p[0] - PLAYER_R), x1 = (int)floorf(p[0] + PLAYER_R - e);
    int y0 = (int)floorf(p[1]), y1 = (int)floorf(p[1] + PLAYER_H - e);
    int z0 = (int)floorf(p[2] - PLAYER_R), z1 = (int)floorf(p[2] + PLAYER_R - e);
    int x, y, z;

    for (y = y0; y <= y1; y++)
        for (z = z0; z <= z1; z++)
            for (x = x0; x <= x1; x++) {
                int b = world_block(x, y, z);

                if (b < 0 || block_solid(b))
                    return 1;
            }
    return 0;
}

/* Move along one horizontal axis, stepping up single blocks on the ground. */
static void move_axis(struct player *pl, int axis, float d)
{
    float p[3], up[3];

    if (d == 0.0f)
        return;
    memcpy(p, pl->feet, sizeof(p));
    p[axis] += d;
    if (!box_blocked(p)) {
        pl->feet[axis] = p[axis];
        return;
    }
    if (pl->on_ground && !pl->flying) {
        memcpy(up, pl->feet, sizeof(up));
        up[1] = floorf(pl->feet[1]) + 1.0f;
        p[1] = up[1];
        if (!box_blocked(up) && !box_blocked(p))
            memcpy(pl->feet, p, sizeof(p));
    }
}

static void update_player(struct player *pl, struct input *in, float dt)
{
    struct camera *c = &pl->cam;
    float fwd, strafe, speed, sy, cy, dx, dz, dy;
    int b, steps, i;

    /* looking */
    c->yaw += TURN_SPEED * dt * (in->key[KEY_E] - in->key[KEY_Q] +
                                 in->key[KEY_RIGHT] - in->key[KEY_LEFT]);
    c->pitch += TURN_SPEED * dt * (in->key[KEY_UP] - in->key[KEY_DOWN]);
    c->yaw += MOUSE_SENS * in->mouse_dx + TTY_TURN * in->t_turn;
    c->pitch += -MOUSE_SENS * in->mouse_dy + TTY_TURN * in->t_look;
    if (c->pitch > 1.55f) c->pitch = 1.55f;
    if (c->pitch < -1.55f) c->pitch = -1.55f;
    c->yaw = fmodf(c->yaw, 2.0f * (float)M_PI);

    if (in->t_fly) {
        pl->flying = !pl->flying;
        pl->vy = 0;
    }

    /* nothing to stand on until the chunk underfoot has loaded */
    if (world_block((int)floorf(pl->feet[0]), 0, (int)floorf(pl->feet[2])) < 0)
        goto out;

    b = world_block((int)floorf(pl->feet[0]), (int)floorf(pl->feet[1] + 0.4f),
                    (int)floorf(pl->feet[2]));
    pl->in_water = (b == BLOCK_WATER);

    /* moving, relative to where we face */
    fwd = (float)(in->key[KEY_W] - in->key[KEY_S]);
    strafe = (float)(in->key[KEY_D] - in->key[KEY_A]);
    speed = pl->flying ? FLY_SPEED
          : (in->key[KEY_LEFTCTRL] || in->key[KEY_RIGHTCTRL]) ? SPRINT_SPEED : WALK_SPEED;
    if (pl->in_water && !pl->flying)
        speed *= SWIM_FACTOR;
    if (fwd != 0 && strafe != 0) {
        fwd *= 0.7071f;
        strafe *= 0.7071f;
    }
    sy = sinf(c->yaw);
    cy = cosf(c->yaw);
    dx = (fwd * sy + strafe * cy) * speed * dt + TTY_STEP * (in->t_move * sy + in->t_strafe * cy);
    dz = (-fwd * cy + strafe * sy) * speed * dt + TTY_STEP * (-in->t_move * cy + in->t_strafe * sy);

    /* vertical speed */
    if (pl->flying) {
        int up = in->key[KEY_SPACE] - (in->key[KEY_LEFTSHIFT] || in->key[KEY_RIGHTSHIFT]);

        pl->vy = up * FLY_SPEED;
    } else if (pl->in_water) {
        pl->vy -= WATER_GRAVITY * dt;
        if (pl->vy < -WATER_SINK)
            pl->vy = -WATER_SINK;
        if (in->key[KEY_SPACE] || in->t_jump)
            pl->vy = SWIM_UP_SPEED;
    } else {
        if ((in->key[KEY_SPACE] || in->t_jump) && pl->on_ground)
            pl->vy = JUMP_SPEED;
        pl->vy -= GRAVITY * dt;
        if (pl->vy < -50.0f)
            pl->vy = -50.0f;
    }
    dy = pl->vy * dt;

    /* small steps so nothing tunnels through a block */
    steps = (int)ceilf(fmaxf(fmaxf(fabsf(dx), fabsf(dz)), fabsf(dy)) / 0.4f);
    if (steps < 1)
        steps = 1;
    for (i = 0; i < steps; i++) {
        float p[3];

        move_axis(pl, 0, dx / steps);
        move_axis(pl, 2, dz / steps);

        memcpy(p, pl->feet, sizeof(p));
        p[1] += dy / steps;
        if (!box_blocked(p)) {
            pl->feet[1] = p[1];
            pl->on_ground = 0;
        } else {
            if (dy < 0.0f) {
                pl->feet[1] = floorf(p[1]) + 1.0f;     /* land on the block */
                pl->on_ground = 1;
            } else {
                pl->feet[1] = floorf(p[1] + PLAYER_H) - PLAYER_H;
            }
            pl->vy = 0.0f;
            dy = 0.0f;
        }
    }
    if (pl->feet[1] > CHUNK_H + 64)
        pl->feet[1] = CHUNK_H + 64;

out:
    c->pos[0] = pl->feet[0];
    c->pos[1] = pl->feet[1] + EYE_HEIGHT;
    c->pos[2] = pl->feet[2];

    in->mouse_dx = in->mouse_dy = 0;
    in->t_move = in->t_strafe = in->t_turn = in->t_look = 0;
    in->t_jump = in->t_fly = 0;
}

static void view_dir(const struct camera *c, float d[3])
{
    d[0] = cosf(c->pitch) * sinf(c->yaw);
    d[1] = sinf(c->pitch);
    d[2] = -cosf(c->pitch) * cosf(c->yaw);
}

static int eye_in_water(const struct camera *c)
{
    return world_block((int)floorf(c->pos[0]), (int)floorf(c->pos[1]),
                       (int)floorf(c->pos[2])) == BLOCK_WATER;
}

/* Is (x, z) dry land with no tree (or tree canopy) within 2 blocks? */
static int open_ground(int x, int z)
{
    struct terrain_column col;
    int dx, dz;

    for (dz = -2; dz <= 2; dz++) {
        for (dx = -2; dx <= 2; dx++) {
            terrain_column_at(x + dx, z + dz, &col);
            if (col.water > col.height || col.feature != FEATURE_NONE)
                return 0;
        }
    }
    return 1;
}

/* Find open dry land near the origin to start on. */
static void find_spawn(float feet[3])
{
    int r, x, z;

    for (r = 0; r < 1024; r += 4) {
        for (z = -r; z <= r; z += 4) {
            for (x = -r; x <= r; x += 4) {
                struct terrain_column col;

                if ((abs(x) != r && abs(z) != r) || !open_ground(x, z))
                    continue;
                terrain_column_at(x, z, &col);
                feet[0] = x + 0.5f;
                feet[1] = (float)col.height;
                feet[2] = z + 0.5f;
                return;
            }
        }
    }
    feet[0] = 0.5f;
    feet[1] = CHUNK_H - 20;
    feet[2] = 0.5f;
}

/* ---- weather ------------------------------------------------------------- */

struct weather_state {
    int shown;                  /* weather being faded to (or shown) */
    int pending;                /* weather under the player, not yet taken */
    float pending_t;
    struct env from, to, cur;   /* cur: what the tables show right now */
    float t;                    /* 0..1 through the fade */
    int step;                   /* last table rebuild, in WEATHER_STEPS */
};

static int parse_weather(const char *name)
{
    int w;

    for (w = 0; w < NUM_WEATHERS; w++) {
        const char *n = weather_name(w);
        int i;

        for (i = 0; n[i] && (n[i] | 0x20) == (name[i] | 0x20); i++)
            ;
        if (!n[i] && !name[i])
            return w;
    }
    return -1;
}

static void weather_start(struct weather_state *ws, int w)
{
    memset(ws, 0, sizeof(*ws));
    ws->shown = ws->pending = w;
    env_for_weather(w, &ws->to);
    ws->from = ws->cur = ws->to;
    ws->t = 1.0f;
    ws->step = WEATHER_STEPS;
    render_set_env(&ws->to);
}

/*
 * Follow the wanted weather (forced, or the terrain's under the player).
 * Fades take WEATHER_FADE seconds; the colour tables are rebuilt only at
 * WEATHER_STEPS points along the way, as that is the costly part.
 */
static void weather_update(struct weather_state *ws, int want, float dt)
{
    if (want >= 0 && want != ws->shown) {
        if (want != ws->pending) {
            ws->pending = want;
            ws->pending_t = 0.0f;
        }
        ws->pending_t += dt;
        if (ws->pending_t >= WEATHER_SETTLE) {
            /* start from whatever is on screen now */
            ws->from = ws->cur;
            env_for_weather(want, &ws->to);
            ws->shown = want;
            ws->t = 0.0f;
            ws->step = 0;
        }
    } else {
        ws->pending = ws->shown;
    }

    if (ws->t < 1.0f) {
        int step;

        ws->t += dt / WEATHER_FADE;
        if (ws->t > 1.0f)
            ws->t = 1.0f;
        step = (int)(ws->t * WEATHER_STEPS);
        if (step != ws->step) {
            float s = (float)step / WEATHER_STEPS;

            env_mix(&ws->cur, &ws->from, &ws->to, s * s * (3.0f - 2.0f * s));
            render_set_env(&ws->cur);
            ws->step = step;
        }
    }
}

/* ---- modes --------------------------------------------------------------- */

static void run_all_jobs(void)
{
    int k;

    for (k = 0; k < 2; k++) {
        while (job_run_one())
            ;
        integrate_results();
    }
}

static int screenshot(const struct camera *cam, const char *path, int breaks, int build,
                      int weather)
{
    float *zbuf = aligned_alloc(64, BAND_H * SCREEN_W * sizeof(float));
    float dir[3];
    int b, guard, hit[3], prev[3], has_hit;
    double t0, t1;

    /* load everything in range, running the jobs on this thread */
    t0 = now_ms();
    for (guard = 0; guard < 100000; guard++) {
        const int (*ring)[2];
        int i, n, missing = 0;
        int pcx = (int)floorf(cam->pos[0] / CHUNK_SIZE);
        int pcz = (int)floorf(cam->pos[2] / CHUNK_SIZE);

        schedule_chunks(cam->pos);
        while (job_run_one())
            ;
        integrate_results();
        n = world_ring(&ring);
        for (i = 0; i < n; i++)
            if (!world_chunk(pcx + ring[i][0], pcz + ring[i][1]))
                missing++;
        if (!missing && jobs_pending() == 0)
            break;
    }
    t1 = now_ms();

    view_dir(cam, dir);
    has_hit = world_raycast(cam->pos, dir, REACH, hit, prev);

    /* test hook: break the targeted block a few times, like holding B */
    for (; breaks > 0 && has_hit; breaks--) {
        printf("breaking block %d %d %d (type %d)\n", hit[0], hit[1], hit[2],
               world_block(hit[0], hit[1], hit[2]));
        break_block(hit);
        run_all_jobs();
        has_hit = world_raycast(cam->pos, dir, REACH, hit, prev);
    }

    /* test hook: one of each hotbar block on the ground ahead, the glass
     * and stairs doubled up so their shapes show */
    if (build) {
        float fx = sinf(cam->yaw), fz = -cosf(cam->yaw);
        int i, k;

        for (i = 0; i < HOTBAR_SLOTS; i++) {
            for (k = 0; k < ((i == 3 || i == 6) ? 2 : 1); k++) {
                float side = (i - (HOTBAR_SLOTS - 1) * 0.5f) * 1.6f;
                int c[3], y;

                c[0] = (int)floorf(cam->pos[0] + fx * 8 + fz * -side + (i == 6 ? fx * k : 0));
                c[2] = (int)floorf(cam->pos[2] + fz * 8 + fx * side + (i == 6 ? fz * k : 0));
                for (y = CHUNK_H - 1; y > 0 && !block_solid(world_block(c[0], y - 1, c[2])); y--)
                    ;
                c[1] = y + (i == 3 ? k : 0);
                place_block(c, i, cam->yaw, NULL);
            }
        }
        run_all_jobs();
        has_hit = world_raycast(cam->pos, dir, REACH, hit, prev);
    }

    {
        struct weather_state ws;
        int w = weather >= 0 ? weather
              : world_weather((int)floorf(cam->pos[0]), (int)floorf(cam->pos[2]));

        weather_start(&ws, w < 0 ? WEATHER_SUNNY : w);
        weather = ws.shown;
    }
    for (guard = 0; guard < 20; guard++) {      /* repeat for stable timing */
        struct view vw = { has_hit ? hit : NULL, eye_in_water(cam), 0, weather, 1.0f, 60 };
        double t2;

        t1 = now_ms();
        render_setup(&frame, cam, &vw);
        t2 = now_ms();
        for (b = 0; b < NUM_BANDS; b++)
            render_band(&frame, back[0], b, zbuf);
        if (guard == 19)
            printf("loaded in %.0f ms; frame: %d polygons from %d chunks, "
                   "setup %.2f ms + draw %.2f ms (one thread)\n",
                   t1 - t0, frame.npolys, frame.nchunks, t2 - t1, now_ms() - t2);
    }
    free(zbuf);
    return write_ppm(path, back[0]);
}

static void play(int autopilot, int forced_weather)
{
    struct player pl;
    struct input in;
    float *zbuf = aligned_alloc(64, BAND_H * SCREEN_W * sizeof(float));
    pthread_t worker;
    double last, stat_t, stat_work = 0;
    float break_timer = 0.0f, place_timer = 0.0f, game_time = 0.0f;
    int cur = 0, stat_frames = 0, prev_f = 0, prev_y = 0, slot = 0;
    struct weather_state ws;
    double fps_t = 0;               /* FPS counter: last sample time */
    int fps_shown = -1;             /* -1 until the first sample */

    memset(&pl, 0, sizeof(pl));
    find_spawn(pl.feet);
    pl.cam.pitch = -0.15f;

    input_open(&in);
    printf("WASD move, Q/E turn, arrows/mouse look, Space jump, Ctrl sprint, F fly,\n"
           "B/left click break, P/right click place, 1-7/wheel pick block,\n"
           "Y cycle weather, Esc quit\n");
    weather_start(&ws, forced_weather >= 0 ? forced_weather : WEATHER_SUNNY);
    printf("cores online: %ld, spawn %.1f %.1f %.1f\n", sysconf(_SC_NPROCESSORS_ONLN),
           pl.feet[0], pl.feet[1], pl.feet[2]);
    fflush(stdout);

    pin_to_cpu(0);
    pthread_create(&worker, NULL, worker_main, NULL);

    last = stat_t = now_ms();
    while (!quitting() && !in.quit) {
        double t = now_ms(), w0;
        float dt = (float)((t - last) / 1000.0), dir[3];
        int hit[3], prev[3], has_hit, want_break, want_place, k;

        last = t;
        if (dt > 0.1f)
            dt = 0.1f;
        game_time += dt;

        /* FPS counter: frames actually shown, averaged over half a second */
        if (t - fps_t >= 500.0) {
            int n = __atomic_exchange_n(&fps_presents, 0, __ATOMIC_RELAXED);

            if (fps_t > 0)
                fps_shown = (int)(n * 1000.0 / (t - fps_t) + 0.5);
            fps_t = t;
        }

        input_poll(&in);

        /* Y: follow the terrain's weather -> sunny -> ... -> rain -> follow */
        if (in.key[KEY_Y] && !prev_y) {
            forced_weather = forced_weather + 1 >= NUM_WEATHERS ? -1 : forced_weather + 1;
            printf("weather: %s\n", forced_weather < 0 ? "from terrain"
                                                        : weather_name(forced_weather));
            fflush(stdout);
        }
        prev_y = in.key[KEY_Y];
        if (autopilot) {
            in.key[KEY_W] = 1;
            pl.cam.yaw += 0.15f * dt;
        }
        if (in.key[KEY_F] && !prev_f)       /* F: toggle flying on press */
            in.t_fly = 1;
        prev_f = in.key[KEY_F];
        want_break = in.key[KEY_B] || in.key[KEY_ENTER] || in.key[BTN_LEFT];
        if (in.t_break) {
            want_break = 1;
            break_timer = 0.0f;
            in.t_break = 0;
        }
        want_place = in.key[KEY_P] || in.key[BTN_RIGHT];
        if (in.t_place) {
            want_place = 1;
            place_timer = 0.0f;
            in.t_place = 0;
        }

        /* hotbar: number keys or the mouse wheel */
        for (k = 0; k < HOTBAR_SLOTS; k++)
            if (in.key[KEY_1 + k])
                slot = k;
        if (in.t_slot) {
            slot = in.t_slot - 1;
            in.t_slot = 0;
        }
        if (in.wheel) {
            slot = ((slot - in.wheel) % HOTBAR_SLOTS + HOTBAR_SLOTS) % HOTBAR_SLOTS;
            in.wheel = 0;
        }

        update_player(&pl, &in, dt);
        integrate_results();

        /* the block under the crosshair: break it, or place against it */
        view_dir(&pl.cam, dir);
        has_hit = world_raycast(pl.cam.pos, dir, REACH, hit, prev);
        if (want_break && has_hit) {
            break_timer -= dt;
            if (break_timer <= 0.0f) {
                break_block(hit);
                break_timer = BREAK_REPEAT;
                has_hit = world_raycast(pl.cam.pos, dir, REACH, hit, prev);
            }
        } else if (!want_break) {
            break_timer = 0.0f;
        }
        if (want_place && has_hit) {
            place_timer -= dt;
            if (place_timer <= 0.0f) {
                place_block(prev, slot, pl.cam.yaw, pl.feet);
                place_timer = BREAK_REPEAT;
                has_hit = world_raycast(pl.cam.pos, dir, REACH, hit, prev);
            }
        } else if (!want_place) {
            place_timer = 0.0f;
        }

        schedule_chunks(pl.cam.pos);

        weather_update(&ws, forced_weather >= 0 ? forced_weather
                            : world_weather((int)floorf(pl.feet[0]), (int)floorf(pl.feet[2])),
                       dt);

        /* wait until this buffer has been shown */
        while (__atomic_load_n(&buf_busy[cur], __ATOMIC_ACQUIRE) && !quitting())
            sched_yield();

        w0 = now_ms();
        {
            struct view vw = { has_hit ? hit : NULL, eye_in_water(&pl.cam), slot,
                               ws.shown, game_time, fps_shown };

            render_setup(&frame, &pl.cam, &vw);
        }
        render_frame(cur, zbuf);
        stat_work += now_ms() - w0;

        /* hand it to the worker for the next vblank */
        while (__atomic_load_n(&present_buf, __ATOMIC_ACQUIRE) >= 0 && !quitting())
            sched_yield();
        __atomic_store_n(&buf_busy[cur], 1, __ATOMIC_RELAXED);
        __atomic_store_n(&present_buf, cur, __ATOMIC_RELEASE);
        cur ^= 1;
        stat_frames++;

        if (t - stat_t >= 2000.0) {
            int pres = __atomic_exchange_n(&stat_presents, 0, __ATOMIC_RELAXED);
            int cus = __atomic_exchange_n(&stat_copy_us, 0, __ATOMIC_RELAXED);
            double secs = (t - stat_t) / 1000.0;

            printf("%5.1f fps | setup+draw %4.1f ms | copy %4.1f ms | %4d polys %3d chunks"
                   " | pos %.1f %.1f %.1f | %s%s%s\n",
                   pres / secs, stat_work / stat_frames,
                   pres ? cus / 1000.0 / pres : 0.0, frame.npolys, frame.nchunks,
                   pl.feet[0], pl.feet[1], pl.feet[2], weather_name(ws.shown),
                   pl.flying ? " flying" : "", pl.in_water ? " swimming" : "");
            fflush(stdout);
            stat_t = t;
            stat_work = 0;
            stat_frames = 0;
        }
    }

    __atomic_store_n(&quit, 1, __ATOMIC_RELAXED);
    pthread_join(worker, NULL);
    input_close(&in);
    free(zbuf);
}

/* test hook: stand at (x, z), hold W for secs, report what happens */
static void physics_test(float x, float z, float yaw_deg, float secs)
{
    struct player pl;
    struct input in;
    struct terrain_column col;
    int i, n = (int)(secs * 60.0f);

    memset(&pl, 0, sizeof(pl));
    memset(&in, 0, sizeof(in));
    terrain_column_at((int)floorf(x), (int)floorf(z), &col);
    pl.feet[0] = x;
    pl.feet[1] = (float)(col.water > col.height ? col.water : col.height);
    pl.feet[2] = z;
    pl.cam.yaw = yaw_deg * (float)M_PI / 180.0f;

    for (i = 0; i < 1000; i++) {            /* load the area */
        schedule_chunks(pl.feet);
        while (job_run_one())
            ;
        integrate_results();
    }
    in.key[KEY_W] = 1;
    for (i = 0; i <= n; i++) {
        if (i % 15 == 0)
            printf("t=%.2fs feet %.2f %.2f %.2f%s%s vy %.1f\n", i / 60.0f,
                   pl.feet[0], pl.feet[1], pl.feet[2], pl.on_ground ? " ground" : "",
                   pl.in_water ? " water" : "", pl.vy);
        update_player(&pl, &in, 1.0f / 60.0f);
    }
}

static void usage(void)
{
    fprintf(stderr,
            "usage: mc [--terrain FILE] [--dist N] [--weather NAME] [--auto]\n"
            "       mc [--terrain FILE] [--dist N] [--weather NAME] [--break N] [--build]\n"
            "          --shot X Y Z YAW PITCH out.ppm\n");
}

int main(int argc, char **argv)
{
    const char *terrain_path = "terrain.bin", *shot_path = NULL;
    struct camera shot_cam;
    int dist = RENDER_DIST_DEFAULT, autopilot = 0, breaks = 0, build = 0, i, rc = 0;
    int weather = -1;

    for (i = 1; i < argc; i++) {
        if (strcmp(argv[i], "--terrain") == 0 && i + 1 < argc) {
            terrain_path = argv[++i];
        } else if (strcmp(argv[i], "--dist") == 0 && i + 1 < argc) {
            dist = atoi(argv[++i]);
        } else if (strcmp(argv[i], "--break") == 0 && i + 1 < argc) {
            breaks = atoi(argv[++i]);
        } else if (strcmp(argv[i], "--physics") == 0 && i + 4 < argc) {
            float x = strtof(argv[i + 1], NULL), z = strtof(argv[i + 2], NULL);
            float yaw = strtof(argv[i + 3], NULL), secs = strtof(argv[i + 4], NULL);

            terrain_open(terrain_path);
            world_init(dist);
            physics_test(x, z, yaw, secs);
            return 0;
        } else if (strcmp(argv[i], "--weather") == 0 && i + 1 < argc) {
            weather = parse_weather(argv[++i]);
            if (weather < 0) {
                fprintf(stderr, "unknown weather %s\n", argv[i]);
                return 2;
            }
        } else if (strcmp(argv[i], "--build") == 0) {
            build = 1;
        } else if (strcmp(argv[i], "--auto") == 0) {
            autopilot = 1;
        } else if (strcmp(argv[i], "--shot") == 0 && i + 6 < argc) {
            shot_cam.pos[0] = strtof(argv[++i], NULL);
            shot_cam.pos[1] = strtof(argv[++i], NULL);
            shot_cam.pos[2] = strtof(argv[++i], NULL);
            shot_cam.yaw = strtof(argv[++i], NULL) * (float)M_PI / 180.0f;
            shot_cam.pitch = strtof(argv[++i], NULL) * (float)M_PI / 180.0f;
            shot_path = argv[++i];
        } else {
            usage();
            return 2;
        }
    }

    terrain_open(terrain_path);
    if (world_init(dist) < 0)
        return 1;
    render_init(world_render_dist());
    for (i = 0; i < 2; i++) {
        back[i] = aligned_alloc(4096, SCREEN_SIZE);     /* page aligned, for the DMA */
        if (!back[i])
            return 1;
        memset(back[i], 0, SCREEN_SIZE);
    }

    if (shot_path) {
        rc = screenshot(&shot_cam, shot_path, breaks, build, weather) ? 1 : 0;
    } else {
        if (fpga_open() < 0)
            return 1;
        signal(SIGINT, on_signal);
        signal(SIGTERM, on_signal);
        fpga_setup_dma(back, 2);        /* the FPGA fetches frames itself, if it can */
        fpga_pick_copy(back[0]);        /* also clears the screen */
        play(autopilot, weather);
        fpga_close();
    }

    for (i = 0; i < 2; i++)
        free(back[i]);
    world_free();
    terrain_close();
    return rc;
}
