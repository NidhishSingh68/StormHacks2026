/*
 * world.c - chunk storage, terrain building, greedy meshing, block edits
 *
 * Threading: the main thread owns chunk state, slots and meshes. The worker
 * thread fills a chunk's blocks only while it is CH_LOADING (the renderer
 * and physics ignore such chunks) and builds meshes into fresh arrays that
 * the main thread then swaps in. After loading, only the main thread writes
 * blocks (when one is broken); a mesh being built at the same moment may
 * see the old or new byte, and is rebuilt anyway because every edit queues
 * a remesh of the chunks it touches.
 */

#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "world.h"
#include "terrain.h"

static struct chunk *grid;          /* CHUNK_GRID * CHUNK_GRID slots */

#define RING_MAX ((2 * RENDER_DIST_MAX + 1) * (2 * RENDER_DIST_MAX + 1))
static int ring[RING_MAX][2];
static int nring, render_dist;

static int ring_cmp(const void *a, const void *b)
{
    const int *p = a, *q = b;

    return (p[0] * p[0] + p[1] * p[1]) - (q[0] * q[0] + q[1] * q[1]);
}

int world_init(int dist)
{
    int i, dx, dz, r2;

    if (dist < 2) dist = 2;
    if (dist > RENDER_DIST_MAX) dist = RENDER_DIST_MAX;
    render_dist = dist;

    grid = calloc(CHUNK_GRID * CHUNK_GRID, sizeof(*grid));
    if (!grid)
        return -1;
    for (i = 0; i < CHUNK_GRID * CHUNK_GRID; i++)
        grid[i].state = CH_EMPTY;

    /* circle of radius dist + 0.5 chunks, nearest first */
    r2 = dist * dist + dist;
    nring = 0;
    for (dz = -dist; dz <= dist; dz++) {
        for (dx = -dist; dx <= dist; dx++) {
            if (dx * dx + dz * dz > r2)
                continue;
            ring[nring][0] = dx;
            ring[nring][1] = dz;
            nring++;
        }
    }
    qsort(ring, nring, sizeof(ring[0]), ring_cmp);
    return 0;
}

void world_free(void)
{
    int i;

    for (i = 0; i < CHUNK_GRID * CHUNK_GRID; i++)
        mesh_free(&grid[i].mesh);
    free(grid);
}

int world_render_dist(void)
{
    return render_dist;
}

int world_ring(const int (**offsets)[2])
{
    *offsets = (const int (*)[2])ring;
    return nring;
}

struct chunk *world_slot(int cx, int cz)
{
    return &grid[(cz & (CHUNK_GRID - 1)) * CHUNK_GRID + (cx & (CHUNK_GRID - 1))];
}

struct chunk *world_chunk(int cx, int cz)
{
    struct chunk *c = world_slot(cx, cz);

    if (__atomic_load_n(&c->state, __ATOMIC_ACQUIRE) != CH_READY)
        return NULL;
    if (c->cx != cx || c->cz != cz)
        return NULL;
    return c;
}

int world_block(int x, int y, int z)
{
    int cx = floor_div(x, CHUNK_SIZE), cz = floor_div(z, CHUNK_SIZE);
    struct chunk *c;

    if (y < 0)
        return BLOCK_BEDROCK;
    if (y >= CHUNK_H)
        return BLOCK_AIR;
    c = world_chunk(cx, cz);
    if (!c)
        return -1;
    return c->blocks[y][z - cz * CHUNK_SIZE][x - cx * CHUNK_SIZE];
}

int world_weather(int x, int z)
{
    int cx = floor_div(x, CHUNK_SIZE), cz = floor_div(z, CHUNK_SIZE);
    struct chunk *c = world_chunk(cx, cz);

    return c ? c->weather[z - cz * CHUNK_SIZE][x - cx * CHUNK_SIZE] : -1;
}

uint8_t world_neighbours(const struct chunk *c)
{
    uint8_t m = 0;

    if (world_chunk(c->cx - 1, c->cz)) m |= NB_XN;
    if (world_chunk(c->cx + 1, c->cz)) m |= NB_XP;
    if (world_chunk(c->cx, c->cz - 1)) m |= NB_ZN;
    if (world_chunk(c->cx, c->cz + 1)) m |= NB_ZP;
    return m;
}

int world_set_block(int x, int y, int z, int block)
{
    int cx = floor_div(x, CHUNK_SIZE), cz = floor_div(z, CHUNK_SIZE);
    int lx = x - cx * CHUNK_SIZE, lz = z - cz * CHUNK_SIZE;
    struct chunk *c = world_chunk(cx, cz), *n;
    int remesh = 1;

    if (!c || y < 0 || y >= CHUNK_H)
        return 0;
    c->blocks[y][lz][lx] = (uint8_t)block;

    /* faces can now appear one block below and beside it */
    if (y - 1 < c->ylo)
        c->ylo = y > 0 ? y - 1 : 0;
    if (y + 1 > c->yhi)
        c->yhi = y + 1;

#define TOUCH(cond, dx, dz, bit)                                            \
    if ((cond) && (n = world_chunk(cx + (dx), cz + (dz))) != NULL) {        \
        if (y < n->ylo) n->ylo = y;                                         \
        remesh |= (bit) << 1;                                               \
    }
    TOUCH(lx == 0, -1, 0, NB_XN);
    TOUCH(lx == CHUNK_SIZE - 1, 1, 0, NB_XP);
    TOUCH(lz == 0, 0, -1, NB_ZN);
    TOUCH(lz == CHUNK_SIZE - 1, 0, 1, NB_ZP);
#undef TOUCH
    return remesh;
}

int world_raycast(const float o[3], const float d[3], float max_dist,
                  int hit[3], int prev[3])
{
    int p[3], step[3], i;
    float tmax[3], tdelta[3], len, t = 0.0f;

    len = sqrtf(d[0] * d[0] + d[1] * d[1] + d[2] * d[2]);
    if (len < 1e-6f)
        return 0;
    for (i = 0; i < 3; i++)
        prev[i] = (int)floorf(o[i]);

    for (i = 0; i < 3; i++) {
        float dir = d[i] / len;

        p[i] = (int)floorf(o[i]);
        if (dir > 0.0f) {
            step[i] = 1;
            tdelta[i] = 1.0f / dir;
            tmax[i] = (p[i] + 1 - o[i]) / dir;
        } else if (dir < 0.0f) {
            step[i] = -1;
            tdelta[i] = -1.0f / dir;
            tmax[i] = (o[i] - p[i]) / -dir;
        } else {
            step[i] = 0;
            tdelta[i] = tmax[i] = 1e30f;
        }
    }

    /* Amanatides & Woo voxel traversal */
    while (t <= max_dist) {
        int b = world_block(p[0], p[1], p[2]);

        if (b < 0)
            return 0;               /* ran into an unloaded chunk */
        if (b != BLOCK_AIR && b != BLOCK_WATER) {
            hit[0] = p[0]; hit[1] = p[1]; hit[2] = p[2];
            return 1;
        }
        prev[0] = p[0]; prev[1] = p[1]; prev[2] = p[2];
        i = (tmax[0] < tmax[1]) ? (tmax[0] < tmax[2] ? 0 : 2)
                                : (tmax[1] < tmax[2] ? 1 : 2);
        t = tmax[i];
        tmax[i] += tdelta[i];
        p[i] += step[i];
    }
    return 0;
}

/* ---- generation ---------------------------------------------------------- */

static uint32_t hash2(int x, int z)
{
    uint32_t h = (uint32_t)x * 0x9E3779B1u + (uint32_t)z * 0x85EBCA77u;

    h ^= h >> 15;
    h *= 0x2C1B3C6Du;
    h ^= h >> 12;
    return h;
}

static void fill_column(struct chunk *c, int x, int z, const struct terrain_column *col)
{
    int h = col->height, w = col->water, s = col->surface, y;
    int sub = (s == BLOCK_SAND) ? BLOCK_SAND
            : (s == BLOCK_STONE) ? BLOCK_STONE : BLOCK_DIRT;

    c->blocks[0][z][x] = BLOCK_BEDROCK;
    for (y = 1; y < h; y++) {
        int depth = h - 1 - y;

        c->blocks[y][z][x] = (uint8_t)(depth == 0 ? s : depth <= 3 ? sub : BLOCK_STONE);
    }
    for (y = h; y < w; y++)
        c->blocks[y][z][x] = BLOCK_WATER;
}

/* Put block b at world (x, y, z) if it falls inside chunk c. */
static void put(struct chunk *c, int x, int y, int z, int b, int only_air)
{
    int lx = x - c->cx * CHUNK_SIZE, lz = z - c->cz * CHUNK_SIZE;

    if (lx < 0 || lx >= CHUNK_SIZE || lz < 0 || lz >= CHUNK_SIZE ||
        y < 0 || y >= CHUNK_H)
        return;
    if (only_air && c->blocks[y][lz][lx] != BLOCK_AIR)
        return;
    c->blocks[y][lz][lx] = (uint8_t)b;
    if (y + 1 > c->yhi)
        c->yhi = y + 1;
}

/* An oak-like tree standing on column (x, z) with ground height h. */
static void place_tree(struct chunk *c, int x, int z, int h)
{
    uint32_t r = hash2(x, z);
    int th = 4 + (int)(r % 3), top = h + th, y, dx, dz;

    if (top + 2 >= CHUNK_H)
        return;

    for (y = top - 3; y <= top; y++) {
        int rad = (y >= top - 1) ? 1 : 2;

        for (dz = -rad; dz <= rad; dz++) {
            for (dx = -rad; dx <= rad; dx++) {
                int corner = (dx == -rad || dx == rad) && (dz == -rad || dz == rad);

                /* round the canopy: drop some corners, always at the top */
                if (corner && (y == top ||
                               ((r >> (((dx + 2) * 5 + (dz + 2) + (top - y) * 7) & 31)) & 1)))
                    continue;
                put(c, x + dx, y, z + dz, BLOCK_LEAVES, 1);
            }
        }
    }
    for (y = h; y < top; y++)
        put(c, x, y, z, BLOCK_LOG, 0);
}

void chunk_generate(struct chunk *c)
{
    static __thread struct terrain_column tiles[3][3][TERRAIN_TILE_COLS];
    int x, z, tx, tz, lo = CHUNK_H;

    for (tz = 0; tz < 3; tz++)
        for (tx = 0; tx < 3; tx++)
            terrain_tile(c->cx + tx - 1, c->cz + tz - 1, tiles[tz][tx]);

    memset(c->blocks, BLOCK_AIR, sizeof(c->blocks));
    c->yhi = 0;

    for (z = 0; z < CHUNK_SIZE; z++) {
        for (x = 0; x < CHUNK_SIZE; x++) {
            const struct terrain_column *col = &tiles[1][1][z * TERRAIN_TILE + x];
            int top = col->water > col->height ? col->water : col->height;

            fill_column(c, x, z, col);
            c->weather[z][x] = col->weather;
            if (top > c->yhi)
                c->yhi = top;
        }
    }

    /*
     * Lowest possible face: one below the lowest ground among this chunk's
     * columns and the neighbouring columns around it (lake beds show
     * through the water).
     */
    for (z = -1; z <= CHUNK_SIZE; z++) {
        for (x = -1; x <= CHUNK_SIZE; x++) {
            const struct terrain_column *col =
                &tiles[(z + TERRAIN_TILE) / TERRAIN_TILE][(x + TERRAIN_TILE) / TERRAIN_TILE]
                      [((z + TERRAIN_TILE) % TERRAIN_TILE) * TERRAIN_TILE +
                       (x + TERRAIN_TILE) % TERRAIN_TILE];

            if (col->height < lo)
                lo = col->height;
        }
    }
    c->ylo = lo > 1 ? lo - 1 : 0;

    /* trees from this tile and the edges of its neighbours (canopy radius 2) */
    for (tz = 0; tz < 3; tz++) {
        for (tx = 0; tx < 3; tx++) {
            for (z = 0; z < TERRAIN_TILE; z++) {
                for (x = 0; x < TERRAIN_TILE; x++) {
                    const struct terrain_column *col = &tiles[tz][tx][z * TERRAIN_TILE + x];
                    int wx = (c->cx + tx - 1) * CHUNK_SIZE + x;
                    int wz = (c->cz + tz - 1) * CHUNK_SIZE + z;
                    int lx = wx - c->cx * CHUNK_SIZE, lz = wz - c->cz * CHUNK_SIZE;

                    if (col->feature != FEATURE_TREE || col->water > col->height)
                        continue;
                    if (lx < -2 || lx > CHUNK_SIZE + 1 || lz < -2 || lz > CHUNK_SIZE + 1)
                        continue;
                    place_tree(c, wx, wz, col->height);
                }
            }
        }
    }
}

/* ---- greedy meshing ------------------------------------------------------ */

struct mesh_ctx {
    const struct chunk *c;
    const struct chunk *nb[4];      /* -x, +x, -z, +z (NULL if not loaded) */
};

/*
 * Block at chunk-local (x, y, z); coordinates may be one step outside the
 * chunk. Missing neighbours count as stone, so no walls are drawn at the
 * edge of the loaded area; below the world is bedrock, above it air.
 */
static int block_near(const struct mesh_ctx *m, int x, int y, int z)
{
    const struct chunk *c = m->c;

    if (y < 0)
        return BLOCK_BEDROCK;
    if (y >= CHUNK_H)
        return BLOCK_AIR;
    if (x < 0)                { c = m->nb[0]; x += CHUNK_SIZE; }
    else if (x >= CHUNK_SIZE) { c = m->nb[1]; x -= CHUNK_SIZE; }
    else if (z < 0)           { c = m->nb[2]; z += CHUNK_SIZE; }
    else if (z >= CHUNK_SIZE) { c = m->nb[3]; z -= CHUNK_SIZE; }
    if (!c)
        return BLOCK_STONE;
    return c->blocks[y][z][x];
}

/*
 * Is the face of block b towards neighbour n visible? Yes against air, and
 * against anything see-through that isn't the same kind of block (so a
 * lake shows its bed, but no faces are drawn between two water blocks).
 */
static inline int face_visible(int b, int n)
{
    return n == BLOCK_AIR || (!block_opaque(n) && n != b);
}

struct quad_list {
    struct quad *q;
    int n, cap;
};

static void emit(struct quad_list *l, const struct quad *q)
{
    if (l->n == l->cap) {
        int cap = l->cap ? l->cap * 2 : 64;
        struct quad *nq = realloc(l->q, cap * sizeof(*nq));

        if (!nq)
            return;             /* out of memory: drop the face */
        l->q = nq;
        l->cap = cap;
    }
    l->q[l->n++] = *q;
}

void mesh_free(struct mesh *m)
{
    free(m->quads);
    memset(m, 0, sizeof(*m));
}

/*
 * Faces of the stairs block b at chunk-local (x, y, z), in half blocks:
 * a bottom slab, plus a step on the raised side. Faces on the block's
 * outline are dropped where an opaque neighbour hides them.
 */
static void stairs_faces(const struct mesh_ctx *m, struct quad_list group[NUM_GROUPS],
                         int b, int x, int y, int z, const int origin[3])
{
    /* box[k] = { lo[3], hi[3] } in half blocks within the cell */
    int box[2][2][3] = { { { 0, 0, 0 }, { 2, 1, 2 } }, { { 0, 1, 0 }, { 2, 2, 2 } } };
    int cell[3] = { x, y, z }, k, a, side;

    switch (b) {
    case BLOCK_STAIRS_XN: box[1][1][0] = 1; break;
    case BLOCK_STAIRS_XP: box[1][0][0] = 1; break;
    case BLOCK_STAIRS_ZN: box[1][1][2] = 1; break;
    default:              box[1][0][2] = 1; break;
    }

    for (k = 0; k < 2; k++) {
        for (a = 0; a < 3; a++) {
            int u = (a + 1) % 3, v = (a + 2) % 3;

            for (side = 0; side < 2; side++) {
                int plane = box[k][side][a], lo[3], hi[3], i;
                struct quad q;

                memcpy(lo, box[k][0], sizeof(lo));
                memcpy(hi, box[k][1], sizeof(hi));

                if (a == 1 && k == 1 && side == 0)
                    continue;               /* step sits on the slab */
                if (a == 1 && k == 0 && side == 1) {
                    /* slab top: only the half the step doesn't cover */
                    for (i = 0; i < 3; i += 2) {
                        if (box[1][1][i] == 1) { lo[i] = 1; hi[i] = 2; }
                        if (box[1][0][i] == 1) { lo[i] = 0; hi[i] = 1; }
                    }
                }
                if (plane == 0 || plane == 2) {
                    int n[3] = { cell[0], cell[1], cell[2] };

                    n[a] += side ? 1 : -1;
                    if (block_opaque(block_near(m, n[0], n[1], n[2])))
                        continue;
                }

                q.axis = (uint8_t)a;
                q.side = (uint8_t)side;
                q.block = (uint8_t)b;
                q.d  = (origin[a] + cell[a]) * QUAD_SCALE + plane;
                q.u0 = (origin[u] + cell[u]) * QUAD_SCALE + lo[u];
                q.u1 = (origin[u] + cell[u]) * QUAD_SCALE + hi[u];
                q.v0 = (origin[v] + cell[v]) * QUAD_SCALE + lo[v];
                q.v1 = (origin[v] + cell[v]) * QUAD_SCALE + hi[v];
                emit(&group[a * 2 + side], &q);
            }
        }
    }
}

void chunk_mesh(struct chunk *c, struct mesh_result *out)
{
    static __thread uint8_t mask[CHUNK_H * CHUNK_SIZE];
    struct quad_list list = { NULL, 0, 0 }, stairs[NUM_GROUPS];
    struct mesh_ctx m;
    struct mesh *me = &out->mesh;
    int origin[3] = { c->cx * CHUNK_SIZE, 0, c->cz * CHUNK_SIZE };
    int lo[3] = { 0, c->ylo, 0 }, hi[3] = { CHUNK_SIZE, c->yhi, CHUNK_SIZE };
    int a, side, x, y, z, g;

    memset(me, 0, sizeof(*me));
    memset(stairs, 0, sizeof(stairs));
    m.c = c;
    m.nb[0] = world_chunk(c->cx - 1, c->cz);
    m.nb[1] = world_chunk(c->cx + 1, c->cz);
    m.nb[2] = world_chunk(c->cx, c->cz - 1);
    m.nb[3] = world_chunk(c->cx, c->cz + 1);
    out->nbmask = (m.nb[0] ? NB_XN : 0) | (m.nb[1] ? NB_XP : 0) |
                  (m.nb[2] ? NB_ZN : 0) | (m.nb[3] ? NB_ZP : 0);
    out->chunk = c;

    if (hi[1] > CHUNK_H) hi[1] = CHUNK_H;
    if (lo[1] < 0) lo[1] = 0;
    me->ymin = lo[1];
    me->ymax = hi[1];

    /* stairs aren't cubes: their faces are made separately, per group */
    for (y = lo[1]; y < hi[1]; y++)
        for (z = 0; z < CHUNK_SIZE; z++)
            for (x = 0; x < CHUNK_SIZE; x++)
                if (block_is_stairs(c->blocks[y][z][x]))
                    stairs_faces(&m, stairs, c->blocks[y][z][x], x, y, z, origin);

    for (a = 0; a < 3; a++) {
        int u = (a + 1) % 3, v = (a + 2) % 3;
        int du = hi[u] - lo[u], dv = hi[v] - lo[v];

        for (side = 0; side < 2; side++) {
            int step = side ? 1 : -1, d, k;

            g = a * 2 + side;
            me->group_start[g] = list.n;
            me->group_dmin[g] = INT32_MAX;
            me->group_dmax[g] = INT32_MIN;

            for (d = lo[a]; d < hi[a] && du > 0 && dv > 0; d++) {
                int i, j, p[3], any = 0;

                /* which faces in this slice are visible */
                for (j = 0; j < dv; j++) {
                    for (i = 0; i < du; i++) {
                        uint8_t b;

                        p[a] = d; p[u] = lo[u] + i; p[v] = lo[v] + j;
                        b = c->blocks[p[1]][p[2]][p[0]];
                        mask[j * du + i] = 0;
                        if (b == BLOCK_AIR || block_is_stairs(b))
                            continue;
                        p[a] += step;
                        if (face_visible(b, block_near(&m, p[0], p[1], p[2]))) {
                            mask[j * du + i] = b;
                            any = 1;
                        }
                    }
                }
                if (!any)
                    continue;

                /* merge equal faces into rectangles */
                for (j = 0; j < dv; j++) {
                    for (i = 0; i < du; ) {
                        uint8_t b = mask[j * du + i];
                        int w, h, ok;
                        struct quad qd;

                        if (!b) {
                            i++;
                            continue;
                        }
                        for (w = 1; i + w < du && mask[j * du + i + w] == b; w++)
                            ;
                        for (h = 1, ok = 1; j + h < dv && ok; ) {
                            for (k = 0; k < w; k++) {
                                if (mask[(j + h) * du + i + k] != b) {
                                    ok = 0;
                                    break;
                                }
                            }
                            if (ok)
                                h++;
                        }
                        for (k = 0; k < h; k++)
                            memset(&mask[(j + k) * du + i], 0, w);

                        qd.axis = (uint8_t)a;
                        qd.side = (uint8_t)side;
                        qd.block = b;
                        qd.d  = (origin[a] + d + side) * QUAD_SCALE;
                        qd.u0 = (origin[u] + lo[u] + i) * QUAD_SCALE;
                        qd.u1 = qd.u0 + w * QUAD_SCALE;
                        qd.v0 = (origin[v] + lo[v] + j) * QUAD_SCALE;
                        qd.v1 = qd.v0 + h * QUAD_SCALE;
                        emit(&list, &qd);
                        i += w;
                    }
                }
            }

            for (k = 0; k < stairs[g].n; k++)
                emit(&list, &stairs[g].q[k]);
            free(stairs[g].q);

            for (k = me->group_start[g]; k < list.n; k++) {
                if (list.q[k].d < me->group_dmin[g]) me->group_dmin[g] = list.q[k].d;
                if (list.q[k].d > me->group_dmax[g]) me->group_dmax[g] = list.q[k].d;
            }
        }
    }
    me->group_start[NUM_GROUPS] = list.n;
    me->quads = list.q;
    me->nquads = list.n;
}
