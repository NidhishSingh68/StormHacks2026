/*
 * world.h - chunk storage, terrain building, meshing, block edits
 */
#ifndef WORLD_H
#define WORLD_H

#include "mc.h"

/*
 * An axis-aligned rectangle of block faces, produced by greedy meshing.
 * axis is the face normal (0 = x, 1 = y, 2 = z); the rectangle lies in the
 * plane axis == d and spans [u0, u1) x [v0, v1) along axes (axis + 1) % 3
 * and (axis + 2) % 3. All coordinates are in world blocks.
 */
struct quad {
    int32_t d;
    int32_t u0, u1, v0, v1;
    uint8_t axis;
    uint8_t side;               /* 1: faces +axis, 0: faces -axis */
    uint8_t block;
};

/*
 * Quads are stored grouped by face direction (group = axis * 2 + side), so
 * a whole group can be skipped when the eye is behind all of its planes.
 */
#define NUM_GROUPS      6

struct mesh {
    struct quad    *quads;
    int             nquads;
    int             group_start[NUM_GROUPS + 1];
    int32_t         group_dmin[NUM_GROUPS], group_dmax[NUM_GROUPS];
    int             ymin, ymax;         /* y extent of all faces */
};

enum { CH_EMPTY, CH_LOADING, CH_READY };

/* Neighbour bits for chunk.nbmask */
#define NB_XN   1
#define NB_XP   2
#define NB_ZN   4
#define NB_ZP   8

struct chunk {
    int             cx, cz;     /* chunk coordinates */
    int             state;      /* CH_*, written by the main thread only */
    int             jobs;       /* queued/running jobs, main thread only */
    int             dirty;      /* needs another mesh once jobs finish */
    uint8_t         nbmask;     /* neighbours that were loaded at mesh time */
    int             ylo, yhi;   /* blocks outside [ylo, yhi) have no faces */
    struct mesh     mesh;       /* owned by the main thread */
    uint8_t         blocks[CHUNK_H][CHUNK_SIZE][CHUNK_SIZE];    /* [y][z][x] */
};

struct mesh_result {
    struct chunk   *chunk;
    struct mesh     mesh;
    uint8_t         nbmask;
};

int  world_init(int render_dist);
void world_free(void);
int  world_render_dist(void);

/* The slot a chunk coordinate maps to (whatever it currently holds). */
struct chunk *world_slot(int cx, int cz);

/* The chunk at (cx, cz) if it is loaded, else NULL. */
struct chunk *world_chunk(int cx, int cz);

/* Block at world coordinates, or -1 if that chunk isn't loaded. */
int world_block(int x, int y, int z);

/*
 * Change a block (main thread). Returns a bitmask of chunks that need a new
 * mesh: bit 0 the chunk itself, then NB_* << 1 for neighbours whose faces
 * touch the changed block.
 */
int world_set_block(int x, int y, int z, int block);

/* Bitmask of loaded neighbours of a chunk (NB_*). */
uint8_t world_neighbours(const struct chunk *c);

/*
 * Chunk offsets within the render distance of the player's chunk, nearest
 * first. Used both to decide what to load and what to draw.
 */
int world_ring(const int (**offsets)[2]);

/*
 * First breakable block along a ray (dir need not be normalised), within
 * max_dist. Returns 1 and the block position on a hit.
 */
int world_raycast(const float origin[3], const float dir[3], float max_dist,
                  int hit[3]);

/* Worker-side jobs. */
void chunk_generate(struct chunk *c);
void chunk_mesh(struct chunk *c, struct mesh_result *out);
void mesh_free(struct mesh *m);

#endif
