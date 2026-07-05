/*
 * voxel.c - occupancy update logic for the DSP voxel map.
 */
#include "voxel.h"

#include <string.h>

/* Log-odds increment per hit and clamp bounds (occupancy grid mapping). */
#define VX_L_HIT   20
#define VX_L_MAX   2000
#define VX_L_MIN  (-2000)

void vx_grid_reset(VoxelGrid *grid) {
    memset(grid->voxels, 0, sizeof(grid->voxels));
}

static int16_t clamp_i16(int32_t v, int32_t lo, int32_t hi) {
    if (v < lo) return (int16_t)lo;
    if (v > hi) return (int16_t)hi;
    return (int16_t)v;
}

void vx_update(VoxelGrid *grid, int ix, int iy, int iz, uint16_t intensity) {
    if (ix < 0 || ix >= VX_NX || iy < 0 || iy >= VX_NY ||
        iz < 0 || iz >= VX_NZ) {
        return; /* out of room bounds */
    }

    Voxel *v = &grid->voxels[vx_index(ix, iy, iz)];

    v->logOdds = clamp_i16((int32_t)v->logOdds + VX_L_HIT, VX_L_MIN, VX_L_MAX);

    if (v->hitCount < UINT16_MAX) {
        v->hitCount++;
    }

    /* Running-ish intensity: saturating accumulate, capped. */
    uint32_t acc = (uint32_t)v->intensity + intensity;
    v->intensity = acc > UINT16_MAX ? UINT16_MAX : (uint16_t)acc;

    /* Track vertical extent of occupancy in this column cell. */
    if (v->hitCount == 1) {
        v->zMin = (uint8_t)iz;
        v->zMax = (uint8_t)iz;
    } else {
        if (iz < v->zMin) v->zMin = (uint8_t)iz;
        if (iz > v->zMax) v->zMax = (uint8_t)iz;
    }
}
