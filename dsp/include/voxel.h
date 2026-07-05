/*
 * voxel.h - DSP-side voxel occupancy map for TI AWR6844AOP.
 *
 * Room: 4m x 6m x 3m, 20cm voxels -> 20 x 30 x 15 = 9000 voxels.
 * The DSP never stores a point cloud; it maintains these 9000 voxels and
 * periodically ships the map to the PC over UART. Keep this in sync with
 * pc/spatial3d/voxel.py.
 */
#ifndef SPATIAL3D_VOXEL_H
#define SPATIAL3D_VOXEL_H

#include <stdint.h>
#include <stddef.h>

#define VX_NX 20
#define VX_NY 30
#define VX_NZ 15
#define VX_COUNT (VX_NX * VX_NY * VX_NZ) /* 9000 */

#define VX_VOXEL_SIZE_M 0.20f

/*
 * On-DSP voxel record. Packed to 8 bytes so the in-memory layout equals the
 * UART wire format (see pc/spatial3d/voxel.py _WIRE = "<hHHBB").
 */
#pragma pack(push, 1)
typedef struct Voxel {
    int16_t  logOdds;   /* occupancy log-odds */
    uint16_t hitCount;  /* CFAR hits accumulated */
    uint16_t intensity; /* accumulated reflection intensity */
    uint8_t  zMin;      /* lowest occupied layer 0..VX_NZ-1 */
    uint8_t  zMax;      /* highest occupied layer 0..VX_NZ-1 */
} Voxel;
#pragma pack(pop)

/* Flatten (ix, iy, iz) with x fastest, matching the PC side. */
static inline size_t vx_index(int ix, int iy, int iz) {
    return (size_t)ix + VX_NX * ((size_t)iy + VX_NY * (size_t)iz);
}

/* The full room map. */
typedef struct VoxelGrid {
    Voxel voxels[VX_COUNT];
} VoxelGrid;

void vx_grid_reset(VoxelGrid *grid);

/* Fold one CFAR detection at (ix, iy, iz) into the map. */
void vx_update(VoxelGrid *grid, int ix, int iy, int iz, uint16_t intensity);

#endif /* SPATIAL3D_VOXEL_H */
