/*
 * main.c - host-buildable smoke test for the voxel engine.
 *
 * On the real target this logic lives inside the mmWave SDK data-path; here we
 * feed a few synthetic detections and print occupancy so the module can be
 * built and debugged on the PC with gcc (see .vscode/launch.json "DSP: voxel").
 */
#include "voxel.h"

#include <stdio.h>

int main(void) {
    static VoxelGrid grid; /* 9000 * 8B = 72000B, keep off the stack */
    vx_grid_reset(&grid);

    /* Simulate a floor row and one furniture column. */
    for (int ix = 0; ix < VX_NX; ++ix) {
        vx_update(&grid, ix, 5, 0, 255);
    }
    for (int iz = 0; iz < 5; ++iz) {
        vx_update(&grid, 6, 8, iz, 180);
    }

    size_t occupied = 0;
    for (size_t i = 0; i < VX_COUNT; ++i) {
        if (grid.voxels[i].logOdds > 0) {
            occupied++;
        }
    }

    printf("VoxelGrid: %d voxels, %zu occupied, %zu bytes/frame\n",
           VX_COUNT, occupied, sizeof(grid.voxels));
    return 0;
}
