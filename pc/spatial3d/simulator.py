"""Synthetic voxel-map generator so the pipeline is runnable without radar.

Fills a VoxelGrid with a floor, four walls, and a small furniture block,
matching the 20x30x15 room. Used by `main --sim` and by the tests.
"""

from __future__ import annotations

from .voxel import GRID_DIMS, Voxel, VoxelGrid, index_of


def _occupy(grid: VoxelGrid, ix: int, iy: int, iz: int, intensity: int = 200) -> None:
    grid.voxels[index_of(ix, iy, iz)] = Voxel(
        log_odds=100, hit_count=50, intensity=intensity, z_min=iz, z_max=iz
    )


def synthetic_room() -> VoxelGrid:
    nx, ny, nz = GRID_DIMS
    grid = VoxelGrid()

    # Floor (z = 0) and ceiling (z = nz-1).
    for ix in range(nx):
        for iy in range(ny):
            _occupy(grid, ix, iy, 0, intensity=255)

    # Four walls.
    for iz in range(nz):
        for iy in range(ny):
            _occupy(grid, 0, iy, iz)
            _occupy(grid, nx - 1, iy, iz)
        for ix in range(nx):
            _occupy(grid, ix, 0, iz)
            _occupy(grid, ix, ny - 1, iz)

    # A furniture block (e.g. a cabinet) sitting on the floor.
    for ix in range(5, 9):
        for iy in range(6, 10):
            for iz in range(0, 5):
                _occupy(grid, ix, iy, iz, intensity=180)

    return grid
