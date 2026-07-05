"""Tests for the voxel model (no hardware / Open3D needed)."""

from spatial3d.simulator import synthetic_room
from spatial3d.voxel import (
    GRID_DIMS,
    VOXEL_COUNT,
    WIRE_SIZE,
    Voxel,
    VoxelGrid,
    coords_of,
    index_of,
)


def test_grid_dims():
    assert VOXEL_COUNT == 20 * 30 * 15 == 9000
    assert WIRE_SIZE == 8


def test_voxel_pack_roundtrip():
    v = Voxel(log_odds=-123, hit_count=4000, intensity=250, z_min=2, z_max=9)
    assert Voxel.unpack(v.pack()) == v


def test_index_coords_roundtrip():
    nx, ny, nz = GRID_DIMS
    for coords in [(0, 0, 0), (nx - 1, ny - 1, nz - 1), (5, 6, 7)]:
        assert coords_of(index_of(*coords)) == coords


def test_grid_bytes_roundtrip():
    grid = synthetic_room()
    restored = VoxelGrid.from_bytes(grid.to_bytes())
    assert [v.occupied for v in restored.voxels] == [v.occupied for v in grid.voxels]


def test_add_points_bins_into_grid():
    grid = VoxelGrid()
    # Points inside the 4x6x3 m room and one outside (should be dropped).
    pts = [(0.1, 0.1, 0.1), (0.15, 0.12, 0.1), (3.9, 5.9, 2.9), (99.0, 0.0, 0.0)]
    binned = grid.add_points(pts)
    assert binned == 3
    # First two land in the same voxel -> hit_count 2.
    assert grid.get(0, 0, 0).hit_count == 2
