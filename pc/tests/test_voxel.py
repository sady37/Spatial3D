"""Tests for the voxel model and UART framing (no hardware / Open3D needed)."""

import io

from spatial3d.simulator import synthetic_room
from spatial3d.uart_reader import PAYLOAD_SIZE, frame, read_grid
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
    assert PAYLOAD_SIZE == 9000 * 8


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


def test_uart_frame_roundtrip():
    grid = synthetic_room()
    stream = io.BytesIO(frame(grid))
    restored = read_grid(stream)
    assert restored.occupied_points() == grid.occupied_points()


def test_uart_resync_on_leading_garbage():
    grid = synthetic_room()
    stream = io.BytesIO(b"\x00\xffnoise" + frame(grid))
    restored = read_grid(stream)
    assert len(restored.occupied_points()) == len(grid.occupied_points())
