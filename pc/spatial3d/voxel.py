"""Voxel data model shared conceptually with the DSP layer.

Room geometry (from project brief):
    4 m (X) x 6 m (Y) x 3 m (Z), voxel edge 20 cm
    -> grid 20 x 30 x 15 = 9000 voxels

UART wire format is a packed 8-byte record per voxel (little-endian):
    int16  logOdds     occupancy log-odds
    uint16 hitCount    number of CFAR hits accumulated
    uint16 intensity   accumulated reflection intensity
    uint8  zMin        lowest occupied layer index (0..14)
    uint8  zMax        highest occupied layer index (0..14)

This mirrors `dsp/include/voxel.h`; keep the two in sync.
"""

from __future__ import annotations

import struct
from dataclasses import dataclass

# --- Fixed spatial configuration -------------------------------------------
VOXEL_SIZE_M = 0.20
GRID_DIMS = (20, 30, 15)  # (nx, ny, nz)
VOXEL_COUNT = GRID_DIMS[0] * GRID_DIMS[1] * GRID_DIMS[2]  # 9000

# UART wire format: little-endian, 8 bytes/voxel.
_WIRE = struct.Struct("<hHHBB")
WIRE_SIZE = _WIRE.size  # 8
assert WIRE_SIZE == 8


@dataclass
class Voxel:
    log_odds: int = 0
    hit_count: int = 0
    intensity: int = 0
    z_min: int = 0
    z_max: int = 0

    @property
    def occupied(self) -> bool:
        return self.log_odds > 0

    def pack(self) -> bytes:
        return _WIRE.pack(
            self.log_odds, self.hit_count, self.intensity, self.z_min, self.z_max
        )

    @classmethod
    def unpack(cls, buf: bytes) -> "Voxel":
        return cls(*_WIRE.unpack(buf))


def index_of(ix: int, iy: int, iz: int) -> int:
    """Flatten (ix, iy, iz) -> linear voxel index (x-fastest ordering)."""
    nx, ny, _ = GRID_DIMS
    return ix + nx * (iy + ny * iz)


def coords_of(index: int) -> tuple[int, int, int]:
    nx, ny, _ = GRID_DIMS
    ix = index % nx
    iy = (index // nx) % ny
    iz = index // (nx * ny)
    return ix, iy, iz


class VoxelGrid:
    """The full 9000-voxel occupancy map."""

    def __init__(self) -> None:
        self.voxels: list[Voxel] = [Voxel() for _ in range(VOXEL_COUNT)]

    def __len__(self) -> int:
        return len(self.voxels)

    def get(self, ix: int, iy: int, iz: int) -> Voxel:
        return self.voxels[index_of(ix, iy, iz)]

    # --- UART sync ---------------------------------------------------------
    def to_bytes(self) -> bytes:
        return b"".join(v.pack() for v in self.voxels)

    @classmethod
    def from_bytes(cls, buf: bytes) -> "VoxelGrid":
        if len(buf) != VOXEL_COUNT * WIRE_SIZE:
            raise ValueError(
                f"expected {VOXEL_COUNT * WIRE_SIZE} bytes, got {len(buf)}"
            )
        grid = cls()
        for i in range(VOXEL_COUNT):
            chunk = buf[i * WIRE_SIZE : (i + 1) * WIRE_SIZE]
            grid.voxels[i] = Voxel.unpack(chunk)
        return grid

    # --- Modeling handoff --------------------------------------------------
    def occupied_points(self) -> list[tuple[float, float, float]]:
        """Return voxel centers (meters) for occupied voxels."""
        pts = []
        half = VOXEL_SIZE_M / 2.0
        for i, v in enumerate(self.voxels):
            if not v.occupied:
                continue
            ix, iy, iz = coords_of(i)
            pts.append(
                (
                    ix * VOXEL_SIZE_M + half,
                    iy * VOXEL_SIZE_M + half,
                    iz * VOXEL_SIZE_M + half,
                )
            )
        return pts
