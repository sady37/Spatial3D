"""Patient monitor: slow Map-A accumulator + minute-level current map + diff.

This is the "patient 3D static-change detector" described in DESIGN-2026-07-05.
No real-time tracker, no trigger — just energy accumulation and differencing.

Coordinate frames:
    Sensor frame (TI demo output):
        x_s = left-right,  y_s = forward (bore),  z_s = up from sensor
    Room frame (what we voxelise into):
        x_r = left-right (same as sensor),
        y_r = horizontal depth on the floor plane,
        z_r = HEIGHT FROM FLOOR (up)

    Sensor mount: height `h` metres, tilted `θ` from vertical (0 = straight down).
    Transform:
        x_r = x_s
        y_r = y_s·sinθ + z_s·cosθ
        z_r = h − y_s·cosθ + z_s·sinθ

    The grid origin (voxel 0,0,0) sits at (grid_origin_x, grid_origin_y, 0) in
    room frame so that the floor is always iz=0.

Key idea:
    Map A  — long-term baseline, very slow EMA (hours/days time constant)
    Current — rolling window of last ~60 s of frames
    Diff   — current minus A → new/moved mass
    Floor band — Z ≤ 0.40 m (iz=0,1) → fall candidate region
"""

from __future__ import annotations

import math
import time
from collections import deque
from dataclasses import dataclass, field

import numpy as np

from .voxel import GRID_DIMS, VOXEL_SIZE_M

# Floor band: Z indices 0 and 1 → 0–0.40 m
FLOOR_Z_MAX_IDX = 2  # exclusive; iz in [0, FLOOR_Z_MAX_IDX)


def sensor_to_room(points_s: np.ndarray, height: float, tilt_deg: float
                   ) -> np.ndarray:
    """Transform (N,3+) sensor-frame XYZ to room-frame XYZ.

    height:   sensor mount height above floor (metres)
    tilt_deg: tilt angle from vertical (degrees); 0 = straight down, 90 = horizontal
    Returns (N,3) room-frame [x, y, z_height_from_floor].
    """
    theta = math.radians(tilt_deg)
    sin_t, cos_t = math.sin(theta), math.cos(theta)
    xs = points_s[:, 0]
    ys = points_s[:, 1]
    zs = points_s[:, 2]
    xr = xs
    yr = ys * sin_t + zs * cos_t
    zr = height - ys * cos_t + zs * sin_t
    return np.column_stack([xr, yr, zr])


def _points_to_energy(points_room: np.ndarray,
                       grid_origin: tuple[float, float, float],
                       shape: tuple[int, int, int]) -> np.ndarray:
    """Bin (N,3) room-frame XYZ points into a 3D energy grid (hit count per voxel)."""
    grid = np.zeros(shape, dtype=np.float64)
    if len(points_room) == 0:
        return grid
    pts = np.asarray(points_room[:, :3], dtype=np.float64)
    ix = ((pts[:, 0] - grid_origin[0]) / VOXEL_SIZE_M).astype(int)
    iy = ((pts[:, 1] - grid_origin[1]) / VOXEL_SIZE_M).astype(int)
    iz = ((pts[:, 2] - grid_origin[2]) / VOXEL_SIZE_M).astype(int)
    mask = ((ix >= 0) & (ix < shape[0]) &
            (iy >= 0) & (iy < shape[1]) &
            (iz >= 0) & (iz < shape[2]))
    np.add.at(grid, (ix[mask], iy[mask], iz[mask]), 1.0)
    return grid


@dataclass
class Snapshot:
    """One frame's contribution, kept in the rolling window."""
    timestamp: float
    energy: np.ndarray  # (nx, ny, nz)


@dataclass
class ClarityMetrics:
    """Quantifies how sharp / blurry Map A currently is."""
    occupied_voxels: int          # voxels with energy > threshold
    total_energy: float           # sum of all energy
    energy_concentration: float   # Gini-like: 0=uniform, 1=all-in-one-voxel
    mean_z_spread: float          # avg std-dev of Z-energy per occupied XY column
    peak_density: float           # mean energy per occupied voxel (higher = sharper)


@dataclass
class FloorAnomaly:
    """Summary of new mass in the floor band (B−A)."""
    diff_energy: float       # total positive diff in floor band
    peak_xy: tuple[int, int] | None  # voxel column with max diff (None if quiet)
    peak_value: float        # max per-column diff


class PatientMonitor:
    """The 'patient 3D static-change detector'.

    Feed it frames via `update(points)`. Query with `get_diff()`,
    `get_clarity()`, `get_floor_anomaly()`.

    Parameters
    ----------
    mount_height : sensor height above floor (metres).
    mount_tilt_deg : tilt from vertical (degrees). 0=straight down, 90=horizontal.
    grid_origin : room-frame offset of voxel (0,0,0) corner (metres).
                  z component should be 0 so iz=0 is the floor.
    alpha_a : per-frame EMA decay for Map A. Tiny = slow absorption.
              Default 1e-4 → at 10 fps, half-life ≈ 700 s ≈ 12 min.
              For production (hours), use ~1e-6.
    window_sec : rolling window duration for the "current" map (seconds).
    occ_threshold : energy threshold to count a voxel as "occupied" in A.
    """

    def __init__(self, *,
                 mount_height: float = 2.00,
                 mount_tilt_deg: float = 35.0,
                 grid_origin: tuple[float, float, float] = (-2.0, 0.0, 0.0),
                 alpha_a: float = 1e-4,
                 window_sec: float = 60.0,
                 occ_threshold: float = 0.5):
        self.mount_height = mount_height
        self.mount_tilt_deg = mount_tilt_deg
        self.grid_origin = grid_origin
        self.alpha_a = alpha_a
        self.window_sec = window_sec
        self.occ_threshold = occ_threshold
        self.shape = GRID_DIMS  # (20, 30, 15)

        # Map A — long-term baseline energy (EMA)
        self.map_a = np.zeros(self.shape, dtype=np.float64)

        # Rolling window of recent snapshots → summed into "current"
        self._window: deque[Snapshot] = deque()
        self._current_sum = np.zeros(self.shape, dtype=np.float64)

        self.frame_count = 0
        self.t_start = time.time()

    # ------------------------------------------------------------------
    # Core update
    # ------------------------------------------------------------------

    def update(self, points: np.ndarray, timestamp: float | None = None) -> None:
        """Process one frame of detected points (N,4) [x,y,z,doppler].

        - Bins points into a per-frame energy grid
        - Updates Map A via EMA
        - Pushes snapshot into the rolling window

        Pass an explicit `timestamp` for offline / test use; defaults to wall clock.
        """
        now = timestamp if timestamp is not None else time.time()
        # Sensor frame → room frame (floor = z=0)
        if len(points) > 0:
            room_pts = sensor_to_room(points, self.mount_height, self.mount_tilt_deg)
        else:
            room_pts = points[:, :3] if len(points.shape) > 1 else np.empty((0, 3))
        energy = _points_to_energy(room_pts, self.grid_origin, self.shape)

        # Update Map A: A ← (1-α)A + α·energy
        self.map_a *= (1.0 - self.alpha_a)
        self.map_a += self.alpha_a * energy

        # Push into rolling window
        snap = Snapshot(timestamp=now, energy=energy)
        self._window.append(snap)
        self._current_sum += energy

        # Evict old snapshots
        cutoff = now - self.window_sec
        while self._window and self._window[0].timestamp < cutoff:
            old = self._window.popleft()
            self._current_sum -= old.energy

        self.frame_count += 1

    # ------------------------------------------------------------------
    # Queries
    # ------------------------------------------------------------------

    def get_current(self) -> np.ndarray:
        """Current rolling-window energy map (copy)."""
        return self._current_sum.copy()

    def get_diff(self) -> np.ndarray:
        """B − A: positive values = new mass not in baseline.

        Normalizes current by window frame count so it's comparable to A.
        """
        n = len(self._window)
        if n == 0:
            return np.zeros(self.shape, dtype=np.float64)
        # Normalize current to per-frame average, then subtract A
        current_avg = self._current_sum / n
        return current_avg - self.map_a

    def get_clarity(self) -> ClarityMetrics:
        """Quantify how sharp Map A is right now."""
        a = self.map_a
        total = float(a.sum())
        if total < 1e-12:
            return ClarityMetrics(0, 0.0, 0.0, 0.0, 0.0)

        occ_mask = a > self.occ_threshold
        n_occ = int(occ_mask.sum())
        peak_density = total / max(n_occ, 1)

        # Energy concentration (Gini coefficient on flat sorted values)
        flat = np.sort(a.ravel())
        n = len(flat)
        cumsum = np.cumsum(flat)
        gini = 1.0 - 2.0 * cumsum.sum() / (n * cumsum[-1]) if cumsum[-1] > 0 else 0.0

        # Mean Z-spread: for each XY column, compute std of the Z-energy profile
        # Only over columns that have meaningful energy
        xy_energy = a.sum(axis=2)  # (nx, ny)
        col_threshold = self.occ_threshold
        z_spreads = []
        nx, ny, nz = self.shape
        z_indices = np.arange(nz, dtype=np.float64)
        for ix in range(nx):
            for iy in range(ny):
                col = a[ix, iy, :]  # (nz,)
                if col.sum() < col_threshold:
                    continue
                weights = col / col.sum()
                mean_z = np.dot(weights, z_indices)
                var_z = np.dot(weights, (z_indices - mean_z) ** 2)
                z_spreads.append(np.sqrt(var_z))
        mean_z_spread = float(np.mean(z_spreads)) if z_spreads else 0.0

        return ClarityMetrics(
            occupied_voxels=n_occ,
            total_energy=total,
            energy_concentration=max(0.0, gini),
            mean_z_spread=mean_z_spread,
            peak_density=peak_density,
        )

    def get_floor_anomaly(self) -> FloorAnomaly:
        """Check for new mass in the floor band (Z ≤ 40cm) relative to A."""
        diff = self.get_diff()
        floor_diff = diff[:, :, :FLOOR_Z_MAX_IDX]

        # Only care about positive diff (new mass, not disappearing mass)
        pos = np.maximum(floor_diff, 0.0)
        total_diff = float(pos.sum())

        # Per-column (XY) sum of floor-band diff
        col_sum = pos.sum(axis=2)  # (nx, ny)
        peak_val = float(col_sum.max()) if col_sum.size > 0 else 0.0
        if peak_val > 0:
            idx = np.unravel_index(col_sum.argmax(), col_sum.shape)
            peak_xy = (int(idx[0]), int(idx[1]))
        else:
            peak_xy = None

        return FloorAnomaly(
            diff_energy=total_diff,
            peak_xy=peak_xy,
            peak_value=peak_val,
        )

    # ------------------------------------------------------------------
    # Persistence
    # ------------------------------------------------------------------

    def save_map_a(self, path: str) -> None:
        """Save Map A to disk as .npy for later reload."""
        np.save(path, self.map_a)

    def load_map_a(self, path: str) -> None:
        """Load a previously saved Map A."""
        loaded = np.load(path)
        if loaded.shape != self.shape:
            raise ValueError(f"shape mismatch: file {loaded.shape} vs grid {self.shape}")
        self.map_a = loaded.astype(np.float64)
