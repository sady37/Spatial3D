"""Energy-density change detection: baseline vs event.

The static 3D energy-density model is the project's core representation. A voxel
accumulates MUSIC *power* (energy), not a point count, so strong reflectors
(a person, furniture) dominate and sparse noise stays low. Normalising each
cloud to its own total energy makes a dense nighttime baseline (many scans) and
a quick event capture (one scan) directly comparable — binary occupancy fails
here because a sparse single scan misses most voxels and everything looks
"gone", whereas the normalised energy *distribution* is robust.

    events, diff, meta = detect_changes(baseline_cloud, event_cloud)
    # events: 'appeared' (person lying -> fall zone) / 'gone' (object removed)
"""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np
from numpy.typing import NDArray

FALL_Z = 0.6   # room-frame height below which an appeared object is a fall/lie


@dataclass
class ChangeEvent:
    """One detected change region between baseline and event energy maps."""
    kind: str                      # 'appeared' | 'gone'
    center: NDArray[np.floating]   # (3,) x,y,z metres (energy-weighted centroid)
    magnitude: float               # summed |normalised energy delta|
    n_voxels: int
    fall_zone: bool                # appeared low (person lying / fall)


def energy_density(
    cloud: NDArray[np.floating],
    voxel_size: float = 0.3,
    x_range=(-3.0, 3.0),
    y_range=(0.0, 7.0),
    z_range=(-0.1, 2.5),
    normalize: bool = True,
) -> tuple[NDArray[np.floating], tuple]:
    """Power-weighted voxel energy grid from a (N, >=4) cloud [x,y,z,power,...].

    Points outside the ranges are dropped (z_range low bound also drops
    below-floor multipath ghosts). With *normalize*, the grid sums to 1 so
    clouds of different point counts compare as energy distributions.
    """
    cloud = np.asarray(cloud, dtype=float)
    xyz = cloud[:, :3]
    power = cloud[:, 3] if cloud.shape[1] > 3 else np.ones(len(cloud))
    m = ((xyz[:, 0] >= x_range[0]) & (xyz[:, 0] < x_range[1]) &
         (xyz[:, 1] >= y_range[0]) & (xyz[:, 1] < y_range[1]) &
         (xyz[:, 2] >= z_range[0]) & (xyz[:, 2] < z_range[1]))
    xyz, power = xyz[m], power[m]

    nx = int(round((x_range[1] - x_range[0]) / voxel_size))
    ny = int(round((y_range[1] - y_range[0]) / voxel_size))
    nz = int(round((z_range[1] - z_range[0]) / voxel_size))
    grid = np.zeros((nx, ny, nz))
    if len(xyz):
        ix = np.clip(((xyz[:, 0] - x_range[0]) / voxel_size).astype(int), 0, nx - 1)
        iy = np.clip(((xyz[:, 1] - y_range[0]) / voxel_size).astype(int), 0, ny - 1)
        iz = np.clip(((xyz[:, 2] - z_range[0]) / voxel_size).astype(int), 0, nz - 1)
        np.add.at(grid, (ix, iy, iz), power)
    if normalize and grid.sum() > 0:
        grid = grid / grid.sum()
    return grid, (x_range, y_range, z_range, voxel_size)


def energy_change(baseline_cloud, event_cloud, **grid_kw):
    """Normalised energy difference (event - baseline) and grid meta."""
    gb, meta = energy_density(baseline_cloud, **grid_kw)
    ga, _ = energy_density(event_cloud, **grid_kw)
    return ga - gb, meta


def _voxel_centers(idxs, meta):
    (xr, yr, zr, vs) = meta
    base = np.array([xr[0], yr[0], zr[0]])
    return base + (idxs + 0.5) * vs


def change_events(diff, meta, rel_threshold: float = 0.3,
                  min_voxels: int = 2, fall_z: float = FALL_Z) -> list[ChangeEvent]:
    """Cluster significant +/- energy deltas into ChangeEvents.

    A voxel is significant if |delta| exceeds *rel_threshold* of the peak
    |delta|. Positive clusters are 'appeared', negative are 'gone'. Each event's
    centre is the energy-weighted centroid; 'appeared' below *fall_z* is flagged
    as a fall/lie.
    """
    from scipy import ndimage

    thr = rel_threshold * np.abs(diff).max() if diff.size else 0.0
    events: list[ChangeEvent] = []
    for sign, kind in ((1, "appeared"), (-1, "gone")):
        mask = (diff * sign) > thr
        labels, n = ndimage.label(mask)
        for c in range(1, n + 1):
            sel = labels == c
            if sel.sum() < min_voxels:
                continue
            idxs = np.argwhere(sel)
            w = np.abs(diff[sel])
            centers = _voxel_centers(idxs, meta)
            centroid = (centers * w[:, None]).sum(axis=0) / w.sum()
            events.append(ChangeEvent(
                kind=kind, center=centroid, magnitude=float(w.sum()),
                n_voxels=int(sel.sum()),
                fall_zone=bool(kind == "appeared" and centroid[2] < fall_z)))
    events.sort(key=lambda e: e.magnitude, reverse=True)
    return events


def detect_changes(baseline_cloud, event_cloud, voxel_size: float = 0.3,
                   rel_threshold: float = 0.3, min_voxels: int = 2,
                   fall_z: float = FALL_Z, **grid_kw):
    """Full pass: energy grids -> normalised diff -> change events.

    Returns (events, diff_grid, meta).
    """
    diff, meta = energy_change(baseline_cloud, event_cloud,
                               voxel_size=voxel_size, **grid_kw)
    return change_events(diff, meta, rel_threshold, min_voxels, fall_z), diff, meta
