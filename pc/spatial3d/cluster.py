"""Cluster a sparse MUSIC point cloud into objects + fit bounding boxes.

mmWave radar sees a table not as a solid block but as a sparse skeleton of its
strong scatterers (edges, corners, legs) — flat faces are specular and mostly
invisible. To recover a "continuous block" you cluster the sparse points into
objects and fit each a bounding box, then label it by height/size. This is the
voxel -> semantic (wall/bed/table/chair) layer of the design.

    labels = cluster_points(xyz, eps=0.45, min_samples=5)
    boxes  = fit_boxes(xyz, labels)      # list[ObjectBox], classified
"""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np
from numpy.typing import NDArray


@dataclass
class ObjectBox:
    """Axis-aligned box for one clustered object (room frame, metres)."""
    label: str
    center: NDArray[np.floating]   # (3,) x,y,z
    size: NDArray[np.floating]     # (3,) extent in x,y,z
    n_points: int

    @property
    def min_bound(self):
        return self.center - self.size / 2

    @property
    def max_bound(self):
        return self.center + self.size / 2

    @property
    def footprint(self):
        return float(max(self.size[0], self.size[1]))


def cluster_points(points: NDArray[np.floating], eps: float = 0.45,
                   min_samples: int = 5) -> NDArray[np.intp]:
    """DBSCAN labels for (N,3) points. -1 = noise. Needs scikit-learn."""
    from sklearn.cluster import DBSCAN
    if len(points) == 0:
        return np.empty((0,), dtype=int)
    return DBSCAN(eps=eps, min_samples=min_samples).fit_predict(points[:, :3])


def classify_box(center, size) -> str:
    """Heuristic furniture label from height + footprint (first-cut).

    Radar sees mostly the floor and low structure (aoaFov looks down), so bands
    are tuned for that. Refine per site once clusters are stable.
    """
    z_top = center[2] + size[2] / 2
    z_bot = center[2] - size[2] / 2
    foot = max(size[0], size[1])
    if z_top < 0.35:
        return "floor"                       # flat, on the ground
    if z_top >= 1.5:
        return "wall/person"                 # tall vertical extent
    if foot >= 1.4 and z_top < 0.7:
        return "bed/sofa"                     # large, low
    if 0.65 <= z_top <= 1.15 and foot >= 0.5:
        return "table"                        # tabletop height, mid footprint
    if 0.4 <= z_top <= 1.1 and foot < 0.7:
        return "chair"                        # compact, seat/back height
    return "object"


def fit_boxes(points: NDArray[np.floating], labels: NDArray[np.intp],
              min_points: int = 6) -> list[ObjectBox]:
    """Axis-aligned bounding box per cluster (skips noise label -1)."""
    boxes: list[ObjectBox] = []
    for lab in sorted(set(labels.tolist())):
        if lab == -1:
            continue
        pts = points[labels == lab, :3]
        if len(pts) < min_points:
            continue
        lo = pts.min(axis=0)
        hi = pts.max(axis=0)
        center = (lo + hi) / 2
        size = np.maximum(hi - lo, 0.05)     # floor tiny extents so box is visible
        boxes.append(ObjectBox(classify_box(center, size), center, size, len(pts)))
    # biggest first
    boxes.sort(key=lambda b: b.n_points, reverse=True)
    return boxes


def cluster_and_box(points: NDArray[np.floating], eps: float = 0.45,
                    min_samples: int = 5, min_points: int = 6):
    """Convenience: (labels, boxes)."""
    labels = cluster_points(points, eps=eps, min_samples=min_samples)
    return labels, fit_boxes(points, labels, min_points=min_points)


def stable_voxel_centers(cloud_xyz, voxel_size=0.3, min_density=10, z_min=-0.1):
    """Reduce a diffuse multi-scan cloud to STABLE voxel centres.

    The raw MUSIC cloud is too diffuse to cluster (one connected blob), so first
    keep only voxels hit by many points (density = confidence across scans),
    after dropping below-floor multipath (z < z_min). Returns (centres, density).
    """
    xyz = cloud_xyz[cloud_xyz[:, 2] >= z_min, :3]
    ijk = np.floor(xyz / voxel_size).astype(int)
    keys, cnt = np.unique(ijk, axis=0, return_counts=True)
    keep = cnt >= min_density
    return (keys[keep] + 0.5) * voxel_size, cnt[keep]


def detect_objects(cloud_xyz, voxel_size=0.3, min_density=10, floor_z=0.35,
                   eps=0.5, min_samples=2, min_points=2, z_min=-0.1):
    """Full furniture pass: ghost-filter -> stable voxels -> drop floor ->
    cluster -> boxes. Returns (boxes, centres_used).

    The floor is a connected plane that otherwise merges every object into one
    cluster, so points below *floor_z* are removed before clustering (the floor
    itself is reported separately by the height bands elsewhere).
    """
    centers, _ = stable_voxel_centers(cloud_xyz, voxel_size, min_density, z_min)
    above = centers[centers[:, 2] >= floor_z]
    labels = cluster_points(above, eps=eps, min_samples=min_samples)
    return fit_boxes(above, labels, min_points=min_points), above


# Box outline colour per semantic label (RGB 0-1), for viewers.
LABEL_COLORS = {
    "table": (0.2, 0.9, 0.2),
    "chair": (0.2, 0.7, 1.0),
    "bed/sofa": (1.0, 0.5, 0.1),
    "wall/person": (1.0, 0.2, 0.2),
    "floor": (0.6, 0.6, 0.6),
    "object": (1.0, 1.0, 0.3),
}
