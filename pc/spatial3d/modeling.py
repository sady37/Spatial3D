"""PC-layer spatial modeling: voxel occupancy -> semantic static model.

All heavy CPU work lives here (per the three-layer architecture): building the
point cloud, RANSAC plane fitting for walls/floor, and DBSCAN clustering for
furniture. Open3D is imported lazily so unit tests and the UART path do not
require it.
"""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

from .voxel import VoxelGrid


@dataclass
class Plane:
    """A fitted plane: n·x + d = 0, with the supporting inlier indices."""

    normal: tuple[float, float, float]
    d: float
    inliers: list[int] = field(default_factory=list)


def grid_to_pointcloud(grid: VoxelGrid) -> np.ndarray:
    """Occupied voxel centers as an (N, 3) float array in meters."""
    pts = grid.occupied_points()
    return np.asarray(pts, dtype=np.float64).reshape(-1, 3)


def fit_planes(points: np.ndarray, max_planes: int = 4,
               distance_threshold: float = 0.05) -> list[Plane]:
    """Iteratively RANSAC-fit dominant planes (floor + walls) via Open3D."""
    import open3d as o3d

    planes: list[Plane] = []
    remaining = o3d.geometry.PointCloud()
    remaining.points = o3d.utility.Vector3dVector(points)

    for _ in range(max_planes):
        if len(remaining.points) < 3:
            break
        model, inliers = remaining.segment_plane(
            distance_threshold=distance_threshold,
            ransac_n=3,
            num_iterations=1000,
        )
        a, b, c, d = model
        planes.append(Plane(normal=(a, b, c), d=d, inliers=list(inliers)))
        remaining = remaining.select_by_index(inliers, invert=True)

    return planes


def cluster_furniture(points: np.ndarray, eps: float = 0.3,
                      min_samples: int = 5) -> np.ndarray:
    """DBSCAN cluster labels for non-planar (furniture) points. -1 = noise."""
    from sklearn.cluster import DBSCAN

    if len(points) == 0:
        return np.empty(0, dtype=int)
    return DBSCAN(eps=eps, min_samples=min_samples).fit_predict(points)


def visualize(points: np.ndarray, planes: list[Plane] | None = None) -> None:
    """Open an Open3D window with the point cloud (debug aid)."""
    import open3d as o3d

    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)
    pcd.paint_uniform_color([0.6, 0.6, 0.6])
    frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.5)
    o3d.visualization.draw_geometries([pcd, frame])
