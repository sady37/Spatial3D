"""Interactive 3D viewer (Open3D) for MUSIC room maps.

Reads a MUSIC point cloud (.npz with 'music_cloud', or a .ply) and opens an
interactive Open3D window — rotate (drag), zoom (scroll), pan (shift-drag).
Far better than a matplotlib 3D PNG for reading spatial structure.

Examples:
    python viz3d.py room_selflearned.ply
    python viz3d.py music_fullroom_voted.npz --voxel --min-density 6
    python viz3d.py music_fullroom_voted.npz --voxel --color density
    python viz3d.py room_full.ply --screenshot preview.png   # headless

Colouring:
    --color height   (default) turbo colormap on Z (floor->ceiling)
    --color density  point/voxel count = confidence (hot colormap)

In the window: mouse to orbit; '+'/'-' point size; 'R' reset view; 'Q' quit.
"""
from __future__ import annotations

import argparse

import numpy as np
import open3d as o3d


def _cmap(name: str):
    """Return a scalar->RGB function, tolerant of matplotlib version."""
    import matplotlib.pyplot as plt
    cm = plt.get_cmap(name)
    return lambda t: cm(np.clip(t, 0, 1))[:, :3]


def _colorize(scalar, name, lo=None, hi=None):
    scalar = np.asarray(scalar, dtype=float)
    lo = scalar.min() if lo is None else lo
    hi = scalar.max() if hi is None else hi
    return _cmap(name)((scalar - lo) / (hi - lo + 1e-9))


def load_points(path: str):
    """Return (xyz float64 (N,3), extra (N,) or None) from .npz or .ply."""
    if path.endswith(".npz"):
        d = np.load(path, allow_pickle=True)
        c = d["music_cloud"]
        extra = c[:, 3] if c.shape[1] > 3 else None   # power
        return c[:, :3].astype(np.float64), extra
    pc = o3d.io.read_point_cloud(path)
    return np.asarray(pc.points), None


def voxel_density(xyz, vs):
    """Voxel centres + point-count (density) per occupied voxel."""
    ijk = np.floor(xyz / vs).astype(int)
    keys, counts = np.unique(ijk, axis=0, return_counts=True)
    centers = (keys + 0.5) * vs
    return centers, counts


def build_geometries(args):
    xyz, extra = load_points(args.input)
    geoms = [o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.5)]

    if args.voxel:
        centers, dens = voxel_density(xyz, args.voxel_size)
        keep = dens >= args.min_density
        centers, dens = centers[keep], dens[keep]
        if args.color == "density":
            cols = _colorize(dens, "hot")
        else:
            cols = _colorize(centers[:, 2], "turbo", -0.5, 2.5)
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(centers)
        pcd.colors = o3d.utility.Vector3dVector(cols)
        # VoxelGrid renders solid, lit cubes (the LEGO look)
        vg = o3d.geometry.VoxelGrid.create_from_point_cloud(
            pcd, voxel_size=args.voxel_size)
        geoms.append(vg)
        print(f"voxel view: {keep.sum()} voxels "
              f"(>= {args.min_density} pts), size {args.voxel_size}m, "
              f"colour={args.color}")
    else:
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(xyz)
        if args.color == "density" and extra is not None:
            cols = _colorize(extra, "hot")
        else:
            cols = _colorize(xyz[:, 2], "turbo", -0.5, 2.5)
        pcd.colors = o3d.utility.Vector3dVector(cols)
        geoms.append(pcd)
        print(f"point view: {len(xyz)} points, colour={args.color}")
    return geoms


def main():
    ap = argparse.ArgumentParser(description="Interactive Open3D viewer for MUSIC maps")
    ap.add_argument("input", help=".npz (music_cloud) or .ply")
    ap.add_argument("--voxel", action="store_true",
                    help="Render voxel cubes (density map) instead of points")
    ap.add_argument("--voxel-size", type=float, default=0.2)
    ap.add_argument("--min-density", type=int, default=1,
                    help="Keep voxels with >= this many points (stable structure)")
    ap.add_argument("--color", choices=["height", "density"], default="height")
    ap.add_argument("--point-size", type=float, default=5.0)
    ap.add_argument("--screenshot", default=None,
                    help="Render off-screen to this PNG instead of a window")
    args = ap.parse_args()

    geoms = build_geometries(args)

    if args.screenshot:
        vis = o3d.visualization.Visualizer()
        vis.create_window(visible=False, width=1400, height=1000)
        for g in geoms:
            vis.add_geometry(g)
        opt = vis.get_render_option()
        opt.point_size = args.point_size
        opt.background_color = np.array([0.08, 0.08, 0.10])
        vis.poll_events()
        vis.update_renderer()
        vis.capture_screen_image(args.screenshot, do_render=True)
        vis.destroy_window()
        print(f"saved {args.screenshot}")
    else:
        print("opening window — drag=orbit, scroll=zoom, shift+drag=pan, Q=quit")
        o3d.visualization.draw_geometries(
            geoms, window_name=f"Spatial3D: {args.input}",
            width=1400, height=1000)


if __name__ == "__main__":
    main()
