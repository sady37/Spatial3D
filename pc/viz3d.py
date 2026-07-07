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


H_MOUNT = 2.0     # radar mount height (room frame): radar sits at (0, 0, H_MOUNT)
TILT_DEG = 35.0   # downward mount tilt


def load_points(path: str):
    """Return (xyz float64 (N,3), extra (N,) or None) from .npz or .ply."""
    if path.endswith(".npz"):
        d = np.load(path, allow_pickle=True)
        c = d["music_cloud"]
        extra = c[:, 3] if c.shape[1] > 3 else None   # power
        return c[:, :3].astype(np.float64), extra
    pc = o3d.io.read_point_cloud(path)
    return np.asarray(pc.points), None


def reference_geometries(h_mount=H_MOUNT, tilt_deg=TILT_DEG,
                         x_range=(-3, 3), y_range=(0, 7)):
    """Radar marker (blue sphere) + boresight ray + floor grid + room box.

    Orients the scene: the radar sits ABOVE the floor at (0, 0, h_mount) and
    looks DOWN into +Y at *tilt_deg*, so the natural view is from the side or
    above, not level.
    """
    geoms = []
    radar = o3d.geometry.TriangleMesh.create_sphere(radius=0.15)
    radar.translate([0, 0, h_mount])
    radar.paint_uniform_color([0.1, 0.4, 1.0])
    radar.compute_vertex_normals()
    geoms.append(radar)

    t = np.radians(tilt_deg)
    # boresight = (0, cos t, -sin t) from radar; hits floor (z=0) at y = h/tan(t)
    floor_hit_y = h_mount / np.tan(t) if t else y_range[1]
    ray = o3d.geometry.LineSet()
    ray.points = o3d.utility.Vector3dVector([[0, 0, h_mount], [0, floor_hit_y, 0]])
    ray.lines = o3d.utility.Vector2iVector([[0, 1]])
    ray.colors = o3d.utility.Vector3dVector([[0.1, 0.8, 1.0]])
    geoms.append(ray)

    box = o3d.geometry.AxisAlignedBoundingBox(
        [x_range[0], y_range[0], -0.1], [x_range[1], y_range[1], 2.5])
    box.color = (0.5, 0.5, 0.5)
    geoms.append(box)

    pts, lns = [], []
    for xg in np.arange(x_range[0], x_range[1] + 0.01, 0.5):
        i = len(pts); pts += [[xg, y_range[0], 0], [xg, y_range[1], 0]]; lns.append([i, i + 1])
    for yg in np.arange(y_range[0], y_range[1] + 0.01, 0.5):
        i = len(pts); pts += [[x_range[0], yg, 0], [x_range[1], yg, 0]]; lns.append([i, i + 1])
    floor = o3d.geometry.LineSet()
    floor.points = o3d.utility.Vector3dVector(pts)
    floor.lines = o3d.utility.Vector2iVector(lns)
    floor.paint_uniform_color([0.25, 0.25, 0.28])
    geoms.append(floor)
    return geoms


# Camera presets (front, up, zoom). NOTE Open3D 'front' is the camera's LOOK
# direction (camera -> scene). The radar sits at (0,0,2) and looks into +Y / down.
VIEWS = {
    "radar": ([0, 1, -0.5], [0, 0, 1], 0.5),      # ALONG the radar line of sight
    "left":  ([0.7, 0.7, -0.45], [0, 0, 1], 0.5),   # into room, yawed to the left
    "right": ([-0.7, 0.7, -0.45], [0, 0, 1], 0.5),  # into room, yawed to the right
    "side":  ([-1, 0, 0.05], [0, 0, 1], 0.6),     # side elevation (Y horizontal)
    "3q":    ([0.5, 0.7, -0.5], [0, 0, 1], 0.55),  # into room, from front-left-above
    "top":   ([0, 0, -1], [0, 1, 0], 0.7),        # bird's-eye
}


def voxel_density(xyz, vs):
    """Voxel centres + point-count (density) per occupied voxel."""
    ijk = np.floor(xyz / vs).astype(int)
    keys, counts = np.unique(ijk, axis=0, return_counts=True)
    centers = (keys + 0.5) * vs
    return centers, counts


def cluster_boxes(xyz, args):
    """Detect furniture (cluster + bounding box) and return Open3D box outlines."""
    from spatial3d.cluster import detect_objects, LABEL_COLORS
    boxes, _ = detect_objects(
        xyz, voxel_size=args.voxel_size if args.voxel else 0.3,
        min_density=args.cluster_density, floor_z=args.floor_z)
    print(f"detected {len(boxes)} objects (cluster + bounding box):")
    geoms = []
    for b in boxes:
        print(f"  {b.label:12s} center=({b.center[0]:+.1f},{b.center[1]:.1f},"
              f"{b.center[2]:.1f})m  size=({b.size[0]:.2f}x{b.size[1]:.2f}"
              f"x{b.size[2]:.2f})m  n={b.n_points}")
        box = o3d.geometry.AxisAlignedBoundingBox(b.min_bound, b.max_bound)
        box.color = LABEL_COLORS.get(b.label, (1, 1, 1))
        geoms.append(box)
    return geoms


def build_geometries(args):
    xyz, extra = load_points(args.input)
    if args.z_min is not None:
        keep = xyz[:, 2] >= args.z_min
        dropped = (~keep).sum()
        xyz = xyz[keep]
        if extra is not None:
            extra = extra[keep]
        print(f"dropped {dropped} points below z={args.z_min} "
              f"(floor-bounce multipath ghosts)")
    geoms = [o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.5)]
    if args.room:
        geoms += reference_geometries()
    if getattr(args, "cluster", False):
        geoms += cluster_boxes(xyz, args)

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
    ap.add_argument("--room", action="store_true",
                    help="Show radar marker + boresight + floor grid + room box")
    ap.add_argument("--cluster", action="store_true",
                    help="Detect furniture (cluster stable voxels + bounding box)")
    ap.add_argument("--cluster-density", type=int, default=10,
                    help="Min voxel density for clustering (stable structure)")
    ap.add_argument("--floor-z", type=float, default=0.35,
                    help="Remove points below this Z before clustering (floor plane)")
    ap.add_argument("--z-min", type=float, default=None,
                    help="Drop points below this Z (use -0.1 to remove "
                         "floor-bounce multipath ghosts — often ~half the cloud)")
    ap.add_argument("--view", choices=list(VIEWS), default=None,
                    help="Camera preset for --screenshot (side/3q/top)")
    ap.add_argument("--screenshot", default=None,
                    help="Render off-screen to this PNG instead of a window")
    args = ap.parse_args()

    geoms = build_geometries(args)

    if args.screenshot:
        vis = o3d.visualization.Visualizer()
        vis.create_window(visible=False, width=1500, height=1000)
        for g in geoms:
            vis.add_geometry(g)
        opt = vis.get_render_option()
        opt.point_size = args.point_size
        opt.background_color = np.array([0.08, 0.08, 0.10])
        if args.view:
            front, up, zoom = VIEWS[args.view]
            ctr = vis.get_view_control()
            ctr.set_lookat([0, 3.0, 0.5])
            ctr.set_front(front)
            ctr.set_up(up)
            ctr.set_zoom(zoom)
        vis.poll_events()
        vis.update_renderer()
        vis.capture_screen_image(args.screenshot, do_render=True)
        vis.destroy_window()
        print(f"saved {args.screenshot}")
    else:
        print("opening window — drag=orbit, scroll=zoom, shift+drag=pan, Q=quit")
        front, up, zoom = VIEWS[args.view or "radar"]  # open along the radar's view
        o3d.visualization.draw_geometries(
            geoms, window_name=f"Spatial3D: {args.input}",
            width=1400, height=1000,
            front=front, up=up, zoom=zoom, lookat=[0, 3.0, 0.4])


if __name__ == "__main__":
    main()
