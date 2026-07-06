"""3D voxel (LEGO-style) static room map.

Converts accumulated static point cloud into a 3D occupancy grid
and renders it as colored blocks. Fall detection = check if energy
shifts from upper voxels to floor-level (Z <= 0.4m) voxels.

Usage:
    python -m spatial3d.voxel_map static_3d_10min.npz --voxel-size 0.3
"""
from __future__ import annotations

import argparse
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.colors import Normalize
import matplotlib.cm as cm


TILT_DEG = 35.0
H_MOUNT = 2.0


def to_room(pts: np.ndarray) -> np.ndarray:
    tilt = np.radians(TILT_DEG)
    x, y, z = pts[:, 0], pts[:, 1], pts[:, 2]
    xr = x
    yr = y * np.cos(tilt) + z * np.sin(tilt)
    zr = -y * np.sin(tilt) + z * np.cos(tilt) + H_MOUNT
    mask = np.isfinite(xr) & np.isfinite(yr) & np.isfinite(zr)
    return np.column_stack([xr[mask], yr[mask], zr[mask]])


def build_voxel_grid(room_pts: np.ndarray, voxel_size: float,
                     x_range=(-3, 3), y_range=(0, 7), z_range=(-0.5, 2.5)):
    x, y, z = room_pts[:, 0], room_pts[:, 1], room_pts[:, 2]
    mask = ((x >= x_range[0]) & (x < x_range[1]) &
            (y >= y_range[0]) & (y < y_range[1]) &
            (z >= z_range[0]) & (z < z_range[1]))
    pts = room_pts[mask]

    nx = int(np.ceil((x_range[1] - x_range[0]) / voxel_size))
    ny = int(np.ceil((y_range[1] - y_range[0]) / voxel_size))
    nz = int(np.ceil((z_range[1] - z_range[0]) / voxel_size))

    ix = np.clip(((pts[:, 0] - x_range[0]) / voxel_size).astype(int), 0, nx - 1)
    iy = np.clip(((pts[:, 1] - y_range[0]) / voxel_size).astype(int), 0, ny - 1)
    iz = np.clip(((pts[:, 2] - z_range[0]) / voxel_size).astype(int), 0, nz - 1)

    grid = np.zeros((nx, ny, nz), dtype=int)
    np.add.at(grid, (ix, iy, iz), 1)

    return grid, (x_range, y_range, z_range)


def render_voxels(grid: np.ndarray, ranges, voxel_size: float,
                  threshold: int = 3, out_path: str = "voxel_map.png"):
    x_range, y_range, z_range = ranges
    occupied = grid >= threshold

    nx, ny, nz = grid.shape
    norm = Normalize(vmin=0, vmax=grid[occupied].max() if occupied.any() else 1)

    fall_z_idx = int(np.ceil((0.4 - z_range[0]) / voxel_size))

    fig = plt.figure(figsize=(16, 10))
    ax = fig.add_subplot(111, projection='3d')

    for ix in range(nx):
        for iy in range(ny):
            for iz in range(nz):
                if not occupied[ix, iy, iz]:
                    continue
                count = grid[ix, iy, iz]
                x0 = x_range[0] + ix * voxel_size
                y0 = y_range[0] + iy * voxel_size
                z0 = z_range[0] + iz * voxel_size

                if iz < fall_z_idx:
                    color = cm.Reds(norm(count))
                elif iz == fall_z_idx:
                    color = cm.Oranges(norm(count))
                else:
                    color = cm.Blues(norm(count))

                _draw_block(ax, x0, y0, z0, voxel_size, color, norm(count))

    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y_room (m) — distance")
    ax.set_zlabel("Z_room (m) — height")
    ax.set_xlim(x_range)
    ax.set_ylim(y_range)
    ax.set_zlim(z_range)
    ax.set_title(f"3D Voxel Map (size={voxel_size}m, threshold≥{threshold} pts)\n"
                 f"Red=fall zone(≤0.4m)  Orange=boundary  Blue=normal",
                 fontsize=12)
    ax.view_init(elev=25, azim=-60)

    plt.tight_layout()
    plt.savefig(out_path, dpi=150, bbox_inches='tight')
    print(f"Saved {out_path}")

    n_total = occupied.sum()
    n_floor = occupied[:, :, :fall_z_idx].sum()
    n_upper = occupied[:, :, fall_z_idx:].sum()
    print(f"Occupied voxels: {n_total} (floor={n_floor}, upper={n_upper})")
    return grid


def _draw_block(ax, x, y, z, s, color, alpha_val):
    alpha = max(0.3, min(0.9, alpha_val))
    vertices = [
        [[x, x+s, x+s, x], [y, y, y, y], [z, z, z+s, z+s]],
        [[x, x+s, x+s, x], [y+s, y+s, y+s, y+s], [z, z, z+s, z+s]],
        [[x, x, x, x], [y, y+s, y+s, y], [z, z, z+s, z+s]],
        [[x+s, x+s, x+s, x+s], [y, y+s, y+s, y], [z, z, z+s, z+s]],
        [[x, x+s, x+s, x], [y, y, y+s, y+s], [z, z, z, z]],
        [[x, x+s, x+s, x], [y, y, y+s, y+s], [z+s, z+s, z+s, z+s]],
    ]
    from mpl_toolkits.mplot3d.art3d import Poly3DCollection
    for v in vertices:
        poly = [list(zip(v[0], v[1], v[2]))]
        face = Poly3DCollection(poly, alpha=alpha)
        face.set_facecolor(color)
        face.set_edgecolor((0.2, 0.2, 0.2, 0.3))
        ax.add_collection3d(face)


def render_floor_plan(grid: np.ndarray, ranges, voxel_size: float,
                      threshold: int = 3, out_path: str = "voxel_floor.png"):
    """Top-down occupancy by height band."""
    x_range, y_range, z_range = ranges
    fall_z_idx = int(np.ceil((0.4 - z_range[0]) / voxel_size))
    desk_z_idx = int(np.ceil((1.0 - z_range[0]) / voxel_size))

    floor_occ = (grid[:, :, :fall_z_idx] >= threshold).sum(axis=2)
    mid_occ = (grid[:, :, fall_z_idx:desk_z_idx] >= threshold).sum(axis=2)
    upper_occ = (grid[:, :, desk_z_idx:] >= threshold).sum(axis=2)

    fig, axes = plt.subplots(1, 3, figsize=(18, 6))
    extent = [y_range[0], y_range[1], x_range[0], x_range[1]]

    for ax, data, title, cmap in [
        (axes[0], floor_occ, f"Floor (Z<0.4m) — FALL ZONE", 'Reds'),
        (axes[1], mid_occ, f"Mid (0.4-1.0m) — desk/chair", 'Oranges'),
        (axes[2], upper_occ, f"Upper (>1.0m) — standing", 'Blues'),
    ]:
        im = ax.imshow(data, origin='lower', extent=extent, aspect='auto', cmap=cmap)
        ax.set_xlabel("Y_room (m) — distance")
        ax.set_ylabel("X (m)")
        ax.set_title(title)
        plt.colorbar(im, ax=ax, label='occupied layers')

    plt.suptitle(f"Voxel Floor Plan (size={voxel_size}m, threshold≥{threshold})", fontsize=13)
    plt.tight_layout()
    plt.savefig(out_path, dpi=150, bbox_inches='tight')
    print(f"Saved {out_path}")


def main():
    p = argparse.ArgumentParser(description="3D Voxel Map (LEGO-style)")
    p.add_argument("input", help="Input .npz file with 'static' array")
    p.add_argument("--voxel-size", type=float, default=0.3, help="Voxel edge length (m)")
    p.add_argument("--threshold", type=int, default=3, help="Min points to mark occupied")
    p.add_argument("--out", default="voxel_map.png", help="Output 3D image")
    p.add_argument("--floor-out", default="voxel_floor.png", help="Output floor plan")
    args = p.parse_args()

    data = np.load(args.input)
    static = data["static"]
    print(f"Loaded {len(static)} static points from {args.input}")

    room = to_room(static)
    print(f"Room-frame valid: {len(room)} points")

    grid, ranges = build_voxel_grid(room, args.voxel_size)
    print(f"Voxel grid: {grid.shape} ({grid.shape[0]}×{grid.shape[1]}×{grid.shape[2]})")
    print(f"Non-empty voxels: {(grid > 0).sum()}, occupied (≥{args.threshold}): {(grid >= args.threshold).sum()}")

    render_voxels(grid, ranges, args.voxel_size, args.threshold, args.out)
    render_floor_plan(grid, ranges, args.voxel_size, args.threshold, args.floor_out)


if __name__ == "__main__":
    main()
