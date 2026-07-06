"""Full-room voxel energy density map from 10-min accumulated point cloud.

This is the '29-deg FFT baseline' that will later be compared against
MUSIC high-resolution maps.
"""
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.colors import LogNorm, Normalize
import matplotlib.cm as cm
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

# ---------- constants ----------
TILT_DEG = 35.0
H_MOUNT = 2.0

X_RANGE = (-3, 3)
Y_RANGE = (0, 7)
Z_RANGE = (-0.5, 2.5)

HEIGHT_ZONES = [
    ("floor",     0.0, 0.3),
    ("table/chair", 0.3, 0.7),
    ("desk",      0.7, 1.2),
    ("standing",  1.2, 2.0),
]


# ---------- helpers ----------
def to_room(pts):
    """Radar -> room coordinate transform."""
    tilt = np.radians(TILT_DEG)
    x, y, z = pts[:, 0], pts[:, 1], pts[:, 2]
    xr = x
    yr = y * np.cos(tilt) + z * np.sin(tilt)
    zr = -y * np.sin(tilt) + z * np.cos(tilt) + H_MOUNT
    mask = np.isfinite(xr) & np.isfinite(yr) & np.isfinite(zr)
    return np.column_stack([xr[mask], yr[mask], zr[mask]])


def voxelize_2d(x, y, vs, x_range, y_range):
    """2D histogram with voxel size vs."""
    nx = int(round((x_range[1] - x_range[0]) / vs))
    ny = int(round((y_range[1] - y_range[0]) / vs))
    mask = ((x >= x_range[0]) & (x < x_range[1]) &
            (y >= y_range[0]) & (y < y_range[1]))
    xp, yp = x[mask], y[mask]
    ix = np.clip(((xp - x_range[0]) / vs).astype(int), 0, nx - 1)
    iy = np.clip(((yp - y_range[0]) / vs).astype(int), 0, ny - 1)
    grid = np.zeros((nx, ny), dtype=int)
    np.add.at(grid, (ix, iy), 1)
    return grid


def voxelize_3d(room, vs, x_range, y_range, z_range):
    """3D histogram with voxel size vs."""
    nx = int(round((x_range[1] - x_range[0]) / vs))
    ny = int(round((y_range[1] - y_range[0]) / vs))
    nz = int(round((z_range[1] - z_range[0]) / vs))
    m = ((room[:, 0] >= x_range[0]) & (room[:, 0] < x_range[1]) &
         (room[:, 1] >= y_range[0]) & (room[:, 1] < y_range[1]) &
         (room[:, 2] >= z_range[0]) & (room[:, 2] < z_range[1]))
    p = room[m]
    if len(p) == 0:
        return np.zeros((nx, ny, nz), dtype=int)
    ix = np.clip(((p[:, 0] - x_range[0]) / vs).astype(int), 0, nx - 1)
    iy = np.clip(((p[:, 1] - y_range[0]) / vs).astype(int), 0, ny - 1)
    iz = np.clip(((p[:, 2] - z_range[0]) / vs).astype(int), 0, nz - 1)
    grid = np.zeros((nx, ny, nz), dtype=int)
    np.add.at(grid, (ix, iy, iz), 1)
    return grid


def draw_block(ax, x, y, z, s, color, alpha):
    """Draw a 3D cube at (x,y,z) with side length s."""
    verts = [
        [[x, x + s, x + s, x], [y, y, y, y], [z, z, z + s, z + s]],
        [[x, x + s, x + s, x], [y + s, y + s, y + s, y + s], [z, z, z + s, z + s]],
        [[x, x, x, x], [y, y + s, y + s, y], [z, z, z + s, z + s]],
        [[x + s, x + s, x + s, x + s], [y, y + s, y + s, y], [z, z, z + s, z + s]],
        [[x, x + s, x + s, x], [y, y, y + s, y + s], [z, z, z, z]],
        [[x, x + s, x + s, x], [y, y, y + s, y + s], [z + s, z + s, z + s, z + s]],
    ]
    for v in verts:
        poly = [list(zip(v[0], v[1], v[2]))]
        face = Poly3DCollection(poly, alpha=alpha)
        face.set_facecolor(color)
        face.set_edgecolor((0.1, 0.1, 0.1, 0.3))
        ax.add_collection3d(face)


# ---------- main ----------
def main():
    # Load data
    data = np.load("static_3d_10min.npz")
    static = data["static"]
    print(f"Loaded static_3d_10min.npz: {len(static)} static points")
    print(f"  Radar X: [{static[:,0].min():.2f}, {static[:,0].max():.2f}]")
    print(f"  Radar Y: [{static[:,1].min():.2f}, {static[:,1].max():.2f}]")
    print(f"  Radar Z: [{static[:,2].min():.2f}, {static[:,2].max():.2f}]")

    # Transform to room coordinates
    room = to_room(static)
    xr, yr, zr = room[:, 0], room[:, 1], room[:, 2]
    print(f"\nRoom-transformed: {len(room)} points")
    print(f"  Room X: [{xr.min():.2f}, {xr.max():.2f}]")
    print(f"  Room Y: [{yr.min():.2f}, {yr.max():.2f}]")
    print(f"  Room Z: [{zr.min():.2f}, {zr.max():.2f}]")

    # ====== Figure 1: Top-down floor plan ======
    print("\n--- Figure 1: Top-down floor plan ---")
    vs = 0.1
    grid_td = voxelize_2d(xr, yr, vs, X_RANGE, Y_RANGE)

    fig, ax = plt.subplots(figsize=(8, 10))
    grid_plot = grid_td.T.astype(float)
    grid_plot[grid_plot == 0] = np.nan
    extent = [X_RANGE[0], X_RANGE[1], Y_RANGE[0], Y_RANGE[1]]
    im = ax.imshow(grid_plot, origin='lower', extent=extent, aspect='equal',
                   cmap='hot', norm=LogNorm(vmin=1, vmax=grid_td.max()),
                   interpolation='nearest')
    cbar = plt.colorbar(im, ax=ax, shrink=0.8, label='Point count per voxel (log)')

    # Mark ObjectC
    ax.plot(1.5, 3.9, 'gD', markersize=10, markeredgecolor='white', linewidth=2,
            label='ObjectC (~1.5, 3.9)')
    ax.legend(loc='upper right', fontsize=9)

    # Grid lines at 1m
    ax.set_xticks(range(int(X_RANGE[0]), int(X_RANGE[1]) + 1))
    ax.set_yticks(range(int(Y_RANGE[0]), int(Y_RANGE[1]) + 1))
    ax.grid(True, alpha=0.3, color='white', linewidth=0.5)
    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y_room (m)")
    ax.set_title("Room Floor Plan — 29° FFT, 10min accumulation, 0.1m voxels")
    ax.set_xlim(X_RANGE)
    ax.set_ylim(Y_RANGE)

    plt.savefig("room_map_topdown.png", dpi=150, bbox_inches='tight')
    plt.close()
    print("  Saved room_map_topdown.png")

    # ====== Figure 2: Height slices ======
    print("\n--- Figure 2: Height slices ---")
    fig, axes = plt.subplots(2, 2, figsize=(14, 16))
    fig.suptitle("Room Height Slices — 29° FFT, 10min, 0.1m voxels", fontsize=14)

    for idx, (zone_name, z_lo, z_hi) in enumerate(HEIGHT_ZONES):
        ax = axes[idx // 2, idx % 2]
        mask = (zr >= z_lo) & (zr < z_hi)
        x_slice, y_slice = xr[mask], yr[mask]
        n_pts = mask.sum()

        grid_slice = voxelize_2d(x_slice, y_slice, vs, X_RANGE, Y_RANGE)
        grid_plot = grid_slice.T.astype(float)
        grid_plot[grid_plot == 0] = np.nan

        vmax = max(grid_slice.max(), 1)
        im = ax.imshow(grid_plot, origin='lower', extent=extent, aspect='equal',
                       cmap='hot', norm=LogNorm(vmin=1, vmax=vmax),
                       interpolation='nearest')
        plt.colorbar(im, ax=ax, shrink=0.8, label='Hits')

        ax.set_xticks(range(int(X_RANGE[0]), int(X_RANGE[1]) + 1))
        ax.set_yticks(range(int(Y_RANGE[0]), int(Y_RANGE[1]) + 1))
        ax.grid(True, alpha=0.3, color='white', linewidth=0.5)
        ax.set_xlabel("X (m)")
        ax.set_ylabel("Y_room (m)")
        ax.set_title(f"{zone_name}: Z=[{z_lo:.1f}, {z_hi:.1f})m  ({n_pts} pts)")
        ax.set_xlim(X_RANGE)
        ax.set_ylim(Y_RANGE)

    plt.tight_layout()
    plt.savefig("room_map_height_slices.png", dpi=150, bbox_inches='tight')
    plt.close()
    print("  Saved room_map_height_slices.png")

    # ====== Figure 3: Side view ======
    print("\n--- Figure 3: Side view ---")
    grid_side = voxelize_2d(yr, zr, vs, Y_RANGE, Z_RANGE)

    fig, ax = plt.subplots(figsize=(12, 5))
    grid_plot = grid_side.T.astype(float)
    grid_plot[grid_plot == 0] = np.nan
    extent_side = [Y_RANGE[0], Y_RANGE[1], Z_RANGE[0], Z_RANGE[1]]
    im = ax.imshow(grid_plot, origin='lower', extent=extent_side, aspect='equal',
                   cmap='hot', norm=LogNorm(vmin=1, vmax=grid_side.max()),
                   interpolation='nearest')
    plt.colorbar(im, ax=ax, shrink=0.8, label='Point count per voxel (log)')

    # Floor and ceiling lines
    ax.axhline(0.0, color='lime', ls='-', lw=1.5, label='Floor (Z=0)')
    ax.axhline(2.0, color='cyan', ls='-', lw=1.5, label='Ceiling (Z=2.0)')

    # Height zone shading
    zone_colors = [(0.0, 0.4, 'red', 'Floor (0-0.4m)'),
                   (0.4, 1.0, 'yellow', 'Furniture (0.4-1.0m)'),
                   (1.0, 2.0, 'green', 'Standing (1.0-2.0m)')]
    for z0, z1, c, lbl in zone_colors:
        ax.axhspan(z0, z1, alpha=0.08, color=c)
        ax.axhline(z0, color=c, ls='--', lw=0.8, alpha=0.5)

    ax.set_xlabel("Y_room (m)")
    ax.set_ylabel("Z_room (m)")
    ax.set_title("Room Side View — 29° FFT, 10min, 0.1m voxels")
    ax.set_xlim(Y_RANGE)
    ax.set_ylim(Z_RANGE)
    ax.legend(loc='upper right', fontsize=8)
    ax.grid(True, alpha=0.2)

    plt.savefig("room_map_side.png", dpi=150, bbox_inches='tight')
    plt.close()
    print("  Saved room_map_side.png")

    # ====== Figure 4: 3D voxel visualization ======
    print("\n--- Figure 4: 3D voxel visualization ---")
    vs3d = 0.2
    min_hits = 5
    grid_3d = voxelize_3d(room, vs3d, X_RANGE, Y_RANGE, Z_RANGE)

    nx, ny, nz = grid_3d.shape
    occupied = grid_3d >= min_hits

    # Collect occupied voxels
    voxels = []
    for xi in range(nx):
        for yi in range(ny):
            for zi in range(nz):
                if occupied[xi, yi, zi]:
                    x0 = X_RANGE[0] + xi * vs3d
                    y0 = Y_RANGE[0] + yi * vs3d
                    z0 = Z_RANGE[0] + zi * vs3d
                    voxels.append((x0, y0, z0, grid_3d[xi, yi, zi]))

    print(f"  3D voxels with >= {min_hits} hits: {len(voxels)}")

    max_hits = max(v[3] for v in voxels) if voxels else 1
    norm = Normalize(vmin=0, vmax=2.0)  # Normalize by Z_room for coloring

    fig = plt.figure(figsize=(20, 9))
    views = [(30, -45), (60, -135)]
    for vidx, (elev, azim) in enumerate(views):
        ax = fig.add_subplot(1, 2, vidx + 1, projection='3d')

        for (x0, y0, z0, hits) in voxels:
            z_center = z0 + vs3d / 2
            # Color by height: red=floor, blue=mid, green=high
            if z_center < 0.5:
                color = (0.9, 0.2, 0.1)   # red — floor
            elif z_center < 1.2:
                color = (0.2, 0.4, 0.9)   # blue — mid
            else:
                color = (0.2, 0.8, 0.3)   # green — high
            # Alpha by energy density
            alpha = 0.2 + 0.7 * min(hits / max_hits, 1.0)
            draw_block(ax, x0, y0, z0, vs3d, color, alpha)

        ax.set_xlabel("X (m)")
        ax.set_ylabel("Y_room (m)")
        ax.set_zlabel("Z_room (m)")
        ax.set_xlim(X_RANGE)
        ax.set_ylim(Y_RANGE)
        ax.set_zlim(Z_RANGE)
        ax.view_init(elev=elev, azim=azim)
        ax.set_title(f"elev={elev}, azim={azim}")

    plt.suptitle(f"Room 3D Voxel Map — 29° FFT, 10min, 0.2m voxels, >= {min_hits} hits\n"
                 "Red=floor  Blue=mid  Green=high  |  Alpha = energy density",
                 fontsize=13)
    plt.tight_layout()
    plt.savefig("room_map_3d.png", dpi=150, bbox_inches='tight')
    plt.close()
    print("  Saved room_map_3d.png")

    # ====== Summary table ======
    print("\n" + "=" * 70)
    print("ROOM MAP SUMMARY  --  29-deg FFT Baseline, 10min accumulation")
    print("=" * 70)

    print(f"\nTotal raw static points:          {len(static)}")
    print(f"Room-transformed points:          {len(room)}")

    # In-bounds points
    in_bounds = ((xr >= X_RANGE[0]) & (xr < X_RANGE[1]) &
                 (yr >= Y_RANGE[0]) & (yr < Y_RANGE[1]) &
                 (zr >= Z_RANGE[0]) & (zr < Z_RANGE[1]))
    print(f"In-bounds points (room):          {in_bounds.sum()}")

    # Occupied voxels at different resolutions
    for res in [0.1, 0.2, 0.3]:
        g = voxelize_3d(room, res, X_RANGE, Y_RANGE, Z_RANGE)
        occ = (g > 0).sum()
        total = g.size
        print(f"Occupied voxels @ {res:.1f}m:           {occ:6d} / {total:6d}  ({100*occ/total:.1f}%)")

    # Per-height-zone statistics
    print(f"\n{'Zone':<15s}  {'Z range':>10s}  {'Points':>8s}  {'Frac':>6s}  {'Occ 0.1m':>9s}")
    print("-" * 55)
    for zone_name, z_lo, z_hi in HEIGHT_ZONES:
        zmask = (zr >= z_lo) & (zr < z_hi) & in_bounds
        n = zmask.sum()
        frac = n / max(in_bounds.sum(), 1)
        g_zone = voxelize_2d(xr[zmask], yr[zmask], 0.1, X_RANGE, Y_RANGE)
        occ = (g_zone > 0).sum()
        print(f"{zone_name:<15s}  [{z_lo:.1f}, {z_hi:.1f})m  {n:8d}  {frac:5.1%}  {occ:9d}")

    # Top 10 densest voxels (0.1m)
    grid_full = voxelize_3d(room, 0.1, X_RANGE, Y_RANGE, Z_RANGE)
    flat = grid_full.ravel()
    top_idx = np.argsort(flat)[::-1][:10]
    nx_f, ny_f, nz_f = grid_full.shape

    print(f"\nTop 10 densest voxels (0.1m):")
    print(f"  {'Rank':>4s}  {'X':>6s}  {'Y':>6s}  {'Z':>6s}  {'Hits':>6s}  Zone")
    print(f"  {'-'*45}")
    for rank, idx in enumerate(top_idx, 1):
        xi, rem = divmod(idx, ny_f * nz_f)
        yi, zi = divmod(rem, nz_f)
        x0 = X_RANGE[0] + xi * 0.1
        y0 = Y_RANGE[0] + yi * 0.1
        z0 = Z_RANGE[0] + zi * 0.1
        hits = flat[idx]
        zone = "?"
        for zn, zl, zh in HEIGHT_ZONES:
            if zl <= z0 + 0.05 < zh:
                zone = zn
                break
        if z0 + 0.05 < HEIGHT_ZONES[0][1]:
            zone = "below floor"
        if z0 + 0.05 >= HEIGHT_ZONES[-1][2]:
            zone = "above standing"
        print(f"  {rank:4d}  {x0:+5.1f}  {y0:5.1f}  {z0:+5.1f}  {hits:6d}  {zone}")

    print(f"\n{'='*70}")
    print("All figures saved. Done.")


if __name__ == "__main__":
    main()
