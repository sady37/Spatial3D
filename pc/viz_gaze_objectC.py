"""Visualize gaze-mode scan of ObjectC vs full-room baseline."""
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from matplotlib.colors import Normalize
import matplotlib.cm as cm

TILT_DEG = 35.0
H_MOUNT = 2.0

def to_room(pts):
    tilt = np.radians(TILT_DEG)
    x, y, z = pts[:, 0], pts[:, 1], pts[:, 2]
    xr = x
    yr = y * np.cos(tilt) + z * np.sin(tilt)
    zr = -y * np.sin(tilt) + z * np.cos(tilt) + H_MOUNT
    mask = np.isfinite(xr) & np.isfinite(yr) & np.isfinite(zr)
    return np.column_stack([xr[mask], yr[mask], zr[mask]])

def draw_block(ax, x, y, z, s, color, alpha):
    verts = [
        [[x,x+s,x+s,x],[y,y,y,y],[z,z,z+s,z+s]],
        [[x,x+s,x+s,x],[y+s,y+s,y+s,y+s],[z,z,z+s,z+s]],
        [[x,x,x,x],[y,y+s,y+s,y],[z,z,z+s,z+s]],
        [[x+s,x+s,x+s,x+s],[y,y+s,y+s,y],[z,z,z+s,z+s]],
        [[x,x+s,x+s,x],[y,y,y+s,y+s],[z,z,z,z]],
        [[x,x+s,x+s,x],[y,y,y+s,y+s],[z+s,z+s,z+s,z+s]],
    ]
    for v in verts:
        poly = [list(zip(v[0], v[1], v[2]))]
        face = Poly3DCollection(poly, alpha=alpha)
        face.set_facecolor(color)
        face.set_edgecolor((0.1, 0.1, 0.1, 0.5))
        ax.add_collection3d(face)

# Load data
gaze = np.load("gaze_objectC_3min.npz")
gaze_static = gaze["static"]
gaze_room = to_room(gaze_static)

baseline = np.load("static_3d_10min.npz")["static"]
baseline_room = to_room(baseline)

print(f"Gaze (3min):    {len(gaze_static)} static pts -> {len(gaze_room)} room pts")
print(f"Baseline (10min): {len(baseline)} static pts -> {len(baseline_room)} room pts")

# --- Figure 1: Point cloud comparison ---
fig, axes = plt.subplots(2, 3, figsize=(18, 12))
fig.suptitle("Gaze Mode ObjectC (3min) vs Full Room Baseline (10min)", fontsize=14)

datasets = [
    (baseline_room, f"Full Room 10min ({len(baseline_room)} pts)"),
    (gaze_room, f"Gaze ObjectC 3min ({len(gaze_room)} pts)"),
]

for row, (room, label) in enumerate(datasets):
    xr, yr, zr = room[:, 0], room[:, 1], room[:, 2]

    ax = axes[row, 0]
    sc = ax.scatter(xr, yr, s=1, alpha=0.3, c=zr, cmap='viridis', vmin=0, vmax=2)
    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y_room (m)")
    ax.set_title(f"Top-down  [{label}]")
    ax.set_xlim(-3, 3)
    ax.set_ylim(0, 7)
    ax.set_aspect('equal')
    ax.axhline(3.9, color='green', ls='--', lw=1, label='ObjectC Y=3.9m')
    ax.axvline(2.1, color='green', ls='--', lw=1, label='ObjectC X=2.1m')
    ax.legend(fontsize=7)
    ax.grid(True, alpha=0.3)

    ax = axes[row, 1]
    ax.scatter(yr, zr, s=1, alpha=0.3, c='steelblue')
    ax.set_xlabel("Y_room (m)")
    ax.set_ylabel("Z_room (m)")
    ax.set_title(f"Side view  [{label}]")
    ax.set_xlim(0, 7)
    ax.set_ylim(-0.5, 2.5)
    ax.axhline(0, color='brown', ls='-', lw=1, label='floor')
    ax.axhline(0.4, color='red', ls='--', lw=0.8, label='fall zone')
    ax.axhline(1.0, color='green', ls='--', lw=0.8, label='ObjectC Z~1.0m')
    ax.legend(fontsize=7)
    ax.grid(True, alpha=0.3)

    ax = axes[row, 2]
    valid = zr[(zr > -0.5) & (zr < 2.5)]
    ax.hist(valid, bins=60, orientation='horizontal', color='steelblue', alpha=0.7)
    ax.set_ylabel("Z_room (m)")
    ax.set_xlabel("Point count")
    ax.set_title(f"Height dist  [{label}]")
    ax.set_ylim(-0.5, 2.5)
    ax.axhline(0.4, color='red', ls='--', lw=0.8, label='fall zone')
    ax.axhline(1.0, color='green', ls='--', lw=0.8, label='ObjectC')
    ax.legend(fontsize=7)
    ax.grid(True, alpha=0.3)

plt.tight_layout()
plt.savefig("gaze_objectC_compare.png", dpi=150, bbox_inches='tight')
print("Saved gaze_objectC_compare.png")

# --- Figure 2: Fine voxel map (0.1m) of gaze data ---
vs = 0.1
x_range, y_range, z_range = (0, 4), (2, 6), (-0.5, 2.5)
nx = int((x_range[1] - x_range[0]) / vs)
ny = int((y_range[1] - y_range[0]) / vs)
nz = int((z_range[1] - z_range[0]) / vs)

def voxelize(room, vs, x_range, y_range, z_range):
    nx = int((x_range[1] - x_range[0]) / vs)
    ny = int((y_range[1] - y_range[0]) / vs)
    nz = int((z_range[1] - z_range[0]) / vs)
    m = ((room[:,0]>=x_range[0]) & (room[:,0]<x_range[1]) &
         (room[:,1]>=y_range[0]) & (room[:,1]<y_range[1]) &
         (room[:,2]>=z_range[0]) & (room[:,2]<z_range[1]))
    p = room[m]
    if len(p) == 0:
        return np.zeros((nx,ny,nz), dtype=int)
    ix = np.clip(((p[:,0]-x_range[0])/vs).astype(int), 0, nx-1)
    iy = np.clip(((p[:,1]-y_range[0])/vs).astype(int), 0, ny-1)
    iz = np.clip(((p[:,2]-z_range[0])/vs).astype(int), 0, nz-1)
    grid = np.zeros((nx,ny,nz), dtype=int)
    np.add.at(grid, (ix,iy,iz), 1)
    return grid

# Compare voxelization at 0.1m
for voxel_size, threshold, label_suffix in [(0.3, 20, "coarse"), (0.1, 5, "fine")]:
    grid_gaze = voxelize(gaze_room, voxel_size, x_range, y_range, z_range)
    grid_base = voxelize(baseline_room, voxel_size, x_range, y_range, z_range)

    n_gaze = (grid_gaze >= threshold).sum()
    n_base = (grid_base >= threshold).sum()
    print(f"\nVoxel {voxel_size}m (threshold={threshold}): gaze={n_gaze} occupied, baseline={n_base} occupied")

    if n_gaze == 0 and n_base == 0:
        continue

    fig = plt.figure(figsize=(20, 10))
    fall_z = int(np.ceil((0.4 - z_range[0]) / voxel_size))

    for idx, (grid, title) in enumerate([
        (grid_base, f"Baseline 10min  ({n_base} voxels @ {voxel_size}m)"),
        (grid_gaze, f"Gaze 3min  ({n_gaze} voxels @ {voxel_size}m)"),
    ]):
        ax = fig.add_subplot(1, 2, idx+1, projection='3d')
        occupied = grid >= threshold
        if not occupied.any():
            ax.set_title(f"{title} — NO DATA")
            continue
        norm = Normalize(vmin=0, vmax=grid[occupied].max())
        nxi, nyi, nzi = grid.shape

        for xi in range(nxi):
            for yi in range(nyi):
                for zi in range(nzi):
                    if not occupied[xi, yi, zi]:
                        continue
                    count = grid[xi, yi, zi]
                    x0 = x_range[0] + xi * voxel_size
                    y0 = y_range[0] + yi * voxel_size
                    z0 = z_range[0] + zi * voxel_size
                    nv = norm(count)
                    if zi < fall_z:
                        color = cm.Reds(nv)
                    else:
                        color = cm.Blues(nv)
                    alpha = max(0.4, nv)
                    draw_block(ax, x0, y0, z0, voxel_size, color, alpha)

        ax.set_xlabel("X (m)")
        ax.set_ylabel("Y (m)")
        ax.set_zlabel("Z (m)")
        ax.set_xlim(x_range)
        ax.set_ylim(y_range)
        ax.set_zlim(z_range)
        ax.set_title(title, fontsize=11)
        ax.view_init(elev=25, azim=-45)

    plt.suptitle(f"ObjectC Region — Voxel {voxel_size}m (threshold>={threshold})", fontsize=14)
    plt.tight_layout()
    fname = f"gaze_objectC_voxel_{label_suffix}.png"
    plt.savefig(fname, dpi=150, bbox_inches='tight')
    print(f"Saved {fname}")

# --- Stats ---
print("\n=== Gaze scan spatial stats ===")
xr, yr, zr = gaze_room[:, 0], gaze_room[:, 1], gaze_room[:, 2]
print(f"  X range: [{xr.min():.2f}, {xr.max():.2f}]")
print(f"  Y range: [{yr.min():.2f}, {yr.max():.2f}]")
print(f"  Z range: [{zr.min():.2f}, {zr.max():.2f}]")
print(f"  Spatial extent: {xr.max()-xr.min():.2f} x {yr.max()-yr.min():.2f} x {zr.max()-zr.min():.2f} m")

# Density comparison in ObjectC region (X=1.5-2.7, Y=3.3-4.5, Z=0.5-1.8)
oc_mask_g = ((xr >= 1.5) & (xr <= 2.7) & (yr >= 3.3) & (yr <= 4.5) & (zr >= 0.5) & (zr <= 1.8))
xb, yb, zb = baseline_room[:, 0], baseline_room[:, 1], baseline_room[:, 2]
oc_mask_b = ((xb >= 1.5) & (xb <= 2.7) & (yb >= 3.3) & (yb <= 4.5) & (zb >= 0.5) & (zb <= 1.8))
volume = 1.2 * 1.2 * 1.3
density_g = oc_mask_g.sum() / volume
density_b = oc_mask_b.sum() / volume
print(f"\n=== ObjectC region density ===")
print(f"  Baseline: {oc_mask_b.sum()} pts in {volume:.2f}m³ = {density_b:.0f} pts/m³")
print(f"  Gaze:     {oc_mask_g.sum()} pts in {volume:.2f}m³ = {density_g:.0f} pts/m³")
if density_b > 0:
    print(f"  Improvement: {density_g/density_b:.1f}x")
