"""Compare gaze scan: empty room vs person lying near ObjectC."""
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

# Load data
empty = np.load("static_3d_10min.npz")["static"]
lying = np.load("gaze_lying_3min.npz")["static"]
empty_room = to_room(empty)
lying_room = to_room(lying)

print(f"Empty: {len(empty)} static -> {len(empty_room)} room pts")
print(f"Lying: {len(lying)} static -> {len(lying_room)} room pts")

# --- Figure 1: Point cloud comparison ---
fig, axes = plt.subplots(2, 3, figsize=(18, 12))
fig.suptitle("Fall Detection Test: Baseline 10min (top) vs Person Lying 60s (bottom)",
             fontsize=14)

datasets = [
    (empty_room, f"Baseline 10min ({len(empty_room)} pts)"),
    (lying_room, f"Person Lying 60s ({len(lying_room)} pts)"),
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
    ax.axhline(0.4, color='red', ls='--', lw=1.5, label='FALL ZONE')
    ax.axhline(1.0, color='green', ls='--', lw=0.8, label='ObjectC')
    ax.legend(fontsize=7)
    ax.grid(True, alpha=0.3)

    ax = axes[row, 2]
    valid = zr[(zr > -0.5) & (zr < 2.5)]
    ax.hist(valid, bins=60, orientation='horizontal', color='steelblue', alpha=0.7)
    ax.set_ylabel("Z_room (m)")
    ax.set_xlabel("Point count")
    ax.set_title(f"Height dist  [{label}]")
    ax.set_ylim(-0.5, 2.5)
    ax.axhline(0.4, color='red', ls='--', lw=1.5, label='FALL ZONE')
    ax.axhline(1.0, color='green', ls='--', lw=0.8, label='ObjectC')
    ax.legend(fontsize=7)
    ax.grid(True, alpha=0.3)

plt.tight_layout()
plt.savefig("lying_compare_pointcloud.png", dpi=150, bbox_inches='tight')
print("Saved lying_compare_pointcloud.png")

# --- Figure 2: Voxel energy density comparison ---
vs = 0.3
x_range, y_range, z_range = (-3, 3), (0, 7), (-0.5, 2.5)
threshold = 10

grid_empty = voxelize(empty_room, vs, x_range, y_range, z_range)
grid_lying = voxelize(lying_room, vs, x_range, y_range, z_range)
grid_diff = grid_lying - grid_empty

nx, ny, nz = grid_empty.shape
fall_z = int(np.ceil((0.4 - z_range[0]) / vs))

# Print energy density comparison
print(f"\n{'='*60}")
print(f"Voxel Energy Density Comparison (0.3m, room coords)")
print(f"{'='*60}")
print(f"{'Voxel (X,Y,Z)':>25s}  {'Empty':>8s}  {'Lying':>8s}  {'Diff':>8s}  Zone")
print(f"{'-'*60}")

changes = []
for xi in range(nx):
    for yi in range(ny):
        for zi in range(nz):
            e = grid_empty[xi, yi, zi]
            l = grid_lying[xi, yi, zi]
            if e >= threshold or l >= threshold:
                x0 = x_range[0] + xi * vs
                y0 = y_range[0] + yi * vs
                z0 = z_range[0] + zi * vs
                d = l - e
                zone = "FLOOR" if zi < fall_z else "UPPER"
                marker = ""
                if d > threshold:
                    marker = " *** NEW"
                    if zi < fall_z:
                        marker = " *** FALL!"
                elif d < -threshold:
                    marker = " (removed)"
                print(f"  ({x0:+.1f}, {y0:.1f}, {z0:+.1f})  {e:8d}  {l:8d}  {d:+8d}  {zone}{marker}")
                if abs(d) > threshold:
                    changes.append((x0, y0, z0, e, l, d, zone))

print(f"\n=== FALL DETECTION SUMMARY ===")
floor_increase = sum(1 for c in changes if c[5] > 0 and c[6] == "FLOOR")
upper_decrease = sum(1 for c in changes if c[5] < 0 and c[6] == "UPPER")
floor_new = sum(1 for c in changes if c[5] > 0 and c[6] == "FLOOR" and c[3] < threshold)
print(f"  Floor voxels with energy increase: {floor_increase}")
print(f"  Upper voxels with energy decrease: {upper_decrease}")
print(f"  New floor voxels (not in baseline): {floor_new}")
if floor_increase > 0:
    print(f"\n  >>> FALL DETECTED: new mass at floor level <<<")
else:
    print(f"\n  >>> No fall detected <<<")

# --- Figure 3: 3D voxel diff map ---
fig = plt.figure(figsize=(20, 10))

for idx, (grid, title, show_diff) in enumerate([
    (grid_empty, "Empty Baseline", False),
    (grid_lying, "Person Lying", False),
    (grid_diff, "DIFFERENCE (Lying - Empty)", True),
]):
    ax = fig.add_subplot(1, 3, idx+1, projection='3d')

    if show_diff:
        pos = grid > threshold
        neg = grid < -threshold
        has_data = pos | neg
        if has_data.any():
            max_abs = max(grid[pos].max() if pos.any() else 1,
                         abs(grid[neg].min()) if neg.any() else 1)
            norm = Normalize(vmin=0, vmax=max_abs)

        for xi in range(nx):
            for yi in range(ny):
                for zi in range(nz):
                    val = grid[xi, yi, zi]
                    if abs(val) <= threshold:
                        continue
                    x0 = x_range[0] + xi * vs
                    y0 = y_range[0] + yi * vs
                    z0 = z_range[0] + zi * vs
                    nv = norm(abs(val))
                    if val > 0:
                        color = (1.0, 0.0, 0.0, 0.8) if zi < fall_z else (0.0, 0.5, 1.0, 0.8)
                    else:
                        color = (0.5, 0.5, 0.5, 0.4)
                    draw_block(ax, x0, y0, z0, vs, color, max(0.4, nv))
    else:
        occupied = grid >= threshold
        if occupied.any():
            norm = Normalize(vmin=0, vmax=grid[occupied].max())
            for xi in range(nx):
                for yi in range(ny):
                    for zi in range(nz):
                        if not occupied[xi, yi, zi]:
                            continue
                        x0 = x_range[0] + xi * vs
                        y0 = y_range[0] + yi * vs
                        z0 = z_range[0] + zi * vs
                        nv = norm(grid[xi, yi, zi])
                        if zi < fall_z:
                            color = cm.Reds(nv)
                        else:
                            color = cm.Blues(nv)
                        draw_block(ax, x0, y0, z0, vs, color, max(0.4, nv))

    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.set_zlabel("Z (m)")
    ax.set_xlim(x_range)
    ax.set_ylim(y_range)
    ax.set_zlim(z_range)
    ax.set_title(title, fontsize=11)
    ax.view_init(elev=25, azim=-45)

plt.suptitle("Fall Detection: Voxel Baseline Subtraction (0.3m)\n"
             "Red=new floor mass  Blue=new upper  Gray=removed",
             fontsize=13)
plt.tight_layout()
plt.savefig("lying_compare_voxel.png", dpi=150, bbox_inches='tight')
print("Saved lying_compare_voxel.png")

# --- Floor zone energy density ---
print(f"\n=== Floor Zone Energy (Z <= 0.4m) ===")
floor_empty = grid_empty[:, :, :fall_z].sum()
floor_lying = grid_lying[:, :, :fall_z].sum()
upper_empty = grid_empty[:, :, fall_z:].sum()
upper_lying = grid_lying[:, :, fall_z:].sum()
print(f"  Empty floor energy:  {floor_empty}")
print(f"  Lying floor energy:  {floor_lying}  (diff: {floor_lying - floor_empty:+d})")
print(f"  Empty upper energy:  {upper_empty}")
print(f"  Lying upper energy:  {upper_lying}  (diff: {upper_lying - upper_empty:+d})")
if floor_lying > floor_empty * 1.2:
    print(f"  >>> Floor energy increased {floor_lying/max(floor_empty,1):.1f}x — FALL SIGNATURE <<<")
