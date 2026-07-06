"""Annotated voxel map with multiple viewing angles."""
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

data = np.load("static_3d_10min.npz")["static"]
room = to_room(data)

vs = 0.3
x_range, y_range, z_range = (-3, 3), (0, 7), (-0.5, 2.5)
nx, ny, nz = 20, 24, 10

m = ((room[:,0]>=x_range[0]) & (room[:,0]<x_range[1]) &
     (room[:,1]>=y_range[0]) & (room[:,1]<y_range[1]) &
     (room[:,2]>=z_range[0]) & (room[:,2]<z_range[1]))
p = room[m]
ix = np.clip(((p[:,0]-x_range[0])/vs).astype(int), 0, nx-1)
iy = np.clip(((p[:,1]-y_range[0])/vs).astype(int), 0, ny-1)
iz = np.clip(((p[:,2]-z_range[0])/vs).astype(int), 0, nz-1)
grid = np.zeros((nx,ny,nz), dtype=int)
np.add.at(grid, (ix,iy,iz), 1)

threshold = 20
fall_z = int(np.ceil((0.4 - z_range[0]) / vs))
occupied = grid >= threshold
norm = Normalize(vmin=0, vmax=grid[occupied].max())

# Annotations for known objects
annotations = {
    (17,13,5): "ObjectC\n椅子",
    (17,13,6): "ObjectC\n桌上",
}

fig = plt.figure(figsize=(20, 10))
views = [(25, -60, "Front-left view"), (25, 60, "Front-right view")]

for idx, (elev, azim, title) in enumerate(views):
    ax = fig.add_subplot(1, 2, idx+1, projection='3d')

    for xi in range(nx):
        for yi in range(ny):
            for zi in range(nz):
                if not occupied[xi, yi, zi]:
                    continue
                count = grid[xi, yi, zi]
                x0 = x_range[0] + xi * vs
                y0 = y_range[0] + yi * vs
                z0 = z_range[0] + zi * vs
                nv = norm(count)

                if zi < fall_z:
                    color = cm.Reds(nv)
                    alpha = max(0.4, nv)
                else:
                    color = cm.Blues(nv)
                    alpha = max(0.4, nv)

                # ObjectC highlight
                if (xi, yi, zi) in annotations:
                    color = (0.0, 0.8, 0.0, 1.0)
                    alpha = 0.8

                draw_block(ax, x0, y0, z0, vs, color, alpha)

                if (xi, yi, zi) in annotations:
                    ax.text(x0+vs/2, y0+vs/2, z0+vs+0.1,
                            annotations[(xi,yi,zi)],
                            fontsize=8, ha='center', color='green',
                            fontweight='bold')

    # Label known objects
    ax.text(-2, 1.8, 0.8, "桌区\nDesk", fontsize=8, ha='center', color='brown')
    ax.text(0, 6.0, 0.3, "墙面\nWall", fontsize=8, ha='center', color='navy')

    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m) — distance")
    ax.set_zlabel("Z (m) — height")
    ax.set_xlim(x_range)
    ax.set_ylim(y_range)
    ax.set_zlim(z_range)
    ax.set_title(f"{title}\nRed=floor  Green=ObjectC  Blue=furniture/wall", fontsize=11)
    ax.view_init(elev=elev, azim=azim)

plt.suptitle("10min Static Voxel Map (0.3m, ≥20pts) — Room Baseline", fontsize=14)
plt.tight_layout()
plt.savefig("voxel_annotated.png", dpi=150, bbox_inches='tight')
print("Saved voxel_annotated.png")

# Print all occupied voxels
print("\n=== All occupied voxels ===")
for xi in range(nx):
    for yi in range(ny):
        for zi in range(nz):
            if grid[xi,yi,zi] >= threshold:
                x0 = x_range[0] + xi*vs
                y0 = y_range[0] + yi*vs
                z0 = z_range[0] + zi*vs
                zone = "FLOOR" if zi < fall_z else "UPPER"
                tag = annotations.get((xi,yi,zi), "")
                print(f"  ({x0:+.1f}, {y0:.1f}, {z0:+.1f}) = {grid[xi,yi,zi]:5d} pts  [{zone}] {tag}")
