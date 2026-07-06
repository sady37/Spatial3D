"""Compare AoA FOV ±70° vs ±45° static 3D maps in room coordinates."""
import numpy as np
import matplotlib.pyplot as plt

TILT_DEG = 35.0
H_MOUNT = 2.0
tilt = np.radians(TILT_DEG)

def to_room(pts):
    x, y, z = pts[:, 0], pts[:, 1], pts[:, 2]
    xr = x
    yr = y * np.cos(tilt) + z * np.sin(tilt)
    zr = -y * np.sin(tilt) + z * np.cos(tilt) + H_MOUNT
    mask = np.isfinite(xr) & np.isfinite(yr) & np.isfinite(zr)
    return np.column_stack([xr[mask], yr[mask], zr[mask]])

fov70 = np.load("static_3d_map_60s.npz")["static"]
fov45 = np.load("static_3d_fov45_60s.npz")["static"]

r70 = to_room(fov70)
r45 = to_room(fov45)

fig, axes = plt.subplots(2, 3, figsize=(18, 12))
fig.suptitle("AoA FOV: ±70° (top) vs ±45° (bottom) — 60s, CFAR 2.0dB, Room Coords", fontsize=14)

datasets = [(r70, f"±70°  ({len(r70)} pts)"),
            (r45, f"±45°  ({len(r45)} pts)")]

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
    ax.axhline(0.85, color='r', ls='--', lw=0.8, label='desk')
    ax.axhline(3.3, color='orange', ls='--', lw=0.8, label='chairs')
    ax.axhline(6.0, color='blue', ls='--', lw=0.8, label='wall')
    ax.legend(fontsize=7)
    ax.grid(True, alpha=0.3)

    ax = axes[row, 1]
    ax.scatter(yr, zr, s=1, alpha=0.3, c='steelblue')
    ax.set_xlabel("Y_room (m)")
    ax.set_ylabel("Z_room (m) — height")
    ax.set_title(f"Side view  [{label}]")
    ax.set_xlim(0, 7)
    ax.set_ylim(-0.5, 2.5)
    ax.axhline(0, color='brown', ls='-', lw=1, label='floor')
    ax.axhline(0.4, color='red', ls='--', lw=0.8, label='fall zone')
    ax.axhline(0.75, color='green', ls='--', lw=0.8, label='desk height')
    ax.axhline(2.0, color='gray', ls='--', lw=0.8, label='mount')
    ax.legend(fontsize=7)
    ax.grid(True, alpha=0.3)

    ax = axes[row, 2]
    valid = zr[(zr > -0.5) & (zr < 2.5)]
    ax.hist(valid, bins=50, orientation='horizontal', color='steelblue', alpha=0.7)
    ax.set_ylabel("Z_room (m)")
    ax.set_xlabel("Point count")
    ax.set_title(f"Height dist  [{label}]")
    ax.set_ylim(-0.5, 2.5)
    ax.axhline(0, color='brown', ls='-', lw=1)
    ax.axhline(0.4, color='red', ls='--', lw=0.8, label='fall zone')
    ax.axhline(0.75, color='green', ls='--', lw=0.8, label='desk')
    ax.legend(fontsize=7)
    ax.grid(True, alpha=0.3)

plt.tight_layout()
plt.savefig("static_fov_compare.png", dpi=150, bbox_inches='tight')
print("Saved static_fov_compare.png")
print(f"\n±70°: {len(r70)} room pts")
print(f"±45°: {len(r45)} room pts  ({len(r45)-len(r70):+d}, {100*(len(r45)-len(r70))/len(r70):+.1f}%)")

# height stats
for label, room in [("±70°", r70), ("±45°", r45)]:
    zr = room[:, 2]
    v = zr[(zr > -0.5) & (zr < 2.5)]
    floor = np.sum((v >= -0.1) & (v <= 0.4))
    desk = np.sum((v > 0.4) & (v <= 1.0))
    upper = np.sum(v > 1.0)
    print(f"  {label}: floor(≤0.4m)={floor}  desk(0.4-1.0m)={desk}  upper(>1.0m)={upper}")
