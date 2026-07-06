"""Compare old (3.0dB) vs new (2.0dB) static 3D maps in room coordinates."""
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

old = np.load("static_3d_60s.npz")["static"]
new = np.load("static_3d_map_60s.npz")["static"]

old_room = to_room(old)
new_room = to_room(new)

fig, axes = plt.subplots(2, 3, figsize=(18, 12))
fig.suptitle("Static 3D Map: CFAR 3.0dB (top) vs 2.0dB (bottom) — 60s Room Coordinates", fontsize=14)

datasets = [(old_room, f"3.0 dB  ({len(old_room)} pts)"),
            (new_room, f"2.0 dB  ({len(new_room)} pts)")]

for row, (room, label) in enumerate(datasets):
    xr, yr, zr = room[:, 0], room[:, 1], room[:, 2]

    # Top-down (X vs Y)
    ax = axes[row, 0]
    ax.scatter(xr, yr, s=1, alpha=0.3, c=zr, cmap='viridis', vmin=0, vmax=2)
    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y_room (m) — distance")
    ax.set_title(f"Top-down  [{label}]")
    ax.set_xlim(-3, 3)
    ax.set_ylim(0, 7)
    ax.set_aspect('equal')
    ax.axhline(0.85, color='r', ls='--', lw=0.8, label='desk ~0.85m')
    ax.axhline(3.3, color='orange', ls='--', lw=0.8, label='chairs ~3.3m')
    ax.axhline(6.0, color='blue', ls='--', lw=0.8, label='wall ~6m')
    ax.legend(fontsize=7)
    ax.grid(True, alpha=0.3)

    # Side view (Y vs Z_room)
    ax = axes[row, 1]
    ax.scatter(yr, zr, s=1, alpha=0.3, c='steelblue')
    ax.set_xlabel("Y_room (m) — distance")
    ax.set_ylabel("Z_room (m) — height")
    ax.set_title(f"Side view  [{label}]")
    ax.set_xlim(0, 7)
    ax.set_ylim(-0.5, 2.5)
    ax.axhline(0, color='brown', ls='-', lw=1, label='floor')
    ax.axhline(0.4, color='red', ls='--', lw=0.8, label='fall zone ≤0.4m')
    ax.axhline(0.75, color='green', ls='--', lw=0.8, label='desk height')
    ax.axhline(2.0, color='gray', ls='--', lw=0.8, label='ceiling/mount')
    ax.legend(fontsize=7)
    ax.grid(True, alpha=0.3)

    # Height histogram
    ax = axes[row, 2]
    valid = zr[(zr > -0.5) & (zr < 2.5)]
    ax.hist(valid, bins=50, orientation='horizontal', color='steelblue', alpha=0.7)
    ax.set_ylabel("Z_room (m) — height")
    ax.set_xlabel("Point count")
    ax.set_title(f"Height distribution  [{label}]")
    ax.set_ylim(-0.5, 2.5)
    ax.axhline(0, color='brown', ls='-', lw=1)
    ax.axhline(0.4, color='red', ls='--', lw=0.8, label='fall zone')
    ax.axhline(0.75, color='green', ls='--', lw=0.8, label='desk')
    ax.legend(fontsize=7)
    ax.grid(True, alpha=0.3)

plt.tight_layout()
plt.savefig("static_map_compare.png", dpi=150, bbox_inches='tight')
print(f"Saved static_map_compare.png")
print(f"\nOld (3.0dB): {len(old)} raw → {len(old_room)} room-valid")
print(f"New (2.0dB): {len(new)} raw → {len(new_room)} room-valid")
print(f"Improvement: +{len(new_room)-len(old_room)} pts (+{100*(len(new_room)-len(old_room))/len(old_room):.0f}%)")
