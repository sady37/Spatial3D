"""Direct voxel difference: event - base, keep only the change. No classifier.

The most honest view of a change: subtract the two normalised energy-density
voxel grids and keep the positive (appeared) voxels. No fall/standing verdict,
no dominant-mass logic — just where energy showed up that the baseline didn't
have. Exports a coloured PLY (open in CloudCompare / MATLAB pcshow) and a
top-down + side-view PNG (2D projections read better than matplotlib 3D).

    python diff_voxel.py --base base_static.npz --event stand_static.npz \
        --out-ply stand_diff.ply --out-png stand_diff.png
"""
import argparse

import matplotlib
matplotlib.use("Agg")
import matplotlib.cm as cm
import matplotlib.pyplot as plt
import numpy as np

from spatial3d.change import energy_change


def write_ply(path, xyz, mag):
    """ASCII PLY of coloured points (colour = hot colormap by magnitude)."""
    c = (cm.hot(mag / (mag.max() or 1.0))[:, :3] * 255).astype(np.uint8)
    with open(path, "w") as f:
        f.write("ply\nformat ascii 1.0\n")
        f.write(f"element vertex {len(xyz)}\n")
        f.write("property float x\nproperty float y\nproperty float z\n")
        f.write("property uchar red\nproperty uchar green\nproperty uchar blue\n")
        f.write("end_header\n")
        for (x, y, z), (r, g, b) in zip(xyz, c):
            f.write(f"{x:.3f} {y:.3f} {z:.3f} {r} {g} {b}\n")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--base", required=True)
    ap.add_argument("--event", required=True)
    ap.add_argument("--out-ply", default="diff.ply")
    ap.add_argument("--out-png", default="diff.png")
    ap.add_argument("--voxel-size", type=float, default=0.15)
    ap.add_argument("--rel-threshold", type=float, default=0.15,
                    help="keep voxels with diff >= this fraction of the peak")
    ap.add_argument("--z-range", type=float, nargs=2, default=[-0.1, 2.5])
    a = ap.parse_args()

    base = np.load(a.base, allow_pickle=True)["music_cloud"]
    event = np.load(a.event, allow_pickle=True)["music_cloud"]
    diff, meta = energy_change(base, event, voxel_size=a.voxel_size,
                               x_range=(-3, 3), y_range=(0, 7),
                               z_range=tuple(a.z_range))
    xr, yr, zr, vs = meta
    app = np.maximum(diff, 0.0)               # only appeared (positive) energy
    thr = a.rel_threshold * app.max()
    idx = np.argwhere(app > thr)
    if len(idx) == 0:
        print("no appeared voxels above threshold")
        return
    centers = np.array([xr[0], yr[0], zr[0]]) + (idx + 0.5) * vs
    mag = app[app > thr]

    write_ply(a.out_ply, centers, mag)

    # 2D projections: top-down (X-Y) and side (Y-Z), point size/colour = mag
    fig, ax = plt.subplots(1, 2, figsize=(14, 6))
    s = 20 + 380 * (mag / mag.max())
    sc0 = ax[0].scatter(centers[:, 0], centers[:, 1], c=mag, s=s,
                        cmap="hot", edgecolors="k", linewidths=0.3)
    ax[0].plot(0, 0, "cs", ms=10); ax[0].set_xlim(-3, 3); ax[0].set_ylim(0, 7)
    ax[0].set_xlabel("X (m)"); ax[0].set_ylabel("Y (m) distance")
    ax[0].set_title("TOP-DOWN  (appeared energy)"); ax[0].set_aspect("equal")
    ax[0].grid(alpha=0.3)
    sc1 = ax[1].scatter(centers[:, 1], centers[:, 2], c=mag, s=s,
                        cmap="hot", edgecolors="k", linewidths=0.3)
    ax[1].set_xlim(0, 7); ax[1].set_ylim(a.z_range[0], a.z_range[1])
    ax[1].set_xlabel("Y (m) distance"); ax[1].set_ylabel("Z (m) height")
    ax[1].set_title("SIDE  (height vs distance)"); ax[1].grid(alpha=0.3)
    fig.colorbar(sc1, ax=ax[1], shrink=0.7, label="norm energy delta")
    fig.suptitle(f"Voxel diff: {a.event} - {a.base}   ({len(idx)} appeared voxels)")
    plt.tight_layout(); plt.savefig(a.out_png, dpi=120, bbox_inches="tight")
    plt.close()
    print(f"saved {a.out_ply} ({len(idx)} voxels) + {a.out_png}")


if __name__ == "__main__":
    main()
