"""Localize the mover: gate range bins by intra-capture motion, then angle it.

Base-free, intra-stream. For each range bin of ONE event cube:
  1. internal motion σ = how much its covariance varies across sub-windows of
     the 20s (static->0, mover->large) — the base-free change signal.
  2. fluctuation covariance R_fluc = R - m·mᴴ — the moving part only.
Keep bins with σ above a gate (drop static floor), run MUSIC on R_fluc to read
the mover's ANGLE, map (bin, az, el) -> room 3D. A real mover should cluster at
one (x,y); range-smear/noise scatters in angle.

    python motion_localize.py --event ev_stand.npz --png stand_loc.png
"""
import argparse

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from spatial3d.cube import Cube
from spatial3d.music import awrl6844_array, music_doa
from spatial3d.music_collect import to_room
from spatial3d.range_music import spherical_to_cart


def internal_sigma(cube_i, nwin=4):
    """Mean pairwise relative covariance diff across nwin sub-windows of one bin."""
    K = cube_i.shape[0]
    w = K // nwin
    Rs = [(cube_i[j * w:(j + 1) * w].conj().T @ cube_i[j * w:(j + 1) * w]) / w
          for j in range(nwin)]
    ds = [np.linalg.norm(Rs[a] - Rs[b]) / (np.linalg.norm(Rs[a]) + 1e-9)
          for a in range(nwin) for b in range(a + 1, nwin)]
    return float(np.mean(ds))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--event", required=True)
    ap.add_argument("--png", default="motion_loc.png")
    ap.add_argument("--out", default=None)
    ap.add_argument("--gate", type=float, default=0.25,
                    help="keep bins with internal σ >= this")
    ap.add_argument("--az-range", type=float, nargs=2, default=[-45, 45])
    ap.add_argument("--el-range", type=float, nargs=2, default=[-45, 30])
    ap.add_argument("--resolution", type=float, default=1.5)
    ap.add_argument("--z-min", type=float, default=-0.3)
    a = ap.parse_args()

    c = Cube.load(a.event)
    array = awrl6844_array()
    rows = []
    for i, b in enumerate(c.bins):
        x = c._valid(i)
        sig = internal_sigma(x)
        if sig < a.gate:                                # drop static bins
            continue
        m = x.mean(axis=0)
        R = (x.conj().T @ x) / len(x)
        Rfluc = R - np.outer(m, m.conj())               # moving part only
        dets = music_doa(Rfluc, array, n_signals=1,
                         az_range=tuple(a.az_range), el_range=tuple(a.el_range),
                         resolution_deg=a.resolution)
        if not dets:
            continue
        az, el, _ = dets[0]                             # strongest peak
        xyz = spherical_to_cart(b * c.dr, np.deg2rad(az), np.deg2rad(el))[0]
        rows.append([xyz[0], xyz[1], xyz[2], sig, float(b), b * c.dr])
    if not rows:
        print("no bins passed the motion gate")
        return
    radar = np.asarray(rows, dtype=np.float32)
    room = to_room(radar)
    room = room[room[:, 2] >= a.z_min]
    if a.out:
        np.savez(a.out, motion_cloud=room)

    sig = room[:, 3]
    s = 40 + 400 * (sig / sig.max())
    fig, ax = plt.subplots(1, 2, figsize=(14, 6))
    ax[0].scatter(room[:, 0], room[:, 1], c=sig, s=s, cmap="viridis",
                  edgecolors="k", linewidths=0.4)
    ax[0].plot(0, 0, "rs", ms=10); ax[0].set_xlim(-3, 3); ax[0].set_ylim(0, 7)
    ax[0].set_aspect("equal"); ax[0].grid(alpha=0.3)
    ax[0].set_xlabel("X (m)"); ax[0].set_ylabel("Y (m) distance")
    ax[0].set_title("TOP-DOWN  (motion, colour=internal σ)")
    sc = ax[1].scatter(room[:, 1], room[:, 2], c=sig, s=s, cmap="viridis",
                       edgecolors="k", linewidths=0.4)
    ax[1].set_xlim(0, 7); ax[1].set_ylim(a.z_min, 2.5)
    ax[1].set_xlabel("Y (m) distance"); ax[1].set_ylabel("Z (m) height")
    ax[1].set_title("SIDE  (height vs distance)"); ax[1].grid(alpha=0.3)
    fig.colorbar(sc, ax=ax[1], shrink=0.7, label="internal σ")
    fig.suptitle(f"Motion localize (σ-gated bins -> fluctuation MUSIC): "
                 f"{a.event}   ({len(room)} bins)")
    plt.tight_layout(); plt.savefig(a.png, dpi=120, bbox_inches="tight"); plt.close()
    print(f"saved {a.png}  ({len(room)} bins, σ gate {a.gate})")


if __name__ == "__main__":
    main()
