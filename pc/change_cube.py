"""Data-domain change detection: subtract covariances FIRST, then image.

The reconstruct-then-subtract pipeline (MUSIC each capture, diff the clouds)
fills the room with noise: MUSIC re-images the SAME static clutter to slightly
different voxels each run (angle-estimate variance) and hallucinates a peak in
every empty bin, so the cloud difference measures reconstruction jitter, not
physical change.

Here we subtract in the DATA domain, per range bin:

    D = R_event - R_base

For an unchanged bin the same static reflectors give R_event ≈ R_base → D ≈ 0 →
nothing to image. Only bins whose covariance actually changed (a person showed
up) have a non-zero D. We gate bins by the residual energy, then read the change
angle with the difference beamformer  P(θ) = aᴴ D a  (positive = appeared,
negative = gone) — its aᴴ R a terms cancel exactly for identical static clutter,
so no jitter noise survives. Peaks map to room-frame 3D points.

    python change_cube.py --base base_cube.npz --event ev_stand.npz \
        --out stand_change.npz --png stand_change.png
"""
import argparse

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from spatial3d.cube import Cube
from spatial3d.music import awrl6844_array, bartlett_spectrum_2d
from spatial3d.music_collect import to_room
from spatial3d.range_music import DR_M, spherical_to_cart


def _load_cov(path):
    """Per-bin covariance dict from a cube npz or a covariance npz."""
    d = np.load(path, allow_pickle=True)
    if "snapshots" in d:
        return Cube.load(path).covariances()
    bins = d["bins"].astype(int)
    return {int(b): d["covariances"][i] for i, b in enumerate(bins)}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--base", required=True)
    ap.add_argument("--event", required=True)
    ap.add_argument("--out", default="change_cube.npz")
    ap.add_argument("--png", default="change_cube.png")
    ap.add_argument("--az-range", type=float, nargs=2, default=[-45, 45])
    ap.add_argument("--el-range", type=float, nargs=2, default=[-45, 30])
    ap.add_argument("--resolution", type=float, default=1.0)
    ap.add_argument("--gate", type=float, default=0.15,
                    help="keep bins whose peak appeared-energy >= this fraction "
                         "of the strongest bin's")
    ap.add_argument("--z-min", type=float, default=-0.3)
    a = ap.parse_args()

    Rb = _load_cov(a.base)
    Re = _load_cov(a.event)
    array = awrl6844_array()
    az = np.deg2rad(np.arange(a.az_range[0], a.az_range[1] + .5, a.resolution))
    el = np.deg2rad(np.arange(a.el_range[0], a.el_range[1] + .5, a.resolution))

    bins = sorted(set(Rb) & set(Re))
    # difference beamformer per bin; peak POSITIVE change = appeared energy.
    # Gate on the RELATIVE change ||D||/||R_base|| so a bin is kept only when its
    # covariance changed a lot vs its own clutter — strong near clutter has large
    # absolute residual noise but small relative change, so it no longer wins.
    per_bin = []
    for b in bins:
        D = Re[b] - Rb[b]
        rel = np.linalg.norm(D) / (np.linalg.norm(Rb[b]) + 1e-9)
        S = bartlett_spectrum_2d(D, array, az, el)      # aᴴ D a (real, signed)
        i, j = np.unravel_index(np.argmax(S), S.shape)
        per_bin.append((b, float(S[i, j]), az[i], el[j], float(rel)))

    peak = max((p[1] for p in per_bin), default=0.0)
    rows = []
    for b, pw, ai, ej, rel in per_bin:
        if rel < a.gate or pw <= 0:                     # gate on relative change
            continue
        r = b * DR_M
        xyz = spherical_to_cart(r, ai, ej)[0]
        rows.append([xyz[0], xyz[1], xyz[2], pw, float(b), r])
    kept = len(rows)
    if not rows:
        print("no bins passed the change gate — scene unchanged")
        return
    radar = np.asarray(rows, dtype=np.float32)
    room = to_room(radar)
    room = room[room[:, 2] >= a.z_min]
    np.savez(a.out, change_cloud=room, dr_m=np.float32(DR_M))

    # top-down + side, point size/colour = appeared energy
    mag = room[:, 3]
    s = 40 + 400 * (mag / (mag.max() or 1))
    fig, ax = plt.subplots(1, 2, figsize=(14, 6))
    ax[0].scatter(room[:, 0], room[:, 1], c=mag, s=s, cmap="hot",
                  edgecolors="k", linewidths=0.4)
    ax[0].plot(0, 0, "cs", ms=10); ax[0].set_xlim(-3, 3); ax[0].set_ylim(0, 7)
    ax[0].set_aspect("equal"); ax[0].grid(alpha=0.3)
    ax[0].set_xlabel("X (m)"); ax[0].set_ylabel("Y (m) distance")
    ax[0].set_title("TOP-DOWN  (appeared energy)")
    sc = ax[1].scatter(room[:, 1], room[:, 2], c=mag, s=s, cmap="hot",
                       edgecolors="k", linewidths=0.4)
    ax[1].set_xlim(0, 7); ax[1].set_ylim(a.z_min, 2.5)
    ax[1].set_xlabel("Y (m) distance"); ax[1].set_ylabel("Z (m) height")
    ax[1].set_title("SIDE  (height vs distance)"); ax[1].grid(alpha=0.3)
    fig.colorbar(sc, ax=ax[1], shrink=0.7, label="appeared energy")
    fig.suptitle(f"Data-domain change (R_event-R_base then image): "
                 f"{a.event} - {a.base}   ({kept} bins)")
    plt.tight_layout(); plt.savefig(a.png, dpi=120, bbox_inches="tight"); plt.close()
    print(f"saved {a.out} + {a.png}  ({kept} bins passed gate, peak={peak:.1f})")


if __name__ == "__main__":
    main()
