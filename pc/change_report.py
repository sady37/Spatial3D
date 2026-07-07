"""Standard change-detection report PNG: event capture vs baseline.

Produces a consistent two-panel top-down figure — FALL ZONE (Z<=0.4m) and FULL
height — of the normalised energy-density difference, with detected change
events marked and a fall alert. Use the same layout for every capture so
results are directly comparable.

    python change_report.py --baseline music_fullroom_voted.npz \
        --event lying_room.npz --out report.png
"""
import argparse

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from spatial3d.change import FALL_Z, energy_change, change_events

X_RANGE = (-3.0, 3.0)
Y_RANGE = (0.0, 7.0)


def _panel(ax, diff, meta, events, title):
    xr, yr, zr, vs = meta
    td = diff.sum(axis=2).T
    v = np.abs(td).max() or 1e-6
    im = ax.imshow(td, origin="lower", extent=[*xr, *yr], aspect="equal",
                   cmap="RdBu_r", vmin=-v, vmax=v)
    ax.set_xlabel("X (m)  left <-> right")
    ax.set_ylabel("Y (m)  distance into room")
    ax.set_title(title, fontsize=11)
    ax.grid(alpha=0.2)
    ax.plot(0, 0, "ks", ms=9)                      # radar at origin (0,0)
    ax.annotate("radar", (0, 0), (0.1, 0.25), fontsize=8)
    for e in events[:6]:
        if e.kind == "appeared":
            ax.plot(e.center[0], e.center[1], "*",
                    color="lime", ms=16, mec="k", mew=0.5)
        else:
            ax.plot(e.center[0], e.center[1], "x", color="blue", ms=12, mew=2)
    return im


def main():
    ap = argparse.ArgumentParser(description="Standard change-detection report")
    ap.add_argument("--baseline", required=True)
    ap.add_argument("--event", required=True)
    ap.add_argument("--out", default="change_report.png")
    ap.add_argument("--voxel-size", type=float, default=0.3)
    ap.add_argument("--fall-alert-mag", type=float, default=0.08,
                    help="Min fall-zone 'appeared' magnitude to raise a FALL alert")
    args = ap.parse_args()

    base = np.load(args.baseline, allow_pickle=True)["music_cloud"]
    event = np.load(args.event, allow_pickle=True)["music_cloud"]

    kw = dict(voxel_size=args.voxel_size, x_range=X_RANGE, y_range=Y_RANGE)
    diff_full, meta_full = energy_change(base, event, z_range=(-0.1, 2.5), **kw)
    diff_fall, meta_fall = energy_change(base, event, z_range=(-0.1, FALL_Z), **kw)
    diff_tall, meta_tall = energy_change(base, event, z_range=(1.0, 1.8), **kw)
    ev_full = change_events(diff_full, meta_full, rel_threshold=0.3, min_voxels=1)
    ev_fall = change_events(diff_fall, meta_fall, rel_threshold=0.3, min_voxels=1)

    # A standing person also puts energy in the fall zone (feet), so a fall is a
    # low-lying appeared mass whose energy is DOMINANTLY low vs the column above
    # (per the design: fall = energy redistributes DOWN, no standing column).
    vs = args.voxel_size
    low_col = np.maximum(diff_fall.sum(axis=2), 0.0)    # (nx,ny) appeared low
    tall_col = np.maximum(diff_tall.sum(axis=2), 0.0)   # (nx,ny) appeared high

    def col_energy(grid, cx, cy, r=0.5):
        ix = int((cx - X_RANGE[0]) / vs); iy = int((cy - Y_RANGE[0]) / vs)
        rr = int(r / vs)
        sub = grid[max(0, ix - rr):ix + rr + 1, max(0, iy - rr):iy + rr + 1]
        return float(sub.max()) if sub.size else 0.0

    def is_fall(e):
        lo = col_energy(low_col, e.center[0], e.center[1])
        hi = col_energy(tall_col, e.center[0], e.center[1])
        return lo > 1.5 * hi                            # energy dominantly low

    strong = [e for e in ev_fall if e.kind == "appeared"
              and e.magnitude >= args.fall_alert_mag]
    fall_hits = [e for e in strong if is_fall(e)]
    standing = [e for e in strong if not is_fall(e)]
    alert = bool(fall_hits)

    fig, axes = plt.subplots(1, 2, figsize=(13, 6.5))
    im0 = _panel(axes[0], diff_fall, meta_fall, ev_fall,
                 f"FALL ZONE  Z<={FALL_Z}m   (fall/lie signal)")
    im1 = _panel(axes[1], diff_full, meta_full, ev_full,
                 "FULL HEIGHT   (all changes)")
    fig.colorbar(im0, ax=axes[0], shrink=0.6, label="norm energy delta")
    fig.colorbar(im1, ax=axes[1], shrink=0.6, label="norm energy delta")

    top = (fall_hits[0] if fall_hits else
           (standing[0] if standing else (ev_fall[0] if ev_fall else None)))
    if alert:
        status = "FALL / LIE DETECTED"
    elif standing:
        status = "person present (standing/upright)"
    else:
        status = "no fall-zone alert"
    loc = (f"  @ X={top.center[0]:+.1f} Y={top.center[1]:.1f} "
           f"mag={top.magnitude:.3f}" if top else "")
    fig.suptitle(f"Change report: {args.event} vs {args.baseline}\n"
                 f"[{status}]{loc}   (green*=appeared, blue x=gone, black=radar)",
                 fontsize=12,
                 color=("red" if alert else "black"))
    plt.tight_layout()
    plt.savefig(args.out, dpi=120, bbox_inches="tight")
    plt.close()

    print(f"{'*** ' + status + ' ***' if alert else status}")
    print(f"  fall-zone events: {len(ev_fall)}, full events: {len(ev_full)}")
    for e in ev_fall[:5]:
        is_fall = any(e is h for h in fall_hits)
        print(f"    {e.kind:8s} X={e.center[0]:+.1f} Y={e.center[1]:.1f} "
              f"Z={e.center[2]:.2f} mag={e.magnitude:.3f}"
              f"{' [FALL]' if is_fall else ''}")
    print(f"  saved {args.out}")


if __name__ == "__main__":
    main()
