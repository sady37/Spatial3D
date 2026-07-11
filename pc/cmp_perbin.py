"""Per-range-bin covariance comparison (R view): event vs base, data domain.

The rawest honest view: for each range bin, the Frobenius energy of R_base and
R_event, and the RELATIVE change  ||R_event - R_base|| / ||R_base||.  A static
scene -> rel ~ 0 at every bin; a person at range r -> a rel spike at that bin.
No MUSIC, no imaging — just which range bins actually changed.

    python cmp_perbin.py --base empty_A.npz --event evK.npz \
        --label AK --date 0710 --mark 4.0
"""
import argparse
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from spatial3d.range_music import DR_M


def _load(path):
    d = np.load(path, allow_pickle=True)
    bins = d["bins"].astype(int)
    key = "covariances" if "covariances" in d else "cov"
    return {int(b): d[key][i] for i, b in enumerate(bins)}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--base", required=True)
    ap.add_argument("--event", required=True)
    ap.add_argument("--label", required=True, help="pair tag, e.g. AK")
    ap.add_argument("--date", default="0710")
    ap.add_argument("--mark", type=float, default=None,
                    help="range (m) to mark with a dashed line, e.g. 4.0")
    ap.add_argument("--gate", type=float, default=0.15)
    a = ap.parse_args()

    Rb, Re = _load(a.base), _load(a.event)
    bins = sorted(set(Rb) & set(Re))
    rng = np.array([b * DR_M for b in bins])
    nb = np.array([np.linalg.norm(Rb[b]) for b in bins])
    ne = np.array([np.linalg.norm(Re[b]) for b in bins])
    dn = np.array([np.linalg.norm(Re[b] - Rb[b]) for b in bins])
    # Normalise the residual by the MEDIAN baseline energy (a robust global
    # scale) rather than per-bin ||R_b||: the latter divides by ~0 at empty
    # near-noise bins and explodes. This shows "change vs a typical bin".
    rel = dn / (np.median(nb) + 1e-9)

    top = int(np.argmax(rel))
    fig, ax = plt.subplots(2, 1, figsize=(12, 7), sharex=True)
    ax[0].plot(rng, nb, "-", color="steelblue", lw=1.3, label=f"||R_base||  ({a.base})")
    ax[0].plot(rng, ne, "-", color="crimson", lw=1.3, label=f"||R_event|| ({a.event})")
    ax[0].set_ylabel("per-bin covariance energy  ||R||")
    ax[0].legend(fontsize=8, loc="upper right"); ax[0].grid(alpha=0.3)
    ax[0].set_title(f"R view (per range bin) — {a.label}   [{a.date}]", fontsize=12)

    ax[1].plot(rng, rel, "-", color="darkorange", lw=1.5)
    ax[1].axhline(a.gate, ls="--", color="gray", lw=1,
                  label=f"change gate {a.gate}")
    ax[1].fill_between(rng, 0, rel, where=rel >= a.gate, color="orange", alpha=0.3)
    ax[1].set_ylabel("relative change  ||R_e-R_b|| / ||R_b||")
    ax[1].set_xlabel("range (m)  = bin x DR")
    ax[1].grid(alpha=0.3)
    ax[1].annotate(f"peak {rel[top]:.2f} @ {rng[top]:.2f} m (bin {bins[top]})",
                   xy=(rng[top], rel[top]),
                   xytext=(rng[top] + 0.2, rel[top] * 0.95 + 0.02),
                   fontsize=9, color="darkred",
                   arrowprops=dict(arrowstyle="->", color="darkred"))
    if a.mark is not None:
        for x in ax:
            x.axvline(a.mark, ls=":", color="green", lw=1.5)
        ax[1].text(a.mark + 0.05, ax[1].get_ylim()[1] * 0.9,
                   f"{a.mark:.1f} m", color="green", fontsize=9)
    out = f"R_perbin_{a.label}_{a.date}.png"
    plt.tight_layout(); plt.savefig(out, dpi=120, bbox_inches="tight"); plt.close()
    print(f"{a.label}: peak rel={rel[top]:.3f} @ {rng[top]:.2f}m (bin {bins[top]}), "
          f"n_bins>gate={(rel >= a.gate).sum()}  -> {out}")


if __name__ == "__main__":
    main()
