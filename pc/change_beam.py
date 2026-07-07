"""Change detection in the DISTORTED reconstruction frame — keep z<0.

The elevation estimate is biased but REPEATABLE (systematic, 0-2° across
captures). So the reconstruction is distorted but CONSISTENT — a wall lands at
the same wrong (x,y,z) in base and event. Therefore: reconstruct base and event
IDENTICALLY (same beamformer energy volume, keeping z<0), normalise each, and
difference. The systematic angle distortion cancels; a new mass (the person)
appears as positive energy at its distorted-but-consistent location.

Uses the Bartlett beamformer energy P(az,el)=aᴴRa (smooth, no MUSIC peak-
hallucination) mapped through spherical_to_cart + to_room into a 3D energy grid.

    python change_beam.py --base base_cube.npz --event ev_lie_legs.npz --png x.png
"""
import argparse

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from spatial3d.cube import Cube
from spatial3d.music import awrl6844_array
from spatial3d.music_collect import to_room
from spatial3d.range_music import spherical_to_cart

XR, YR, ZR, VS = (-3, 3), (0, 7), (-2.5, 2.5), 0.2


def covs_of(path):
    d = np.load(path, allow_pickle=True)
    if "snapshots" in d:
        c = Cube.load(path); return c.covariances(), c.dr
    b = d["bins"].astype(int)
    return {int(x): d["covariances"][i] for i, x in enumerate(b)}, float(d["dr_m"])


def energy_volume(covs, dr, array, az, el):
    """Beamformer energy accumulated into a (x,y,z) grid, keeping z<0."""
    nx = int((XR[1] - XR[0]) / VS); ny = int((YR[1] - YR[0]) / VS)
    nz = int((ZR[1] - ZR[0]) / VS)
    G = np.zeros((nx, ny, nz))
    A = array.steering_matrix(az, el)                        # (naz,nel,16)
    for b, R in covs.items():
        P = np.einsum("ijk,kl,ijl->ij", A.conj(), R, A).real  # (naz,nel)
        r = b * dr
        # map every (az,el) cell to room (x,y,z)
        for ia in range(len(az)):
            xyz = spherical_to_cart(r, np.full(len(el), az[ia]), el)  # (nel,3)
            room = to_room(xyz)
            ix = ((room[:, 0] - XR[0]) / VS).astype(int)
            iy = ((room[:, 1] - YR[0]) / VS).astype(int)
            iz = ((room[:, 2] - ZR[0]) / VS).astype(int)
            ok = ((ix >= 0) & (ix < nx) & (iy >= 0) & (iy < ny) &
                  (iz >= 0) & (iz < nz))
            np.add.at(G, (ix[ok], iy[ok], iz[ok]), P[ia, ok])
    s = G.sum()
    return G / s if s > 0 else G


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--base", required=True)
    ap.add_argument("--event", required=True)
    ap.add_argument("--png", default="change_beam.png")
    ap.add_argument("--res", type=float, default=3.0)
    a = ap.parse_args()

    array = awrl6844_array()
    az = np.deg2rad(np.arange(-45, 46, a.res))
    el = np.deg2rad(np.arange(-45, 31, a.res))
    Cb, drb = covs_of(a.base); Ce, dre = covs_of(a.event)
    Gb = energy_volume(Cb, drb, array, az, el)
    Ge = energy_volume(Ce, dre, array, az, el)
    D = Ge - Gb                                              # appeared = positive
    app = np.maximum(D, 0.0)

    xr = np.linspace(XR[0], XR[1], app.shape[0])
    yr = np.linspace(YR[0], YR[1], app.shape[1])
    zr = np.linspace(ZR[0], ZR[1], app.shape[2])
    top = app.sum(2).T                                       # (y,x)
    side = app.sum(0).T                                      # (z,y)

    fig, ax = plt.subplots(1, 2, figsize=(14, 6))
    ax[0].imshow(top, origin="lower", extent=[*XR, *YR], aspect="equal",
                 cmap="hot"); ax[0].plot(0, 0, "cs", ms=9)
    ax[0].set_xlabel("X (m)"); ax[0].set_ylabel("Y (m) distance")
    ax[0].set_title("TOP-DOWN  appeared energy (event-base)")
    im = ax[1].imshow(side, origin="lower", extent=[*YR, *ZR], aspect="auto",
                      cmap="hot")
    ax[1].axhline(0, color="cyan", lw=1, ls="--")           # floor Z=0
    ax[1].set_xlabel("Y (m) distance"); ax[1].set_ylabel("Z (m) height  (dashed=floor)")
    ax[1].set_title("SIDE  (keeps z<0 distorted frame)")
    fig.colorbar(im, ax=ax[1], shrink=0.7, label="norm appeared energy")
    # peak
    pk = np.unravel_index(app.argmax(), app.shape)
    fig.suptitle(f"Distorted-frame change: {a.event} - {a.base}   "
                 f"peak @ X={xr[pk[0]]:+.1f} Y={yr[pk[1]]:.1f} Z={zr[pk[2]]:+.1f}m")
    plt.tight_layout(); plt.savefig(a.png, dpi=120, bbox_inches="tight"); plt.close()
    print(f"saved {a.png}  peak X={xr[pk[0]]:+.2f} Y={yr[pk[1]]:.2f} Z={zr[pk[2]]:+.2f}m")


if __name__ == "__main__":
    main()
