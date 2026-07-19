"""Half-kneel descent: recover height trajectory from the track-bin-cube.

Premise (confirmed by hk_recon.py): during a half-kneel fall the CFAR point
cloud dies for ~5 s (pcN=0) while the track-bin-cube keeps streaming ~7
zero-Doppler 16-antenna snapshots per frame (e_vec). Those snapshots carry the
angle information the point cloud lost. Single-snapshot Bartlett beamforming of
each e_vec gives (az, el); with the event's own range that is a 3D point, and
the highest body point per frame is a height trajectory THROUGH the blind
window.

    z_radar(event) = r * sin(el)                 # bias-free vertical, radar frame
    room height    = to_room(spherical_to_cart)  # tilt/mount applied (calibrated)

Deliverables per fall-take (dropout window):
  - top-of-body height trajectory z(t) across the 5 s blind window
  - descent rate (cm/s), total drop, settle height, descent duration
  - a temporal criterion separating a half-kneel from a full fall / a sit

Validation (beamforming is only trustworthy if it agrees with ground truth):
  - at dropout-edge frames where pc points coexist, compare event-z to pc-z
  - beamformed azimuth vs the persistent track azimuth atan2(t_x, t_y)
  - tilt calibration: standing baseline should land ~1.4 m, fallen body ~0.2 m
"""
import argparse
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

from spatial3d.music import awrl6844_array
from spatial3d.range_music import spherical_to_cart

FILES = ["case/fall_hk_chairL_1.npz", "case/fall_hk_chairL_2.npz",
         "case/fall_hk_chairR.npz", "case/fall_hk_chairR_2.npz"]

# Beamform grid (radar frame; boresight +y). El range covers well below
# boresight because the radar is tilted down onto the floor.
AZ = np.deg2rad(np.arange(-50, 51, 1.0))
EL = np.deg2rad(np.arange(-60, 31, 1.0))
ARR = awrl6844_array()
STEER = ARR.steering_matrix(AZ, EL)          # (naz, nel, 16)


def to_room(x, y, z, tilt_deg, h_mount):
    """Radar-frame (x,y,z) -> room vertical height. X-tilt down + mount lift."""
    t = np.radians(tilt_deg)
    zr = -y * np.sin(t) + z * np.cos(t) + h_mount
    return zr


def beamform_events(evec, erange):
    """Per event: single-snapshot Bartlett peak -> (az, el, radar xyz, power)."""
    # P(az,el) = |a(az,el)^H x|^2, argmax over the grid
    resp = np.abs(np.einsum("aek,nk->nae", STEER.conj(), evec)) ** 2  # (N,naz,nel)
    N = evec.shape[0]
    out = np.zeros((N, 6))                    # az,el,x,y,z,power
    for i in range(N):
        ia, ie = np.unravel_index(resp[i].argmax(), resp[i].shape)
        az, el = AZ[ia], EL[ie]
        xyz = np.asarray(spherical_to_cart(float(erange[i]), az, el)).ravel()
        out[i] = [az, el, xyz[0], xyz[1], xyz[2], resp[i, ia, ie]]
    return out


def find_takes(ef, pf, fr_lo, fr_hi, min_len=20):
    """Dropout windows (pcN<3) that carry events = the fall-take blind windows."""
    frames = np.arange(fr_lo, fr_hi + 1)
    pc_n = np.array([(pf == f).sum() for f in frames])
    ev_n = np.array([(ef == f).sum() for f in frames])
    drop = (pc_n < 3) & (ev_n > 0)
    idx = np.where(drop)[0]
    if len(idx) == 0:
        return []
    segs = np.split(idx, np.where(np.diff(idx) > 1)[0] + 1)
    return [(int(frames[s[0]]), int(frames[s[-1]])) for s in segs
            if len(s) >= min_len]


def analyze(path, tilt_deg, h_mount, png=None):
    d = np.load(path, allow_pickle=True)
    ts = d["ts"]
    ef, er, ev = d["e_frame"], d["e_range"], d["e_vec"]
    pf, pxyz = d["p_frame"], d["pc_xyz"]
    tf, tx, ty, tz = d["t_frame"], d["t_x"], d["t_y"], d["t_z"]
    fr_lo = int(min(ef.min(), pf.min(), tf.min()))
    fr_hi = int(max(ef.max(), pf.max(), tf.max()))
    fps = (fr_hi - fr_lo + 1) / (ts[-1] - ts[0])
    name = path.split("/")[-1]
    takes = find_takes(ef, pf, fr_lo, fr_hi)
    print(f"\n===== {name}   {fps:.1f} fps   {len(takes)} half-kneel take(s)")

    fig, axes = plt.subplots(len(takes), 1, figsize=(11, 3.2 * len(takes)),
                             squeeze=False)
    results = []
    for ti, (f0, f1) in enumerate(takes):
        # include a 15-frame lead-in (pc alive) for the pre-fall standing height
        lead = 15
        fa, fb = f0 - lead, f1 + 3
        fzt, ftt, fpc = [], [], []          # (t, z) samples: beamformed / track / pc
        for f in range(fa, fb + 1):
            m = ef == f
            if m.any():
                bo = beamform_events(ev[m], er[m])
                # top-of-body: 90th pct of room height, power-weighted top few
                zroom = to_room(bo[:, 2], bo[:, 3], bo[:, 4], tilt_deg, h_mount)
                ztop = np.percentile(zroom, 90)
                fzt.append((f, ztop, np.median(zroom), bo))
            mt = tf == f
            if mt.any():
                zt_room = to_room(tx[mt], ty[mt], tz[mt], tilt_deg, h_mount)
                ftt.append((f, float(np.max(zt_room))))
            mp = pf == f
            if mp.sum() >= 3:
                zp = to_room(pxyz[mp, 0], pxyz[mp, 1], pxyz[mp, 2],
                             tilt_deg, h_mount)
                fpc.append((f, float(np.percentile(zp, 90))))

        if not fzt:
            continue
        F = np.array([r[0] for r in fzt])
        Ztop = np.array([r[1] for r in fzt])
        t = (F - f0) / fps                                    # s, 0 = blind start

        # descent stats over the blind window
        drop = Ztop.max() - Ztop.min()
        # rate = slope of a robust line over the descending portion
        imax, imin = int(Ztop.argmax()), int(Ztop.argmin())
        if imin > imax:
            dt = (F[imin] - F[imax]) / fps
            rate = (Ztop[imin] - Ztop[imax]) / dt if dt > 0 else 0.0
        else:
            dt, rate = 0.0, 0.0
        settle = np.median(Ztop[-8:])
        start_h = np.median(Ztop[:5])
        results.append(dict(take=ti, f0=f0, f1=f1, start=start_h, settle=settle,
                            drop=drop, rate=rate, descent_s=dt))
        print(f"  take{ti} f{f0}-{f1} ({(f1-f0)/fps:.1f}s blind): "
              f"start {start_h*100:5.0f}cm -> settle {settle*100:5.0f}cm  "
              f"drop {drop*100:5.0f}cm  descent {dt:.1f}s  "
              f"rate {rate*100:+6.1f} cm/s")

        ax = axes[ti, 0]
        ax.axvspan((f0 - f0) / fps, (f1 - f0) / fps, color="0.9",
                   label="blind (pc=0)")
        ax.plot(t, Ztop * 100, "-o", ms=3, color="C3", label="cube top-of-body")
        if ftt:
            tt = (np.array([r[0] for r in ftt]) - f0) / fps
            ax.plot(tt, np.array([r[1] for r in ftt]) * 100, ".", ms=4,
                    color="0.5", label="firmware track z")
        if fpc:
            tp = (np.array([r[0] for r in fpc]) - f0) / fps
            ax.plot(tp, np.array([r[1] for r in fpc]) * 100, "s", ms=6,
                    color="C0", mfc="none", label="pc z90 (edge)")
        ax.axhline(20, color="r", lw=0.8, ls=":")
        ax.axhline(80, color="g", lw=0.8, ls=":")
        ax.set_ylabel("height cm"); ax.set_ylim(-20, 170)
        ax.set_title(f"{name} take{ti}: descent {dt:.1f}s, "
                     f"rate {rate*100:+.0f}cm/s, drop {drop*100:.0f}cm")
        ax.legend(fontsize=7, loc="upper right")
    axes[-1, 0].set_xlabel("time (s), 0 = blind-window start")
    fig.tight_layout()
    out = png or f"hk_beam_{name.replace('.npz','')}.png"
    fig.savefig(out, dpi=110); plt.close()
    print(f"  saved {out}")
    return results


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("files", nargs="*", default=FILES)
    ap.add_argument("--tilt", type=float, default=25.0)
    ap.add_argument("--hmount", type=float, default=2.0)
    a = ap.parse_args()
    allres = []
    for f in (a.files or FILES):
        allres.append((f, analyze(f, a.tilt, a.hmount)))
    # cross-take summary -> the temporal criterion
    print("\n===== TEMPORAL SIGNATURE (all half-kneel takes) =====")
    rates, drops, descs, settles = [], [], [], []
    for f, rs in allres:
        for r in rs:
            rates.append(r["rate"] * 100); drops.append(r["drop"] * 100)
            descs.append(r["descent_s"]); settles.append(r["settle"] * 100)
    if rates:
        import numpy as np
        print(f"  n={len(rates)} takes")
        print(f"  descent rate  {np.mean(rates):+.0f} +/- {np.std(rates):.0f} cm/s"
              f"  [{min(rates):+.0f},{max(rates):+.0f}]")
        print(f"  total drop    {np.mean(drops):.0f} +/- {np.std(drops):.0f} cm"
              f"  [{min(drops):.0f},{max(drops):.0f}]")
        print(f"  descent time  {np.mean(descs):.1f} +/- {np.std(descs):.1f} s"
              f"  [{min(descs):.1f},{max(descs):.1f}]")
        print(f"  settle height {np.mean(settles):.0f} +/- {np.std(settles):.0f} cm"
              f"  [{min(settles):.0f},{max(settles):.0f}]")


if __name__ == "__main__":
    main()
