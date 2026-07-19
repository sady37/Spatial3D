"""Half-kneel descent + stay-down, with static-clutter rejection.

Redefined deliverable (the cube can't give absolute height: array geometry is
unverified and single-snapshot beamforming locks onto high static clutter). So:

  descent RATE   <- point cloud BEFORE the blackout (body is still moving/detected)
  stay-down      <- inside the ~5 s blind window, firmware down-flag + cube events
  settle height  <- clutter-rejected body-top just before + around the blackout

Static-clutter rejection: the recordings carry a fixed blob (~y1.8m, z1.4m) that
is NOT the person (the person sits at y~3.3m, z~-0.35m). Per frame we cluster the
points and keep the cluster co-located with the persistent track (t_x, t_y); that
drops the clutter and any detached ceiling ghost without ever gating on height
(so a genuinely upright torso in a half-kneel is preserved).

    python hk_descent.py                 # all four, prints per-take signature
    python hk_descent.py --png           # + trajectory plots
"""
import argparse
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

from spatial3d.music_collect import to_room

# The 4 STANDARD half-kneel cases (falltest_20260719.jsonl). Each recording is
# ONE sit->half-kneel fall at go_t + fall_rel_s; the 3 cube bursts (rel ~8/16/24)
# capture the AFTERMATH, not the descent. Aligning to go_t pins the single fall
# (critical for chairL_1: its fall is at frame ~1272, NOT the earlier activity).
FILES = ["case/fall_hk_chairL_1.npz", "case/fall_hk_chairL_2.npz",
         "case/fall_hk_chairR.npz", "case/fall_hk_chairR_2.npz"]
GO_T = {"fall_hk_chairL_1.npz": 1784485730.0, "fall_hk_chairL_2.npz": 1784485803.0,
        "fall_hk_chairR.npz": 1784486183.9, "fall_hk_chairR_2.npz": 1784486941.9}
FALL_REL_S = 5.0        # fall happens ~this many s after go (sit by T3, fall ~T5)


def body_points(pxyz_room, tcx, tcy, radius=0.8):
    """Keep points whose horizontal (x,y) is within *radius* of the track center.

    Rejects the fixed near-range clutter blob and detached ghosts by spatial
    co-location with the persistent track, NOT by height gating.
    """
    dx = pxyz_room[:, 0] - tcx
    dy = pxyz_room[:, 1] - tcy
    return pxyz_room[np.hypot(dx, dy) <= radius]


def body_height(d, fr, fps, tilt, hmount):
    """Clutter-rejected (z_top, z_med, n) for one frame, or None."""
    pf, pxyz = d["p_frame"], d["pc_xyz"]
    tf, tx, ty = d["t_frame"], d["t_x"], d["t_y"]
    mp = pf == fr
    if mp.sum() < 3:
        return None
    room = to_room(pxyz[mp], tilt_deg=tilt, h_mount=hmount)
    mt = tf == fr
    if mt.any():
        # track x,y in room frame (track z unreliable, ignore it)
        tr = to_room(np.array([[tx[mt][0], ty[mt][0], 0.0]]),
                     tilt_deg=tilt, h_mount=hmount)
        bp = body_points(room, tr[0, 0], tr[0, 1])
    else:
        bp = room
    if len(bp) < 3:
        bp = room                                   # fall back to raw if over-pruned
    z = bp[:, 2]
    return np.percentile(z, 90), np.median(z), len(bp)


def find_takes(d, fr_lo, fr_hi, min_len=20):
    ef, pf = d["e_frame"], d["p_frame"]
    frames = np.arange(fr_lo, fr_hi + 1)
    pc_n = np.array([(pf == f).sum() for f in frames])
    ev_n = np.array([(ef == f).sum() for f in frames])
    drop = (pc_n < 3) & (ev_n > 0)
    idx = np.where(drop)[0]
    if not len(idx):
        return []
    segs = np.split(idx, np.where(np.diff(idx) > 1)[0] + 1)
    return [(int(frames[s[0]]), int(frames[s[-1]])) for s in segs if len(s) >= min_len]


def analyze(path, tilt, hmount, make_png):
    d = np.load(path, allow_pickle=True)
    ts = d["ts"]
    ef, tf, tdown = d["e_frame"], d["t_frame"], d["t_down"]
    name = path.split("/")[-1]
    fps = (len(ts) - 1) / (ts[-1] - ts[0])
    go = GO_T.get(name)
    if go is None:
        print(f"\n===== {name}: no go_t label, skip"); return []
    # frame index by nearest epoch timestamp; frame i corresponds to ts[i]
    def fr_at(rel):
        return int(np.argmin(np.abs(ts - (go + rel))))
    fall_fr = fr_at(FALL_REL_S)
    print(f"\n===== {name}   {fps:.1f} fps   fall @go+{FALL_REL_S:.0f}s = frame {fall_fr}")

    UP, FLOOR = 0.60, 0.25          # upright / floor band (m)
    # DESCENT window: sit (rel 2s) .. settle (rel 9s), around the single fall
    f_a, f_b = fr_at(2.0), fr_at(9.0)
    zt = [(f, body_height(d, f, fps, tilt, hmount)) for f in range(f_a, f_b + 1)]
    zt = [(f, v[0]) for f, v in zt if v is not None]
    if len(zt) < 5:
        print("  <5 pc frames in descent window, skip"); return []
    F = np.array([r[0] for r in zt]); Ztop = np.array([r[1] for r in zt])
    t = ts[F] - go                                       # s relative to go
    Zs = np.convolve(Ztop, np.ones(3) / 3, mode="same")

    h_stand = float(np.percentile(Ztop, 95))
    h_settle = float(np.median(Ztop[t >= FALL_REL_S + 1.5])) if (t >= FALL_REL_S + 1.5).any() \
        else float(np.median(Ztop[-4:]))
    drop = h_stand - h_settle
    # threshold crossing: last upright -> first floor after it
    i_up = np.where(Zs >= UP)[0]; i_fl = np.where(Zs <= FLOOR)[0]
    dt = rate = np.nan
    if len(i_up) and len(i_fl):
        k_up = i_up[-1]; after = i_fl[i_fl > k_up]
        if len(after):
            k_fl = after[0]; dt = t[k_fl] - t[k_up]
            rate = (Zs[k_fl] - Zs[k_up]) / dt if dt > 0 else np.nan
    captured = h_stand >= UP and h_settle <= FLOOR + 0.20 and np.isfinite(rate)

    # STAY-DOWN: over the 3 aftermath cube bursts (rel ~8..26s)
    after_a, after_b = fr_at(6.0), fr_at(26.0)
    aft = np.arange(after_a, after_b + 1)
    ev_cov = float(np.mean([(ef == f).any() for f in aft]))
    dn = [int(tdown[tf == f].max()) if (tf == f).any() else 0 for f in aft]
    down_frac = float(np.mean(dn))

    kind = "DESCENT" if captured else "partial"
    rt = f"{rate*100:+6.0f}cm/s" if np.isfinite(rate) else "   --  "
    dts = f"{dt:.1f}s" if np.isfinite(dt) else " -- "
    print(f"  [{kind:8s}] stand {h_stand*100:5.0f} -> settle {h_settle*100:5.0f}cm"
          f"  drop {drop*100:4.0f}  rate {rt} desc {dts}"
          f" || aftermath: cubeCov={ev_cov:.2f} down={down_frac:.2f}")
    row = dict(name=name, kind=kind, h_stand=h_stand, h_settle=h_settle, drop=drop,
               rate=rate, descent_s=dt, ev_cov=ev_cov, down_frac=down_frac,
               captured=captured)
    if make_png:
        fig, ax = plt.subplots(figsize=(11, 3.4))
        ax.plot(t, Ztop * 100, "-o", ms=4, color="C0",
                label="clutter-rejected body-top (pc)")
        ax.axvline(FALL_REL_S, color="C3", ls="--", lw=1, label="fall (go+5s)")
        for r in (8, 16, 24):
            ax.axvspan(r - 0.2, r + 5, color="0.9", alpha=.6)
        ax.axhline(80, color="g", ls=":", lw=.8); ax.axhline(20, color="r", ls=":", lw=.8)
        ax.set_ylabel("height cm"); ax.set_ylim(-60, 180)
        ax.set_xlabel("time (s) relative to go (grey = cube bursts)")
        ax.set_title(f"{name}  [{kind}] drop {drop*100:.0f}cm rate {rt} desc {dts}")
        ax.legend(fontsize=7)
        fig.tight_layout()
        out = f"hk_descent_{name.replace('.npz','')}.png"
        fig.savefig(out, dpi=110); plt.close()
        print(f"  saved {out}")
    return [row]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("files", nargs="*")
    ap.add_argument("--tilt", type=float, default=25.0)
    ap.add_argument("--hmount", type=float, default=2.0)
    ap.add_argument("--png", action="store_true")
    a = ap.parse_args()
    allrows = []
    for f in (a.files or FILES):
        allrows += analyze(f, a.tilt, a.hmount, a.png)
    if allrows:
        desc = [r for r in allrows if r["captured"]]
        print(f"\n===== HALF-KNEEL SIGNATURE  ({len(desc)}/{len(allrows)} standard "
              f"cases cleanly captured the descent) =====")
        print("  -- descent-captured cases only --")
        for k, lab, sc in [("h_stand", "stand", 100), ("h_settle", "settle", 100),
                           ("drop", "drop", 100), ("rate", "rate", 100),
                           ("descent_s", "descent_s", 1)]:
            v = np.array([r[k] for r in desc]) * sc
            v = v[np.isfinite(v)]
            if len(v):
                print(f"  {lab:10s} {v.mean():+7.1f} +/- {v.std():5.1f}"
                      f"  [{v.min():+.0f},{v.max():+.0f}]")
        print("  -- all 4 cases: aftermath stay-down confirmer --")
        for k, lab in [("down_frac", "down_frac"), ("ev_cov", "cube_cov")]:
            v = np.array([r[k] for r in allrows])
            print(f"  {lab:10s} {v.mean():+7.2f} +/- {v.std():5.2f}"
                  f"  [{v.min():.2f},{v.max():.2f}]")


if __name__ == "__main__":
    main()
