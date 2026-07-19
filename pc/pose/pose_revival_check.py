"""On-chip pose MLP revival check — before/after the tracker cfg change
(stateParam static2free 40->100, maxAcceleration Z 0.1->1.0).

Reads a scene npz's RECORDED firmware pose legs (t_pose class + t_fprob falling_prob, TLV 321)
and the track presence, and reports EXACTLY the two things the cfg change should move:

  Q1 (vertical dynamics freed -> velZ alive): at the FALL INSTANT does falling_prob rise off 0
     and does t_pose emit Falling(3)?  (before: falling_prob max ~0, Falling frames = 0)
  Q2 (static2free raised -> track survives a lie): when the person is down, does the track stay
     alive (not drop ~4s) and does t_pose hold Lying(2)?

The fall instant is found track-INDEPENDENTLY from the raw 3001 cloud centroid height drop-rate
(dH/dt), so it works even when the track froze/dropped in the OLD recordings.

Usage:  python3 pose/pose_revival_check.py <a.npz> [<b.npz> ...]   # pass old + new to compare
"""
import os, sys, math
import numpy as np

POSE = {0: "Stood", 1: "Sat", 2: "Lying", 3: "Falling", 255: "invalid"}
MOUNT, TILT = 2.0, 25.0


def cloud_height(d):
    """Per-frame 3001-cloud median world height + the ground-range, for fall-instant + drop-rate."""
    th = math.radians(TILT)
    pf, pcx = d["p_frame"], d["pc_xyz"]
    nfr = int(d["ts"].shape[0])
    H = np.full(nfr, np.nan)
    for fi in range(nfr):
        m = pf == fi
        if m.any():
            z, y = pcx[m, 2], pcx[m, 1]
            H[fi] = np.median(MOUNT + z * math.cos(th) - y * math.sin(th))
    return H


def analyze(path):
    d = np.load(path)
    if "t_pose" not in d:
        print(f"  {os.path.basename(path)}: pre-pose schema (no t_pose) — skip"); return None
    ts = d["ts"] - d["ts"][0]
    nfr = int(ts.shape[0])
    tf, tid, tpose, tfp = d["t_frame"], d["t_tid"], d["t_pose"], d["t_fprob"]

    # ---- pose class histogram (all track-frames) ----
    hist = {k: 0 for k in POSE}
    for pv in tpose:
        hist[int(pv) if int(pv) in POSE else 255] += 1
    fprob = np.asarray(tfp, float)

    # ---- track presence / persistence ----
    present = np.zeros(nfr, bool)
    present[np.clip(tf.astype(int), 0, nfr - 1)] = True
    # longest continuous alive run and longest gap (in seconds), primary = most-frequent tid
    def runs(mask):
        best = cur = 0
        for b in mask:
            cur = cur + 1 if b else 0
            best = max(best, cur)
        return best
    fps = nfr / max(ts[-1], 1e-6)
    alive_run_s = runs(present) / fps
    gap_run_s = runs(~present) / fps

    # ---- fall instant from cloud-height drop-rate (track-independent) ----
    H = cloud_height(d)
    rate = np.gradient(np.where(np.isnan(H), np.nanmedian(H), H)) * fps  # m/s
    fall_fi = int(np.nanargmin(rate)) if np.isfinite(rate).any() else 0
    fall_t = ts[fall_fi]

    # ---- pose/fprob in a +-2s window around the fall instant ----
    w = (tf >= fall_fi - int(2 * fps)) & (tf <= fall_fi + int(2 * fps))
    win_poses = [POSE.get(int(p), "?") for p in tpose[w]]
    win_fpmax = float(fprob[w].max()) if w.any() else 0.0

    # ---- down-window: when the cloud is low (lying), does the track hold + read Lying? ----
    low = H < 0.0
    low_frames = np.where(low)[0]
    lying_hold = 0.0
    if len(low_frames):
        lo, hi = low_frames[0], low_frames[-1]
        low_present = present[lo:hi + 1]
        lying_hold = runs(low_present) / fps
        wl = (tf >= lo) & (tf <= hi)
        lying_frac = (np.isin(tpose[wl], [2]).mean() if wl.any() else 0.0)
    else:
        lying_frac = 0.0

    return {
        "file": os.path.basename(path), "dur": float(ts[-1]), "nfr": nfr,
        "hist": hist, "fp_max": float(fprob.max()) if len(fprob) else 0.0,
        "fp_pos": int((fprob > 0.05).sum()),
        "alive_run_s": alive_run_s, "gap_run_s": gap_run_s,
        "present_frac": float(present.mean()),
        "fall_t": float(fall_t), "fall_rate": float(rate[fall_fi]),
        "win_falling": win_poses.count("Falling"), "win_fpmax": win_fpmax,
        "win_poses": win_poses,
        "lying_hold_s": lying_hold, "lying_frac": lying_frac,
    }


def report(r):
    if r is None:
        return
    h = r["hist"]
    print(f"\n=== {r['file']}  ({r['dur']:.0f}s, {r['nfr']} frames) ===")
    print(f"  pose class frames: Stood={h[0]} Sat={h[1]} Lying={h[2]} "
          f"Falling={h[3]} invalid={h[255]}")
    print(f"  falling_prob: max={r['fp_max']:.3f}  frames>0.05={r['fp_pos']}   "
          f"(BEFORE-fix baseline: max~0, Falling=0, Lying=0)")
    print(f"  track presence: {100*r['present_frac']:.0f}% frames, longest ALIVE run="
          f"{r['alive_run_s']:.1f}s, longest GAP={r['gap_run_s']:.1f}s")
    print(f"  ⓵ FALL INSTANT @ t={r['fall_t']:.1f}s (cloud drop {r['fall_rate']:.1f} m/s):")
    print(f"      Falling(3) frames in ±2s = {r['win_falling']}   "
          f"falling_prob max in ±2s = {r['win_fpmax']:.3f}")
    print(f"      pose seq around fall: {r['win_poses']}")
    print(f"  ⓶ WHILE DOWN (cloud below floor): track alive run={r['lying_hold_s']:.1f}s, "
          f"Lying(2) fraction={100*r['lying_frac']:.0f}%")
    # verdicts
    q1 = "✅ REVIVED" if (r["win_falling"] > 0 or r["win_fpmax"] > 0.1) else "❌ still dead"
    q2 = "✅ holds" if (r["lying_hold_s"] > 6 and r["lying_frac"] > 0.3) else "❌ drops/not-Lying"
    print(f"  => Q1 fall MLP (Falling/fprob): {q1}   |   Q2 lying track+pose: {q2}")


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print(__doc__); sys.exit(1)
    for p in sys.argv[1:]:
        report(analyze(p))
