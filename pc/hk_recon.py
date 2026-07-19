"""Reconnaissance on the half-kneel fall cubes before beamforming.

Establish, per capture:
  - frame range / fps (from ts + p_frame/e_frame/t_frame)
  - the point-cloud dropout window (frames with few/zero pc points)
  - track state timeline: t_z, t_pose, t_down, t_fprob across frames
  - event (e_vec) density per frame, and whether events survive the dropout
    (the whole premise: cube recovers height where the point cloud dies)
"""
import sys
import numpy as np

FILES = ["case/fall_hk_chairL_1.npz", "case/fall_hk_chairL_2.npz",
         "case/fall_hk_chairR.npz", "case/fall_hk_chairR_2.npz"]


def recon(path):
    d = np.load(path, allow_pickle=True)
    ts = d["ts"]
    pf = d["p_frame"]; ef = d["e_frame"]; tf = d["t_frame"]
    fr_lo = int(min(pf.min() if len(pf) else 1e9,
                    ef.min() if len(ef) else 1e9,
                    tf.min() if len(tf) else 1e9))
    fr_hi = int(max(pf.max() if len(pf) else -1,
                    ef.max() if len(ef) else -1,
                    tf.max() if len(tf) else -1))
    dur = ts[-1] - ts[0] if len(ts) > 1 else 0
    print(f"\n===== {path}")
    print(f"  frames {fr_lo}..{fr_hi} ({fr_hi-fr_lo+1}), ts dur {dur:.1f}s"
          f"  ~{(fr_hi-fr_lo+1)/dur:.1f} fps" if dur > 0 else "")
    # per-frame counts
    frames = np.arange(fr_lo, fr_hi + 1)
    pc_n = np.array([(pf == f).sum() for f in frames])
    ev_n = np.array([(ef == f).sum() for f in frames])
    tk_n = np.array([(tf == f).sum() for f in frames])
    # dropout = pc points scarce
    drop = pc_n < 3
    # print a compact timeline every few frames
    tz = d["t_z"]; tpose = d["t_pose"]; tdown = d["t_down"]; tfp = d["t_fprob"]
    print("  frame | pcN evN tkN | t_z(cm) pose down fprob | DROP")
    for i, f in enumerate(frames):
        m = tf == f
        zc = f"{tz[m].max()*100:6.0f}" if m.any() else "   -- "
        ps = f"{int(tpose[m][0])}" if m.any() else "-"
        dn = f"{int(tdown[m].max())}" if m.any() else "-"
        fp = f"{tfp[m].max():.2f}" if m.any() else " -- "
        flag = "DROP" if drop[i] else ""
        # only print transitions / dropout region to keep it readable
        if drop[i] or (i > 0 and drop[i] != drop[i-1]) or i % 10 == 0:
            print(f"  {f:5d} | {pc_n[i]:3d} {ev_n[i]:3d} {tk_n[i]:3d} |"
                  f" {zc}  {ps:>3} {dn:>3}  {fp} | {flag}")
    # dropout summary
    if drop.any():
        idx = np.where(drop)[0]
        segs = np.split(idx, np.where(np.diff(idx) > 1)[0] + 1)
        for s in segs:
            f0, f1 = frames[s[0]], frames[s[-1]]
            ev_in = ev_n[s].sum()
            print(f"  >> DROPOUT frames {f0}..{f1} ({len(s)} fr) : "
                  f"pc≈0 but {ev_in} events present "
                  f"({'CUBE SURVIVES' if ev_in > 0 else 'cube also gone'})")
    else:
        print("  (no pc dropout < 3 pts)")


if __name__ == "__main__":
    fs = sys.argv[1:] or FILES
    for f in fs:
        recon(f)
