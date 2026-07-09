"""Coherent empty-vs-sit detection: BASE -> soft reset -> SIT, one session.

Reproduces the production flow — soft reset (= gaze-profile switch) is the only
reset available in deployment and was proven innocent by reset_ab_test (ratio
1.37). Captured minutes apart so scene aging is only ~2%, so the empty-vs-sit
covariance difference is the PERSON, not drift.

  1. person OUT, chair present  -> capture BASE (K snapshots)
  2. soft reset (sensorStop -> resend same cfg -> sensorStart)
  3. write status=COME_SIT, wait for --trigger (person sits, holds still)
  4. capture SIT
  5. report rel(R) & dirSim vs the ~0.02 empty same-session floor

    python sit_detect.py --out sit_detect.npz --trigger /tmp/go --status /tmp/st
"""
import argparse
import os
import time

import numpy as np

from reset_ab_test import (CFG, CLI, DATA, collect_segment, cov_mean, drain,
                           dir_sim, pack_cube, rel_cov, soft_reset)
from spatial3d.range_music import DR_M
from spatial3d.uart_reader import RadarSession


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default="sit_detect.npz")
    ap.add_argument("--trigger", required=True)
    ap.add_argument("--status", required=True)
    ap.add_argument("--k", type=int, default=300)
    ap.add_argument("--preheat", type=float, default=180.0)
    ap.add_argument("--settle", type=float, default=30.0)
    ap.add_argument("--post-reset", type=float, default=60.0,
                    help="wait after soft reset before prompting to sit")
    ap.add_argument("--start", type=int, default=87)
    ap.add_argument("--num", type=int, default=184)
    ap.add_argument("--seg-timeout", type=float, default=90.0)
    a = ap.parse_args()
    for f in (a.trigger, a.status):
        if os.path.exists(f):
            os.remove(f)

    s = RadarSession(CLI, DATA)
    s.start_drain()
    live, t0 = 0, time.time()
    while time.time() - t0 < 6 and live < 5:
        f = s.get_frame(timeout=1.0)
        if f is not None and f.range_antenna() is not None:
            live += 1
    if live >= 5:
        print(f"attached to EXISTING stream -> settle {a.settle:.0f}s", flush=True)
        drain(s, a.settle)
    else:
        print(f"no stream -> cfg + preheat {a.preheat:.0f}s", flush=True)
        s.send_cfg(CFG, echo=False)
        drain(s, a.preheat)

    # 1) empty BASE (person out, chair present)
    print("  capturing BASE (empty) ...", flush=True)
    acc_base = collect_segment(s, a.k, a.start, a.num, a.seg_timeout)
    print(f"  BASE: {len(acc_base.snaps)} bins", flush=True)

    # 2) soft reset = production gaze-profile switch (proven innocent)
    print("  [soft reset] (production event switch) ...", flush=True)
    soft_reset(s)
    drain(s, a.post_reset)

    # 3) prompt operator to sit, wait for trigger
    with open(a.status, "w") as fh:
        fh.write("COME_SIT\n")
    print(f"  >>> COME IN, SIT on the chair, hold still, then touch {a.trigger}",
          flush=True)
    while not os.path.exists(a.trigger):
        s.get_frame(timeout=0.3)

    # 4) SIT event
    print("  capturing SIT ...", flush=True)
    acc_sit = collect_segment(s, a.k, a.start, a.num, a.seg_timeout)
    print(f"  SIT: {len(acc_sit.snaps)} bins", flush=True)
    s.close()

    # 5) save cubes + cov/mean, report
    cb, mb = cov_mean(acc_base); cs, ms = cov_mean(acc_sit)
    common = sorted(set(cb) & set(cs))
    bins = np.array(common, dtype=np.int32)
    save = {"bins": bins, "dr_m": np.float32(DR_M)}
    for name, acc in (("base", acc_base), ("sit", acc_sit)):
        cube, cnt = pack_cube(acc, common, a.k)
        save[f"cube_{name}"] = cube
        save[f"counts_{name}"] = cnt
    save["cov_base"] = np.stack([cb[b] for b in common]).astype(np.complex64)
    save["cov_sit"] = np.stack([cs[b] for b in common]).astype(np.complex64)
    save["mean_base"] = np.stack([mb[b] for b in common]).astype(np.complex64)
    save["mean_sit"] = np.stack([ms[b] for b in common]).astype(np.complex64)
    np.savez(a.out, **save)

    r, _ = rel_cov(cb, cs); d = dir_sim(mb, ms)
    # per-bin rel to locate the person in range
    rels = {b: np.linalg.norm(cb[b] - cs[b]) / (np.linalg.norm(cb[b]) + 1e-9)
            for b in common}
    top = sorted(rels.items(), key=lambda kv: -kv[1])[:6]
    print("\n" + "=" * 52, flush=True)
    print(f"  EMPTY vs SIT (coherent, soft-reset between):", flush=True)
    print(f"    rel(R) = {r:.3f}   dirSim = {d:.4f}", flush=True)
    print(f"    (empty same-session floor ~0.02 / dirSim ~0.9999)", flush=True)
    print(f"  top-6 changed bins (person in range):", flush=True)
    for b, v in top:
        print(f"    bin{b} R={b*DR_M:.2f}m  rel={v:.3f}", flush=True)
    print(f"  SAVED {a.out}", flush=True)


if __name__ == "__main__":
    main()
