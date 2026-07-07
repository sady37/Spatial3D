"""Continuous-stream gaze capture: keep the demo streaming, grab K snapshots on trigger.

Solves two problems: (1) no per-event reset/reconfig — the demo streams
continuously so it never sleeps; (2) the operator controls timing — collection
starts only when a trigger file appears, so you lie down *then* fire it.

    python gaze_stream.py --out cov.npz --trigger /tmp/go --status /tmp/st &
    # wait until <status> says STREAMING, then: touch /tmp/go
"""
import argparse
import os
import time

import numpy as np

from spatial3d.range_music import DR_M, N_VIRT_ANT, BinAccumulator
from spatial3d.uart_reader import RadarSession

CLI = "/dev/cu.usbmodem0000RA441"
DATA = "/dev/cu.usbmodem0000RA444"
CFG = "/Users/sady3721/project/TI/Tiinstall/profile_music_5fps_fullroom.cfg"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", required=True)
    ap.add_argument("--trigger", required=True)
    ap.add_argument("--status", required=True)
    ap.add_argument("--k", type=int, default=100)
    ap.add_argument("--start", type=int, default=87)
    ap.add_argument("--num", type=int, default=184)
    ap.add_argument("--collect-timeout", type=float, default=40.0)
    a = ap.parse_args()

    for f in (a.trigger, a.status):
        if os.path.exists(f):
            os.remove(f)

    s = RadarSession(CLI, DATA)
    s.start_drain()
    print("config + start streaming ...", flush=True)
    s.send_cfg(CFG, echo=False)

    # confirm streaming (drain a few frames)
    live = 0
    t0 = time.time()
    while time.time() - t0 < 8 and live < 5:
        f = s.get_frame(timeout=1.0)
        if f is not None and f.range_antenna() is not None:
            live += 1
    if live < 5:
        with open(a.status, "w") as fh:
            fh.write("STALLED\n")
        print("STALLED — demo not streaming", flush=True)
        s.close()
        return
    with open(a.status, "w") as fh:
        fh.write("STREAMING\n")
    print("STREAMING — waiting for trigger ...", flush=True)

    # keep draining (demo stays awake) until the trigger file appears
    while not os.path.exists(a.trigger):
        s.get_frame(timeout=0.3)     # flush so the queue stays fresh

    print("TRIGGERED — collecting K snapshots ...", flush=True)
    acc = BinAccumulator(k=a.k, n_ant=N_VIRT_ANT)
    bins = range(a.start, a.start + a.num)
    t0 = time.time()
    while time.time() - t0 < a.collect_timeout and acc.min_count(bins) < a.k:
        f = s.get_frame(timeout=1.0)
        if f is None:
            continue
        ra = f.range_antenna()
        if ra is not None:
            acc.add(ra)
    s.close()

    covs = acc.covariances(min_snapshots=10)
    binsA = np.array(sorted(covs), dtype=np.int32)
    cov = np.stack([covs[int(b)] for b in binsA]).astype(np.complex64)
    cnt = np.array([len(acc.snaps[int(b)]) for b in binsA], dtype=np.int32)
    np.savez(a.out, bins=binsA, covariances=cov, counts=cnt,
             dr_m=np.float32(DR_M))
    print(f"SAVED {a.out}  {len(binsA)} bins, min/bin={acc.min_count(bins)}",
          flush=True)


if __name__ == "__main__":
    main()
