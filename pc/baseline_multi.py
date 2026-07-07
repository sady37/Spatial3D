"""Transient-robust covariance baseline: N rounds of K snapshots, spaced, median-combined.

A single high-K covariance has no transient rejection — someone walking through
the ~60s capture contaminates it. Here the demo streams continuously (no stop /
no reset) and we collect N separate K-snapshot covariance sets spaced *interval*
seconds apart, then per range bin take the element-wise MEDIAN across rounds.
A transient that hits one round is out-voted by the others.

    python baseline_multi.py --out baseline_cov.npz --rounds 3 --k 300 --interval 30
"""
import argparse
import time

import numpy as np

from spatial3d.range_music import DR_M, N_VIRT_ANT, BinAccumulator
from spatial3d.uart_reader import RadarSession

CLI = "/dev/cu.usbmodem0000RA441"
DATA = "/dev/cu.usbmodem0000RA444"
CFG = "/Users/sady3721/project/TI/Tiinstall/profile_music_5fps_fullroom.cfg"


def collect_round(session, k, start, num, timeout):
    acc = BinAccumulator(k=k, n_ant=N_VIRT_ANT)
    bins = range(start, start + num)
    t0 = time.time()
    while time.time() - t0 < timeout and acc.min_count(bins) < k:
        f = session.get_frame(timeout=1.0)
        if f is None:
            continue
        ra = f.range_antenna()
        if ra is not None:
            acc.add(ra)
    return acc.covariances(min_snapshots=10)


def drain(session, seconds):
    t0 = time.time()
    while time.time() - t0 < seconds:
        session.get_frame(timeout=0.5)     # keep the stream alive (no sleep)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", required=True)
    ap.add_argument("--rounds", type=int, default=3)
    ap.add_argument("--k", type=int, default=300)
    ap.add_argument("--interval", type=float, default=30.0)
    ap.add_argument("--start", type=int, default=87)
    ap.add_argument("--num", type=int, default=184)
    ap.add_argument("--round-timeout", type=float, default=90.0)
    a = ap.parse_args()

    s = RadarSession(CLI, DATA)
    s.start_drain()
    print("config + stream ...", flush=True)
    s.send_cfg(CFG, echo=False)

    rounds = []
    for r in range(a.rounds):
        print(f"round {r + 1}/{a.rounds}: collecting K={a.k} ...", flush=True)
        covs = collect_round(s, a.k, a.start, a.num, a.round_timeout)
        print(f"  round {r + 1}: {len(covs)} bins", flush=True)
        rounds.append(covs)
        if r < a.rounds - 1:
            print(f"  idle {a.interval:.0f}s (stream kept alive) ...", flush=True)
            drain(s, a.interval)
    s.close()

    # per-bin element-wise median across rounds (bins present in >= 2 rounds)
    from collections import Counter
    bin_counts = Counter(b for rc in rounds for b in rc)
    keep = sorted(b for b, c in bin_counts.items() if c >= max(2, (a.rounds + 1) // 2))
    covs_out = []
    for b in keep:
        mats = [rc[b] for rc in rounds if b in rc]
        stack = np.stack(mats)                      # (R, 16, 16) complex
        med = (np.median(stack.real, axis=0)
               + 1j * np.median(stack.imag, axis=0)).astype(np.complex64)
        covs_out.append(med)
    bins = np.asarray(keep, dtype=np.int32)
    cov = np.stack(covs_out).astype(np.complex64) if covs_out else \
        np.empty((0, N_VIRT_ANT, N_VIRT_ANT), np.complex64)
    cnt = np.full(len(bins), a.k, dtype=np.int32)
    np.savez(a.out, bins=bins, covariances=cov, counts=cnt,
             dr_m=np.float32(DR_M))
    print(f"SAVED {a.out}: {len(bins)} bins, median of {a.rounds} rounds "
          f"(K={a.k} each)", flush=True)


if __name__ == "__main__":
    main()
