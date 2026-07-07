"""Transient-robust BASE CUBE: N rounds of K snapshots, spaced, median-combined.

A single high-K capture has no transient rejection — someone walking through the
~60s window contaminates it. The demo streams continuously (no stop/reset); we
collect N separate K-snapshot rounds spaced *interval* seconds apart and take the
per-bin element-wise MEDIAN across rounds, so a transient that hits one round is
out-voted.

Unlike the old version (median covariance only), this saves the full set of
reference statistics needed to compare ANY event against the base:

    covariances : median R           -> static energy/structure baseline (MUSIC)
    variance    : median trace(R)-|m|² -> motion NOISE FLOOR per bin (liveness ref)
    fluctuation : median R-m·mᴴ        -> baseline moving-clutter covariance

The event's motion is only meaningful as EXCESS over this base floor — an empty
room still has some per-bin fluctuation (multipath jitter, clutter), so "real
motion" = event variance well above the base variance at that bin.

    python baseline_multi.py --out base_cube.npz --rounds 3 --k 300 --interval 30
"""
import argparse
import time
from collections import Counter

import numpy as np

from spatial3d.range_music import DR_M, N_VIRT_ANT, BinAccumulator
from spatial3d.uart_reader import RadarSession

CLI = "/dev/cu.usbmodem0000RA441"
DATA = "/dev/cu.usbmodem0000RA444"
CFG = "/Users/sady3721/project/TI/Tiinstall/profile_music_5fps_fullroom.cfg"


def collect_round(session, k, start, num, timeout):
    """Collect one round; return per-bin (covariance, variance, fluctuation)."""
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
    covs, var, fluc = {}, {}, {}
    for b, lst in acc.snaps.items():
        if len(lst) < 10:
            continue
        x = np.stack(lst, axis=0)                       # (K, 16)
        m = x.mean(axis=0)
        R = (x.conj().T @ x) / len(x)
        covs[b] = R.astype(np.complex64)
        fluc[b] = (R - np.outer(m, m.conj())).astype(np.complex64)
        var[b] = float(np.mean(np.sum(np.abs(x) ** 2, axis=1))
                       - np.sum(np.abs(m) ** 2))
    return covs, var, fluc


def drain(session, seconds):
    t0 = time.time()
    while time.time() - t0 < seconds:
        session.get_frame(timeout=0.5)                  # keep the stream alive


def _median_complex(mats):
    stack = np.stack(mats)
    return (np.median(stack.real, axis=0)
            + 1j * np.median(stack.imag, axis=0)).astype(np.complex64)


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

    rounds = []                                          # list of (covs,var,fluc)
    for r in range(a.rounds):
        print(f"round {r + 1}/{a.rounds}: collecting K={a.k} ...", flush=True)
        rr = collect_round(s, a.k, a.start, a.num, a.round_timeout)
        print(f"  round {r + 1}: {len(rr[0])} bins", flush=True)
        rounds.append(rr)
        if r < a.rounds - 1:
            print(f"  idle {a.interval:.0f}s (stream kept alive) ...", flush=True)
            drain(s, a.interval)
    s.close()

    # keep bins present in a majority of rounds; per-bin median across rounds
    need = max(2, (a.rounds + 1) // 2)
    counts = Counter(b for covs, _, _ in rounds for b in covs)
    keep = sorted(b for b, c in counts.items() if c >= need)

    cov_out, fluc_out, var_out = [], [], []
    for b in keep:
        cov_out.append(_median_complex([c[b] for c, _, _ in rounds if b in c]))
        fluc_out.append(_median_complex([f[b] for _, _, f in rounds if b in f]))
        var_out.append(float(np.median([v[b] for _, v, _ in rounds if b in v])))

    bins = np.asarray(keep, dtype=np.int32)
    empty = np.empty((0, N_VIRT_ANT, N_VIRT_ANT), np.complex64)
    # Also keep the PER-ROUND covariance/variance (same continuous stream, empty
    # room) so the round-to-round difference can be measured — the best-case
    # noise floor of the covariance when nothing physically changed.
    zero = np.zeros((N_VIRT_ANT, N_VIRT_ANT), np.complex64)
    rounds_cov = np.stack([
        np.stack([c[b] if b in c else zero for b in keep])
        for c, _, _ in rounds]) if keep else empty[None]
    rounds_var = np.stack([
        np.asarray([v.get(b, 0.0) for b in keep], np.float32)
        for _, v, _ in rounds]) if keep else np.empty((a.rounds, 0), np.float32)
    np.savez(
        a.out,
        bins=bins,
        covariances=np.stack(cov_out).astype(np.complex64) if cov_out else empty,
        fluctuation=np.stack(fluc_out).astype(np.complex64) if fluc_out else empty,
        variance=np.asarray(var_out, dtype=np.float32),
        rounds_cov=rounds_cov.astype(np.complex64),
        rounds_var=rounds_var,
        counts=np.full(len(bins), a.k, dtype=np.int32),
        dr_m=np.float32(DR_M),
    )
    vmax = max(var_out) if var_out else 0.0
    print(f"SAVED {a.out}: {len(bins)} bins, median of {a.rounds} rounds "
          f"(K={a.k}). base variance floor: max={vmax:.1f}", flush=True)


if __name__ == "__main__":
    main()
