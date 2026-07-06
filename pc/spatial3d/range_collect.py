"""Collect range-profile snapshots for A/B comparison.

Saves per-bin mean + variance as .npz for offline analysis.

Usage:
    .venv/bin/python -m spatial3d.range_collect --data-port /dev/cu.usbmodem0000RA444 --duration 30 --out empty.npz
    # (person walks in)
    .venv/bin/python -m spatial3d.range_collect --data-port /dev/cu.usbmodem0000RA444 --duration 30 --out person.npz
    # compare:
    .venv/bin/python -m spatial3d.range_collect --compare empty.npz person.npz
"""

from __future__ import annotations

import argparse
import sys
import time

import numpy as np

from .range_monitor import parse_range_profile, TLV_RANGE_PROFILE, N_BINS, BIN_WIDTH_M
from .uart_reader import DATA_BAUD, iter_frames


def collect(data_port: str, baud: int, duration: float) -> tuple[np.ndarray, np.ndarray]:
    """Collect range profiles, return (mean, variance) per bin."""
    profiles = []
    t0 = time.time()
    for frame in iter_frames(data_port, baud):
        if time.time() - t0 > duration:
            break
        for tlv in frame.tlvs:
            if tlv.type == TLV_RANGE_PROFILE:
                p = parse_range_profile(tlv.payload)
                if p is not None:
                    profiles.append(p)
        if len(profiles) % 50 == 0 and profiles:
            print(f"  collected {len(profiles)} profiles ({time.time()-t0:.0f}s)...")

    stack = np.array(profiles)
    return stack.mean(axis=0), stack.var(axis=0)


def compare(file_a: str, file_b: str, max_bin: int = 75):
    """Compare two saved snapshots and show where they differ."""
    a = np.load(file_a)
    b = np.load(file_b)

    mean_a, var_a = a["mean"], a["var"]
    mean_b, var_b = b["mean"], b["var"]

    mean_diff = mean_b - mean_a
    var_diff = var_b - var_a

    print(f"Comparing {file_a} vs {file_b}")
    print(f"Range: bins 4-{max_bin} (0.28m - {max_bin * BIN_WIDTH_M:.1f}m)\n")

    print("=== Mean magnitude change (B - A) ===")
    print("Positive = more energy in B (person present?)")
    top_mean = np.argsort(np.abs(mean_diff[4:max_bin]))[::-1][:15] + 4
    for idx in top_mean:
        r = idx * BIN_WIDTH_M
        print(f"  bin {idx:3d} ({r:5.2f}m): A={mean_a[idx]:.0f}  B={mean_b[idx]:.0f}"
              f"  Δ={mean_diff[idx]:+.0f}")

    print("\n=== Variance change (B - A) ===")
    print("Positive = more fluctuation in B (micro-motion?)")
    top_var = np.argsort(np.abs(var_diff[4:max_bin]))[::-1][:15] + 4
    for idx in top_var:
        r = idx * BIN_WIDTH_M
        print(f"  bin {idx:3d} ({r:5.2f}m): A_var={var_a[idx]:.0f}  B_var={var_b[idx]:.0f}"
              f"  Δ={var_diff[idx]:+.0f}")

    # Variance ratio (where A has non-trivial variance)
    print("\n=== Variance ratio (B / A) where A > 10 ===")
    mask = var_a > 10
    ratio = np.where(mask, var_b / var_a, 0)
    top_ratio = np.argsort(ratio[4:max_bin])[::-1][:10] + 4
    for idx in top_ratio:
        if ratio[idx] < 0.1:
            break
        r = idx * BIN_WIDTH_M
        print(f"  bin {idx:3d} ({r:5.2f}m): ratio={ratio[idx]:.1f}x"
              f"  (A_var={var_a[idx]:.0f} → B_var={var_b[idx]:.0f})")


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description="Collect range profiles for comparison")
    ap.add_argument("--data-port", help="radar DATA UART")
    ap.add_argument("--baud", type=int, default=DATA_BAUD)
    ap.add_argument("--duration", type=float, default=30.0)
    ap.add_argument("--out", help="save snapshot to .npz")
    ap.add_argument("--compare", nargs=2, metavar=("EMPTY", "PERSON"),
                    help="compare two snapshots")
    args = ap.parse_args(argv)

    if args.compare:
        compare(args.compare[0], args.compare[1])
        return 0

    if not args.data_port or not args.out:
        print("need --data-port and --out (or --compare A B)", file=sys.stderr)
        return 2

    print(f"Collecting for {args.duration}s from {args.data_port}...")
    mean, var = collect(args.data_port, args.baud, args.duration)
    np.savez(args.out, mean=mean, var=var)
    print(f"Saved to {args.out}")

    # Quick summary of near-field
    print(f"\nSummary (bins 4-75, 0.28-5.27m):")
    for i in range(4, 75):
        if var[i] > 50:
            r = i * BIN_WIDTH_M
            print(f"  bin {i:3d} ({r:5.2f}m): mean={mean[i]:.0f} var={var[i]:.0f}")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
