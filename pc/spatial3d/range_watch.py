"""Live range-profile monitor CLI.

Reads TLV frames from the radar DATA port, feeds range profiles (TLV type 2)
into RangeProfileMonitor, prints presence detection status.

Usage:
    cd /Users/sady3721/project/owl/Spatial3D/pc
    .venv/bin/python -m spatial3d.range_watch --data-port /dev/cu.usbmodemRA444_0 --cli-port /dev/cu.usbmodemRA444_2 --cfg profile_4T4R_tdm_low_cfar.cfg

If radar is already running (config sent), omit --cli-port/--cfg.
"""

from __future__ import annotations

import argparse
import signal
import sys
import time

import numpy as np

from .range_monitor import (
    RangeProfileMonitor, RangeStatus, Posture, N_BINS, BIN_WIDTH_M, bin_to_range,
)
from .uart_reader import DATA_BAUD, iter_frames

POSTURE_LABELS = {
    Posture.UNKNOWN: "???",
    Posture.ABSENT: "---",
    Posture.STANDING: "STAND",
    Posture.LYING: "LYING",
}


def _print_status(status: RangeStatus) -> None:
    elapsed = status.elapsed_s
    bands_str = ""
    if status.presence_bands:
        total_width = sum(b.width_bins for b in status.presence_bands)
        parts = []
        for b in status.presence_bands:
            parts.append(f"{b.range_start_m:.1f}-{b.range_end_m:.1f}m({b.width_bins}bins)")
        bands_str = " | w=" + str(total_width) + " " + ", ".join(parts)
    else:
        bands_str = " | no presence"

    ready = "OK" if status.baseline_ready else "warming"
    posture = POSTURE_LABELS[status.posture]
    print(
        f"[{elapsed:5.0f}s] f={status.frame_count:5d} "
        f"[{posture:5s}] "
        f"totVar={status.total_variance:6.0f} "
        f"trend={status.variance_trend:.2f} "
        f"peak=bin{status.max_variance_bin}({bin_to_range(status.max_variance_bin):.2f}m)"
        f"{bands_str}"
    )


def _print_profile_bar(mon: RangeProfileMonitor, top_n: int = 10) -> None:
    """Print top-N bins by variance as a compact bar chart."""
    var = mon.get_variance_profile()
    order = np.argsort(var)[::-1][:top_n]
    if var[order[0]] == 0:
        return
    scale = 40.0 / max(var[order[0]], 1.0)
    for idx in order:
        v = var[idx]
        if v < 1.0:
            break
        bar = "#" * int(v * scale)
        print(f"  bin{idx:3d} ({bin_to_range(idx):5.2f}m) var={v:7.1f} {bar}")


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description="Spatial3D range-profile monitor")
    ap.add_argument("--data-port", required=True, help="radar DATA UART device")
    ap.add_argument("--baud", type=int, default=DATA_BAUD)
    ap.add_argument("--cli-port", help="CLI UART (to send config before monitoring)")
    ap.add_argument("--cfg", help="path to .cfg file (requires --cli-port)")
    ap.add_argument("--mount-height", type=float, default=2.0)
    ap.add_argument("--mount-tilt", type=float, default=35.0,
                    help="tilt from vertical (degrees)")
    ap.add_argument("--var-window", type=int, default=50,
                    help="frames to keep for variance (default 50 ≈ 5s)")
    ap.add_argument("--threshold", type=float, default=1000.0,
                    help="variance threshold for presence declaration")
    ap.add_argument("--status-interval", type=float, default=3.0,
                    help="seconds between status prints")
    ap.add_argument("--detail", action="store_true",
                    help="print per-bin variance bar chart")
    ap.add_argument("--duration", type=float, default=None,
                    help="stop after N seconds (default: run forever)")
    args = ap.parse_args(argv)

    # Send config if requested
    if args.cli_port and args.cfg:
        from .uart_reader import send_config
        print(f"[range] sending config {args.cfg} via {args.cli_port}...")
        send_config(args.cli_port, args.cfg)
        print("[range] config sent, waiting 1s for data to start...")
        time.sleep(1.0)

    mon = RangeProfileMonitor(
        mount_height=args.mount_height,
        mount_tilt_deg=args.mount_tilt,
        var_window=args.var_window,
        presence_threshold=args.threshold,
    )

    running = [True]
    def _stop(sig, frame):
        running[0] = False
        print("\n[range] stopping...")
    signal.signal(signal.SIGINT, _stop)

    print(f"[range] listening on {args.data_port} @ {args.baud}")
    print(f"[range] mount: {args.mount_height}m, tilt={args.mount_tilt}° from vert")
    print(f"[range] variance window={args.var_window} frames, threshold={args.threshold}")

    last_status = time.time()
    t_start = last_status

    for frame in iter_frames(args.data_port, args.baud):
        if not running[0]:
            break
        if args.duration and (time.time() - t_start) > args.duration:
            break

        mon.update(frame)
        now = time.time()

        if now - last_status >= args.status_interval:
            status = mon.get_status()
            _print_status(status)
            if args.detail:
                _print_profile_bar(mon)
            last_status = now

    # Final summary
    print("\n--- Final status ---")
    status = mon.get_status()
    _print_status(status)
    if status.presence_bands:
        _print_profile_bar(mon)

    var = mon.get_variance_profile()
    top5 = np.argsort(var)[::-1][:5]
    print("\nTop 5 bins by variance:")
    for idx in top5:
        r = bin_to_range(idx)
        from .range_monitor import range_to_height
        h = range_to_height(r, args.mount_height, args.mount_tilt)
        print(f"  bin {idx:3d}: range={r:.2f}m  height={h:.2f}m  variance={var[idx]:.1f}")

    print(f"\n[range] done. {mon.frame_count} frames processed.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
