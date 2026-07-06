"""Patient monitor CLI — the 'sit and watch' loop.

Reads live TLV frames from the radar DATA port (or replays saved frames),
feeds them into PatientMonitor, and prints periodic status.

Usage (live):
    cd pc
    .venv/bin/python -m spatial3d.watch \\
        --data-port /dev/cu.usbmodem0000RA444 --baud 1250000

Usage (sim / offline):
    .venv/bin/python -m spatial3d.watch --sim --sim-fall-at 120

Usage (load a previously saved Map A as starting baseline):
    .venv/bin/python -m spatial3d.watch --data-port ... --load-a saved_map_a.npy
"""

from __future__ import annotations

import argparse
import itertools
import signal
import sys
import time

import numpy as np

from .monitor import PatientMonitor, VOXEL_SIZE_M
from .uart_reader import DATA_BAUD


def _sim_frames(n_frames: int, fall_at: int | None):
    """Generate synthetic frames: static furniture + optional fall event."""
    from .tlv import Frame, FrameHeader, Tlv

    rng = np.random.default_rng(42)

    # Static scene: a few furniture-like clusters
    furniture = np.array([
        [1.0, 2.0, 0.75],   # table
        [1.1, 2.1, 0.80],
        [0.9, 1.9, 0.70],
        [3.0, 4.0, 0.40],   # low shelf
        [3.1, 4.1, 0.35],
    ], dtype=np.float32)

    for i in range(n_frames):
        pts_list = []

        # Static furniture (always present, with noise)
        for base in furniture:
            if rng.random() > 0.3:  # 70% detection probability
                noise = rng.normal(0, 0.05, size=3).astype(np.float32)
                pt = base + noise
                pts_list.append(pt)

        # Person standing (before fall)
        if fall_at is None or i < fall_at:
            # Standing person at (2.0, 3.0) — energy at Z ~1.0-1.6m
            for z in np.linspace(0.8, 1.5, 4):
                if rng.random() > 0.2:
                    pt = np.array([2.0, 3.0, z], dtype=np.float32)
                    pt += rng.normal(0, 0.08, size=3).astype(np.float32)
                    pts_list.append(pt)
        elif fall_at is not None and i >= fall_at:
            # Fallen person: energy at Z ~0.1-0.3m, spread on floor
            for dx in np.linspace(-0.3, 0.3, 3):
                for dy in np.linspace(-0.2, 0.2, 2):
                    if rng.random() > 0.2:
                        pt = np.array([2.0 + dx, 3.0 + dy, 0.15], dtype=np.float32)
                        pt += rng.normal(0, 0.05, size=3).astype(np.float32)
                        pts_list.append(pt)

        if pts_list:
            xyz = np.stack(pts_list)
            # Add doppler column (zeros for static)
            points = np.c_[xyz, np.zeros(len(xyz), dtype=np.float32)]
        else:
            points = np.empty((0, 4), dtype=np.float32)

        yield points
        time.sleep(0.1)  # ~10 fps


def _live_frames(data_port: str, baud: int):
    """Yield (N,4) point arrays from live UART."""
    from .uart_reader import iter_frames
    for frame in iter_frames(data_port, baud):
        yield frame.detected_points()


def _print_status(mon: PatientMonitor, elapsed: float) -> None:
    """One-line periodic status."""
    clarity = mon.get_clarity()
    floor = mon.get_floor_anomaly()

    loc_str = ""
    if floor.peak_xy is not None:
        px, py = floor.peak_xy
        x_m = mon.grid_origin[0] + px * VOXEL_SIZE_M
        y_m = mon.grid_origin[1] + py * VOXEL_SIZE_M
        loc_str = f" @({x_m:.1f},{y_m:.1f})m"

    print(
        f"[{elapsed:6.0f}s] "
        f"frames={mon.frame_count:5d} | "
        f"A: occ={clarity.occupied_voxels:4d} "
        f"conc={clarity.energy_concentration:.3f} "
        f"Zsprd={clarity.mean_z_spread:.2f} "
        f"dens={clarity.peak_density:.4f} | "
        f"floor_Δ={floor.diff_energy:.3f}{loc_str}"
    )


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description="Spatial3D patient monitor")
    ap.add_argument("--data-port", help="radar DATA UART device")
    ap.add_argument("--baud", type=int, default=DATA_BAUD)
    ap.add_argument("--sim", action="store_true", help="run with synthetic data")
    ap.add_argument("--sim-frames", type=int, default=600, help="sim frame count")
    ap.add_argument("--sim-fall-at", type=int, default=None,
                    help="sim: frame number when fall occurs (None=no fall)")
    ap.add_argument("--mount-height", type=float, default=2.00,
                    help="sensor height above floor (metres)")
    ap.add_argument("--mount-tilt", type=float, default=35.0,
                    help="sensor tilt from vertical (degrees)")
    ap.add_argument("--grid-origin", type=float, nargs=3, default=[-2.0, 0.0, 0.0],
                    metavar=("OX", "OY", "OZ"),
                    help="room-frame offset of voxel (0,0,0)")
    ap.add_argument("--alpha", type=float, default=1e-4,
                    help="Map A EMA decay per frame (smaller=slower)")
    ap.add_argument("--window", type=float, default=60.0,
                    help="current-map rolling window (seconds)")
    ap.add_argument("--status-interval", type=float, default=5.0,
                    help="seconds between status prints")
    ap.add_argument("--load-a", help="load a saved Map A (.npy) as starting baseline")
    ap.add_argument("--save-a", help="save Map A to this path on exit")
    args = ap.parse_args(argv)

    if not args.sim and not args.data_port:
        print("need --sim or --data-port", file=sys.stderr)
        return 2

    mon = PatientMonitor(
        mount_height=args.mount_height,
        mount_tilt_deg=args.mount_tilt,
        grid_origin=tuple(args.grid_origin),
        alpha_a=args.alpha,
        window_sec=args.window,
    )

    if args.load_a:
        mon.load_map_a(args.load_a)
        print(f"[watch] loaded Map A from {args.load_a}")

    # Graceful shutdown
    running = [True]
    def _stop(sig, frame):
        running[0] = False
        print("\n[watch] stopping...")
    signal.signal(signal.SIGINT, _stop)

    if args.sim:
        print(f"[watch] sim mode: {args.sim_frames} frames"
              + (f", fall at frame {args.sim_fall_at}" if args.sim_fall_at else ""))
        source = _sim_frames(args.sim_frames, args.sim_fall_at)
    else:
        print(f"[watch] live: {args.data_port} @ {args.baud}")
        source = _live_frames(args.data_port, args.baud)

    t_start = time.time()
    last_status = t_start

    for points in source:
        if not running[0]:
            break
        mon.update(points)
        now = time.time()
        if now - last_status >= args.status_interval:
            _print_status(mon, now - t_start)
            last_status = now

    # Final status
    _print_status(mon, time.time() - t_start)

    if args.save_a:
        mon.save_map_a(args.save_a)
        print(f"[watch] saved Map A to {args.save_a}")

    print(f"[watch] done. {mon.frame_count} frames processed.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
