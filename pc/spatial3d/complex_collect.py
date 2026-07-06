"""Collect point cloud + per-antenna complex data for MUSIC processing.

Like static_collect.py but also captures the complex antenna data (TLV type 8)
alongside the point cloud. Both are saved to the output .npz file.

Requires firmware modification to emit TLV_ANTENNA_COMPLEX. If the firmware
has not been modified, the collector falls back to saving only the point cloud
(same as static_collect.py) with a clear warning.

Usage:
    python -m spatial3d.complex_collect \
        --data-port /dev/cu.usbmodem0000RA444 \
        --duration 600 --out music_10min.npz
"""

from __future__ import annotations

import argparse
import sys
import time

import numpy as np

from .tlv import TLV_DETECTED_POINTS, TLV_ANTENNA_COMPLEX
from .uart_reader import send_config, iter_frames


def main():
    p = argparse.ArgumentParser(
        description="Point cloud + antenna complex data collector")
    p.add_argument("--cli-port", help="CLI UART port (sends config)")
    p.add_argument("--data-port", required=True, help="DATA UART port")
    p.add_argument("--cfg", default="profile_extreme_cfar.cfg",
                   help="Config file to send")
    p.add_argument("--duration", type=float, default=30,
                   help="Collection duration in seconds")
    p.add_argument("--out", default="complex_3d.npz",
                   help="Output file (.npz)")
    args = p.parse_args()

    if args.cli_port:
        print(f"Sending config: {args.cfg}")
        send_config(args.cli_port, args.cfg)
        time.sleep(0.5)

    print(f"Collecting for {args.duration}s from {args.data_port} ...")

    all_points: list[np.ndarray] = []
    all_complex: list[np.ndarray] = []
    frame_count = 0
    det_frames = 0
    complex_frames = 0
    total_det = 0
    warned_no_complex = False
    t0 = time.time()

    try:
        for frame in iter_frames(args.data_port):
            elapsed = time.time() - t0
            if elapsed >= args.duration:
                break

            frame_count += 1
            pts = frame.detected_points()
            cx = frame.antenna_complex()
            n = len(pts)
            total_det += n

            if n > 0:
                det_frames += 1
                all_points.append(pts)

                if cx is not None and len(cx) == n:
                    complex_frames += 1
                    all_complex.append(cx)
                elif cx is not None and len(cx) != n:
                    # Mismatch between point count and complex count
                    print(f"\n  WARNING: frame {frame_count}: {n} points "
                          f"but {len(cx)} complex entries (mismatch)")
                    all_complex.append(np.zeros((n, 16), dtype=np.complex64))
                elif not warned_no_complex and n > 0:
                    print(f"\n  WARNING: frame {frame_count}: {n} points "
                          f"but no antenna complex TLV")

            # After 10 frames with detections but no complex data, print clear message
            if det_frames == 10 and complex_frames == 0 and not warned_no_complex:
                warned_no_complex = True
                print("\n  No antenna complex TLV detected. "
                      "Firmware modification required.")
                print("  Falling back to point-cloud-only mode.\n")

            if frame_count % 10 == 0:
                cx_status = f"cx={complex_frames}" if complex_frames > 0 else "cx=NONE"
                avg = total_det / frame_count
                print(f"\r  {elapsed:5.1f}s  frames={frame_count}  "
                      f"det={det_frames}  pts={total_det}  "
                      f"avg={avg:.1f}/frame  {cx_status}",
                      end="", flush=True)

    except KeyboardInterrupt:
        pass

    print()
    elapsed = time.time() - t0
    print(f"\nDone: {frame_count} frames in {elapsed:.1f}s "
          f"({frame_count/max(elapsed,0.1):.1f} fps)")
    print(f"  Frames with detections: {det_frames}/{frame_count}")
    print(f"  Frames with complex data: {complex_frames}/{det_frames}")
    print(f"  Total detected points: {total_det}")

    if not all_points:
        print("  NO POINTS DETECTED")
        return

    cloud = np.concatenate(all_points, axis=0)
    print(f"  Point cloud shape: {cloud.shape}")
    print(f"  X range: [{cloud[:,0].min():.2f}, {cloud[:,0].max():.2f}]")
    print(f"  Y range: [{cloud[:,1].min():.2f}, {cloud[:,1].max():.2f}]")
    print(f"  Z range: [{cloud[:,2].min():.2f}, {cloud[:,2].max():.2f}]")
    print(f"  Doppler range: [{cloud[:,3].min():.3f}, {cloud[:,3].max():.3f}]")

    # Filter near-zero doppler (static)
    static_mask = np.abs(cloud[:, 3]) < 0.5
    static = cloud[static_mask]
    print(f"\n  Static points (|doppler|<0.5): {len(static)}/{len(cloud)}")

    has_complex = len(all_complex) > 0 and complex_frames > 0

    if has_complex:
        antenna_cx = np.concatenate(all_complex, axis=0)
        static_cx = antenna_cx[static_mask]
        print(f"  Antenna complex shape: {antenna_cx.shape}")
        print(f"  Static complex shape: {static_cx.shape}")
        np.savez(args.out,
                 cloud=cloud,
                 static=static,
                 antenna_complex=antenna_cx,
                 static_complex=static_cx)
        print(f"  Saved (with complex data) to {args.out}")
    else:
        print("\n  WARNING: No complex antenna data collected.")
        print("  Saving point cloud only (firmware mod needed for MUSIC).")
        np.savez(args.out, cloud=cloud, static=static)
        print(f"  Saved (point cloud only) to {args.out}")


if __name__ == "__main__":
    main()
