"""Collect static 3D point cloud with extreme CFAR settings.

Sends extreme-CFAR config (clutterRemoval=0, threshold=0dB),
then accumulates detected points over time. Each frame's TLV type 1
gives (x, y, z, doppler) for CFAR-detected targets. With low threshold
and no clutter removal, static targets should appear.

Usage:
    python -m spatial3d.static_collect --cli-port /dev/cu.usbmodemRA4431 \
        --data-port /dev/cu.usbmodemRA4434 --duration 30 --out static_3d.npz
"""

from __future__ import annotations

import argparse
import sys
import time

import numpy as np

from .tlv import TLV_DETECTED_POINTS
from .uart_reader import send_config, iter_frames


def main():
    p = argparse.ArgumentParser(description="Static 3D point cloud collector")
    p.add_argument("--cli-port", help="CLI UART port (sends config)")
    p.add_argument("--data-port", required=True, help="DATA UART port")
    p.add_argument("--cfg", default="profile_extreme_cfar.cfg",
                   help="Config file to send")
    p.add_argument("--duration", type=float, default=30,
                   help="Collection duration in seconds")
    p.add_argument("--out", default="static_3d.npz",
                   help="Output file (.npz)")
    args = p.parse_args()

    if args.cli_port:
        print(f"Sending config: {args.cfg}")
        send_config(args.cli_port, args.cfg)
        time.sleep(0.5)

    print(f"Collecting for {args.duration}s from {args.data_port} ...")

    all_points = []
    frame_count = 0
    det_frames = 0
    total_det = 0
    t0 = time.time()

    try:
        for frame in iter_frames(args.data_port):
            elapsed = time.time() - t0
            if elapsed >= args.duration:
                break

            frame_count += 1
            pts = frame.detected_points()
            n = len(pts)
            total_det += n

            if n > 0:
                det_frames += 1
                all_points.append(pts)

            if frame_count % 10 == 0:
                avg = total_det / frame_count
                print(f"\r  {elapsed:5.1f}s  frames={frame_count}  "
                      f"det_frames={det_frames}  total_pts={total_det}  "
                      f"avg={avg:.1f}/frame", end="", flush=True)

    except KeyboardInterrupt:
        pass

    print()
    elapsed = time.time() - t0
    print(f"\nDone: {frame_count} frames in {elapsed:.1f}s "
          f"({frame_count/max(elapsed,0.1):.1f} fps)")
    print(f"  Frames with detections: {det_frames}/{frame_count}")
    print(f"  Total detected points: {total_det}")

    if all_points:
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

        np.savez(args.out, cloud=cloud, static=static)
        print(f"  Saved to {args.out}")
    else:
        print("  NO POINTS DETECTED — extreme CFAR did not produce detections")
        print("  Static targets may not pass CFAR even at threshold=0")


if __name__ == "__main__":
    main()
