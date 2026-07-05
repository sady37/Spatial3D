"""Entry point for the PC spatial-modeling layer.

Two modes:
    # offline synthetic room (no hardware)
    python -m spatial3d.main --sim --viz

    # live: send cfg to CLI port, accumulate N frames from DATA port, model
    python -m spatial3d.main --data-port /dev/cu.usbmodem0000RA444 \
        --cli-port /dev/cu.usbmodem0000RA441 --cfg profile.cfg --frames 300 --viz

See docs/GET-DATA-FROM-TI.md for how the radar starts streaming in the first place.
"""

from __future__ import annotations

import argparse
import itertools
import sys

from . import modeling
from .simulator import synthetic_room
from .voxel import VoxelGrid


def _load_sim() -> VoxelGrid:
    print("[main] using synthetic room voxel map")
    return synthetic_room()


def _load_live(args) -> VoxelGrid:
    from . import uart_reader

    if args.cfg:
        print(f"[main] sending cfg {args.cfg} -> {args.cli_port}")
        uart_reader.send_config(args.cli_port, args.cfg)

    print(f"[main] accumulating {args.frames} frames from {args.data_port}")
    grid = VoxelGrid()
    total = 0
    for frame in itertools.islice(uart_reader.iter_frames(args.data_port), args.frames):
        pts = frame.detected_points()
        total += grid.add_points(pts, origin=tuple(args.origin))
    print(f"[main] binned {total} radar points into the grid")
    return grid


def main(argv=None) -> int:
    p = argparse.ArgumentParser(description="Spatial3D PC modeling")
    p.add_argument("--sim", action="store_true", help="use synthetic room")
    p.add_argument("--data-port", help="radar DATA UART")
    p.add_argument("--cli-port", help="radar CLI UART")
    p.add_argument("--cfg", help="radar profile .cfg to send on the CLI port")
    p.add_argument("--frames", type=int, default=300, help="frames to accumulate")
    p.add_argument("--origin", type=float, nargs=3, default=[0.0, 0.0, 0.0],
                   metavar=("OX", "OY", "OZ"), help="sensor->room offset (m)")
    p.add_argument("--viz", action="store_true", help="open Open3D viewer")
    args = p.parse_args(argv)

    if args.sim:
        grid = _load_sim()
    elif args.data_port:
        grid = _load_live(args)
    else:
        print("[main] need --sim or --data-port", file=sys.stderr)
        return 2

    points = modeling.grid_to_pointcloud(grid)
    print(f"[main] occupied voxels: {len(points)}")
    if len(points) < 3:
        print("[main] too few occupied voxels to model")
        return 0

    planes = modeling.fit_planes(points)
    print(f"[main] fitted {len(planes)} planes (floor/walls):")
    for i, pl in enumerate(planes):
        print(f"    plane {i}: n={tuple(round(x, 3) for x in pl.normal)} "
              f"d={pl.d:.3f} inliers={len(pl.inliers)}")

    if args.viz:
        modeling.visualize(points, planes)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
