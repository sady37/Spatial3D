"""Entry point for the PC spatial-modeling layer.

Examples:
    python -m spatial3d.main --sim --viz          # synthetic room, visualize
    python -m spatial3d.main --port /dev/tty.usbserial-XXXX
"""

from __future__ import annotations

import argparse
import sys

from . import modeling
from .simulator import synthetic_room
from .voxel import VoxelGrid


def load_grid(args: argparse.Namespace) -> VoxelGrid:
    if args.sim:
        print("[main] using synthetic room voxel map")
        return synthetic_room()

    if not args.port:
        print("[main] no --port given and --sim not set", file=sys.stderr)
        sys.exit(2)

    from . import uart_reader

    print(f"[main] reading voxel map from {args.port} @ {args.baud}")
    stream = uart_reader.open_serial(args.port, args.baud)
    return uart_reader.read_grid(stream)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Spatial3D PC modeling")
    parser.add_argument("--sim", action="store_true", help="use synthetic room")
    parser.add_argument("--port", help="UART serial port")
    parser.add_argument("--baud", type=int, default=921600)
    parser.add_argument("--viz", action="store_true", help="open Open3D viewer")
    args = parser.parse_args(argv)

    grid = load_grid(args)
    points = modeling.grid_to_pointcloud(grid)
    print(f"[main] occupied voxels: {len(points)}")

    planes = modeling.fit_planes(points)
    print(f"[main] fitted {len(planes)} planes (floor/walls):")
    for i, p in enumerate(planes):
        print(f"    plane {i}: n={tuple(round(x, 3) for x in p.normal)} "
              f"d={p.d:.3f} inliers={len(p.inliers)}")

    if args.viz:
        modeling.visualize(points, planes)

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
