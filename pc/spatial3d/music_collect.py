"""End-to-end per-bin MUSIC pipeline: covariances -> DOA -> 3D voxel map.

Consumes per-bin covariances (either acquired live via the rolling layered
scan, or loaded from a music_cov.npz produced by complex_collect.py), runs
2D MUSIC per range bin to super-resolve arrival angle(s), maps each
(range_bin, az, el) to a radar-frame 3D point, transforms into the room frame,
and (optionally) renders a 3D voxel map with coarse height-band semantics.

This replaces the old per-detection refine path: the firmware indexes complex
data by range bin, so DOA is estimated per bin from a cross-frame covariance,
not per point-cloud detection.

Usage (offline, from saved covariances):
    python -m spatial3d.music_collect --from-npz music_cov.npz \
        --voxel-size 0.3 --out music_room.npz --voxel-out music_voxel.png

Usage (live end-to-end):
    python -m spatial3d.music_collect --cli-port /dev/cu.usbmodem0000RA441 \
        --data-port /dev/cu.usbmodem0000RA444 --cfg profile_4T4R_music.cfg \
        --snapshots 50 --out music_room.npz
"""

from __future__ import annotations

import argparse

import numpy as np

from .music import awrl6844_array, spatial_smoothing_2d, subarray_array
from .range_music import DR_M, covariances_to_points

# Room-frame transform (mount geometry) — kept in sync with voxel_map.
TILT_DEG = 35.0
H_MOUNT = 2.0


def to_room(pts: np.ndarray, tilt_deg: float = TILT_DEG,
            h_mount: float = H_MOUNT) -> np.ndarray:
    """Radar-frame (x, y, z, ...) -> room frame, applying tilt + mount height.

    Rotates about the X axis by the downward mounting tilt and lifts by the
    mount height. Extra columns (power, bin, range) are carried through. Same
    convention as voxel_map.to_room.
    """
    tilt = np.radians(tilt_deg)
    x, y, z = pts[:, 0], pts[:, 1], pts[:, 2]
    xr = x
    yr = y * np.cos(tilt) + z * np.sin(tilt)
    zr = -y * np.sin(tilt) + z * np.cos(tilt) + h_mount
    room = np.column_stack([xr, yr, zr])
    if pts.shape[1] > 3:
        room = np.column_stack([room, pts[:, 3:]])
    mask = np.isfinite(xr) & np.isfinite(yr) & np.isfinite(zr)
    return room[mask]


def build_covariances(stacks: dict[int, np.ndarray] | None,
                      covs: dict[int, np.ndarray] | None,
                      smooth_sub: tuple[int, int] | None):
    """Return (covariances, array) for MUSIC.

    With *smooth_sub* set, 2D forward-backward spatial smoothing is applied to
    per-bin snapshot *stacks* (required) and a matching sub-array is returned;
    this decorrelates coherent same-range scatterers. Otherwise the raw 16x16
    covariances and the full 4x4 array are used.
    """
    if smooth_sub is not None:
        if not stacks:
            raise ValueError("spatial smoothing needs snapshot stacks "
                             "(acquire live or save with --save-snapshots)")
        smoothed = {b: spatial_smoothing_2d(s, grid=(4, 4), sub=smooth_sub)
                    for b, s in stacks.items()}
        return smoothed, subarray_array(smooth_sub)
    if covs is None:
        # Derive plain covariances from stacks.
        from .music import estimate_covariance
        covs = {b: estimate_covariance(s) for b, s in (stacks or {}).items()}
    return covs, awrl6844_array()


def semantic_label(z_room: float) -> str:
    """Coarse height-band semantic tag for a room-frame Z (metres).

    A first-cut classifier: height bands map to likely furniture. Refine with
    per-cluster geometry (extent, verticality) once the voxel map is stable.
    """
    if z_room < 0.4:
        return "floor"          # floor / fall zone
    if z_room < 0.7:
        return "bed_low"        # bed / low seat
    if z_room < 1.1:
        return "table_chair"    # table top / chair back
    if z_room < 1.8:
        return "standing"       # standing person / shelf
    return "wall_high"          # high wall / ceiling fixtures


def _load_npz(path: str):
    """Load covariances (+ optional snapshot stacks) from a music_cov.npz."""
    data = np.load(path, allow_pickle=True)
    bins = data["bins"].astype(int)
    covs = {int(b): data["covariances"][i] for i, b in enumerate(bins)}
    stacks = None
    if "snapshots" in data and "snap_bins" in data:
        sb = data["snap_bins"].astype(int)
        # Uniform-shape stacks come back as a 3D object array; force complex.
        stacks = {int(b): np.asarray(data["snapshots"][i], dtype=np.complex64)
                  for i, b in enumerate(sb)}
    dr = float(data["dr_m"]) if "dr_m" in data else DR_M
    return covs, stacks, dr


def run_pipeline(covs, stacks, dr, args) -> np.ndarray:
    """Covariances -> per-bin MUSIC DOA -> room-frame points with semantics.

    Returns (M, 7): [x_room, y_room, z_room, power, bin, range_m, label_id].
    """
    smooth_sub = None
    if args.smooth:
        se, sa = (int(v) for v in args.smooth.split("x"))
        smooth_sub = (se, sa)

    covariances, array = build_covariances(stacks, covs, smooth_sub)
    method = getattr(args, "method", "music").upper()
    print(f"{method} over {len(covariances)} bins "
          f"(array={array.n_antennas} ant, smooth={smooth_sub}) ...")

    radar_pts = covariances_to_points(
        covariances, array, dr=dr,
        n_signals=args.n_signals,
        az_range=tuple(args.az_range), el_range=tuple(args.el_range),
        resolution_deg=args.resolution,
        max_peaks_per_bin=args.max_peaks,
        method=getattr(args, "method", "music"),
    )
    print(f"  MUSIC points: {len(radar_pts)}")
    if len(radar_pts) == 0:
        return np.empty((0, 7), dtype=np.float32)

    room = to_room(radar_pts)  # (M, 6): x,y,z,power,bin,range
    labels = np.array([_LABEL_ID[semantic_label(z)] for z in room[:, 2]],
                      dtype=np.float32)
    return np.column_stack([room, labels]).astype(np.float32)


_LABELS = ["floor", "bed_low", "table_chair", "standing", "wall_high"]
_LABEL_ID = {name: i for i, name in enumerate(_LABELS)}


def main():
    p = argparse.ArgumentParser(
        description="Per-bin MUSIC pipeline: covariances -> DOA -> 3D voxel map")
    src = p.add_argument_group("source (live or from file)")
    src.add_argument("--from-npz", help="Load per-bin covariances from .npz")
    src.add_argument("--cli-port", help="CLI UART port (live)")
    src.add_argument("--data-port", help="DATA UART port (live)")
    src.add_argument("--cfg", help="MUSIC .cfg (live)")
    src.add_argument("--snapshots", type=int, default=None,
                     help="K per bin (live; default: cfg '% spatial3d:' or 50)")
    src.add_argument("--min-snapshots", type=int, default=None,
                     help="Min snapshots for a covariance (default: cfg or 10)")
    src.add_argument("--roll", action="store_true",
                     help="Full-room 3-layer calibration scan (live). "
                          "Default: single active window (real-time model).")

    mus = p.add_argument_group("MUSIC")
    mus.add_argument("--n-signals", type=int, default=None,
                     help="Fixed sources per bin (default: MDL auto)")
    mus.add_argument("--resolution", type=float, default=1.0,
                     help="Angle grid spacing (deg)")
    mus.add_argument("--az-range", type=float, nargs=2, default=[-45.0, 45.0])
    mus.add_argument("--el-range", type=float, nargs=2, default=[-45.0, 20.0])
    mus.add_argument("--max-peaks", type=int, default=3,
                     help="Max DOA peaks kept per bin")
    mus.add_argument("--smooth", default=None,
                     help="2D spatial smoothing sub-array, e.g. '3x3' or '2x2' "
                          "(needs snapshot stacks)")
    mus.add_argument("--method", choices=["music", "fft"], default="music",
                     help="DOA method: music (super-res) or fft (Bartlett baseline)")

    out = p.add_argument_group("output")
    out.add_argument("--voxel-size", type=float, default=0.3)
    out.add_argument("--out", default="music_room.npz")
    out.add_argument("--voxel-out", default=None,
                     help="Render a 3D voxel PNG here (needs matplotlib)")
    out.add_argument("--floor-out", default=None,
                     help="Render a floor-plan PNG here")
    out.add_argument("--occ-threshold", type=int, default=3,
                     help="Min points for a voxel to count as occupied in one scan")

    conf = p.add_argument_group("confidence (multi-scan voting)")
    conf.add_argument("--repeat", type=int, default=1,
                      help="Run N independent scans and vote per voxel "
                           "(rejects transients, confirms stable structure)")
    conf.add_argument("--vote-min", type=int, default=None,
                      help="Keep voxels occupied in >= this many scans "
                           "(default: majority = ceil(N/2))")
    args = p.parse_args()

    if not (args.from_npz or (args.cli_port and args.data_port and args.cfg)):
        p.error("provide --from-npz OR (--cli-port --data-port --cfg)")
        return

    repeat = args.repeat
    if args.from_npz and repeat > 1:
        print("  --repeat ignored for --from-npz (deterministic); using 1")
        repeat = 1

    # --- run repeat scans, keep a voxel grid + cloud from each ---
    from .voxel_map import build_voxel_grid

    grids: list[np.ndarray] = []
    clouds: list[np.ndarray] = []
    ranges = None
    for i in range(repeat):
        if repeat > 1:
            print(f"\n=== scan {i + 1}/{repeat} ===")
        covs, stacks, dr = _acquire_source(args)
        if not covs and not stacks:
            print("  NO COVARIANCES — skipping scan")
            continue
        cloud = run_pipeline(covs, stacks, dr, args)
        if len(cloud) == 0:
            print("  NO MUSIC POINTS this scan")
            continue
        clouds.append(cloud)
        grid, ranges = build_voxel_grid(cloud[:, :3], args.voxel_size)
        grids.append(grid)

    if not clouds:
        print("\n  NO MUSIC POINTS from any scan — check FOV/thresholds/covariances")
        return

    # --- vote across scans ---
    vote_min = args.vote_min if args.vote_min is not None else (len(grids) + 1) // 2
    votes = np.sum([g >= args.occ_threshold for g in grids], axis=0).astype(np.int32)
    kept = int((votes >= vote_min).sum())
    if len(grids) > 1:
        print(f"\n  Voting across {len(grids)} scans (occ>= {args.occ_threshold}/scan): "
              f"keep voxels in >= {vote_min} scans -> {kept} stable voxels")
        per_scan = [int((g >= args.occ_threshold).sum()) for g in grids]
        print(f"  per-scan occupied: {per_scan}")

    all_cloud = np.concatenate(clouds, axis=0)
    np.savez(args.out,
             music_cloud=all_cloud[:, :6],
             labels=all_cloud[:, 6].astype(np.int32),
             label_names=np.array(_LABELS),
             votes=votes, vote_min=np.int32(vote_min),
             voxel_size=np.float32(args.voxel_size))
    print(f"\n  Saved {len(all_cloud)} points ({len(grids)} scans) to {args.out}")

    if args.voxel_out or args.floor_out:
        if len(grids) == 1:
            _render_grid(grids[0], ranges, args.occ_threshold, args)  # density
        else:
            _render_voted(votes, ranges, vote_min, args)              # confidence


def _acquire_source(args):
    """Get (covs, stacks, dr) from a saved npz or one live acquisition."""
    if args.from_npz:
        print(f"Loading covariances from {args.from_npz}")
        return _load_npz(args.from_npz)

    from .complex_collect import acquire_covariances

    class _A:  # adapt to acquire_covariances' arg interface (None => read cfg)
        cli_port = args.cli_port
        data_port = args.data_port
        cfg = args.cfg
        snapshots = args.snapshots
        min_snapshots = args.min_snapshots
        rounds = None
        layer_timeout = None
        timeout = None
        layers = None
        roll = args.roll
    bins, cov, cnt, acc = acquire_covariances(_A)   # resolves _A from cfg
    covs = {int(b): cov[i] for i, b in enumerate(bins)}
    stacks = acc.stacks(min_snapshots=_A.min_snapshots) if args.smooth else None
    return covs, stacks, DR_M


def _render_grid(grid: np.ndarray, ranges, threshold: int, args) -> None:
    """Single-scan render: colour scales with point density."""
    from .voxel_map import render_voxels, render_floor_plan

    print(f"  Voxel grid {grid.shape}, occupied>= {threshold}: {(grid >= threshold).sum()}")
    if args.voxel_out:
        render_voxels(grid, ranges, args.voxel_size, threshold=threshold,
                      out_path=args.voxel_out)
    if args.floor_out:
        render_floor_plan(grid, ranges, args.voxel_size, threshold=threshold,
                          out_path=args.floor_out)


def _render_voted(votes: np.ndarray, ranges, vote_min: int, args) -> None:
    """Multi-scan render: colour scales with vote count (confidence)."""
    from .voxel_map import render_voxels, render_floor_plan

    n_occ = int((votes >= vote_min).sum())
    print(f"  Voted voxel grid {votes.shape}, kept (>= {vote_min} scans): {n_occ}")
    if args.voxel_out:
        render_voxels(votes, ranges, args.voxel_size, threshold=vote_min,
                      out_path=args.voxel_out)
    if args.floor_out:
        render_floor_plan(votes, ranges, args.voxel_size, threshold=vote_min,
                          out_path=args.floor_out)


if __name__ == "__main__":
    main()
