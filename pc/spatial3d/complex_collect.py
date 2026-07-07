"""Acquisition: rolling layered range-antenna scan -> per-bin covariances.

The Spatial3D firmware emits a zero-Doppler range-antenna TLV (type 8), but
one ~82-bin *layer* per frame fits the UART budget. This collector rolls
through the layers (re-sending ``rangeAntennaOutput`` over CLI *while* the DATA
port keeps draining, via RadarSession) and accumulates K snapshots per range
bin. Each bin's snapshots build a spatial covariance matrix, saved to .npz for
offline MUSIC (see music_collect.py) or fed straight into the live pipeline.

This replaces the old per-detection complex path: the firmware indexes complex
data by RANGE BIN, not by detection, so covariance is built per bin across
frames rather than per point within a frame.

Usage:
    python -m spatial3d.complex_collect \
        --cli-port /dev/cu.usbmodem0000RA441 \
        --data-port /dev/cu.usbmodem0000RA444 \
        --cfg /path/to/profile_4T4R_music.cfg \
        --snapshots 50 --out music_cov.npz
"""

from __future__ import annotations

import argparse
import time

import numpy as np

from .range_music import (
    LAYERS,
    N_VIRT_ANT,
    BinAccumulator,
    parse_layers_from_cfg,
    parse_music_params_from_cfg,
)
from .uart_reader import RadarSession


# Hardcoded fallbacks, used only when neither the CLI flag nor the cfg set a value.
_DEFAULTS = dict(snapshots=50, min_snapshots=10, rounds=4,
                 layer_timeout=12.0, timeout=90.0)


def _pick(cli_value, cfg_value, default):
    """Resolve a parameter: explicit CLI flag > cfg directive > hardcoded default."""
    if cli_value is not None:
        return cli_value
    if cfg_value is not None:
        return cfg_value
    return default


def resolve_params(args) -> dict:
    """Fill acquisition params from cfg (layers + ``% spatial3d:`` K/dwell),
    letting any explicit CLI flag win. Mutates *args* in place and returns the
    resolved dict for convenience."""
    cfg_params = parse_music_params_from_cfg(args.cfg)
    layers = (_parse_layers(args.layers) if args.layers
              else parse_layers_from_cfg(args.cfg))
    resolved = {"layers": layers}
    for key, default in _DEFAULTS.items():
        val = _pick(getattr(args, key, None), cfg_params.get(key), default)
        resolved[key] = val
        setattr(args, key, val)      # write back so downstream sees resolved value
    args.layers = layers
    return resolved


def roll_and_accumulate(
    session: RadarSession,
    layers: list[tuple[int, int]] = LAYERS,
    k: int = 50,
    rounds: int = 4,
    per_layer_timeout: float = 12.0,
    total_timeout: float = 90.0,
    switch: str = "restart",
    reconfig=None,
    verbose: bool = True,
) -> BinAccumulator:
    """Roll through *layers*, accumulating up to *k* snapshots per bin.

    For each layer the range-antenna window is switched (via ``switch``:
    "restart" = sensorStop/reconfig/sensorStart, the safe default; "live" =
    re-send ``rangeAntennaOutput`` while streaming, which WEDGES the current
    firmware) and frames are consumed until every bin in the layer holds *k*
    snapshots or the per-layer timeout elapses. Multiple *rounds* let lagging
    bins catch up. Returns the populated accumulator.
    """
    acc = BinAccumulator(k=k, n_ant=N_VIRT_ANT)
    t_start = time.time()

    for rnd in range(rounds):
        all_full = True
        for (start_bin, num_bins) in layers:
            if acc.is_layer_full(start_bin, num_bins):
                continue
            all_full = False
            if switch == "none":
                pass  # single-window: demo already streams this from sensorStart
            elif switch == "reconfig":
                reconfig(start_bin, num_bins)   # full cfg resend for this window
            elif switch == "live":
                session.set_layer(start_bin, num_bins, echo=verbose)
            else:
                session.restart_layer(start_bin, num_bins, echo=verbose)
            t_layer = time.time()
            got = 0
            while time.time() - t_layer < per_layer_timeout:
                if time.time() - t_start > total_timeout:
                    break
                frame = session.get_frame(timeout=1.0)
                if frame is None:
                    continue
                ra = frame.range_antenna()
                if ra is not None:
                    got += acc.add(ra)
                if acc.is_layer_full(start_bin, num_bins):
                    break
            if verbose:
                mn = acc.min_count(range(start_bin, start_bin + num_bins))
                print(f"  round {rnd} layer bins {start_bin}-"
                      f"{start_bin + num_bins - 1}: +{got} snaps, "
                      f"min/bin={mn}/{k}")
            if time.time() - t_start > total_timeout:
                if verbose:
                    print("  total timeout reached")
                return acc
        if all_full:
            if verbose:
                print(f"  all layers full after {rnd} round(s)")
            break
    return acc


def _covariances_to_arrays(covs: dict[int, np.ndarray], counts: dict[int, int]):
    """Pack {bin: R} into aligned (bins, covariances, counts) arrays."""
    bins = np.asarray(sorted(covs), dtype=np.int32)
    if len(bins) == 0:
        return bins, np.empty((0, N_VIRT_ANT, N_VIRT_ANT), np.complex64), \
            np.empty((0,), np.int32)
    cov = np.stack([covs[b] for b in bins]).astype(np.complex64)
    cnt = np.asarray([counts.get(int(b), 0) for b in bins], dtype=np.int32)
    return bins, cov, cnt


def acquire_covariances(args) -> tuple[np.ndarray, np.ndarray, np.ndarray, BinAccumulator]:
    """Open the radar, run the scan, return (bins, covs, counts, acc).

    Default is SINGLE-WINDOW (real-time model): configure once and collect on
    the cfg's active rangeAntennaOutput window — no layer switching, so the
    one-time startup/calibration is paid once and there is no per-layer
    sensorStart. Pass ``roll=True`` for the full-room 3-layer calibration scan
    (nighttime), which switches windows via sensorStop/reconfig/sensorStart.
    """
    r = resolve_params(args)
    roll = getattr(args, "roll", False)
    if roll:
        layers, switch = r["layers"], "reconfig"
        mode = f"ROLL {len(layers)} layers"
    else:
        layers, switch = [r["layers"][0]], "none"   # active window only
        mode = "SINGLE-WINDOW"
    print(f"Scan plan from cfg: {mode} "
          f"(bins {layers[0][0]}-{layers[-1][0] + layers[-1][1] - 1}), "
          f"K={r['snapshots']} snapshots/bin")
    # Give each layer enough dwell to reach K at ~10 fps (+margin), and scale the
    # total accordingly — the defaults (12s/90s) are too short for large K.
    k = r["snapshots"]
    per_layer = max(r["layer_timeout"], k / 10.0 * 1.3 + 6.0)
    total = max(r["timeout"], per_layer * len(layers) * 1.4 + 20.0)

    print(f"Opening session CLI={args.cli_port} DATA={args.data_port}")
    with RadarSession(args.cli_port, args.data_port) as session:
        print(f"Sending config: {args.cfg}")
        session.send_cfg(args.cfg)                 # initial calib + layer[0] active
        if session.cli_errors:
            print(f"  WARNING: CLI errors on: {session.cli_errors}")

        def _reconfig(sb, nb):
            # rangeAntennaOutput is only honoured in a full config parse, so
            # resend the whole cfg with this layer's window. Calibration is
            # cached (restore cfg) so this is fast.
            session.send_cfg(args.cfg, echo=False, layer=(sb, nb))
            session.flush_frames()

        acc = roll_and_accumulate(
            session, layers=layers, k=k,
            rounds=r["rounds"], per_layer_timeout=per_layer,
            total_timeout=total, switch=switch,
            reconfig=_reconfig if switch == "reconfig" else None,
        )
        print(f"Frames read={session.frames_read} dropped={session.frames_dropped}")

    counts = acc.counts()
    covs = acc.covariances(min_snapshots=args.min_snapshots)
    bins, cov, cnt = _covariances_to_arrays(covs, counts)
    print(f"Bins seen: {len(counts)}  |  bins with cov "
          f"(>= {args.min_snapshots} snaps): {len(bins)}")
    return bins, cov, cnt, acc


def _parse_layers(spec: str) -> list[tuple[int, int]]:
    """Parse 'start:num,start:num,...' into a layer list."""
    out = []
    for part in spec.split(","):
        s, n = part.split(":")
        out.append((int(s), int(n)))
    return out


def main():
    p = argparse.ArgumentParser(
        description="Rolling layered range-antenna scan -> per-bin covariances")
    p.add_argument("--cli-port", required=True, help="CLI UART port")
    p.add_argument("--data-port", required=True, help="DATA UART port")
    p.add_argument("--cfg", required=True, help="MUSIC .cfg (with sensorStart)")
    # Defaults are None so cfg values win unless a flag is explicitly given.
    # Resolution order: CLI flag > '% spatial3d:' cfg directive > hardcoded default.
    p.add_argument("--snapshots", type=int, default=None,
                   help="Target snapshots per bin K (default: cfg or 50)")
    p.add_argument("--min-snapshots", type=int, default=None,
                   help="Min snapshots for a bin to yield a covariance "
                        "(default: cfg or 10)")
    p.add_argument("--rounds", type=int, default=None,
                   help="Max passes over all layers (default: cfg or 4)")
    p.add_argument("--layer-timeout", type=float, default=None,
                   help="Per-layer dwell timeout in s (default: cfg or 12)")
    p.add_argument("--timeout", type=float, default=None,
                   help="Total acquisition timeout in s (default: cfg or 90)")
    p.add_argument("--layers", default=None,
                   help="Override layers as 'start:num,start:num,...' "
                        "(default: read from cfg rangeAntennaOutput lines)")
    p.add_argument("--roll", action="store_true",
                   help="Full-room multi-layer calibration scan (nighttime). "
                        "Default: single active window (real-time model).")
    p.add_argument("--save-snapshots", action="store_true",
                   help="Also save raw per-bin snapshot stacks (large)")
    p.add_argument("--out", default="music_cov.npz", help="Output .npz")
    args = p.parse_args()

    bins, cov, cnt, acc = acquire_covariances(args)

    if len(bins) == 0:
        print("  NO COVARIANCES — check firmware TLV type 8 and CLI errors")
        return

    save = dict(bins=bins, covariances=cov, counts=cnt,
                dr_m=np.float32(0.0234375))
    if args.save_snapshots:
        stacks = acc.stacks(min_snapshots=args.min_snapshots)
        # Ragged if K varies; store as an object array keyed by index order.
        save["snap_bins"] = np.asarray(sorted(stacks), dtype=np.int32)
        save["snapshots"] = np.array(
            [stacks[b] for b in sorted(stacks)], dtype=object)
    np.savez(args.out, **save)
    print(f"  Saved {len(bins)} bin covariances to {args.out}")


if __name__ == "__main__":
    main()
