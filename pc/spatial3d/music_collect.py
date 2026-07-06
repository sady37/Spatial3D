"""End-to-end MUSIC pipeline collector: collect -> MUSIC refine -> save.

Reads TLV frames from the radar, extracts point cloud + per-antenna complex
data, runs MUSIC angle refinement on each static detection, and saves both
FFT-resolution and MUSIC-refined point clouds.

Requires:
  - Firmware mod to emit TLV_ANTENNA_COMPLEX (type 8)
  - spatial3d.music module with refine_angles()

If complex TLV data is not available (firmware not modified), falls back to
FFT-only mode and saves the original point cloud without MUSIC refinement.

Usage:
    python -m spatial3d.music_collect \
        --data-port /dev/cu.usbmodem0000RA444 \
        --duration 600 --out music_room_map.npz
"""

from __future__ import annotations

import argparse
import sys
import time

import numpy as np

from .tlv import TLV_DETECTED_POINTS, TLV_ANTENNA_COMPLEX
from .uart_reader import send_config, iter_frames

# Lazy import of music module -- may not exist yet
_music_available = None
_refine_angles = None


def _load_music():
    """Try to import refine_angles from spatial3d.music. Cache result."""
    global _music_available, _refine_angles
    if _music_available is not None:
        return _music_available
    try:
        from .music import refine_angles
        _refine_angles = refine_angles
        _music_available = True
    except ImportError:
        _music_available = False
    return _music_available


def _cart_to_spherical(pts: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Convert (N, 3+) Cartesian (x, y, z, ...) to (range, azimuth_rad, elevation_rad).

    Convention: azimuth = atan2(x, y), elevation = asin(z / range).
    """
    x, y, z = pts[:, 0], pts[:, 1], pts[:, 2]
    r = np.sqrt(x**2 + y**2 + z**2)
    az = np.arctan2(x, y)
    el = np.where(r > 0, np.arcsin(np.clip(z / np.maximum(r, 1e-9), -1, 1)), 0.0)
    return r, az, el


def _spherical_to_cart(r: np.ndarray, az: np.ndarray, el: np.ndarray) -> np.ndarray:
    """Convert (range, azimuth_rad, elevation_rad) to (N, 3) Cartesian (x, y, z)."""
    cos_el = np.cos(el)
    x = r * cos_el * np.sin(az)
    y = r * cos_el * np.cos(az)
    z = r * np.sin(el)
    return np.column_stack([x, y, z])


def _refine_one(complex_vec: np.ndarray, fft_az: float, fft_el: float,
                search_half_deg: float = 15.0) -> tuple[float, float]:
    """Run MUSIC refinement on a single detection's complex data.

    Returns refined (azimuth_rad, elevation_rad).
    Falls back to FFT angles on any error.
    """
    half_rad = np.deg2rad(search_half_deg)
    try:
        az_refined, el_refined = _refine_angles(
            complex_vec,
            az_center=fft_az,
            el_center=fft_el,
            az_range=half_rad,
            el_range=half_rad,
        )
        return az_refined, el_refined
    except Exception:
        return fft_az, fft_el


def main():
    p = argparse.ArgumentParser(
        description="MUSIC pipeline collector: collect + refine + save")
    p.add_argument("--cli-port", help="CLI UART port (sends config)")
    p.add_argument("--data-port", required=True, help="DATA UART port")
    p.add_argument("--cfg", default="profile_extreme_cfar.cfg",
                   help="Config file to send")
    p.add_argument("--duration", type=float, default=30,
                   help="Collection duration in seconds")
    p.add_argument("--search-deg", type=float, default=15.0,
                   help="MUSIC search half-window in degrees (default 15)")
    p.add_argument("--out", default="music_room_map.npz",
                   help="Output file (.npz)")
    args = p.parse_args()

    # Check MUSIC module availability
    has_music = _load_music()
    if has_music:
        print("MUSIC module loaded (spatial3d.music)")
    else:
        print("WARNING: spatial3d.music not available. "
              "MUSIC refinement disabled; FFT-only mode.")

    if args.cli_port:
        print(f"Sending config: {args.cfg}")
        send_config(args.cli_port, args.cfg)
        time.sleep(0.5)

    print(f"Collecting for {args.duration}s from {args.data_port} ...")

    fft_points: list[np.ndarray] = []
    music_points: list[np.ndarray] = []
    frame_count = 0
    det_frames = 0
    complex_frames = 0
    music_refined_count = 0
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
            # TODO(spatial3d): firmware now emits a per-range-bin zero-Doppler block
            # (frame.range_antenna()), not per-detection vectors. The has_cx/len==n
            # path below no longer aligns; rework to map detections->bins by range and
            # accumulate across frames for MUSIC covariance.
            cx = frame.antenna_complex()
            n = len(pts)
            total_det += n

            if n == 0:
                continue

            det_frames += 1
            fft_points.append(pts)

            has_cx = cx is not None and len(cx) == n
            if has_cx:
                complex_frames += 1

            # After 10 detection frames with no complex data, warn once
            if det_frames == 10 and complex_frames == 0 and not warned_no_complex:
                warned_no_complex = True
                print("\n  No antenna complex TLV detected. "
                      "Firmware modification required.")
                print("  Running in FFT-only mode.\n")

            # MUSIC refinement on static points
            if has_cx and has_music:
                # Static mask
                static_mask = np.abs(pts[:, 3]) < 0.5
                r, az, el = _cart_to_spherical(pts)

                refined_az = az.copy()
                refined_el = el.copy()

                for i in range(n):
                    if static_mask[i]:
                        ref_az, ref_el = _refine_one(
                            cx[i], az[i], el[i], args.search_deg)
                        refined_az[i] = ref_az
                        refined_el[i] = ref_el
                        music_refined_count += 1

                xyz_music = _spherical_to_cart(r, refined_az, refined_el)
                music_pts = np.column_stack([xyz_music, pts[:, 3]])
                music_points.append(music_pts.astype(np.float32))
            else:
                # No complex data or no MUSIC module -- pass through FFT points
                music_points.append(pts)

            if frame_count % 10 == 0:
                mode = "MUSIC" if (has_cx and has_music) else "FFT"
                avg = total_det / frame_count
                print(f"\r  {elapsed:5.1f}s  frames={frame_count}  "
                      f"det={det_frames}  pts={total_det}  "
                      f"avg={avg:.1f}/frame  refined={music_refined_count}  "
                      f"mode={mode}",
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
    print(f"  MUSIC-refined detections: {music_refined_count}")

    if not fft_points:
        print("  NO POINTS DETECTED")
        return

    fft_cloud = np.concatenate(fft_points, axis=0)
    music_cloud = np.concatenate(music_points, axis=0)

    fft_static_mask = np.abs(fft_cloud[:, 3]) < 0.5
    music_static_mask = np.abs(music_cloud[:, 3]) < 0.5

    fft_static = fft_cloud[fft_static_mask]
    music_static = music_cloud[music_static_mask]

    print(f"\n  FFT cloud: {fft_cloud.shape}")
    print(f"  FFT static: {fft_static.shape}")
    print(f"  MUSIC cloud: {music_cloud.shape}")
    print(f"  MUSIC static: {music_static.shape}")

    if music_refined_count > 0:
        # Compare angular spread
        fft_r, fft_az, fft_el = _cart_to_spherical(fft_static)
        mus_r, mus_az, mus_el = _cart_to_spherical(music_static)
        print(f"\n  FFT azimuth std:   {np.rad2deg(np.std(fft_az)):.2f} deg")
        print(f"  MUSIC azimuth std: {np.rad2deg(np.std(mus_az)):.2f} deg")
        print(f"  FFT elev std:      {np.rad2deg(np.std(fft_el)):.2f} deg")
        print(f"  MUSIC elev std:    {np.rad2deg(np.std(mus_el)):.2f} deg")

    np.savez(args.out,
             fft_cloud=fft_cloud,
             music_cloud=music_cloud,
             fft_static=fft_static,
             music_static=music_static)
    print(f"\n  Saved to {args.out}")


if __name__ == "__main__":
    main()
