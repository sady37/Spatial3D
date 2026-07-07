"""Range-bin domain glue for server-side per-bin MUSIC.

The Spatial3D firmware emits a zero-Doppler range-antenna TLV (type 8): for a
contiguous window of range bins, the coherent-mean 16-virtual-antenna vector.
Because the firmware only fits one ~82-bin *layer* per frame over the UART
budget, the server rolls through the layers (re-sending ``rangeAntennaOutput``)
and accumulates, per range bin, K snapshots across frames. Each bin's K
snapshots build a spatial covariance matrix; per-bin MUSIC then super-resolves
the arrival angle(s), and (range_bin, az, el) becomes a 3D point.

Geometry (from profile_4T4R_music.cfg):
    Fs = 20 Msps, numAdcSamples = 1024 -> 512 range bins (real sampling)
    ADC bandwidth = 125 MHz/us * 51.2 us = 6.4 GHz
    dR = c / (2 B) = 2.34375 cm/bin ;  R_max = 512 * dR = 12.0 m
    Room slant range 2.03-7.73 m -> bins 87-330, scanned as 3 layers.

`range` here is *slant* range from the sensor; the room transform (tilt +
mount height) is applied downstream in voxel_map.to_room().
"""

from __future__ import annotations

import math
import re
from dataclasses import dataclass, field

import numpy as np
from numpy.typing import NDArray

from .music import AntennaArray, estimate_covariance, music_doa
from .tlv import RangeAntenna

# --- range geometry (profile_4T4R_music.cfg) --------------------------------
DR_M = 0.0234375          # metres per range bin (2.34375 cm)
N_VIRT_ANT = 16

# Default 3-layer scan plan: (start_bin, num_bins). One layer fits the UART
# budget per frame; the server rolls through all three (AWRL6844.md 5.5).
LAYERS: list[tuple[int, int]] = [(87, 82), (169, 82), (251, 80)]


def bin_range_m(bin_idx: int | NDArray, dr: float = DR_M) -> float | NDArray:
    """Slant range (metres) for a range-bin index."""
    return np.asarray(bin_idx) * dr


# --- parametric scan plan from mount geometry -------------------------------
# UART: DATA link 1.25 Mbps; ~60% usable at the framing overhead observed.
UART_KB_S = 73.7
FRAME_OVERHEAD_KB = 1.0     # frame header + point-cloud TLV per frame


def room_scan_plan(
    mount_height_m: float = 2.0,
    tilt_deg: float = 35.0,
    el_down_deg: float = 45.0,
    room_far_horiz_m: float = 6.0,
    dr: float = DR_M,
    fps: float = 10.0,
    uart_kb_s: float = UART_KB_S,
    overhead_kb: float = FRAME_OVERHEAD_KB,
    n_ant: int = N_VIRT_ANT,
) -> dict:
    """Compute the range-antenna window(s) for a room from mount geometry.

    The near limit is the slant range to the closest floor point the radar can
    see: along the steepest downward ray (tilt + downward-elevation-FOV below
    horizontal), the floor is ``H/sin(theta)`` away. The far limit is the
    diagonal to the room's farthest floor point, ``hypot(D_far, H)``. Range
    resolution is unchanged by fps, but a lower fps doubles the per-frame UART
    budget, so more bins fit in one window.

    Returns a dict with start/end bins, whether it fits one window at *fps*,
    how many layers otherwise, the ready-to-paste ``rangeAntennaOutput`` line,
    and the ``H_MOUNT`` to use in voxel_map.to_room().
    """
    theta = min(np.deg2rad(tilt_deg + el_down_deg), np.deg2rad(89.9))
    r_min = mount_height_m / np.sin(theta)
    r_max = float(np.hypot(room_far_horiz_m, mount_height_m))
    start_bin = int(round(r_min / dr))
    end_bin = int(round(r_max / dr))
    total_bins = end_bin - start_bin + 1

    bytes_per_bin = n_ant * 4
    frame_budget_kb = uart_kb_s / fps - overhead_kb
    max_bins = max(0, int(frame_budget_kb * 1024 / bytes_per_bin))
    fits = total_bins <= max_bins
    n_layers = math.ceil(total_bins / max_bins) if max_bins else 0
    win_bins = total_bins if fits else max_bins

    return {
        "mount_height_m": mount_height_m,
        "tilt_deg": tilt_deg,
        "near_horiz_m": float(mount_height_m / np.tan(theta)),
        "r_min_m": float(r_min),
        "r_max_m": r_max,
        "start_bin": start_bin,
        "end_bin": end_bin,
        "total_bins": total_bins,
        "dr_cm": dr * 100,
        "fps": fps,
        "max_bins_per_frame": max_bins,
        "fits_one_window": fits,
        "n_layers": n_layers,
        "rangeAntennaOutput": f"rangeAntennaOutput {start_bin} {win_bins} 1",
        "H_MOUNT": mount_height_m,
    }


def format_scan_plan(p: dict) -> str:
    """Human-readable summary of a room_scan_plan() result."""
    lines = [
        f"Mount H={p['mount_height_m']}m tilt={p['tilt_deg']}deg  "
        f"dR={p['dr_cm']:.2f}cm/bin",
        f"  near floor: {p['near_horiz_m']:.2f}m horiz -> slant "
        f"{p['r_min_m']:.2f}m = bin {p['start_bin']}",
        f"  far  floor: slant {p['r_max_m']:.2f}m = bin {p['end_bin']}",
        f"  room needs {p['total_bins']} bins ({p['start_bin']}-{p['end_bin']})",
        f"  @ {p['fps']:.0f} fps: budget {p['max_bins_per_frame']} bins/frame",
    ]
    if p["fits_one_window"]:
        lines.append(f"  -> FITS ONE WINDOW:  {p['rangeAntennaOutput']}")
    else:
        lines.append(f"  -> needs {p['n_layers']} layers "
                     f"(too many bins for one frame at this fps)")
    lines.append(f"  set voxel_map H_MOUNT = {p['H_MOUNT']}")
    return "\n".join(lines)


# --- read the scan plan straight from the .cfg ------------------------------
_RA_RE = re.compile(r"rangeAntennaOutput\s+(\d+)\s+(\d+)\s+([01])")
# Optional Spatial3D directive, e.g. "% spatial3d: snapshots=50 min_snapshots=10"
_DIRECTIVE_RE = re.compile(r"spatial3d\s*[:=]?\s*(.*)", re.IGNORECASE)
_KV_RE = re.compile(r"(\w+)\s*=\s*([0-9]*\.?[0-9]+)")


def parse_layers_from_cfg(cfg_path: str) -> list[tuple[int, int]]:
    """Read the range-antenna layer plan from a .cfg.

    Every ``rangeAntennaOutput <start> <num> <enable>`` occurrence is picked up
    — both the *active* command and the ``% Layer 2/3:`` comment lines the cfg
    documents them in — so the whole rolling plan comes from one source of
    truth. Disable lines (num == 0) are ignored; duplicates are collapsed and
    the result is sorted by start bin. Falls back to :data:`LAYERS` if none.
    """
    layers: list[tuple[int, int]] = []
    seen: set[tuple[int, int]] = set()
    with open(cfg_path) as f:
        for raw in f:
            m = _RA_RE.search(raw)
            if not m:
                continue
            start, num, enable = int(m.group(1)), int(m.group(2)), int(m.group(3))
            if enable == 0 or num == 0 or (start, num) in seen:
                continue
            seen.add((start, num))
            layers.append((start, num))
    layers.sort()
    return layers or list(LAYERS)


def parse_music_params_from_cfg(cfg_path: str) -> dict[str, float | int]:
    """Read MUSIC/acquisition params from ``% spatial3d: k=v ...`` directives.

    K (snapshots) and dwell settings are not native TI cfg fields, so they live
    in a Spatial3D comment directive that the radar ignores. Example::

        % spatial3d: snapshots=50 min_snapshots=10 rounds=4

    Returns a {key: number} dict (ints where the value has no decimal point).
    """
    params: dict[str, float | int] = {}
    with open(cfg_path) as f:
        for raw in f:
            line = raw.strip()
            if not line.startswith("%"):
                continue
            m = _DIRECTIVE_RE.search(line)
            if not m:
                continue
            for key, val in _KV_RE.findall(m.group(1)):
                params[key.lower()] = float(val) if "." in val else int(val)
    return params


def spherical_to_cart(r: float | NDArray, az_rad: float | NDArray,
                      el_rad: float | NDArray) -> NDArray[np.floating]:
    """(range, azimuth, elevation) -> radar-frame Cartesian (x, y, z).

    Boresight is +y (forward). Convention matches music.AntennaArray's
    steering vector: x = r cos(el) sin(az), y = r cos(el) cos(az), z = r sin(el).
    Accepts scalars or equal-length arrays; returns (N, 3).
    """
    r = np.atleast_1d(np.asarray(r, dtype=np.float64))
    az = np.atleast_1d(np.asarray(az_rad, dtype=np.float64))
    el = np.atleast_1d(np.asarray(el_rad, dtype=np.float64))
    cos_el = np.cos(el)
    x = r * cos_el * np.sin(az)
    y = r * cos_el * np.cos(az)
    z = r * np.sin(el)
    return np.column_stack([x, y, z])


# --- per-bin snapshot accumulator -------------------------------------------
@dataclass
class BinAccumulator:
    """Accumulate up to K zero-Doppler antenna snapshots per range bin.

    Frames arrive as range-antenna blocks (one layer each). Feed each block
    with :meth:`add`; snapshots for a bin stop being collected once it reaches
    ``k``. For a *static* target the antenna response is identical frame to
    frame and only the noise is independent, so K frames give K decorrelated
    noise realisations for a well-conditioned covariance (AWRL6844.md 5.2).
    """

    k: int = 50
    n_ant: int = N_VIRT_ANT
    snaps: dict[int, list[NDArray[np.complexfloating]]] = field(default_factory=dict)

    def add(self, ra: RangeAntenna) -> int:
        """Add one range-antenna block. Returns how many snapshots it stored."""
        if ra is None or ra.num_bins == 0:
            return 0
        stored = 0
        for i in range(ra.num_bins):
            b = ra.start_bin + i
            lst = self.snaps.setdefault(b, [])
            if len(lst) < self.k:
                lst.append(np.asarray(ra.data[i], dtype=np.complex64))
                stored += 1
        return stored

    def counts(self) -> dict[int, int]:
        return {b: len(v) for b, v in self.snaps.items()}

    def min_count(self, bins: list[int] | range | None = None) -> int:
        """Fewest snapshots held by any bin in *bins* (0 if a bin is unseen)."""
        if bins is None:
            bins = list(self.snaps)
        if not bins:
            return 0
        return min(len(self.snaps.get(b, [])) for b in bins)

    def is_layer_full(self, start_bin: int, num_bins: int) -> bool:
        return self.min_count(range(start_bin, start_bin + num_bins)) >= self.k

    def covariances(self, min_snapshots: int = 10,
                    spatial_smoothing: int = 0) -> dict[int, NDArray]:
        """Build a covariance matrix per bin with >= *min_snapshots* snapshots.

        With ``spatial_smoothing`` > 0, forward-backward smoothing is applied
        (see music.estimate_covariance); use it (or music.spatial_smoothing_2d
        upstream) to decorrelate coherent same-range scatterers.
        """
        out: dict[int, NDArray] = {}
        for b, lst in self.snaps.items():
            if len(lst) < min_snapshots:
                continue
            snaps = np.stack(lst, axis=0)  # (K, n_ant)
            out[b] = estimate_covariance(snaps, spatial_smoothing=spatial_smoothing)
        return out

    def stacks(self, min_snapshots: int = 10) -> dict[int, NDArray]:
        """Raw (K, n_ant) snapshot stacks per bin (for 2D spatial smoothing)."""
        return {b: np.stack(lst, axis=0)
                for b, lst in self.snaps.items() if len(lst) >= min_snapshots}


# --- per-bin DOA -> 3D points -----------------------------------------------
def covariances_to_points(
    covariances: dict[int, NDArray],
    array: AntennaArray,
    dr: float = DR_M,
    n_signals: int | None = None,
    n_snapshots: int = 50,
    az_range: tuple[float, float] = (-45.0, 45.0),
    el_range: tuple[float, float] = (-45.0, 20.0),
    resolution_deg: float = 1.0,
    max_peaks_per_bin: int = 3,
    min_rel_db: float = -6.0,
) -> NDArray[np.floating]:
    """Run per-bin 2D MUSIC and map peaks to radar-frame 3D points.

    Parameters
    ----------
    covariances    : {bin_idx: (N, N) covariance matrix}.
    array          : AntennaArray matching the covariance dimension.
    dr             : metres per range bin.
    n_signals      : fixed source count, or None for MDL auto per bin.
    az_range/el_range/resolution_deg : MUSIC scan grid (degrees).
    max_peaks_per_bin : keep at most this many peaks per bin.
    min_rel_db     : drop peaks weaker than this vs the bin's strongest peak.

    Returns
    -------
    (M, 6) float array: columns [x, y, z, power, bin_idx, range_m].
    """
    rows: list[list[float]] = []
    for b in sorted(covariances):
        R = covariances[b]
        dets = music_doa(
            R, array, n_signals=n_signals, n_snapshots=n_snapshots,
            az_range=az_range, el_range=el_range, resolution_deg=resolution_deg,
        )
        if not dets:
            continue
        peak_power = dets[0][2]
        r = b * dr
        for az_deg, el_deg, power in dets[:max_peaks_per_bin]:
            if 10.0 * np.log10(power / peak_power + 1e-30) < min_rel_db:
                break
            xyz = spherical_to_cart(r, np.deg2rad(az_deg), np.deg2rad(el_deg))[0]
            rows.append([xyz[0], xyz[1], xyz[2], power, float(b), r])
    if not rows:
        return np.empty((0, 6), dtype=np.float32)
    return np.asarray(rows, dtype=np.float32)
