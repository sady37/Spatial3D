"""Range-profile-based patient monitor.

Works directly with the range profile (TLV type 2): 128 uint16 log-magnitude
bins covering 0-9m at ~7cm resolution.

Presence detection via temporal variance:
    Static objects (walls, furniture) -> range profile constant -> low variance
    Person (breathing, micro-sway)   -> range profile fluctuates -> HIGH variance

Fall detection via posture signature:
    Standing: wide variance band (~8 bins, 0.56m), total variance ~70-90k
    Lying:    narrow variance band (~2-3 bins, 0.14m), total variance ~20-25k
    Fall = sustained drop in total variance + band narrowing

Two detection tiers:
    Strong presence: absolute variance > 1000 (standing person micro-motion)
    Weak presence:   variance > 5x calibrated empty-room variance (lying body)

Geometry: sensor at height H, tilt theta from vertical.
    A person at slant range R is at height H - R*cos(theta).
"""

from __future__ import annotations

import math
import time
from collections import deque
from dataclasses import dataclass
from enum import Enum
from pathlib import Path

import numpy as np

N_BINS = 128
MAX_RANGE_M = 9.0
BIN_WIDTH_M = MAX_RANGE_M / N_BINS  # ~0.0703 m
MIN_RANGE_BIN = 4   # skip near-field DC artifacts (<0.28m)
MAX_RANGE_BIN = 75  # skip far-field noise (>5.3m)

TLV_RANGE_PROFILE = 2


class Posture(Enum):
    UNKNOWN = "unknown"
    ABSENT = "absent"
    STANDING = "standing"
    LYING = "lying"


def parse_range_profile(tlv_payload: bytes) -> np.ndarray | None:
    if len(tlv_payload) < N_BINS * 2:
        return None
    return np.frombuffer(tlv_payload[: N_BINS * 2], dtype="<u2").astype(np.float64)


def bin_to_range(b: int | float) -> float:
    return b * BIN_WIDTH_M


def range_to_height(slant_range: float, mount_height: float, tilt_deg: float) -> float:
    theta = math.radians(tilt_deg)
    return mount_height - slant_range * math.cos(theta)


@dataclass
class PresenceBand:
    bin_start: int
    bin_end: int
    range_start_m: float
    range_end_m: float
    peak_bin: int
    peak_variance: float
    mean_height_m: float
    width_bins: int = 0
    tier: str = "strong"  # "strong" (abs threshold) or "weak" (ratio threshold)

    def __post_init__(self):
        self.width_bins = self.bin_end - self.bin_start


@dataclass
class RangeStatus:
    frame_count: int
    elapsed_s: float
    baseline_ready: bool
    calibrated: bool
    presence_bands: list[PresenceBand]
    body_range_m: tuple[float, float] | None  # (min, max) range of all detected presence
    total_variance: float
    max_variance_bin: int
    max_variance: float
    posture: Posture = Posture.UNKNOWN
    posture_confidence: float = 0.0
    variance_trend: float = 1.0


# Calibrated from real data (AWRL6844, 2.0m mount, 35deg tilt)
STANDING_TOTVAR_MIN = 40000.0
LYING_TOTVAR_MAX = 35000.0
STANDING_BANDWIDTH_MIN = 5  # bins
LYING_BANDWIDTH_MAX = 4     # bins
PRESENCE_TOTVAR_MIN = 3000.0  # lowered: lying person has ~5-6k

STRONG_VAR_THRESHOLD = 1000.0  # absolute variance for confident detection
WEAK_VAR_RATIO = 5.0           # current_var / calibration_var for weak detection
WEAK_VAR_FLOOR = 100.0         # minimum absolute variance even in ratio mode


class RangeProfileMonitor:

    def __init__(self, *,
                 mount_height: float = 2.0,
                 mount_tilt_deg: float = 35.0,
                 baseline_frames: int = 100,
                 var_window: int = 50):
        self.mount_height = mount_height
        self.mount_tilt_deg = mount_tilt_deg
        self.baseline_frames = baseline_frames
        self.var_window = var_window

        self._window: deque[np.ndarray] = deque(maxlen=var_window)
        self._baseline = np.zeros(N_BINS, dtype=np.float64)
        self._baseline_alpha = 0.002

        # Empty-room calibration (variance per bin)
        self._cal_var: np.ndarray | None = None

        self._totvar_history: deque[float] = deque(maxlen=60)

        self.frame_count = 0
        self.t_start = time.time()

    def load_calibration(self, path: str) -> None:
        """Load empty-room calibration (.npz with 'var' array)."""
        data = np.load(path)
        self._cal_var = data["var"].astype(np.float64)
        # Enforce minimum to avoid division by zero
        self._cal_var = np.maximum(self._cal_var, 1.0)

    def save_calibration(self, path: str) -> None:
        """Save current variance profile as calibration."""
        var = self._compute_variance()
        np.savez(path, var=var, mean=self._baseline)

    @property
    def calibrated(self) -> bool:
        return self._cal_var is not None

    def update(self, frame) -> None:
        profile = None
        for tlv in frame.tlvs:
            if tlv.type == TLV_RANGE_PROFILE:
                profile = parse_range_profile(tlv.payload)
                break
        if profile is None:
            return

        self._window.append(profile)

        if self.frame_count == 0:
            self._baseline = profile.copy()
        else:
            self._baseline *= (1.0 - self._baseline_alpha)
            self._baseline += self._baseline_alpha * profile

        self.frame_count += 1

    def _compute_variance(self) -> np.ndarray:
        if len(self._window) < 5:
            return np.zeros(N_BINS)
        stack = np.array(self._window)
        var = stack.var(axis=0)
        var[:MIN_RANGE_BIN] = 0.0
        var[MAX_RANGE_BIN:] = 0.0
        return var

    def _find_bands(self, var: np.ndarray) -> list[PresenceBand]:
        """Find presence bands using dual thresholds."""
        # Build a per-bin detection mask with tier info
        detected = np.zeros(MAX_RANGE_BIN, dtype=int)  # 0=no, 1=weak, 2=strong
        for i in range(MIN_RANGE_BIN, MAX_RANGE_BIN):
            if var[i] >= STRONG_VAR_THRESHOLD:
                detected[i] = 2
            elif (self._cal_var is not None
                  and var[i] >= WEAK_VAR_FLOOR
                  and var[i] >= self._cal_var[i] * WEAK_VAR_RATIO):
                detected[i] = 1

        bands = []
        in_band = False
        start = 0
        for i in range(MIN_RANGE_BIN, MAX_RANGE_BIN):
            if detected[i] > 0 and not in_band:
                start = i
                in_band = True
            elif (detected[i] == 0 or i == MAX_RANGE_BIN - 1) and in_band:
                end = i + 1 if detected[i] > 0 else i
                peak = start + int(var[start:end].argmax())
                center_range = bin_to_range((start + end) / 2)
                has_strong = any(detected[j] == 2 for j in range(start, end))
                bands.append(PresenceBand(
                    bin_start=start,
                    bin_end=end,
                    range_start_m=bin_to_range(start),
                    range_end_m=bin_to_range(end),
                    peak_bin=peak,
                    peak_variance=float(var[peak]),
                    mean_height_m=range_to_height(center_range,
                                                   self.mount_height,
                                                   self.mount_tilt_deg),
                    tier="strong" if has_strong else "weak",
                ))
                in_band = False
        return bands

    def _classify_posture(self, total_var: float, bands: list[PresenceBand]) -> tuple[Posture, float]:
        if total_var < PRESENCE_TOTVAR_MIN and not bands:
            return Posture.ABSENT, 0.8

        total_width = sum(b.width_bins for b in bands) if bands else 0

        standing_score = 0.0
        lying_score = 0.0

        if total_var >= STANDING_TOTVAR_MIN:
            standing_score += 0.5
        elif total_var <= LYING_TOTVAR_MAX:
            lying_score += 0.5

        if total_width >= STANDING_BANDWIDTH_MIN:
            standing_score += 0.5
        elif 0 < total_width <= LYING_BANDWIDTH_MAX:
            lying_score += 0.5

        # With calibration, even weak-detection bands count for body extent
        if self._cal_var is not None and bands:
            weak_width = sum(b.width_bins for b in bands if b.tier == "weak")
            if weak_width > 0 and total_var < LYING_TOTVAR_MAX:
                lying_score += 0.3

        if standing_score > lying_score:
            return Posture.STANDING, min(standing_score, 1.0)
        elif lying_score > standing_score:
            return Posture.LYING, min(lying_score, 1.0)
        return Posture.UNKNOWN, 0.0

    def get_status(self) -> RangeStatus:
        elapsed = time.time() - self.t_start
        ready = self.frame_count >= self.baseline_frames

        if len(self._window) < 5:
            return RangeStatus(
                frame_count=self.frame_count, elapsed_s=elapsed,
                baseline_ready=ready, calibrated=self.calibrated,
                presence_bands=[], body_range_m=None,
                total_variance=0.0, max_variance_bin=0, max_variance=0.0,
            )

        var = self._compute_variance()
        total_var = float(var[MIN_RANGE_BIN:MAX_RANGE_BIN].sum())
        max_bin = int(var[:MAX_RANGE_BIN].argmax())
        max_var = float(var[max_bin])
        bands = self._find_bands(var)

        self._totvar_history.append(total_var)
        avg_totvar = np.mean(self._totvar_history) if self._totvar_history else total_var
        trend = total_var / avg_totvar if avg_totvar > 0 else 1.0

        posture, confidence = self._classify_posture(total_var, bands)

        body_range = None
        if bands:
            body_range = (bands[0].range_start_m, bands[-1].range_end_m)

        return RangeStatus(
            frame_count=self.frame_count, elapsed_s=elapsed,
            baseline_ready=ready, calibrated=self.calibrated,
            presence_bands=bands, body_range_m=body_range,
            total_variance=total_var, max_variance_bin=max_bin,
            max_variance=max_var, posture=posture,
            posture_confidence=confidence, variance_trend=trend,
        )

    def get_diff_profile(self) -> np.ndarray:
        if len(self._window) < 2:
            return np.zeros(N_BINS)
        current = np.mean(self._window, axis=0)
        return current - self._baseline

    def get_variance_profile(self) -> np.ndarray:
        if len(self._window) < 2:
            return np.zeros(N_BINS)
        var = np.array(self._window).var(axis=0)
        var[:MIN_RANGE_BIN] = 0.0
        var[MAX_RANGE_BIN:] = 0.0
        return var
