"""COMPUTE layer — pure vitals/geometry from a radar window. NO I/O, NO web.

Display and compute are DELIBERATELY separated (user requirement): everything
here is a pure function of a captured window, so the exact same call validates
the algorithm offline on a recorded cube (chairL + watch) and drives the live
web page. `analyze()` returns one JSON-able dict; the server only renders it.

Reuses the VALIDATED building blocks — do not re-derive their math here:
  - living_gate.living_window   -> presence (RR-band spatial concentration+cluster)
  - bcg_vitals.estimate_rr      -> RR + breathing f0
  - chairL chest-cluster HR     -> RR-anchored chest bin + interp autocorr (watch MAE 1.5)
  - range_music / music_collect -> MUSIC DOA -> room-frame XYZ (target position, fall)

Bin-adjustable: pass hr_bin_lo/hr_bin_hi to force the HR search into a range-bin
window (the web page's "adjust bin position" control) instead of auto-selecting.
"""
from __future__ import annotations
import os
import sys
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from bcg_vitals import demod_channels, bandpass, estimate_rr, autocorr_peak, RR_LO, RR_HI
from living_gate import living_window
from spatial3d.range_music import covariances_to_points, spherical_to_cart  # noqa: F401
from spatial3d.music import awrl6844_array
from spatial3d.music_collect import to_room, TILT_DEG, H_MOUNT

HRLO, HRHI = 1.0, 1.7            # validated resting cardiac band
AZ_RANGE = (-45.0, 45.0)
EL_RANGE = (-45.0, 20.0)
FALL_Z_MAX = 0.6                 # m; breathing centroid below this (room frame) = on the floor
STAND_Z_MIN = 0.9               # m; above this = upright torso


# ---------------------------------------------------------------- HR (chairL) --
def _cardiac_snr(disp, fps, f0):
    """Tallest NON-harmonic peak in [HRLO,HRHI] over the local floor -> (snr, bpm).
    The chairL selector: a clean heartbeat bin scores high; a breathing-harmonic
    pile-up bin scores low (its energy is AT the notched harmonics)."""
    n = len(disp)
    f = np.fft.rfftfreq(n, 1 / fps)
    X = 2 * np.abs(np.fft.rfft(disp - disp.mean())) / n
    band = (f >= HRLO) & (f <= HRHI)
    nharm = np.zeros_like(f, bool)
    for k in range(1, 12):
        nharm |= np.abs(f - k * f0) <= 0.035
    cand = band & ~nharm
    if not cand.any():
        return 0.0, None
    floor = np.median(X[cand])
    j = np.where(cand)[0][np.argmax(X[cand])]
    return float(X[j] / (floor + 1e-9)), float(f[j] * 60)


def _hr_chest_cluster(disp, bins, fps, f0, hr_bin_lo=None, hr_bin_hi=None):
    """RR-anchored chest-cluster HR (chairL, watch MAE 1.5).

    abdomen = max breathing amplitude bin. Body = bins with real breathing.
    chest = body bin maximizing cardiac-SNR (naturally opposite the abdomen, low
    breathing). HR = interp-autocorr[HRLO,HRHI] over the chest +/-2 CONTIGUOUS
    cluster, median-fused (trimmed of the single-bin outliers). If hr_bin_lo/hi
    given, restrict the chest search to that range-bin window (web bin control)."""
    rr_amp = np.array([bandpass(d, fps, RR_LO, 0.6).std() for d in disp])
    ab = int(np.argmax(rr_amp))
    body = rr_amp > 0.12 * rr_amp.max()
    csnr = np.zeros(len(bins))
    for i in range(len(bins)):
        if not body[i]:
            continue
        if hr_bin_lo is not None and not (hr_bin_lo <= int(bins[i]) <= hr_bin_hi):
            continue
        csnr[i], _ = _cardiac_snr(disp[i], fps, f0)
    if csnr.max() <= 0:
        return None, None, dict(abdomen_bin=int(bins[ab]), chest_bin=None, csnr=0.0)
    ch = int(np.argmax(csnr))
    # contiguous +/-2 cluster around the chest bin, still within the body span
    cl = [i for i in range(max(0, ch - 2), min(len(bins), ch + 3)) if body[i]]
    hrs, hts = [], []
    for i in cl:
        bpm, h = autocorr_peak(bandpass(disp[i], fps, HRLO, HRHI), fps,
                               int(HRLO * 60), int(HRHI * 60), interp=True)
        if bpm:
            hrs.append(bpm); hts.append(h)
    if not hrs:
        return None, None, dict(abdomen_bin=int(bins[ab]), chest_bin=int(bins[ch]), csnr=float(csnr[ch]))
    hr = float(np.median(hrs))
    strength = float(np.median(hts))
    diag = dict(abdomen_bin=int(bins[ab]), chest_bin=int(bins[ch]),
                cluster_bins=[int(bins[i]) for i in cl], csnr=round(float(csnr[ch]), 1))
    return round(hr, 1), round(strength, 2), diag


# ---------------------------------------------------------------- geometry --
def _window_covariances(cube_win, bins, want_bins):
    """Per-bin (16,16) covariance from the window snapshots, only for want_bins."""
    covs = {}
    idx = {int(b): i for i, b in enumerate(bins)}
    for b in want_bins:
        i = idx.get(int(b))
        if i is None:
            continue
        X = cube_win[i]                       # (T,16)
        if X.shape[0] < 16:
            continue
        covs[int(b)] = (X.conj().T @ X) / X.shape[0]
    return covs


def _target_xyz(cube_win, bins, dr, center_bin, tilt_deg=None, h_mount=None):
    """MUSIC DOA of the breathing target (strongest peak on the bins around the
    occupied range). Returns range_m + cross-range x (both mount-INDEPENDENT), and
    height z ONLY when a mount calibration (tilt_deg,h_mount) is supplied — z and
    therefore fall are meaningless without the real rig geometry."""
    want = [b for b in bins if abs(int(b) - int(center_bin)) <= 3]
    covs = _window_covariances(cube_win, bins, want)
    if not covs:
        return None
    try:
        pts = covariances_to_points(covs, awrl6844_array(), dr=dr,
                                    az_range=AZ_RANGE, el_range=EL_RANGE,
                                    resolution_deg=2.0, max_peaks_per_bin=1)
    except Exception:
        return None
    if pts.shape[0] == 0:
        return None
    p = pts[np.argmax(pts[:, 3])]             # [x,y,z_radar,power,bin,range]
    rng = float(p[5])
    x_cross = round(float(p[0]), 2)           # radar-frame cross-range: mount-independent
    out = dict(range_m=round(rng, 2), x=x_cross, z=None, calibrated=False)
    if tilt_deg is not None and h_mount is not None:
        room = to_room(p[None, :3], tilt_deg, h_mount)[0]
        out.update(x=round(float(room[0]), 2), z=round(float(room[2]), 2), calibrated=True)
    return out


def measurable_range(bins, dr):
    """The sensor's coverage volume for the display: D (range) span from the bin
    extent, X (cross-range) half-width and Z (height) span from the MUSIC az/el
    scan limits and the mount (tilt/height)."""
    d_lo, d_hi = float(min(bins)) * dr, float(max(bins)) * dr
    corners = []
    for r in (d_lo, d_hi):
        for az in AZ_RANGE:
            for el in EL_RANGE:
                xyz = spherical_to_cart(r, np.deg2rad(az), np.deg2rad(el))[0]
                corners.append(to_room(xyz[None, :], TILT_DEG, H_MOUNT)[0])
    corners = np.array(corners)
    return dict(d_min=round(d_lo, 2), d_max=round(d_hi, 2),
                x_min=round(float(corners[:, 0].min()), 2),
                x_max=round(float(corners[:, 0].max()), 2),
                z_min=round(float(corners[:, 2].min()), 2),
                z_max=round(float(corners[:, 2].max()), 2),
                az_deg=list(AZ_RANGE), el_deg=list(EL_RANGE),
                bin_lo=int(min(bins)), bin_hi=int(max(bins)))


# ---------------------------------------------------------------- main entry --
def analyze(cube_win, bins, dr, fps, hr_bin_lo=None, hr_bin_hi=None,
            tilt_deg=None, h_mount=None):
    """One window -> full state dict (JSON-able). cube_win=(nbin,T,16) complex,
    bins=(nbin,) range-bin indices. Pure: same call for live and replay."""
    bins = np.asarray(bins).astype(int)
    disp = demod_channels(cube_win, bins)                 # (nbin,T) mm
    T = disp.shape[1]
    out = dict(fps=round(fps, 2), win_s=round(T / fps, 1), n_bins=len(bins))

    live = living_window(disp, bins, dr, fps)
    out["present"] = bool(live["present"])
    out["occ_conc"] = live["conc"]; out["occ_span"] = live["span"]
    out["range_m"] = live["range_m"]

    if not out["present"]:
        out.update(hr=None, rr=None, hr_strength=None, fall=False, posture="empty",
                   target=None, hr_diag=None)
        return out

    rr, f0, _, _ = estimate_rr(disp, fps)
    out["rr"] = round(rr) if rr else None

    hr, strength, diag = _hr_chest_cluster(disp, bins, fps, f0, hr_bin_lo, hr_bin_hi)
    out["hr"] = hr; out["hr_strength"] = strength; out["hr_diag"] = diag

    center = diag.get("chest_bin") or int(round(live["range_m"] / dr))
    tgt = _target_xyz(cube_win, bins, dr, center, tilt_deg, h_mount)
    out["target"] = tgt

    # posture / fall from the breathing-target height (room frame). ONLY when the
    # mount is calibrated — z is otherwise meaningless (would false-alarm FALL).
    # Heuristic v1: upright torso centroid >=0.9m; a person on the floor <0.6m.
    if tgt is not None and tgt.get("calibrated"):
        z = tgt["z"]
        if z < FALL_Z_MAX:
            out["posture"] = "on_floor"; out["fall"] = True
        elif z >= STAND_Z_MIN:
            out["posture"] = "upright"; out["fall"] = False
        else:
            out["posture"] = "low"; out["fall"] = False
    else:
        out["posture"] = "uncalibrated"; out["fall"] = False
    return out
