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
# --- HR confidence gate (stop reporting a harmonic-locked value as if it were HR) ---
STRENGTH_MIN = 0.30              # min autocorr periodicity strength to trust the value
HARM_MARGIN_BPM = 5.0           # HR within this of any k*RR => likely a breathing harmonic
# --- pose from the RR (breathing) energy centroid height (room frame) ---
FALL_Z = 0.6                     # z below this = on the floor
SIT_STAND_Z = 1.05               # z in [FALL_Z, this) = sitting; >= this = standing
POSE_TILT_DEFAULT = 0.0          # radar horizontal, ~1 m surface: rough default so pose
POSE_MOUNT_DEFAULT = 1.0         # shows live; pass --tilt/--mount for a calibrated height


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


CHEST_SEARCH_BINS = 20          # search the chest within +-this of the abdomen anchor
CHEST_ALPHA = 0.3               # C/B ratio softener (avoid /0 at zero-breathing bins)


def _hr_chest_cluster(disp, bins, fps, f0, hr_bin_lo=None, hr_bin_hi=None):
    """RR-anchored, GRADIENT chest selection (user's body-model: from the breathing
    anchor, walk down the fluctuation gradient -> chest = adjacent bin where
    breathing has DROPPED but cardiac is present). Validated on case cubes: this
    picks the low-breathing/cardiac bin (chairL 172/182, lie41 168), i.e. the
    C/B-ratio peak, NOT the abdomen (whose high cardiac-SNR is harmonic leakage).

    chest = argmax over the anchor neighbourhood of  cardiac_snr / (B/B_anchor + a)
    -> favours cardiac-strong AND breathing-weak. HR = interp-autocorr[HRLO,HRHI]
    over the chest +/-2 contiguous cluster, median-fused. hr_bin_lo/hi restricts
    the search to a range-bin window (web bin control)."""
    rr_amp = np.array([bandpass(d, fps, RR_LO, 0.6).std() for d in disp])
    ab = int(np.argmax(rr_amp))                       # STEP 1: breathing anchor = abdomen
    b_anchor = rr_amp[ab] + 1e-9
    # STEP 2: gradient chest score in the anchor neighbourhood
    score = np.full(len(bins), -1.0)
    for i in range(len(bins)):
        if abs(int(bins[i]) - int(bins[ab])) > CHEST_SEARCH_BINS:
            continue
        if hr_bin_lo is not None and not (hr_bin_lo <= int(bins[i]) <= hr_bin_hi):
            continue
        c, _ = _cardiac_snr(disp[i], fps, f0)
        score[i] = c / (rr_amp[i] / b_anchor + CHEST_ALPHA)   # C/B ratio (gradient)
    if score.max() <= 0:
        return None, None, dict(abdomen_bin=int(bins[ab]), chest_bin=None, csnr=0.0)
    ch = int(np.argmax(score))
    csnr_ch, _ = _cardiac_snr(disp[ch], fps, f0)
    # contiguous +/-2 cluster around the chest bin
    cl = list(range(max(0, ch - 2), min(len(bins), ch + 3)))
    hrs, hts = [], []
    for i in cl:
        bpm, h = autocorr_peak(bandpass(disp[i], fps, HRLO, HRHI), fps,
                               int(HRLO * 60), int(HRHI * 60), interp=True)
        if bpm:
            hrs.append(bpm); hts.append(h)
    if not hrs:
        return None, None, dict(abdomen_bin=int(bins[ab]), chest_bin=int(bins[ch]), csnr=round(float(csnr_ch), 1))
    hr = float(np.median(hrs))
    strength = float(np.median(hts))
    diag = dict(abdomen_bin=int(bins[ab]), chest_bin=int(bins[ch]),
                cluster_bins=[int(bins[i]) for i in cl], csnr=round(float(csnr_ch), 1),
                chest_score=round(float(score[ch]), 1))
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


def _hr_confidence(hr, strength, rr):
    """Three-level trust so a harmonic-lock isn't shown as a real reading, WITHOUT
    hiding a value that is often right. Returns (level, reason):
      'weak'      periodicity too weak -> don't trust the number at all
      'harmonic'  strong but sits on a breathing harmonic k*RR -> COULD be the
                  residue the estimator locks onto; show it but mark 存疑
      'ok'        strong AND clear of every harmonic -> trust
    (At normal RR the true resting HR often coincides with ~6*RR — the wall — so
    'harmonic' is common at rest and does NOT mean the value is wrong, only unproven.)"""
    if hr is None:
        return "none", "无"
    if strength is None or strength < STRENGTH_MIN:
        return "weak", "弱周期"
    if rr and rr > 0:
        k = round(hr / rr)
        if k >= 1 and abs(hr - k * rr) <= HARM_MARGIN_BPM:
            return "harmonic", f"≈{k}×RR"
    return "ok", "ok"


def _rr_centroid(cube_win, disp, bins, dr, fps, tilt_deg, h_mount):
    """Breathing-energy-WEIGHTED spatial centroid (per user: pose from the RR
    centroid, not a single MUSIC peak -> far more stable). Each breathing bin's
    DOA is weighted by its RR-band displacement amplitude; the weighted mean is
    the torso centroid. Returns dict(range_m, x, z, calibrated)."""
    rr_amp = np.array([np.std(bandpass(d, fps, RR_LO, RR_HI)) for d in disp])
    if rr_amp.max() <= 0:
        return None
    body = np.where(rr_amp > 0.25 * rr_amp.max())[0]        # the breathing bins
    if len(body) == 0:
        return None
    covs = _window_covariances(cube_win, bins, [int(bins[i]) for i in body])
    if not covs:
        return None
    tilt = POSE_TILT_DEFAULT if tilt_deg is None else tilt_deg
    mnt = POSE_MOUNT_DEFAULT if h_mount is None else h_mount
    try:
        pts = covariances_to_points(covs, awrl6844_array(), dr=dr,
                                    az_range=AZ_RANGE, el_range=EL_RANGE,
                                    resolution_deg=2.0, max_peaks_per_bin=1)
    except Exception:
        return None
    if pts.shape[0] == 0:
        return None
    amp_by_bin = {int(bins[i]): float(rr_amp[i]) for i in body}
    room = to_room(pts[:, :3], tilt, mnt)                   # (M,3) room x,y,z
    w = np.array([amp_by_bin.get(int(b), 0.0) for b in pts[:, 4]])  # RR-energy weights
    if w.sum() <= 0:
        return None
    c = (room * w[:, None]).sum(0) / w.sum()                # weighted centroid
    rng = float((pts[:, 5] * w).sum() / w.sum())
    return dict(range_m=round(rng, 2), x=round(float(c[0]), 2),
                z=round(float(c[2]), 2), calibrated=(tilt_deg is not None and h_mount is not None))


def _pose(z):
    """Fall / Sit / Stand from the breathing-centroid height (room frame)."""
    if z is None:
        return "unknown"
    if z < FALL_Z:
        return "fall"
    if z < SIT_STAND_Z:
        return "sit"
    return "stand"


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
        out.update(hr=None, rr=None, hr_strength=None, hr_confident=False,
                   hr_level="none", hr_reason="empty", fall=False, pose="empty",
                   pose_calibrated=False, target=None, hr_diag=None)
        return out

    rr, f0, _, _ = estimate_rr(disp, fps)
    out["rr"] = round(rr) if rr else None

    hr, strength, diag = _hr_chest_cluster(disp, bins, fps, f0, hr_bin_lo, hr_bin_hi)
    level, reason = _hr_confidence(hr, strength, rr)
    out["hr"] = hr; out["hr_strength"] = strength; out["hr_diag"] = diag
    out["hr_level"] = level; out["hr_reason"] = reason; out["hr_confident"] = (level == "ok")

    # pose from the RR (breathing) energy centroid — stable vs single-peak MUSIC.
    tgt = _rr_centroid(cube_win, disp, bins, dr, fps, tilt_deg, h_mount)
    out["target"] = tgt
    z = tgt["z"] if tgt else None
    out["pose"] = _pose(z)
    out["pose_calibrated"] = bool(tgt and tgt.get("calibrated"))
    out["fall"] = (out["pose"] == "fall")
    return out
