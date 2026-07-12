"""Respiratory phase-domain harmonic cancellation.

User's insight: breathing harmonics ride on n*f0(t) and DRIFT with the breathing
period; the heart rate is asynchronous and does NOT. The static Hz-comb notch fails
because f0 wanders, so a fixed-frequency comb misaligns. Fix: resample the signal onto
a uniform BREATHING-PHASE grid. There every breathing harmonic sits at an EXACT integer
(cycles-per-breath) regardless of f0 drift, so one integer comb-notch removes ALL of
them cleanly. The cardiac component sits at f_h/f0 (e.g. 2.0/0.32 = 6.3 cyc/breath) --
non-integer -> survives. Transform back to time; the heart should emerge.

Also a DIAGNOSTIC of the raw claim: per window, does the dominant cardiac-band peak
track n*f0(t) (harmonic) or stay independent (heart)?

    python phase_deharm_probe.py
"""
import numpy as np
from scipy.signal import hilbert, find_peaks
from bcg_vitals import (bandpass, sqi, fft_peak, autocorr_peak, beat_count,
                        demod_channels, RR_LO, RR_HI)

CARD_LO, CARD_HI = 1.0, 2.5
NOTCH_HW = 0.16          # half-width around each integer harmonic (cyc/breath)
NHARM = 18               # notch harmonics k=1..18


def phase_deharmonic(sig, breath, fps, n_phase=8192):
    """Remove breathing harmonics via phase-domain integer notch. Returns time-domain
    de-harmonic signal (same length as sig)."""
    phi = np.unwrap(np.angle(hilbert(breath)))          # breathing phase (rad)
    phi = phi - phi[0]
    cyc = phi / (2 * np.pi)                              # breaths elapsed, monotonic
    if cyc[-1] < 3:
        return None
    grid = np.linspace(0, cyc[-1], n_phase)             # uniform in breathing phase
    sp = np.interp(grid, cyc, sig)                      # signal vs breathing phase
    # FFT over phase: bin k = k cycles-per-breath = harmonic number
    F = np.fft.rfft(sp - sp.mean())
    kf = np.fft.rfftfreq(n_phase, d=grid[1] - grid[0])  # cycles per breath
    for n in range(1, NHARM + 1):
        F[np.abs(kf - n) <= NOTCH_HW] = 0               # notch each integer harmonic
    sp_clean = np.fft.irfft(F, n=n_phase)
    return np.interp(cyc, grid, sp_clean)               # back to original time grid


def read_hr(sig, fps, tag):
    b = bandpass(sig, fps, CARD_LO, CARD_HI)
    ff = fft_peak(b, fps, CARD_LO, CARD_HI)
    ac, h = autocorr_peak(b, fps, int(CARD_LO * 60), int(CARD_HI * 60))
    bc = beat_count(b, fps, hi_bpm=int(CARD_HI * 60), height=0.4)
    print(f"    {tag:16s} FFT={ff and ff*60:6.1f}  ac={ac and ac:6.1f}(h={h:.2f})  beat={bc:6.1f}")


def track_independence(sig, breath, fps, win_s=10, step_s=2):
    """Does the dominant cardiac-band peak track n*f0(t)? Returns correlation of the
    per-window cardiac peak with the per-window breathing rate."""
    n = int(win_s * fps); step = int(step_s * fps)
    fc, f0 = [], []
    for s in range(0, len(sig) - n, step):
        seg = sig[s:s + n]; bseg = breath[s:s + n]
        ffc = fft_peak(bandpass(seg, fps, CARD_LO, CARD_HI), fps, CARD_LO, CARD_HI)
        ff0 = fft_peak(bseg, fps, RR_LO, RR_HI)
        if ffc and ff0:
            fc.append(ffc); f0.append(ff0)
    fc, f0 = np.array(fc), np.array(f0)
    if len(fc) < 4:
        return None, None, None
    corr = float(np.corrcoef(fc, f0)[0, 1])
    ratio = float(np.median(fc / f0))                   # harmonic number if locked
    return corr, ratio, len(fc)


def analyze(path, fps, true_hr):
    d = np.load(path, allow_pickle=True)
    cube = np.asarray(d["snapshots"], dtype=np.complex64)
    counts = d["counts"].astype(int); bins = d["bins"].astype(int)
    C = cube[:, :int(counts.min()), :]
    chans = demod_channels(C, bins)
    rr_sqi = np.array([sqi(bandpass(c, fps, RR_LO, RR_HI), fps, RR_LO, RR_HI)
                       for c in chans])
    top = np.argsort(rr_sqi)[::-1][:5]
    breath = bandpass(chans[top[0]], fps, RR_LO, RR_HI)

    print(f"\n=== {path}  true {true_hr} ===")
    corr, ratio, nfr = track_independence(chans[top[0]], breath, fps)
    print(f"  [diagnostic] dominant cardiac-band peak vs breathing rate: "
          f"corr={corr:+.2f}  median fc/f0={ratio:.1f}  "
          f"({'tracks f0 -> HARMONIC' if corr and corr > 0.5 else 'independent -> not locked'})")
    for i in top[:3]:
        raw = chans[i]
        clean = phase_deharmonic(raw, bandpass(chans[top[0]], fps, RR_LO, RR_HI), fps)
        print(f"  bin {bins[i]} ({bins[i]*0.0234375:.2f}m)")
        read_hr(raw, fps, "raw")
        if clean is not None:
            read_hr(clean, fps, "phase-deharm")


for path, fps, truth in [("tachy2_cube.npz", 18.78, "128->91 (Q)"),
                         ("tachy3_cube.npz", 18.78, "84-87 (S)"),
                         ("sit39_cube.npz", 18.78, "~81")]:
    analyze(path, fps, truth)
