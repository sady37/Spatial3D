"""Breathing-synchronous template subtraction (the constructive form of the user's
insight: harmonics are phase-LOCKED to the breathing cycle, the heart is NOT).

Info is proven present (bin-65 tracked the true tachy descent under oracle selection;
resting HR matches Apple Watch at these ranges = TI-demo parity). So the miss is
ALGORITHMIC. This removes breathing by SYNCHRONOUS AVERAGING, which cancels everything
locked to the breathing period (fundamental + ALL harmonics + any non-sinusoidal
shape) in one shot, far cleaner than an integer notch:
  1. breathing phase phi(t) via Hilbert of the RR-band signal
  2. resample displacement onto uniform phase, reshape (n_cycles, N_per_cycle)
  3. template = median over cycles (the breathing-locked common waveform)
  4. residual = each cycle - template   -> breathing & harmonics gone, heart survives
  5. back to time, estimate HR in a WIDE band (no 102 cap)

Test: tachy2 (true 128->91) should now clear >100; tachy3/sit39 ~85/81.

    python synced_template_probe.py
"""
import numpy as np
from scipy.signal import hilbert
from bcg_vitals import (bandpass, sqi, fft_peak, autocorr_peak, beat_count,
                        demod_channels, estimate_rr, RR_LO, RR_HI)

WIDE_LO, WIDE_HI = 0.8, 2.6      # heart band, NO resting cap
NPC = 96                          # resample points per breathing cycle


def template_subtract(disp, breath, fps):
    """Return time-domain residual after subtracting the breathing-synchronous
    template. disp/breath are same-length real arrays."""
    phi = np.unwrap(np.angle(hilbert(breath)))
    phi = phi - phi[0]
    ncyc = int(phi[-1] // (2 * np.pi))
    if ncyc < 4:
        return None
    # uniform phase grid over whole cycles
    grid = np.linspace(0, ncyc * 2 * np.pi, ncyc * NPC, endpoint=False)
    sp = np.interp(grid, phi, disp)                  # displacement vs breathing phase
    M = sp.reshape(ncyc, NPC)
    template = np.median(M, axis=0)                  # breathing-locked common waveform
    resid = (M - template).reshape(-1)               # heart survives
    # back to time grid
    return np.interp(phi, grid, resid, left=0, right=0)


def read(sig, fps, tag):
    b = bandpass(sig, fps, WIDE_LO, WIDE_HI)
    ff = fft_peak(b, fps, WIDE_LO, WIDE_HI)
    ac, h = autocorr_peak(b, fps, int(WIDE_LO * 60), int(WIDE_HI * 60))
    bc = beat_count(b, fps, hi_bpm=int(WIDE_HI * 60), height=0.4)
    print(f"    {tag:16s} FFT={ff and ff*60:6.1f}  ac={ac and ac:6.1f}(h={h:.2f})  beat={bc:6.1f}")
    return ff and ff * 60


def analyze(path, fps, true_hr):
    d = np.load(path, allow_pickle=True)
    cube = np.asarray(d["snapshots"], dtype=np.complex64)
    counts = d["counts"].astype(int); bins = d["bins"].astype(int)
    C = cube[:, :int(counts.min()), :]
    chans = demod_channels(C, bins)
    rr_sqi = np.array([sqi(bandpass(c, fps, RR_LO, RR_HI), fps, RR_LO, RR_HI)
                       for c in chans])
    top = np.argsort(rr_sqi)[::-1][:5]
    breath = bandpass(chans[top[0]], fps, RR_LO, RR_HI)   # shared breathing ref

    print(f"\n=== {path}  true {true_hr} ===")
    vals = []
    for i in top[:4]:
        resid = template_subtract(chans[i], breath, fps)
        print(f"  bin {bins[i]} ({bins[i]*0.0234375:.2f}m)")
        read(chans[i], fps, "raw")
        if resid is not None:
            v = read(resid, fps, "template-sub")
            if v: vals.append(v)
    if vals:
        print(f"  --> median template-sub HR across bins = {np.median(vals):.0f} bpm")


for path, fps, truth in [("tachy2_cube.npz", 18.78, "128->91 (Q)"),
                         ("tachy3_cube.npz", 18.78, "84-87 (S)"),
                         ("sit39_cube.npz", 18.78, "~81"),
                         ("lie41_cube.npz", 18.78, "~77")]:
    analyze(path, fps, truth)
