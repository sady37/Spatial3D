"""Respiratory-gated BCG beat detection: find the J/K/L waves BETWEEN breaths.

User's insight: the breathing PM-harmonic comb that poisons the cardiac band is a
nonlinear product of phase VELOCITY (e^{jphi}); it is strongest mid-breath (max
diaphragm velocity) and vanishes at the respiratory TURNING POINTS (end-inspiration /
end-expiration, where breathing velocity -> 0). In those pause windows the large
breathing motion is locally flat (a near-constant offset the bandpass removes), so the
small sharp cardiac J-K-L beats are exposed. Separate on the TIME axis, not frequency
or space.

Method:
  1. breathing reference = SQI-top breathing bin, bandpass RR band -> breath(t)
  2. low-velocity gate = |d/dt breath| below its P-th percentile (the pauses)
  3. cardiac = bandpass [0.9-3.0]Hz on the chest bins
  4. detect beats (sharp peaks) but KEEP only those inside pause windows
  5. HR = 60 / median inter-beat interval  (robust to gaps between pauses)
  cross-check: FFT of the pause-weighted cardiac signal.

Test: tachy2 (true 128->91) should read >100; tachy3/sit39 (true ~85/81) ~85.

    python respgate_probe.py
"""
import numpy as np
from scipy.signal import find_peaks
from bcg_vitals import (bandpass, sqi, fft_peak, demod_channels, RR_LO, RR_HI)

CARD_LO, CARD_HI = 0.9, 3.0      # wide cardiac band (resting..tachy), keeps J-peak shape
GATE_PCTL = 15                   # keep frames whose |breath velocity| is in lowest 15%
ACCEL = True                     # detect J-peaks in acceleration domain (sharpens transient)


def analyze(path, fps, true_hr):
    d = np.load(path, allow_pickle=True)
    cube = np.asarray(d["snapshots"], dtype=np.complex64)
    counts = d["counts"].astype(int); bins = d["bins"].astype(int)
    C = cube[:, :int(counts.min()), :]
    chans = demod_channels(C, bins)

    rr_sqi = np.array([sqi(bandpass(c, fps, RR_LO, RR_HI), fps, RR_LO, RR_HI)
                       for c in chans])
    btop = np.argsort(rr_sqi)[::-1][:6]

    # breathing reference + low-velocity (pause) gate
    breath = bandpass(chans[btop[0]], fps, RR_LO, RR_HI)
    vel = np.abs(np.gradient(breath))
    gate = vel < np.percentile(vel, GATE_PCTL)          # True in the pauses
    duty = gate.mean()

    hrs_beat, hrs_fft = [], []
    for i in btop:
        card = bandpass(chans[i], fps, CARD_LO, CARD_HI)
        if ACCEL:                                       # accel sharpens the J transient
            card = np.gradient(np.gradient(card))
        card = card / (card.std() + 1e-9)
        # --- beat detect over whole signal, then keep beats inside pauses ---
        dist = max(1, int(fps / (CARD_HI)))             # min beat spacing
        pk, _ = find_peaks(card, distance=dist, height=0.5)
        pk_gated = pk[gate[pk]]
        n_gated = len(pk_gated)
        if n_gated > 3:
            ibi = np.diff(pk_gated) / fps               # inter-beat intervals (s)
            ibi = ibi[(ibi > 0.4) & (ibi < 1.2)]        # 50-150 bpm physiologic
            if len(ibi) > 2:
                hrs_beat.append(60.0 / np.median(ibi))
        # expected beats over gated duration IF true HR held: diagnose halving
        gated_secs = gate.sum() / fps
        # --- cross-check: FFT of pause-weighted cardiac signal ---
        wsig = card * gate                              # zero out non-pause frames
        ff = fft_peak(wsig, fps, CARD_LO, CARD_HI)
        if ff:
            hrs_fft.append(ff * 60)

    beat_hr = float(np.median(hrs_beat)) if hrs_beat else None
    fft_hr = float(np.median(hrs_fft)) if hrs_fft else None
    print(f"  {path:18s} true {true_hr:10s} | pause-duty {duty:.0%} | "
          f"gated-beat HR = {beat_hr and round(beat_hr):>4} bpm | "
          f"pause-FFT HR = {fft_hr and round(fft_hr):>4} bpm "
          f"| per-bin beat {[round(x) for x in hrs_beat]}")


print("respiratory-gated BCG beat detection (J/K/L between breaths):\n")
for path, fps, truth in [("tachy2_cube.npz", 18.78, "128->91"),
                         ("tachy3_cube.npz", 18.78, "84-87"),
                         ("sit39_cube.npz", 18.78, "~81"),
                         ("lie41_cube.npz", 18.78, "~77"),
                         ("fall20_cube.npz", 18.78, "~80")]:
    analyze(path, fps, truth)
