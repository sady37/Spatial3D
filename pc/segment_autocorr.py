"""Test the explanation: is NON-STATIONARITY the killer (not a hard SNR wall)?

The working pipeline reads resting HR by autocorr-integrating the FULL 120s. A sweeping
post-exercise HR can't be integrated (freq moves) -> it smears. Prediction: cut the
record into ~15s segments (quasi-stationary within each) and run the SAME autocorr,
wide band [1.0-2.5] no cap. If:
  - tachy2/1 segments DESCEND toward each segment's true HR (>81, sloping down)
  - sit39/tachy3 segments stay FLAT ~81/85
then non-stationarity is the killer and the info is there per-segment. If tachy
segments ALSO read flat ~80, then per-segment SNR is too low -> leans back to SNR.

    python segment_autocorr.py
"""
import numpy as np
from bcg_vitals import (bandpass, sqi, demod_channels, estimate_rr,
                        hr_band_search, hr_fft_value, RR_LO, RR_HI)

FPS = 18.78
SEG_S, STEP_S = 15, 5
LO, HI = 1.0, 2.5                  # wide cardiac band, NO resting cap


def run(path, truth_fn, label):
    d = np.load(path, allow_pickle=True)
    cube = np.asarray(d["snapshots"], dtype=np.complex64)
    counts = d["counts"].astype(int); bins = d["bins"].astype(int)
    K = int(counts.min())
    C = cube[:, :K, :]
    chans_full = demod_channels(C, bins)
    _, f0, _, _ = estimate_rr(chans_full, FPS)

    # full-record reference (wide band)
    full = hr_band_search(chans_full, FPS, f0, LO, HI, topk=8)
    print(f"\n=== {path}  ({label})  f0={f0:.3f}Hz ===")
    print(f"  FULL-record autocorr[{LO}-{HI}] = {full['hr'] and round(full['hr'])} bpm")

    n = int(SEG_S * FPS); step = int(STEP_S * FPS)
    rows = []
    for s in range(0, K - n, step):
        seg = C[:, s:s + n, :]
        ch = demod_channels(seg, bins)
        r = hr_band_search(ch, FPS, f0, LO, HI, topk=8)
        ff, _, _ = hr_fft_value(ch, FPS, f0, LO, HI, topk=8)
        tc = s / FPS + SEG_S / 2
        rows.append((tc, r["hr"], ff, truth_fn(np.array([tc]))[0], r["spread"]))

    print(f"  {'t(s)':>5} {'ac_hr':>6} {'fft_hr':>7} {'truth':>6} {'spread':>7}")
    ac_err, fft_err = [], []
    for tc, ac, ff, tr, sp in rows:
        print(f"  {tc:>5.0f} {ac and round(ac):>6} {ff and round(ff):>7} {tr:>6.0f} {sp:>7.1f}")
        if ac: ac_err.append(abs(ac - tr))
        if ff: fft_err.append(abs(ff - tr))
    acs = [r[1] for r in rows if r[1]]
    if len(acs) > 2:
        slope = np.polyfit([r[0] for r in rows if r[1]], acs, 1)[0] * (rows[-1][0]-rows[0][0])
        print(f"  --> autocorr segments: median|err|={np.median(ac_err):.0f}bpm  "
              f"slope over record={slope:+.0f}bpm  (truth slope {truth_fn(np.array([rows[-1][0]]))[0]-truth_fn(np.array([rows[0][0]]))[0]:+.0f})")


run("tachy2_cube.npz", lambda t: np.where(t <= 60, 131 - 21*t/60, 110 - 19*(t-60)/60), "Q true 131->91")
run("tachy1_cube.npz", lambda t: 122 - 27*t/120, "P true >110 desc")
run("tachy3_cube.npz", lambda t: np.full_like(t, 85.0), "S true 84-87 flat")
run("sit39_cube.npz", lambda t: np.full_like(t, 81.0), "true 81 flat")
