"""Breathing-synchronous template subtraction, read out CORRECTLY.

The cardiac is a periodic phase change that is NOT breath-locked. Synchronous averaging
over breathing cycles builds the breath-locked waveform (fundamental + ALL its shape
harmonics); subtracting it leaves the asynchronous cardiac -- even where cardiac
coincides with a harmonic (sport33: 95 = 5*RR), because the cardiac isn't phase-locked
so the average doesn't capture it. Prior attempt failed only on READOUT (wideband
argmax locked the low residue). Here: autocorr period in [1.0-2.2Hz] on the residual.

    python template_readout.py
"""
import numpy as np
from bcg_vitals import (bandpass, sqi, autocorr_peak, fft_peak, demod_channels,
                        estimate_rr, RR_LO, RR_HI)
from synced_template_probe import template_subtract

FPS = 18.8
LO, HI = 1.0, 2.2


def analyze(path, truth):
    d = np.load(path, allow_pickle=True)
    cube = np.asarray(d["snapshots"], dtype=np.complex64)
    counts = d["counts"].astype(int); bins = d["bins"].astype(int)
    C = cube[:, :int(counts.min()), :]
    chans = demod_channels(C, bins)
    _, f0, _, _ = estimate_rr(chans, FPS)
    rr_sqi = np.array([sqi(bandpass(c, FPS, RR_LO, RR_HI), FPS, RR_LO, RR_HI) for c in chans])
    top = np.argsort(rr_sqi)[::-1][:6]
    breath = bandpass(chans[top[0]], FPS, RR_LO, RR_HI)

    raw_ac, sub_ac, sub_fft = [], [], []
    for i in top:
        resid = template_subtract(chans[i], breath, FPS)
        rb = bandpass(chans[i], FPS, LO, HI)
        a0, _ = autocorr_peak(rb, FPS, int(LO * 60), int(HI * 60))
        if a0: raw_ac.append(a0)
        if resid is not None:
            sb = bandpass(resid, FPS, LO, HI)
            a1, _ = autocorr_peak(sb, FPS, int(LO * 60), int(HI * 60))
            ff = fft_peak(sb, FPS, LO, HI)
            if a1: sub_ac.append(a1)
            if ff: sub_fft.append(ff * 60)
    def med(x): return round(np.median(x)) if x else None
    print(f"  {path:16s} true {truth:10s} | raw-ac {med(raw_ac)} | "
          f"template-sub ac {med(sub_ac)}  fft {med(sub_fft)}  "
          f"(RR {f0*60:.0f}, 5xRR={round(5*f0*60)})")


print("breathing-template subtraction, autocorr readout on residual:\n")
analyze("sport33_cube.npz", "95->82")
analyze("sit33_cube.npz", "82")
analyze("tachy2_cube.npz", "131->91")
analyze("tachy3_cube.npz", "84-87")
