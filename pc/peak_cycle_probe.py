"""Peak-aligned breathing-cycle synchronous subtraction (user's idea: find breathing
PEAKS by energy, take the segment between two peaks as one cycle).

Detect breathing peaks (displacement maxima) -> true cycle boundaries (robust to the
irregular post-exercise breathing rate that Hilbert phase smears). Resample each
peak-to-peak cycle to a common length, median-stack -> the breath-locked waveform
(fundamental + ALL shape harmonics), subtract per cycle -> residual = asynchronous
cardiac. Autocorr the residual in [1.0-2.2Hz]. Cardiac survives even at 95 = 5*RR
because it is NOT breath-locked, so peak-cycle averaging can't capture it.

    python peak_cycle_probe.py
"""
import numpy as np
from scipy.signal import find_peaks
from bcg_vitals import (bandpass, sqi, autocorr_peak, fft_peak, demod_channels,
                        estimate_rr, RR_LO, RR_HI)

FPS = 18.8
LO, HI = 1.0, 2.2
NPC = 64                          # resample points per breathing cycle


def peak_cycle_subtract(disp, breath, fps):
    """Subtract the peak-aligned breathing-cycle template. Returns residual (len=T)."""
    b = breath / (breath.std() + 1e-9)
    # breathing peaks: min spacing = half a breath period (RR up to ~30rpm -> 1s)
    pk, _ = find_peaks(b, distance=int(fps * 0.8))
    if len(pk) < 5:
        return None
    resid = disp.copy().astype(float)
    cycles, spans = [], []
    for a, c in zip(pk[:-1], pk[1:]):
        seg = disp[a:c]
        if len(seg) < 8:
            continue
        # resample this cycle to NPC points (normalizes variable cycle length)
        xi = np.linspace(0, len(seg) - 1, NPC)
        cycles.append(np.interp(xi, np.arange(len(seg)), seg))
        spans.append((a, c))
    if len(cycles) < 4:
        return None
    template = np.median(np.array(cycles), axis=0)      # breath-locked common waveform
    for (a, c) in spans:
        seg_len = c - a
        # map template back to this cycle's length and subtract
        xi = np.linspace(0, NPC - 1, seg_len)
        resid[a:c] = disp[a:c] - np.interp(xi, np.arange(NPC), template)
    return resid


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

    sub_ac, sub_fft = [], []
    for i in top:
        resid = peak_cycle_subtract(chans[i], breath, FPS)
        if resid is None:
            continue
        rb = bandpass(resid, FPS, LO, HI)
        a, _ = autocorr_peak(rb, FPS, int(LO * 60), int(HI * 60))
        ff = fft_peak(rb, FPS, LO, HI)
        if a: sub_ac.append(a)
        if ff: sub_fft.append(ff * 60)

    def med(x): return round(np.median(x)) if x else None
    print(f"  {path:16s} true {truth:10s} | peak-cycle-sub ac {med(sub_ac)}  fft {med(sub_fft)} "
          f" per-bin ac {[round(x) for x in sub_ac]}  (RR {f0*60:.0f}, 5xRR={round(5*f0*60)})")


print("peak-aligned breathing-cycle subtraction (autocorr on residual):\n")
for p, t in [("sport33_cube.npz", "95->82"), ("sit33_cube.npz", "82"),
             ("tachy2_cube.npz", "131->91"), ("tachy3_cube.npz", "84-87")]:
    analyze(p, t)
