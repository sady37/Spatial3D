"""Find the heart by the EXCESS it adds on top of a breathing harmonic.

When HR coincides with n*f0 (sport33: RR 19rpm -> 5th harmonic = 95 = true HR), you
can't separate them in frequency. But breathing harmonics follow a SMOOTH decaying
envelope; the cardiac energy piled onto one harmonic makes THAT harmonic anomalously
tall vs the envelope its neighbors define. So: measure A_n at each n*f0, fit a smooth
baseline from the OTHER harmonics, and the harmonic with the largest positive excess
(within the cardiac band) points at HR ~= n*f0. Time-resolve it: as HR descends
106->82 it should hand the excess from the 5th harmonic down toward the 4th.

    python harmonic_excess.py sport33_cube.npz
    python harmonic_excess.py sit33_cube.npz     # resting control
"""
import sys
import numpy as np
from bcg_vitals import (bandpass, sqi, fft_peak, demod_channels, estimate_rr,
                        RR_LO, RR_HI)

FPS = 18.8
NH = 8


def harm_amps(sig, fps, f0, nh=NH):
    f = np.fft.rfftfreq(len(sig), 1 / fps)
    S = np.abs(np.fft.rfft(sig - sig.mean()))
    amps = []
    for n in range(1, nh + 1):
        m = np.abs(f - n * f0) <= 0.04            # nearest bins to n*f0
        amps.append(float(S[m].max()) if m.any() else 0.0)
    return np.array(amps)


def excess(amps):
    """Local excess: each harmonic vs the geometric mean of its two neighbours.
    A cardiac bump piled on harmonic n pushes A_n above what n-1,n+1 interpolate.
    Robust to the smooth comb decay and to zero low harmonics."""
    exc = np.ones_like(amps)
    for i in range(1, len(amps) - 1):
        lo, hi = amps[i - 1], amps[i + 1]
        base = np.sqrt(max(lo, 1e-9) * max(hi, 1e-9))
        exc[i] = amps[i] / (base + 1e-9)
    return exc


def analyze(path):
    d = np.load(path, allow_pickle=True)
    cube = np.asarray(d["snapshots"], dtype=np.complex64)
    counts = d["counts"].astype(int); bins = d["bins"].astype(int)
    C = cube[:, :int(counts.min()), :]
    chans = demod_channels(C, bins)
    _, f0, _, _ = estimate_rr(chans, FPS)
    rr_sqi = np.array([sqi(bandpass(c, FPS, RR_LO, RR_HI), FPS, RR_LO, RR_HI) for c in chans])
    top = np.argsort(rr_sqi)[::-1][:6]
    print(f"\n=== {path}  f0={f0:.3f}Hz ({f0*60:.0f}rpm) ===")
    print("  harmonics n*f0 (bpm):", [round(n * f0 * 60) for n in range(1, NH + 1)])

    # whole-record, SQI-top bins averaged
    A = np.mean([harm_amps(bandpass(chans[i], FPS, 0.7, 3.2), FPS, f0) for i in top], 0)
    exc = excess(A)
    print("  amp     :", [round(a, 1) for a in A])
    print("  excess  :", [round(e, 2) for e in exc])
    card = [(n, exc[n - 1]) for n in range(1, NH + 1) if 1.0 <= n * f0 <= 2.3]
    best = max(card, key=lambda t: t[1])
    print(f"  -> excess peak in cardiac band at n={best[0]} = {round(best[0]*f0*60)}bpm "
          f"(excess {best[1]:.2f})")

    # time-resolved: does the excess-harmonic move as HR descends?
    n15 = int(15 * FPS); step = int(10 * FPS)
    print("  time-resolved excess-HR:", end=" ")
    for s in range(0, C.shape[1] - n15 + 1, step):
        seg = demod_channels(C[:, s:s + n15, :], bins)
        A = np.mean([harm_amps(bandpass(seg[i], FPS, 0.7, 3.2), FPS, f0) for i in top], 0)
        e = excess(A)
        cand = [(n, e[n - 1]) for n in range(1, NH + 1) if 1.0 <= n * f0 <= 2.3]
        b = max(cand, key=lambda t: t[1])
        print(f"[{int(s/FPS)}]{round(b[0]*f0*60)}", end=" ")
    print()


for p in sys.argv[1:] or ["sport33_cube.npz"]:
    analyze(p)
