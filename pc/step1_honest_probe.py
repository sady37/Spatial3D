"""Honest probe of the resting-HR mainline: is the [1.0,1.7]Hz autocorr detecting
a heartbeat, or is the band prior + coarse lag grid manufacturing ~80bpm?

For each cube we take the SQI-top HR bins, bandpass to [1.0,1.7]Hz, and look at:
  - the full in-band autocorrelation curve (not just its argmax)
  - the winning lag's height AND its MARGIN over the neighbour lags
  - the same for an EMPTY room (control): if empty also peaks at lag=14 with a
    similar height, the number is noise shaped by the band, i.e. force-fit.
"""
import numpy as np
from bcg_vitals import demod_channels, bandpass, sqi

FPS = 18.78
LO, HI = 1.0, 1.7

def load(path, t0=0, t1=1e9):
    d = np.load(path, allow_pickle=True)
    cube = np.asarray(d["snapshots"], dtype=np.complex64)
    bins = d["bins"].astype(int)
    K = int(d["counts"].astype(int).min())
    i0, i1 = int(t0*FPS), min(K, int(t1*FPS))
    return demod_channels(cube[:, i0:i1, :], bins), bins

def band_autocorr(sig, fps, lo_bpm=60, hi_bpm=102):
    ac = np.correlate(sig, sig, "full")[len(sig)-1:]
    ac = ac / (ac[0] + 1e-12)
    l0, l1 = int(fps/(hi_bpm/60)), int(fps/(lo_bpm/60))
    lags = np.arange(l0, l1+1)
    return lags, ac[l0:l1+1]

def analyze(path, label, t0=0, t1=1e9):
    chans, bins = load(path, t0, t1)
    hr_sqi = np.array([sqi(bandpass(c, FPS, LO, HI), FPS, LO, HI) for c in chans])
    top = np.argsort(hr_sqi)[::-1][:8]
    print(f"\n==== {label} ====")
    win_lags, heights, margins = [], [], []
    for i in top:
        sig = bandpass(chans[i], FPS, LO, HI)
        lags, ac = band_autocorr(sig, FPS)
        k = int(np.argmax(ac))
        win_lag = lags[k]; h = ac[k]
        # margin = winning height minus the best competing lag (>=2 lags away)
        comp = ac.copy(); comp[max(0,k-1):k+2] = -1
        margin = h - comp.max()
        bpm = FPS/win_lag*60
        win_lags.append(win_lag); heights.append(h); margins.append(margin)
        print(f"  bin{bins[i]:3d}: win lag={win_lag} ({bpm:5.1f}bpm) height={h:+.3f} "
              f"margin_vs_next={margin:+.3f}  curve={np.array2string(ac, precision=2, suppress_small=True)}")
    print(f"  --> median winning height={np.median(heights):.3f}  "
          f"median margin={np.median(margins):.3f}  "
          f"winning lags={sorted(set(win_lags))}")

analyze("sit33_cube.npz", "sit33 (truth ~82)")
analyze("sit39_cube.npz", "sit39 (truth ~81)")
analyze("lie41_cube.npz", "lie41 (truth ~77)")
analyze("fall20_cube.npz", "fall20 (truth ~80)")
analyze("emptyL_cube.npz", "emptyL (CONTROL: no person)")
