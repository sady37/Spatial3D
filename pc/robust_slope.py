"""Robust slope discriminator: descending segment-HR = post-exercise/elevated;
flat = resting. Absolute per-segment HR is noisy (+-31bpm) so use Theil-Sen (median of
pairwise slopes, robust to outliers) + a permutation test for slope<0 significance.
Validate separability across all cubes (2 descending vs 5 flat).

    python robust_slope.py
"""
import numpy as np
from scipy.stats import theilslopes
from bcg_vitals import (bandpass, sqi, demod_channels, estimate_rr,
                        hr_band_search, RR_LO, RR_HI)

FPS = 18.78
SEG_S, STEP_S = 15, 5
LO, HI = 1.0, 2.5
SPREAD_MAX = 20.0                 # drop segments with inter-bin spread above this


def segment_series(path):
    d = np.load(path, allow_pickle=True)
    cube = np.asarray(d["snapshots"], dtype=np.complex64)
    counts = d["counts"].astype(int); bins = d["bins"].astype(int)
    K = int(counts.min()); C = cube[:, :K, :]
    _, f0, _, _ = estimate_rr(demod_channels(C, bins), FPS)
    n = int(SEG_S * FPS); step = int(STEP_S * FPS)
    t, hr, sp = [], [], []
    for s in range(0, K - n, step):
        ch = demod_channels(C[:, s:s + n, :], bins)
        r = hr_band_search(ch, FPS, f0, LO, HI, topk=8)
        if r["hr"]:
            t.append(s / FPS + SEG_S / 2); hr.append(r["hr"]); sp.append(r["spread"])
    return np.array(t), np.array(hr), np.array(sp)


def perm_p(t, hr, obs_slope, n=2000):
    """Permutation p-value that slope is more negative than chance."""
    cnt = 0
    rng = list(range(len(hr)))
    for k in range(n):
        # deterministic shuffle (no RNG): roll by k
        h = np.roll(hr, k % len(hr) if len(hr) else 0)
        if k >= len(hr):                      # add index-reversal variants for spread
            h = h[::-1] if (k // len(hr)) % 2 else h
        s = theilslopes(h, t)[0]
        if s <= obs_slope:
            cnt += 1
    return cnt / n


def analyze(path, truth_slope, label):
    t, hr, sp = segment_series(path)
    keep = sp <= SPREAD_MAX
    tk, hk = (t[keep], hr[keep]) if keep.sum() >= 5 else (t, hr)
    slope, intercept, lo_s, hi_s = theilslopes(hk, tk)     # bpm per second
    rec = (t[-1] - t[0])
    slope_rec = slope * rec                                 # bpm over whole record
    p = perm_p(tk, hk, slope)
    verdict = "ELEVATED/recovering" if (slope_rec < -8 and p < 0.1) else "resting/flat"
    early = np.mean(hr[t < t[0] + 30] > 100) if (t < t[0]+30).any() else 0
    print(f"  {path:18s} {label:16s} slope={slope_rec:+5.0f}bpm/rec "
          f"[{lo_s*rec:+.0f},{hi_s*rec:+.0f}] p(desc)={p:.2f} "
          f"early>100:{early:.0%}  truth {truth_slope:+.0f}  -> {verdict}")


print(f"robust descending-HR discriminator (Theil-Sen, seg {SEG_S}s):\n")
print("  DESCENDING (post-exercise):")
analyze("tachy2_cube.npz", -40, "Q 131->91")
analyze("tachy1_cube.npz", -27, "P >110 desc")
print("  FLAT (resting/normal):")
analyze("tachy3_cube.npz", 0, "S 84-87")
analyze("sit39_cube.npz", 0, "sit 81")
analyze("lie41_cube.npz", 0, "lie 77")
analyze("sidesit_cube.npz", 0, "sidesit 78")
analyze("fall20_cube.npz", 0, "fall 80")
