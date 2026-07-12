"""Prototype: FFT-based tachycardia region vote vs the current autocorr vote.
Finding (tachy_diag): at near range the true tachy HR shows up in the per-bin
FFT peak of the FULL band, but the autocorr locks to breathing harmonics / half
rate in [1.0-1.7]. So gate tachycardia on the FFT peak location, not autocorr.

vote_fft = fraction of SQI-top bins whose FULL-band [lo,tachy_hi] FFT peak sits
above `split` (1.7Hz=102bpm). HIGH iff frac>=0.5; value = median of those peaks.
Must fire on tachy1/tachy2 (real elevated HR) and STAY LOW on the resting cubes."""
import numpy as np
from bcg_vitals import (demod_channels, estimate_rr, bandpass, sqi, fft_peak,
                        autocorr_peak, occupancy)

FPS = 18.8
LO, SPLIT, TACHY_HI = 1.0, 1.7, 2.4
CASES = [("tachy2(near,HR110-131)", "tachy2_cube.npz"),
         ("tachy1(far,HR110+)", "tachy1_cube.npz"),
         ("sit39(rest81)", "sit39_cube.npz"),
         ("sidesit(rest81)", "sidesit_cube.npz"),
         ("lie41(rest87)", "lie41_cube.npz"),
         ("fall20(rest81)", "fall20_cube.npz"),
         ("emptyT(none)", "emptyT_cube.npz")]


def vote(chans, fps, f0, topk=8):
    # SQI over full band so we don't bias bin selection toward the low sub-band
    s = np.array([sqi(bandpass(c, fps, LO, TACHY_HI, notch_f0=f0), fps, LO, TACHY_HI)
                  for c in chans])
    top = np.argsort(s)[::-1][:topk]
    ff_all, ac_all = [], []
    for i in top:
        sig = bandpass(chans[i], fps, LO, TACHY_HI, notch_f0=f0)   # notch harmonics
        ff = fft_peak(sig, fps, LO, TACHY_HI)                      # FFT on NOTCHED
        if ff:
            ff_all.append(ff * 60)
        bpm, _ = autocorr_peak(sig, fps, int(LO * 60), int(TACHY_HI * 60))
        if bpm:
            ac_all.append(bpm)
    ff_all, ac_all = np.array(ff_all), np.array(ac_all)
    frac_fft = float(np.mean(ff_all > SPLIT * 60)) if len(ff_all) else 0.0
    frac_ac = float(np.mean(ac_all > SPLIT * 60)) if len(ac_all) else 0.0
    hi = ff_all[ff_all > SPLIT * 60]
    val = float(np.median(hi)) if len(hi) else None
    return frac_fft, frac_ac, val, ff_all


print(f"{'case':26} occ   {'frac_fft':>8} {'frac_ac':>8}  fft>102 median   all fft peaks")
for name, path in CASES:
    d = np.load(path, allow_pickle=True)
    cube = np.asarray(d["snapshots"], dtype=np.complex64)
    K = int(d["counts"].astype(int).min()); bins = d["bins"].astype(int)
    cube = cube[:, :K, :]
    # analyze the first 20s (where tachy HR is highest); resting is stationary anyway
    W = min(K, int(20 * FPS))
    chans = demod_channels(cube[:, :W, :], bins)
    occ = occupancy(chans, FPS)
    _, f0, _, _ = estimate_rr(chans, FPS)
    ff_frac, ac_frac, val, ffs = vote(chans, FPS, f0)
    print(f"{name:26} {'P' if occ['present'] else '-'}    {ff_frac:8.0%} {ac_frac:8.0%}  "
          f"{val and round(val)}bpm        {np.round(np.sort(ffs)).astype(int).tolist()}")
