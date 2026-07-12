"""Crack tachy2 by SPATIAL breath-null (free the cardiac the mean-steering beamformer
suppresses) + per-segment tracking (handle the 131->91 sweep). For each candidate bin,
null the breathing spatial subspace, then per 15s segment take the residual FFT peak in
[1.2-2.3Hz] and see if it tracks the true descent. Check chest bins (92/93) vs bin 65.
True: 0-60s 131->110, 60-120s 109->91.
"""
import numpy as np
from bcg_vitals import bandpass, sqi, fft_peak, demod_channels, estimate_rr, RR_LO, RR_HI
from spatial_null_probe import per_antenna_disp, breath_subspace

FPS = 18.78
CLO, CHI = 1.2, 2.3          # cardiac search (72-138 bpm), above most breathing harmonics


def truth(t):
    return np.where(t <= 60, 131 - 21 * t / 60, 110 - 19 * (t - 60) / 60)


d = np.load("tachy2_cube.npz", allow_pickle=True)
cube = np.asarray(d["snapshots"], np.complex64)
counts = d["counts"].astype(int); bins = d["bins"].astype(int)
C = cube[:, :int(counts.min()), :]
chans = demod_channels(C, bins)
_, f0, _, _ = estimate_rr(chans, FPS)
rr = np.array([sqi(bandpass(c, FPS, RR_LO, RR_HI), FPS, RR_LO, RR_HI) for c in chans])
chest = int(np.argsort(rr)[::-1][0])
print(f"f0={f0:.3f}Hz; chest(breath-SQI) bin={bins[chest]}")

n = int(15 * FPS); step = int(10 * FPS)
cand_bins = [65, 92, 93, 99, 100, 105]
for bval in cand_bins:
    idx = int(np.where(bins == bval)[0][0]) if bval in bins else None
    if idx is None:
        continue
    D = per_antenna_disp(C[idx])
    Ub, _, _ = breath_subspace(D, FPS, f0)
    Dr = D @ (np.eye(16) - Ub @ Ub.T)
    z = Dr.sum(1)                                   # combine nulled channels
    traj = []
    for s in range(0, C.shape[1] - n + 1, step):
        seg = bandpass(z[s:s + n], FPS, CLO, CHI)
        ff = fft_peak(seg, FPS, CLO, CHI)
        traj.append(round(ff * 60) if ff else 0)
    tc = np.array([(s + n / 2) / FPS for s in range(0, C.shape[1] - n + 1, step)])
    err = np.median(np.abs(np.array(traj) - truth(tc)))
    print(f"  bin {bval} ({bval*0.0234:.2f}m): {traj}  |med err {err:.0f}| slope "
          f"{np.polyfit(tc,traj,1)[0]*120:+.0f}")
print(f"  (truth ~{[round(x) for x in truth(tc)]})")
