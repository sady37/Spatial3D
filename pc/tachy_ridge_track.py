"""Falsification test: does tachy2 bin-65's breath-nulled ridge TRACK the known
true-HR descent (131->110->91 over 120s)? A real cardiac source follows the Apple
Watch trajectory; a lucky-of-44 noise bin does not. Prints the ridge time series.

Known truth (NEXT.md): 0-60s 128->110 (peak 131);  61-120s 109->91.

    python tachy_ridge_track.py
"""
import numpy as np
from bcg_vitals import (bandpass, sqi, fft_peak, demod_channels, RR_LO, RR_HI)
from spatial_null_probe import per_antenna_disp, breath_subspace
from tachy_existence_probe import stft_peaks, CARD_LO, CARD_HI


def residual_ridge(path, fps, bin_id):
    d = np.load(path, allow_pickle=True)
    cube = np.asarray(d["snapshots"], dtype=np.complex64)
    counts = d["counts"].astype(int); bins = d["bins"].astype(int)
    C = cube[:, :int(counts.min()), :]
    chans = demod_channels(C, bins)
    rr_sqi = np.array([sqi(bandpass(c, fps, RR_LO, RR_HI), fps, RR_LO, RR_HI)
                       for c in chans])
    rtop = np.argsort(rr_sqi)[::-1][:8]
    f0 = float(np.median([x for x in (fft_peak(chans[i], fps, RR_LO, RR_HI)
                                      for i in rtop) if x]))
    idx = int(np.where(bins == bin_id)[0][0])
    D = per_antenna_disp(C[idx])
    Ub, _, _ = breath_subspace(D, fps, f0)
    Dr = D @ (np.eye(16) - Ub @ Ub.T)
    Dh = np.column_stack([bandpass(Dr[:, a], fps, CARD_LO, CARD_HI) for a in range(16)])
    e = (Dh ** 2).sum(0); z = Dh @ np.sqrt(e / (e.sum() + 1e-12))
    return stft_peaks(z, fps) * 60      # bpm per ~1s frame (8s window)


for path, b, truth in [("tachy2_cube.npz", 65, "131->110->91 (should DESCEND)"),
                       ("sit39_cube.npz", 152, "~81 flat (resting control)")]:
    ridge = residual_ridge(path, 18.78, b)
    # median in 20s blocks to see the trajectory through frame jitter
    blk = 20
    print(f"\n{path} bin {b}  truth: {truth}")
    print("  20s-block median bpm:", end=" ")
    fps_frames = 1  # ~1 frame/s
    for s in range(0, len(ridge), blk):
        seg = ridge[s:s + blk]
        if len(seg): print(f"[{s}-{s+len(seg)}s]{np.median(seg):.0f}", end=" ")
    print()
