"""Close the gap: run the persistence test on the SPATIALLY-NULLED residual too.

tachy_existence_probe.py showed no tachy ridge in the collapsed best bin. This asks
the same estimator-independent question of the breath-subspace-nulled residual across
ALL bins: does ANY spatial projection produce a persistent [1.83-2.18]Hz ridge that
tachy2 has and sit39 lacks? Reports the BEST bin (max tachy-frame fraction) per cube.

    python tachy_existence_spatial.py
"""
import numpy as np
from bcg_vitals import (bandpass, sqi, fft_peak, demod_channels, LAMBDA_MM,
                        RR_LO, RR_HI)
from spatial_null_probe import per_antenna_disp, breath_subspace
from tachy_existence_probe import stft_peaks, TACHY_LO, TACHY_HI, CARD_LO, CARD_HI


def analyze(path, fps, true_hr):
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

    best = (-1.0, None, None)
    for i in range(len(bins)):
        D = per_antenna_disp(C[i])
        Ub, ev, r = breath_subspace(D, fps, f0)
        Dr = D @ (np.eye(16) - Ub @ Ub.T)
        Dh = np.column_stack([bandpass(Dr[:, a], fps, CARD_LO, CARD_HI)
                              for a in range(16)])
        e = (Dh ** 2).sum(0); z = Dh @ np.sqrt(e / (e.sum() + 1e-12))
        peaks = stft_peaks(z, fps)
        frac = float(np.mean((peaks >= TACHY_LO) & (peaks <= TACHY_HI)))
        if frac > best[0]:
            best = (frac, bins[i], float(np.median(peaks) * 60))

    print(f"{path:20s} TRUE {true_hr:10s}  BEST spatial-residual bin {best[1]}: "
          f"tachy-frames {best[0]:5.1%}  median {best[2]:.0f}bpm")


if __name__ == "__main__":
    print("Best-over-all-bins tachy ridge persistence on breath-nulled residual:\n")
    analyze("tachy2_cube.npz", 18.78, "110-131")
    analyze("sit39_cube.npz", 18.78, "~81")
    analyze("tachy1_cube.npz", 18.78, ">110 far")
