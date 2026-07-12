"""Probe: does the 16-antenna SPATIAL axis separate heart from breath?

Rationale (see NEXT.md 物理本质 / memory tachy-miss-algorithmic): every one of the
17 failed methods ran AFTER demod_channels collapses 16 antennas -> 1 via the mean
steering vector (bcg_vitals.py:116). All failures were therefore on the FREQUENCY
axis, where breathing PM-harmonics (J_n(beta_r), beta_r~12.6 -> ~12 strong sidebands)
fill the tachy band and can exceed true cardiac J_1 -> "pick strongest" locks a
harmonic. But those harmonics are sidebands of ONE physical source (diaphragm/ribcage,
distributed) so they share its SPATIAL signature. A spatial filter that nulls the
breathing source nulls ALL its harmonics at once, regardless of frequency -- something
no frequency method can do.

This probe: per-antenna phase demod (NOT collapsed) at chest bins, then a MaxSNR /
GEVD spatial filter w = argmax (w^H R_heart w)/(w^H R_breath w). Compare the HR read
from the collapsed baseline channel vs the spatially-separated channel, on:
  - tachy2 (near ~2m, TRUE HR 110-131) -- should now read HIGH, not 81
  - sit39   (resting, TRUE ~81)         -- MUST stay ~81 (no fabricated tachy)

    python spatial_sep_probe.py
"""
import numpy as np
from scipy.linalg import eigh
from bcg_vitals import (bandpass, sqi, fft_peak, autocorr_peak, beat_count,
                        demod_channels, LAMBDA_MM, RR_LO, RR_HI)

BREATH_LO, BREATH_HI = 0.12, 0.6      # breathing fundamental (distributed source)
HEART_LO, HEART_HI = 1.0, 2.5         # full cardiac search band (resting..tachy)


def per_antenna_disp(X):
    """X: (T,16) complex slow-time for one range bin -> (T,16) mm displacement,
    one phase-demod channel PER antenna (spatial dimension preserved)."""
    phi = np.unwrap(np.angle(X), axis=0)               # (T,16) rad
    return -LAMBDA_MM / (4 * np.pi) * (phi - phi.mean(0))


def spatial_cov(D):
    """D: (T,16) real band-filtered displacement -> (16,16) spatial covariance."""
    Dc = D - D.mean(0)
    return (Dc.T @ Dc) / len(Dc)


def maxsnr_filter(D, fps):
    """GEVD spatial filter maximizing heart-band / breath-band power ratio.
    Returns (w, z) where z(t)=D_heart @ w is the separated 1-D signal."""
    Db = np.column_stack([bandpass(D[:, a], fps, BREATH_LO, BREATH_HI)
                          for a in range(D.shape[1])])
    Dh = np.column_stack([bandpass(D[:, a], fps, HEART_LO, HEART_HI)
                          for a in range(D.shape[1])])
    Rb = spatial_cov(Db) + 1e-9 * np.eye(D.shape[1])   # breath (to be nulled)
    Rh = spatial_cov(Dh)                               # heart  (to be passed)
    evals, evecs = eigh(Rh, Rb)                         # generalized eig
    w = evecs[:, -1]                                    # max heart/breath ratio
    z = Dh @ w
    return w, z, float(evals[-1])


def read_hr(sig, fps, label):
    """Report FFT peak, autocorr, beat-count over the full cardiac band."""
    ff = fft_peak(sig, fps, HEART_LO, HEART_HI)
    ac, h = autocorr_peak(sig, fps, int(HEART_LO * 60), int(HEART_HI * 60))
    bc = beat_count(sig, fps, hi_bpm=int(HEART_HI * 60), height=0.4)
    print(f"    {label:22s} FFT={ff and ff*60:6.1f}  "
          f"autocorr={ac and ac:6.1f}(h={h:.2f})  beat-cnt={bc:6.1f}")
    return ff and ff * 60


def analyze(path, fps, true_hr):
    d = np.load(path, allow_pickle=True)
    cube = np.asarray(d["snapshots"], dtype=np.complex64)
    counts = d["counts"].astype(int); bins = d["bins"].astype(int)
    K = int(counts.min())
    C = cube[:, :K, :]                                  # (nbin, T, 16)

    # pick chest bins = top breathing-SQI on the collapsed channel (same as pipeline)
    chans = demod_channels(C, bins)
    rr_sqi = np.array([sqi(bandpass(c, fps, RR_LO, RR_HI), fps, RR_LO, RR_HI)
                       for c in chans])
    top = np.argsort(rr_sqi)[::-1][:4]

    print(f"\n=== {path}  (fps={fps}, TRUE HR {true_hr}) ===")
    for i in top:
        X = C[i]                                        # (T,16)
        base = chans[i]                                 # collapsed baseline channel
        base_h = bandpass(base, fps, HEART_LO, HEART_HI)
        D = per_antenna_disp(X)                         # (T,16) spatial
        w, z, ratio = maxsnr_filter(D, fps)
        print(f"  bin {bins[i]} ({bins[i]*0.0234375:.2f}m)  GEVD heart/breath={ratio:.2f}")
        read_hr(base_h, fps, "collapsed (baseline)")
        read_hr(z,     fps, "spatial MaxSNR")


if __name__ == "__main__":
    analyze("tachy2_cube.npz", 18.78, "110-131")
    analyze("sit39_cube.npz", 18.78, "~81")
