"""Probe v2: null the WHOLE breathing spatial subspace, then hunt the cardiac point.

v1 (spatial_sep_probe.py) failed for two reasons: (a) it selected top-breathing-SQI
bins = diaphragm/belly, NOT the precordium; (b) it nulled only the breathing
fundamental [0.12-0.6], but the poisoning harmonics are PM sidebands of that same
distributed source and can occupy a (slightly) frequency-dependent spatial mode.

v2 implements the blueprint literally:
  1. Per antenna, phase-demod each range bin (spatial dim kept).
  2. Build the breathing spatial subspace U_b from the covariance of the full
     harmonic COMB (n*f0, n=1..8) -- this captures the distributed source across ALL
     its harmonics at once. Its RANK answers "distributed(high-rank) vs point(low)".
  3. Project every antenna onto (I - U_b U_b^H): removes the breathing source AND
     every harmonic regardless of frequency -- the thing no frequency method can do.
  4. On the residual, search ALL bins for the cleanest cardiac peak.

Verdict test: tachy2 should surface ~110-131; sit39 must stay ~81 (no fabrication).

    python spatial_null_probe.py
"""
import numpy as np
from bcg_vitals import (bandpass, sqi, fft_peak, autocorr_peak, beat_count,
                        demod_channels, LAMBDA_MM, RR_LO, RR_HI)

HEART_LO, HEART_HI = 1.0, 2.5


def per_antenna_disp(X):
    phi = np.unwrap(np.angle(X), axis=0)
    return -LAMBDA_MM / (4 * np.pi) * (phi - phi.mean(0))


def comb_filter(x, fps, f0, nmax=8, hw=0.06):
    """Keep only energy within +-hw of n*f0 (n=1..nmax): the breathing harmonic comb."""
    f = np.fft.rfftfreq(len(x), 1 / fps)
    X = np.fft.rfft(x - x.mean())
    keep = np.zeros_like(f, dtype=bool)
    for n in range(1, nmax + 1):
        keep |= (f >= n * f0 - hw) & (f <= n * f0 + hw)
    X[~keep] = 0
    return np.fft.irfft(X, n=len(x))


def breath_subspace(D, fps, f0, rank_thresh=0.95):
    """D:(T,16) -> orthonormal basis U_b of the breathing (comb) spatial subspace,
    keeping modes up to rank_thresh cumulative energy. Returns (U_b, evals)."""
    Dc = np.column_stack([comb_filter(D[:, a], fps, f0) for a in range(D.shape[1])])
    R = (Dc.T @ Dc) / len(Dc)
    ev, U = np.linalg.eigh(R)
    ev = ev[::-1]; U = U[:, ::-1]                     # descending
    cum = np.cumsum(ev) / ev.sum()
    r = int(np.searchsorted(cum, rank_thresh) + 1)
    return U[:, :r], ev, r


def cardiac_read(sig, fps):
    ff = fft_peak(sig, fps, HEART_LO, HEART_HI)
    ac, h = autocorr_peak(sig, fps, int(HEART_LO * 60), int(HEART_HI * 60))
    bc = beat_count(sig, fps, hi_bpm=int(HEART_HI * 60), height=0.4)
    return (ff and ff * 60), ac, h, bc


def analyze(path, fps, true_hr):
    d = np.load(path, allow_pickle=True)
    cube = np.asarray(d["snapshots"], dtype=np.complex64)
    counts = d["counts"].astype(int); bins = d["bins"].astype(int)
    K = int(counts.min())
    C = cube[:, :K, :]

    # global breathing f0 from the collapsed channels (for the comb)
    chans = demod_channels(C, bins)
    rr_sqi = np.array([sqi(bandpass(c, fps, RR_LO, RR_HI), fps, RR_LO, RR_HI)
                       for c in chans])
    rtop = np.argsort(rr_sqi)[::-1][:8]
    f0s = [fft_peak(chans[i], fps, RR_LO, RR_HI) for i in rtop]
    f0 = float(np.median([x for x in f0s if x]))
    print(f"\n=== {path}  (fps={fps}, TRUE {true_hr})  breath f0={f0:.3f}Hz "
          f"({f0*60:.0f}rpm) ===")

    rows = []
    for i in range(len(bins)):
        X = C[i]
        D = per_antenna_disp(X)
        Ub, ev, r = breath_subspace(D, fps, f0)
        P = np.eye(16) - Ub @ Ub.T                    # null the breathing subspace
        Dr = D @ P                                     # residual (T,16)
        Dh = np.column_stack([bandpass(Dr[:, a], fps, HEART_LO, HEART_HI)
                              for a in range(16)])
        # combine residual antennas by cardiac-band energy
        e = (Dh ** 2).sum(0)
        w = e / (e.sum() + 1e-12)
        z = Dh @ np.sqrt(w)
        ff, ac, h, bc = cardiac_read(z, fps)
        card_e = float((z ** 2).sum())
        rows.append((bins[i], bins[i] * 0.0234375, r, ff, ac, h, bc, card_e))

    rows.sort(key=lambda t: -t[7])                     # by residual cardiac energy
    print(f"  breath-subspace rank (median over bins) = "
          f"{int(np.median([t[2] for t in rows]))}")
    print(f"  {'bin':>4} {'range':>6} {'Rrank':>5} {'FFT':>6} {'acorr':>6} "
          f"{'h':>4} {'beat':>6}   (top-8 by residual cardiac energy)")
    for b, rng, r, ff, ac, hh, bc, ce in rows[:8]:
        print(f"  {b:>4} {rng:>5.2f}m {r:>5} {ff or 0:>6.1f} {ac or 0:>6.1f} "
              f"{hh:>4.2f} {bc:>6.1f}")


if __name__ == "__main__":
    analyze("tachy2_cube.npz", 18.78, "110-131")
    analyze("sit39_cube.npz", 18.78, "~81")
