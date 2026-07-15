"""EXPERIMENTAL algorithm — reference-cancellation HR (NOT in the main bcg_vitals pipeline).

STATUS (2026-07-14): the MECHANISM is correct, the SELF-VERIFICATION is NOT usable at 4m.
Kept here (not in bcg_vitals) because it may work at CLOSE range / higher SNR — do not
wire it into the main process until validated on a close capture. See memory
next-crack-rr-harmonic (0714 canceller block).

WHY IT'S THE RIGHT MECHANISM: HR extraction under a radial geometry is a CANCELLATION
problem, not a subspace-separability problem. Breathing is one source polluting many bins;
chest bin = breathing + heart, abdomen bin ~ pure breathing. Borrow the abdomen phase
clock theta_abd as a breathing reference and LS-cancel its FULL harmonic comb
{cos k*theta, sin k*theta} from a chest bin's phase — this removes even the 3f-5f harmonics
sitting ON the heartbeat, which MUSIC/GEVD cannot (they treat each harmonic as a separate
'source' and fold breathing into the signal subspace, pushing the cardiac into the noise
subspace). The cardiac then emerges as the residual FFT peak. On lie_long (radial) this
reads 71 = truth; MUSIC/GEVD on the same data read the noise floor.

WHY THE SELF-VERIFICATION FAILS AT 4m (the honest blocker): at ~4m the cardiac SNR is at
the noise floor, and narrowband filtering makes ANY noise look periodic + stable, so the
verification metrics (spectral prominence, two-half stability) certify the empty room (68
bpm, prom 8.5, halves 68/70) and wrong answers (chairL 65 / truth 80, sitR 72 / truth 103)
with the SAME confidence as the one true success (lie_long 71). prom does not even rank-
correlate with correctness (chairL-wrong 16.6 > lie_long-right 15.2). There is NO self-
contained confidence signal at this SNR — only the external Apple-Watch truth distinguishes
71 (real) from 68 (noise). So this must NOT ship as a 'verified HR': it would confidently
certify noise, worse than abstaining. Occupancy is gated upstream (bcg_vitals.occupancy),
so 'empty' isn't the operational worry; the killer is that it certifies WRONG HR for a
PRESENT person. Revisit only with a close-range (~1.5-2.5m) capture where the residual
prominence should genuinely separate from the empty null.

Usage:
    import bcg_vitals as bv
    from tests.reference_cancel_hr import reference_cancel_hr
    chans = bv.demod_channels(cube, bins)
    r = reference_cancel_hr(chans, fps=18.78, bins=bins)   # r['verified'] unreliable @4m
"""
import os, sys
import numpy as np
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
import bcg_vitals as bv


def _analytic(x):
    """analytic signal (Hilbert) via FFT."""
    n = len(x); X = np.fft.fft(x); H = np.zeros(n)
    if n % 2 == 0:
        H[0] = H[n // 2] = 1; H[1:n // 2] = 2
    else:
        H[0] = 1; H[1:(n + 1) // 2] = 2
    return np.fft.ifft(X * H)


def reference_cancel_hr(chans, fps, bins=None, kmax=20, stab_tol=6.0, prom_min=3.0):
    """Reference-cancellation HR with self-verification. Returns dict(hr, verified,
    confidence, method, abdomen, chest, prom, half_bpm, f_rr, reason).
    ⚠️ verified=True is NOT trustworthy at 4m (see module docstring) — mechanism only.

    Borrow the strongest-breathing (abdomen) bin's phase clock as a breathing reference,
    LS-cancel its harmonic comb from each range-separated chest candidate, and take the
    residual FFT peak as HR. Self-verify by two-half stability + spectral prominence;
    abstain (verified=False) when no candidate qualifies (colocated / harmonic-locked)."""
    nbin = len(chans)
    def _bin(i): return int(bins[i]) if bins is not None else i
    Pb = np.array([np.mean(bv.bandpass(c, fps, bv.RR_LO, bv.RR_HI) ** 2) for c in chans])
    ab = int(np.argmax(Pb))
    theta = np.unwrap(np.angle(_analytic(bv.bandpass(chans[ab], fps, bv.RR_LO, bv.RR_HI))))
    f_rr = float(np.median(np.gradient(theta)) * fps / (2 * np.pi))
    cols = [np.ones_like(theta)]
    for k in range(1, kmax + 1):
        cols += [np.cos(k * theta), np.sin(k * theta)]
    X = np.vstack(cols).T
    floor = np.median(Pb)
    cand = [i for i in range(nbin) if abs(_bin(i) - _bin(ab)) >= 3 and Pb[i] > 2 * floor]

    def resid_peak(r, lo=1.0, hi=2.2):
        r = bv.bandpass(r, fps, lo, hi)
        f = np.fft.rfftfreq(len(r), 1 / fps); S = np.abs(np.fft.rfft(r)) ** 2
        m = (f >= lo) & (f <= hi); fb, Sb = f[m], S[m]
        pk = int(np.argmax(Sb))
        return fb[pk] * 60, float(Sb[pk] / (np.median(Sb) + 1e-12))

    T = len(theta); h = T // 2
    best = None
    for i in cand:
        beta, *_ = np.linalg.lstsq(X, chans[i], rcond=None)
        hr_full, prom = resid_peak(chans[i] - X @ beta)
        b1, *_ = np.linalg.lstsq(X[:h], chans[i][:h], rcond=None)
        hr1, _ = resid_peak(chans[i][:h] - X[:h] @ b1)
        b2, *_ = np.linalg.lstsq(X[h:], chans[i][h:], rcond=None)
        hr2, _ = resid_peak(chans[i][h:] - X[h:] @ b2)
        stab = abs(hr1 - hr2)
        if stab <= stab_tol and prom >= prom_min and 55 <= hr_full <= 140:
            score = prom / (1 + stab)
            if best is None or score > best["score"]:
                best = dict(hr=hr_full, chest=_bin(i), prom=prom, stab=stab, score=score,
                            hr1=hr1, hr2=hr2)
    if best:
        return dict(hr=best["hr"], verified=True, confidence="HIGH", method="reference-cancel",
                    abdomen=_bin(ab), chest=best["chest"], prom=round(best["prom"], 1),
                    half_bpm=(round(best["hr1"]), round(best["hr2"])), f_rr=f_rr, reason="")
    return dict(hr=None, verified=False, confidence="LOW", method="none", abdomen=_bin(ab),
                chest=None, prom=0.0, half_bpm=None, f_rr=f_rr,
                reason="HR not verifiable: no range-separated pure-breathing reference bin "
                       "with a stable off-harmonic cardiac residual (colocated or harmonic-locked)")
