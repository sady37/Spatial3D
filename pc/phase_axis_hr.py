"""THE PHASE / TEMPORAL AXIS attack on RR-harmonic entanglement (2026-07-14).

Context: HR extraction dies two ways — (1) FREQUENCY axis when HR==k*RR (cardiac line
coincident with a breathing harmonic; no FFT/autocorr/notch splits them), (2) SPATIAL
axis when chest/abdomen are colocated in range (no range-decouple; see
bcg_vitals.chest_decoupled_hr, works only on RADIAL geometry). User's lead: a THIRD axis
— PHASE/TIME. A breathing harmonic is phase-LOCKED to the breath (deterministic waveform
-> harmonic k has phase exactly k*theta(t)); the cardiac is an INDEPENDENT oscillator,
phase-incoherent with breathing. So use the respiratory PHASE CLOCK theta(t) to remove
everything breathing-locked and see if the cardiac survives.

Distinct from tachy2_phasefold (FAILED): that fit a warped-time breathing WAVEFORM and
subtracted it. Here the removal is by the phase clock — exact-by-construction for any
harmonic shape. Two forms implemented:
  harmonic_residual()  = JOINT least-squares projection onto {cos k*theta, sin k*theta}
                         (the OPTIMAL linear synchronous canceller; strongest form)
  lockin_hr()          = per-k heterodyne exp(-i k*theta) -> harmonic->DC, cardiac->Delta_f

===================  RESULT: NEGATIVE for the hard cell (documented)  ===================
Tested vs Apple-Watch truth + an EMPTY-ROOM null (the honest control the earlier work
lacked). The phase clock IS a better harmonic canceller than the frequency notch, and on
RADIAL geometry it recovers HR as well as the spatial method (lie_long 70~=71, chairL
75~=80, residual prominence above the empty null). BUT it provides NO new escape for the
cell that matters — COLOCATED and/or ELEVATED HR, where frequency+space already died:
  - lie_short (colocated resting 71-77): median 72 is right but prominence == empty-null
    level -> indistinguishable from noise, not trustworthy.
  - sitR       (colocated + ELEVATED 96-110): locks LOW 70-84, same failure as all others.
  - sit33/tachy2 (near range 2-3.3m): best-PLV/prom lines miss truth (60-76 vs 81; 61 vs
    131->91). Even at 1.5-2.5m PLV maxes ~0.20 vs a 0.10 empty-null floor, and the most
    COHERENT surviving line is the imperfectly-cancelled harmonic RESIDUE, not the cardiac.
Root cause = the SAME SNR wall (hr-tracking-noise-limited): after removing the ~10x
stronger breathing-locked component, the ~4m cardiac phase is below the residual floor,
so "most coherent residual line" picks harmonic leakage, not the heart. The phase axis
joins frequency and space as an SNR-limited dead end for the colocated/elevated cell.
The ONLY validated escape remains SPATIAL range-decouple on RADIAL geometry.
=========================================================================================

    python phase_axis_hr.py            # runs the full case table + empty null
"""
import sys, numpy as np
sys.path.insert(0, 'pc')
try:
    import bcg_vitals as bv
except ModuleNotFoundError:
    sys.path.insert(0, '.'); import bcg_vitals as bv

RR_LO, RR_HI = bv.RR_LO, bv.RR_HI


def _analytic_band(c, fps, lo, hi):
    x = c - c.mean()
    f = np.fft.rfftfreq(len(x), 1 / fps)
    X = np.fft.rfft(x); X[(f < lo) | (f > hi)] = 0
    n = len(x); full = np.zeros(n, complex); full[:len(X)] = X
    if n % 2 == 0: full[1:len(X) - 1] *= 2
    else:          full[1:len(X)] *= 2
    return np.fft.ifft(full)


def resp_phase(chans, fps):
    """Respiratory phase clock theta(t) = Hilbert phase of the strongest breathing bin."""
    Pb = np.array([(np.abs(np.fft.rfft(bv.bandpass(c, fps, RR_LO, RR_HI))) ** 2).sum()
                   for c in chans])
    ab = int(np.argmax(Pb))
    theta = np.unwrap(np.angle(_analytic_band(chans[ab], fps, RR_LO, RR_HI)))
    dth = np.diff(theta) * fps / (2 * np.pi)
    return theta, float(np.median(dth[dth > 0.05])), ab, Pb


def harmonic_residual(c, theta, Kmax=22):
    """OPTIMAL synchronous canceller: LS-project c onto {1, cos k*theta, sin k*theta}
    for k=1..Kmax and return the residual (everything breathing-locked removed)."""
    cols = [np.ones_like(theta)]
    for k in range(1, Kmax + 1):
        cols += [np.cos(k * theta), np.sin(k * theta)]
    X = np.vstack(cols).T
    beta, *_ = np.linalg.lstsq(X, c, rcond=None)
    return c - X @ beta


def hr_from_residual(r, fps, lo=1.0, hi=2.5):
    x = r - r.mean()
    f = np.fft.rfftfreq(len(x), 1 / fps)
    S = np.abs(np.fft.rfft(x)) ** 2
    m = (f >= lo) & (f <= hi)
    return f[m][np.argmax(S[m])] * 60, float(S[m].max() / (np.median(S[m]) + 1e-12))


def lockin_hr(c, fps, theta, f_rr, k, band_hw=0.35, dc_notch=0.05):
    """Per-k heterodyne: harmonic->DC (removed), cardiac->Delta_f. Returns (hr, plv)."""
    fc = k * f_rr
    if fc < 0.8 or fc > 2.6: return None
    a = _analytic_band(c, fps, fc - band_hw, fc + band_hw)
    p = a * np.exp(-1j * k * theta); p = p - p.mean()
    f = np.fft.fftfreq(len(p), 1 / fps); P = np.abs(np.fft.fft(p)) ** 2
    m = (np.abs(f) > dc_notch) & (np.abs(f) < band_hw)
    idx = np.where(m)[0]; j = idx[np.argmax(P[idx])]; delta = f[j]
    hr = (fc + delta) * 60
    if not (55 <= hr <= 140): return None
    t = np.arange(len(p)) / fps
    q = p * np.exp(-1j * 2 * np.pi * delta * t)
    return hr, float(np.abs(np.mean(q / (np.abs(q) + 1e-12))))


def analyze(path, fps, use_chest=True):
    d = np.load(path, allow_pickle=True)
    cube = np.asarray(d['snapshots'], dtype=np.complex64)
    bins = d['bins'].astype(int); K = int(d['counts'].astype(int).min())
    chans = bv.demod_channels(cube[:, :K, :], bins)
    theta, f_rr, ab, Pb = resp_phase(chans, fps)
    cand = [i for i in range(len(chans))
            if 0.01 * Pb[ab] < Pb[i] < 0.5 * Pb[ab] and abs(bins[i] - bins[ab]) >= 3]
    sel = (sorted(cand, key=lambda i: -Pb[i])[:6] if (use_chest and cand) else [ab])
    hrs = [(int(bins[i]),) + hr_from_residual(harmonic_residual(chans[i], theta), fps)
           for i in sel]
    return hrs, f_rr, int(bins[ab])


CASES = [
    ("pc/case/chairL_sit_20260713_225001.npz", "80 near-radial ctrl"),
    ("pc/case/lie_long_20260714.npz", "69-73 RADIAL (works)"),
    ("pc/case/lie_short_20260714.npz", "71-77 COLOCATED"),
    ("pc/case/sitR_172500.npz", "96-110 COLOC+ELEVATED"),
    ("pc/case/sit33_cube.npz", "81 @3.3m"),
    ("pc/case/tachy2_cube.npz", "131->91 @2.2m dyn"),
]

if __name__ == "__main__":
    fps = 18.78
    nh, _, _ = analyze("pc/case/empty_20260714.npz", fps)
    null_prom = np.median([p for _, _, p in nh])
    print(f"EMPTY-NULL residual prominence (median) = {null_prom:.1f}  "
          f"[a real cardiac line must clearly beat this]\n")
    for path, truth in CASES:
        hrs, f_rr, ab = analyze(path, fps)
        med = np.median([h for _, h, _ in hrs])
        top = sorted(hrs, key=lambda r: -r[2])[:3]
        flag = "" if max(p for _, _, p in hrs) > 2 * null_prom else "  <= NULL-LEVEL"
        print(f"{path.split('/')[-1]:32s} RR={f_rr*60:4.1f}  med-HR={med:5.0f}  "
              f"TRUTH {truth}{flag}")
        for b, h, p in top:
            print(f"      bin{b} HR={h:6.1f}  prom={p:5.1f}")
