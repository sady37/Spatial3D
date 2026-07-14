"""Is the JKL template a real folded cardiac morphology, or noise fitted by the fold?
Decisive tests on sit33 (resting, the template's own source):
  A) split-half reproducibility: fold 1st half vs 2nd half -> correlation
  B) null: fold at an INCOMMENSURATE (non-cardiac) frequency -> spurious template
  C) epoch-folding periodogram: real-f variance vs null-f variance (SNR of the fold)
"""
import numpy as np
from bcg_vitals import demod_channels, bandpass, sqi, estimate_rr, RR_LO, RR_HI

SHAPE_LO = 0.7

def _profile(s, f, fps, N=64):
    t = np.arange(len(s)) / fps
    ph = (f * t) % 1.0
    idx = np.minimum((ph * N).astype(int), N - 1)
    prof = np.zeros(N); cnt = np.zeros(N)
    np.add.at(prof, idx, s); np.add.at(cnt, idx, 1)
    prof = prof / np.maximum(cnt, 1)
    return prof - prof.mean()

def best_f(s, fps, flo, fhi, nf=800, N=64):
    fs = np.linspace(flo, fhi, nf)
    var = np.array([np.var(_profile(s, f, fps, N)) for f in fs])
    return fs[int(np.argmax(var))], var, fs

def load_chest(path, fps, t0=0, t1=None):
    d = np.load(path, allow_pickle=True)
    cube = np.asarray(d["snapshots"], np.complex64)
    counts = d["counts"].astype(int); bins = d["bins"].astype(int)
    C = cube[:, :int(counts.min()), :]
    chans = demod_channels(C, bins)
    _, f0, _, _ = estimate_rr(chans, fps)
    rr = np.array([sqi(bandpass(c, fps, RR_LO, RR_HI), fps, RR_LO, RR_HI) for c in chans])
    idx = int(np.argsort(rr)[::-1][0])
    i0 = int(t0*fps); i1 = int(t1*fps) if t1 else chans.shape[1]
    shape_hi = min(7.0, fps/2 - 0.5)
    s = bandpass(chans[idx][i0:i1], fps, SHAPE_LO, shape_hi, notch_f0=f0)
    return s / (s.std()+1e-9), f0

def corr(a, b):
    a = a - a.mean(); b = b - b.mean()
    return float(np.dot(a, b) / (np.linalg.norm(a)*np.linalg.norm(b) + 1e-9))

for path, fps, flo, fhi, tag in [("sit33_cube.npz", 20.0, 1.2, 1.7, "sit33 RESTING (template source)"),
                                  ("tachy2_cube.npz", 18.78, 1.9, 2.3, "tachy2 0-30s TACHY")]:
    print("\n===== %s =====" % tag)
    t1 = 30 if "tachy2" in path else None
    s, f0 = load_chest(path, fps, 0, t1)
    f, var, fs = best_f(s, fps, flo, fhi)
    dur = len(s)/fps
    print("  folded f = %.3fHz = %.1f bpm   (%d cardiac cycles in %.0fs)"
          % (f, f*60, int(dur*f), dur))

    # C) epoch-folding SNR: peak variance vs median (null) variance across the search
    fold_snr = var.max() / np.median(var)
    print("  epoch-folding periodogram: peakVar/medVar = %.2f  (>~3 = real periodicity)"
          % fold_snr)

    # A) split-half reproducibility at the SAME f
    half = len(s)//2
    g1 = _profile(s[:half], f, fps); g2 = _profile(s[half:], f, fps)
    print("  A) split-half fold correlation = %.2f   (>0.7 = reproducible morphology)"
          % corr(g1, g2))

    # B) null: fold at an incommensurate freq (golden-ratio off the HR, still in-band-ish)
    fnull = f * 1.313
    gN = _profile(s, fnull, fps); gR = _profile(s, f, fps)
    print("  B) null-freq(%.2fHz) fold var = %.4f  vs  real-freq fold var = %.4f  (ratio %.1fx)"
          % (fnull, np.var(gN), np.var(gR), np.var(gR)/(np.var(gN)+1e-9)))

    # harmonic content of the folded template (this is what becomes the fingerprint)
    N = 64; phi = np.arange(N)/N*2*np.pi
    hmag = [np.hypot(2/N*np.sum(gR*np.cos(m*phi)), 2/N*np.sum(gR*np.sin(m*phi))) for m in range(1,5)]
    hmag = np.array(hmag); hmag = hmag/hmag[0]
    print("  fingerprint from THIS fold |h1..h4| = %s" % np.round(hmag,3))
