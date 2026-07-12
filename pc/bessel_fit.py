"""User's insight: for a fixed waveform the harmonic decay is a FIXED FORMULA. For the
radar PM signal z = A*e^{j*beta*sin(2pi f0 t)} the breathing harmonics are EXACTLY
|J_n(beta)| (Bessel). Fit beta from the clean low harmonics (n=1..4, cardiac-free since
HR>90bpm=n>=5), predict the breathing comb J_5,6,7, subtract -> residual = cardiac.
Works on the COMPLEX signal (where Bessel applies), not the unwrapped displacement.

    python bessel_fit.py
"""
import numpy as np
from scipy.special import jv
from scipy.optimize import minimize_scalar
from bcg_vitals import bandpass, sqi, demod_channels, estimate_rr, RR_LO, RR_HI

FPS = 18.78


def harm_amps_complex(z, f0, nh=8):
    """Harmonic magnitudes of the COMPLEX signal at n*f0 (two-sided -> use +n)."""
    z = z - z.mean()
    f = np.fft.fftfreq(len(z), 1 / FPS)
    S = np.abs(np.fft.fft(z))
    return np.array([float(S[np.abs(f - n * f0) <= 0.04].max())
                     if (np.abs(f - n * f0) <= 0.04).any() else 0.0 for n in range(1, nh + 1)])


def fit_beta(A14):
    """Fit beta so c*|J_n(beta)| matches A_1..A_4 (log-domain, scale-free)."""
    n = np.arange(1, len(A14) + 1)
    logA = np.log(A14 + 1e-9)
    def err(beta):
        Jn = np.abs(jv(n, beta)) + 1e-9
        c = np.mean(logA - np.log(Jn))               # best log-scale
        return np.sum((logA - (np.log(Jn) + c)) ** 2)
    r = minimize_scalar(err, bounds=(0.5, 14), method="bounded")
    beta = r.x
    Jn = np.abs(jv(n, beta))
    c = np.exp(np.mean(logA - np.log(Jn + 1e-9)))
    return beta, c


def analyze(path, truelabel):
    d = np.load(path, allow_pickle=True)
    cube = np.asarray(d["snapshots"], np.complex64)
    counts = d["counts"].astype(int); bins = d["bins"].astype(int)
    C = cube[:, :int(counts.min()), :]
    chans = demod_channels(C, bins)
    _, f0, _, _ = estimate_rr(chans, FPS)
    rr = np.array([sqi(bandpass(c, FPS, RR_LO, RR_HI), FPS, RR_LO, RR_HI) for c in chans])
    top = np.argsort(rr)[::-1][:6]

    n = int(20 * FPS); step = int(10 * FPS)
    traj = []
    for s in range(0, C.shape[1] - n + 1, step):
        hrs = []
        for i in top:
            X = C[i, s:s + n]; m = X.mean(0); m /= (np.linalg.norm(m) + 1e-9)
            z = X @ m.conj()
            A = harm_amps_complex(z, f0)
            if (A[:4] > 0).sum() < 3:
                continue
            beta, c = fit_beta(A[:4])
            pred = c * np.abs(jv(np.arange(1, 9), beta))    # predicted breathing comb
            resid = A - pred                                # cardiac excess (can be <0)
            ns = np.array([5, 6, 7, 8])                     # cardiac band (95-152 for f0~.3)
            e = np.clip(resid[ns - 1], 0, None)
            if e.max() <= 0.15 * A[ns - 1].max():           # no real excess
                continue
            hr = np.sum(e * ns) / (e.sum() + 1e-9) * f0 * 60
            hrs.append(hr)
        traj.append(round(np.median(hrs)) if hrs else 0)
    nz = [x for x in traj if x]
    frac_hi = np.mean([x > 100 for x in nz]) if nz else 0
    verdict = "TACHY" if (nz and np.median(nz) > 100 and frac_hi >= 0.5) else "not-tachy"
    print(f"  {path:16s} true {truelabel:12s} f0={f0*60:.0f} beta~fit "
          f"med={round(np.median(nz)) if nz else '--':>4} hi%={frac_hi:.0%} "
          f"-> {verdict:9s}  {traj}")


print("Bessel-comb fit + cardiac-excess tachy detector:\n")
for p, t in [("tachy2_cube.npz", "131->91 T"), ("tachy3_cube.npz", "84-87"),
             ("sport33_cube.npz", "95->82"), ("sit33_cube.npz", "82"),
             ("sit39_cube.npz", "81"), ("lie41_cube.npz", "77"), ("fall20_cube.npz", "80")]:
    analyze(p, t)
