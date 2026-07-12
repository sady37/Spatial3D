"""Method A: sin^k (exponentiated-cosine) respiration-waveform model (per the radar
literature: evenly-exponentiated sines capture the asymmetric, harmonic-rich breathing
profile). Fit the shape power p + scale to the CLEAN low harmonics (n=1..4, cardiac-
free), predict the breathing harmonic comb at ALL n, subtract from the measured
spectrum, and read the RESIDUAL SPECTRAL PEAK in the cardiac band (not a harmonic
centroid -> fixes the Bessel high-bias). Per 20s segment; track + detect.

    python method_a_sink.py
"""
import numpy as np
from scipy.optimize import minimize_scalar
from bcg_vitals import bandpass, sqi, demod_channels, estimate_rr, RR_LO, RR_HI

FPS = 18.78
CLO, CHI = 1.0, 2.4


def model_harm(p, nh=8, N=1024):
    """Harmonic magnitudes of an exponentiated-cosine breathing waveform (peaked,
    harmonic-rich; power p sets how many harmonics)."""
    t = np.linspace(0, 1, N, endpoint=False)
    b = (0.5 + 0.5 * np.cos(2 * np.pi * t)) ** p
    b = b - b.mean()
    S = np.abs(np.fft.rfft(b))
    return S[1:nh + 1]


# precompute model harmonic patterns over a grid of p
_PGRID = np.linspace(0.5, 8, 40)
_MODELS = {round(p, 3): model_harm(p) for p in _PGRID}


def fit_and_excess(A):
    """A = measured harmonic amps n=1..8. Fit p to n=1..4, predict comb, subtract."""
    logA = np.log(A[:4] + 1e-9)
    best = (1e18, None)
    for p, M in _MODELS.items():
        m = M[:4]; lm = np.log(m + 1e-9)
        c = np.mean(logA - lm)                       # log-scale
        e = np.sum((logA - (lm + c)) ** 2)
        if e < best[0]:
            best = (e, (p, c))
    p, c = best[1]
    pred = np.exp(c) * _MODELS[p]                     # predicted breathing comb n=1..8
    resid = np.clip(A - pred, 0, None)
    return resid, p


def seg_hr(z, f0):
    z = z - z.mean()
    f = np.fft.rfftfreq(len(z), 1 / FPS)
    S = np.abs(np.fft.rfft(z))
    A = np.array([float(S[np.abs(f - n * f0) <= 0.04].max()) if (np.abs(f - n * f0) <= 0.04).any()
                  else 1e-9 for n in range(1, 9)])
    if (A[:4] > 0).sum() < 3:
        return None
    resid, p = fit_and_excess(A)
    # build a residual spectrum: keep S but subtract predicted breathing at each n*f0
    Sr = S.copy()
    for n in range(1, 9):
        mask = np.abs(f - n * f0) <= 0.04
        if mask.any():
            Sr[mask] = np.clip(Sr[mask] - (A[n - 1] - resid[n - 1]), 0, None)
    band = (f >= CLO) & (f <= CHI)
    if not band.any() or Sr[band].max() <= 0:
        return None
    return f[band][np.argmax(Sr[band])] * 60


def run(path, truth):
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
            h = seg_hr(chans[i][s:s + n], f0)         # real displacement (sin^k domain)
            if h: hrs.append(h)
        traj.append(round(np.median(hrs)) if hrs else 0)
    nz = [x for x in traj if x]
    med = round(np.median(nz)) if nz else 0
    hi = np.mean([x > 100 for x in nz]) if nz else 0
    v = "TACHY" if (med > 110 and hi >= 0.5) else "ok"
    print(f"  {path:16s} true {truth:11s} med={med:>4} hi%={hi:.0%} -> {v:6s} {traj}")


print("Method A: sin^k respiration model subtract + residual peak\n")
for p, t in [("tachy2_cube.npz", "131->91 T"), ("sport33_cube.npz", "95->82"),
             ("tachy3_cube.npz", "84-87"), ("sit33_cube.npz", "82"),
             ("sit39_cube.npz", "81"), ("lie41_cube.npz", "77"), ("fall20_cube.npz", "80")]:
    run(p, t)
