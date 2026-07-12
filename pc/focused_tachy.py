"""Focus on the 3 RELIABLE-ground-truth cubes (others had inaccurate truth):
  tachy2  ~2.1m  true 131->91  (near, tachy)   -- must READ HIGH + track descent
  tachy3  ~2.2m  true 84-87    (near, normal)  -- must NOT be tachy
  sport33 ~3.3m  true 101->106->82 (far, mild) -- elevated early, recovering

Refine the Bessel-excess method: fit beta from clean low harmonics n=1..4, predict the
breathing comb, and take the cardiac-band harmonic n* whose POSITIVE excess is a LOCAL
MAX (a real bump, not the centroid -> fixes the high bias). Parabolic-interpolate the
excess across n*-1,n*,n*+1 for a sub-harmonic HR (so the sweep is tracked, not snapped
to n*f0). Per 15s segment.

    python focused_tachy.py
"""
import numpy as np
from scipy.special import jv
from scipy.optimize import minimize_scalar
from bcg_vitals import bandpass, sqi, demod_channels, estimate_rr, RR_LO, RR_HI

FPS = 18.78


def harm_amps(z, f0, nh=9):
    z = z - z.mean()
    f = np.fft.fftfreq(len(z), 1 / FPS)
    S = np.abs(np.fft.fft(z))
    return np.array([float(S[np.abs(f - n * f0) <= 0.04].max()) if (np.abs(f - n * f0) <= 0.04).any()
                     else 0.0 for n in range(1, nh + 1)])


def fit_beta(A14):
    n = np.arange(1, len(A14) + 1); logA = np.log(A14 + 1e-9)
    def err(b):
        Jn = np.abs(jv(n, b)) + 1e-9
        return np.sum((logA - (np.log(Jn) + np.mean(logA - np.log(Jn)))) ** 2)
    b = minimize_scalar(err, bounds=(0.5, 13), method="bounded").x
    Jn = np.abs(jv(np.arange(1, 10), b))
    c = np.exp(np.mean(logA - np.log(np.abs(jv(n, b)) + 1e-9)))
    return c * Jn


def seg_hr(z, f0):
    A = harm_amps(z, f0)
    if (A[:4] > 0).sum() < 3:
        return None
    pred = fit_beta(A[:4])
    exc = A - pred                                    # cardiac excess (signed)
    # candidate harmonics in cardiac band (95-138bpm for f0~.3 -> n where n*f0 in [1.4,2.35])
    ns = [n for n in range(4, 9) if 1.4 <= n * f0 <= 2.35]
    best = None
    for n in ns:
        i = n - 1
        if exc[i] <= 0.15 * A.max():
            continue
        if exc[i] >= exc[i - 1] and exc[i] >= exc[i + 1]:   # LOCAL MAX bump
            # parabolic sub-harmonic refine on the excess
            y0, y1, y2 = exc[i - 1], exc[i], exc[i + 1]
            denom = y0 - 2 * y1 + y2
            dn = 0.5 * (y0 - y2) / denom if abs(denom) > 1e-9 else 0.0
            dn = np.clip(dn, -0.5, 0.5)
            if best is None or exc[i] > best[1]:
                best = ((n + dn) * f0 * 60, exc[i])
    return best[0] if best else None


def run(path, truth):
    d = np.load(path, allow_pickle=True)
    cube = np.asarray(d["snapshots"], np.complex64)
    counts = d["counts"].astype(int); bins = d["bins"].astype(int)
    C = cube[:, :int(counts.min()), :]
    chans = demod_channels(C, bins)
    _, f0, _, _ = estimate_rr(chans, FPS)
    rr = np.array([sqi(bandpass(c, FPS, RR_LO, RR_HI), FPS, RR_LO, RR_HI) for c in chans])
    top = np.argsort(rr)[::-1][:6]
    n = int(15 * FPS); step = int(10 * FPS)
    traj = []
    for s in range(0, C.shape[1] - n + 1, step):
        hrs = []
        for i in top:
            X = C[i, s:s + n]; m = X.mean(0); m /= (np.linalg.norm(m) + 1e-9)
            h = seg_hr(X @ m.conj(), f0)
            if h: hrs.append(h)
        traj.append(round(np.median(hrs)) if hrs else 0)
    nz = [x for x in traj if x]
    med = round(np.median(nz)) if nz else 0
    slope = 0
    if len(nz) > 3:
        tt = np.arange(len(traj))[np.array(traj) > 0]
        slope = round(np.polyfit(tt, nz, 1)[0] * len(traj))
    print(f"  {path:16s} f0={f0*60:.0f} true {truth:14s} med={med:>4} slope={slope:+d} | {traj}")


print("Focused (3 reliable cubes): Bessel local-bump + sub-harmonic refine\n")
run("tachy2_cube.npz", "131->91 TACHY")
run("tachy3_cube.npz", "84-87 normal")
run("sport33_cube.npz", "101->82 mild")
