"""Method B: Prony / linear-prediction super-resolution (per the radar literature:
MUSIC/Prony resolve cardiac from respiration harmonics better than FFT). Per segment,
bandpass to the cardiac band, fit an LP (Prony) pole model, and among the poles in
[1.0-2.5Hz] take the strongest one that is NOT sitting on a breathing harmonic n*f0
(super-resolution separates the cardiac pole from the harmonic poles). Track + detect.

    python method_b_prony.py
"""
import numpy as np
from bcg_vitals import bandpass, sqi, demod_channels, estimate_rr, RR_LO, RR_HI

FPS = 18.78
CLO, CHI = 1.0, 2.5
ORDER = 20


def prony_pole_hr(x, f0):
    x = x - x.mean()
    N = len(x)
    if N < ORDER * 2:
        return None
    # forward linear prediction: x[n] ~ sum_{k=1..O} a_k x[n-k]
    M = np.column_stack([x[ORDER - k:N - k] for k in range(1, ORDER + 1)])
    y = x[ORDER:N]
    a, *_ = np.linalg.lstsq(M, y, rcond=None)
    roots = np.roots(np.concatenate([[1.0], -a]))
    freqs = np.angle(roots) * FPS / (2 * np.pi)
    mags = np.abs(roots)
    cand = []
    for fr, mg in zip(freqs, mags):
        if CLO <= fr <= CHI and 0.85 <= mg <= 1.06:      # in band, near unit circle
            bpm = fr * 60
            # reject poles sitting on a breathing harmonic n*f0
            on_harm = any(abs(bpm - n * f0 * 60) < 4 for n in range(3, 9))
            cand.append((bpm, mg, on_harm))
    if not cand:
        return None
    free = [c for c in cand if not c[2]]                 # non-harmonic poles (cardiac)
    pool = free if free else cand
    pool.sort(key=lambda c: -c[1])                       # strongest (closest to circle)
    return pool[0][0]


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
            x = bandpass(chans[i][s:s + n], FPS, CLO, CHI)
            h = prony_pole_hr(x, f0)
            if h: hrs.append(h)
        traj.append(round(np.median(hrs)) if hrs else 0)
    nz = [x for x in traj if x]
    med = round(np.median(nz)) if nz else 0
    hi = np.mean([x > 100 for x in nz]) if nz else 0
    v = "TACHY" if (med > 110 and hi >= 0.5) else "ok"
    print(f"  {path:16s} true {truth:11s} med={med:>4} hi%={hi:.0%} -> {v:6s} {traj}")


print("Method B: Prony super-resolution, non-harmonic cardiac pole\n")
for p, t in [("tachy2_cube.npz", "131->91 T"), ("sport33_cube.npz", "95->82"),
             ("tachy3_cube.npz", "84-87"), ("sit33_cube.npz", "82"),
             ("sit39_cube.npz", "81"), ("lie41_cube.npz", "77"), ("fall20_cube.npz", "80")]:
    run(p, t)
