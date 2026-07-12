"""Crack tachy2 via the user's insight: the cardiac shows as EXCESS over the breathing
decay. The clean low harmonics (n=1..4, below the cardiac band -> pure breathing) fix
the breathing envelope; per short segment, whichever cardiac-band harmonic (n=5,6,7 =
95,114,133) most EXCEEDS that envelope marks the current HR. As HR sweeps 131->91 the
excess should walk n=7 -> n=6 -> n=5. Sub-harmonic interpolation refines within.

    python tachy2_excess_track.py
"""
import numpy as np
from bcg_vitals import bandpass, sqi, demod_channels, estimate_rr, LAMBDA_MM, RR_LO, RR_HI

FPS = 18.78


def truth(t):
    return np.where(t <= 60, 131 - 21 * t / 60, 110 - 19 * (t - 60) / 60)


def seg_excess_hr(disp, f0):
    disp = disp - disp.mean()
    f = np.fft.rfftfreq(len(disp), 1 / FPS)
    S = np.abs(np.fft.rfft(disp))
    A = np.array([float(S[np.abs(f - n * f0) <= 0.05].max()) if (np.abs(f - n * f0) <= 0.05).any()
                  else 1e-9 for n in range(1, 9)])
    n14 = np.arange(1, 5)
    good = A[:4] > 0
    if good.sum() < 2:
        return None
    c = np.polyfit(n14[good], np.log(A[:4][good]), 1)
    base = np.exp(np.polyval(c, np.arange(1, 9)))
    exc = A / base
    # cardiac band harmonics n=5,6,7 (95,114,133); pick max-excess, refine by excess-weighting
    ns = np.array([5, 6, 7])
    e = exc[ns - 1]
    if e.max() < 1.3:                              # no real excess this segment
        return None
    # excess-weighted centroid of the harmonic freqs -> sub-harmonic HR
    w = np.clip(e - 1.0, 0, None)
    hr = np.sum(w * ns * f0 * 60) / (w.sum() + 1e-9)
    return hr


def run(path, truelabel):
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
        hrs = [seg_excess_hr(chans[i][s:s + n], f0) for i in top]
        hrs = [h for h in hrs if h]
        traj.append(round(np.median(hrs)) if hrs else 0)
    nz = [x for x in traj if x]
    frac_hi = np.mean([x > 100 for x in nz]) if nz else 0.0
    verdict = "TACHY" if (nz and np.median(nz) > 100 and frac_hi >= 0.5) else "not-tachy"
    print(f"  {path:16s} true {truelabel:12s} f0={f0*60:.0f} "
          f"med={round(np.median(nz)) if nz else '--':>4} hi%={frac_hi:.0%} "
          f"segs={len(nz)}/{len(traj)} -> {verdict}   {traj}")


print("excess-harmonic tachy detector across cubes:\n")
run("tachy2_cube.npz", "131->91 T")
run("tachy3_cube.npz", "84-87")
run("sit33_cube.npz", "82")
run("sport33_cube.npz", "95->82")
run("sit39_cube.npz", "81")
run("lie41_cube.npz", "77")
run("fall20_cube.npz", "80")
