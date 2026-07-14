"""Apply the RR-anchored chest-cluster HR method to MANY radar cubes and see if it
outputs DIVERSE, truth-tracking values (real) or collapses to 1-2 fixed grid values
(artifact). Resting cubes use band [1.0,1.7]; tachy cubes get a wide [1.4,2.4] pass.

    python3 hr_method_survey.py
"""
import numpy as np
from bcg_vitals import demod_channels, bandpass, autocorr_peak, RR_LO, RR_HI

CUBES = [  # (file, truth or None, is_tachy)
    ("sit33_cube.npz", 82, False), ("sit39_cube.npz", 81, False),
    ("lie41_cube.npz", 77, False), ("fall20_cube.npz", 80, False),
    ("sidesit_cube.npz", None, False), ("sport33_cube.npz", None, False),
    ("tachy1_cube.npz", None, True), ("tachy2_cube.npz", None, True),
    ("tachy3_cube.npz", None, True),
    ("emptyL_cube.npz", 0, False), ("emptychair_20260713_192151.npz", 0, False),
]
FPS = 18.78


def load(path):
    d = np.load(path, allow_pickle=True)
    C = np.asarray(d["snapshots"], np.complex64)[:, :int(d["counts"].astype(int).min()), :]
    bins = d["bins"].astype(int)
    try:
        ts = np.asarray(d["frame_ts"], float)[:C.shape[1]]; s = ts[-1] - ts[0]
        fps = (len(ts) - 1) / (s / 1000 if s > 1e4 else s)
    except KeyError:
        fps = FPS
    return C, bins, fps


def hr_cluster(disp, fps, f0):
    N = disp.shape[1]; f = np.fft.rfftfreq(N, 1 / fps)
    rr = np.array([bandpass(d, fps, RR_LO, 0.6).std() for d in disp])
    above = np.where(rr > 0.15 * rr.max())[0]
    lo_b, hi_b = int(above.min()), int(above.max())

    def cs(i):
        d = disp[i]; X = 2 * np.abs(np.fft.rfft(d - d.mean())) / N
        band = (f >= 1.0) & (f <= 1.7); nh = np.zeros_like(f, bool)
        for k in range(1, 12):
            nh |= np.abs(f - k * f0) <= 0.035
        c = band & ~nh
        return X[np.where(c)[0][np.argmax(X[c])]] / (np.median(X[band & ~nh]) + 1e-9)
    csn = {i: cs(i) for i in range(lo_b, hi_b + 1)}
    hot = max(csn, key=csn.get)
    return [i for i in range(hot - 2, hot + 3) if lo_b <= i <= hi_b], rr, csn[hot]


def hr_windows(disp, chidx, fps, lo, hi):
    sigs = [bandpass(disp[c], fps, lo, hi) for c in chidx]
    N = disp.shape[1]; W = int(30 * fps); vals = []
    for s in range(0, N - W + 1, int(15 * fps)):
        hv = [autocorr_peak(sg[s:s + W], fps, int(lo * 60), int(hi * 60), interp=True)[0]
              for sg in sigs]
        hv = sorted(v for v in hv if v)
        if hv:
            k = len(hv); l = max(1, k // 4) if k >= 3 else 0
            vals.append(float(np.mean(hv[l:k - l] if k - 2 * l >= 1 else hv)))
    return np.array(vals)


def main():
    print("RR-anchored chest-cluster HR across cubes (interp; do values SPREAD or stick?)\n")
    print(f"  {'cube':16} {'RR':>4} {'chest bins':>12} {'cSNR':>5} {'HR med':>7} {'HR std':>6} "
          f"{'truth':>6} {'err':>5}")
    meds = []
    for path, truth, tachy in CUBES:
        try:
            C, bins, fps = load(path)
        except FileNotFoundError:
            print(f"  {path:16} (missing)"); continue
        disp = demod_channels(C, bins); N = disp.shape[1]; f = np.fft.rfftfreq(N, 1 / fps)
        rr_amp = [bandpass(x, fps, RR_LO, 0.6).std() for x in disp]
        Xr = np.abs(np.fft.rfft(bandpass(disp[int(np.argmax(rr_amp))], fps, RR_LO, RR_HI)))
        m = (f >= RR_LO) & (f <= RR_HI); f0 = f[m][np.argmax(Xr[m])]
        chidx, rr, csnr = hr_cluster(disp, fps, f0)
        lo, hi = (1.4, 2.4) if tachy else (1.0, 1.7)
        v = hr_windows(disp, chidx, fps, lo, hi)
        if len(v) == 0:
            print(f"  {path.replace('_cube.npz',''):16} (no HR)"); continue
        med, std = np.median(v), np.std(v)
        cb = f"{bins[chidx[0]]}-{bins[chidx[-1]]}"
        err = "" if truth is None else f"{med-truth:+.0f}"
        tag = " [tachy band]" if tachy else ("" if truth != 0 else " [NULL]")
        print(f"  {path.replace('_cube.npz','').replace('.npz',''):16} {f0*60:4.0f} {cb:>12} "
              f"{csnr:5.1f} {med:7.1f} {std:6.1f} {str(truth) if truth is not None else '?':>6} {err:>5}{tag}")
        if truth not in (0, None):
            meds.append(med)
    if meds:
        print(f"\n  resting HR spread across cubes: {min(meds):.0f}–{max(meds):.0f} bpm "
              f"(range {max(meds)-min(meds):.0f}) — if it were a fixed artifact this would be ~0")


if __name__ == "__main__":
    main()
