"""User's idea: breathing harmonics are generated at the breathing PEAKS / steep parts
(max displacement -> strongest PM nonlinearity). The TROUGH period (end-expiration,
baseline, quiet) has the weakest harmonic contamination, so compute HR from ONLY the
trough segments. Implemented as a MASKED autocorrelation: use only sample pairs where
BOTH ends fall in the breathing trough -> the cardiac period survives, the breathing
harmonics (weak in troughs) do not, and there are no gating-window artifacts.

    python trough_gated.py
"""
import numpy as np
from bcg_vitals import bandpass, sqi, demod_channels, estimate_rr, RR_LO, RR_HI

FPS = 18.78
LO, HI = 1.0, 2.5
TROUGH_PCTL = 40          # keep samples where breathing is in its lowest 40% (trough)


def masked_autocorr_hr(x, mask, fps, lo=LO, hi=HI):
    x = (x - x.mean()) * mask
    num = np.correlate(x, x, "full")[len(x) - 1:]
    den = np.correlate(mask.astype(float), mask.astype(float), "full")[len(x) - 1:]
    ac = num / (den + 1e-9)
    if ac[0] <= 0:
        return None
    ac = ac / ac[0]
    l0, l1 = int(fps / (hi)), int(fps / (lo))
    if l1 <= l0 + 1 or l1 >= len(ac):
        return None
    k = l0 + int(np.argmax(ac[l0:l1]))
    return fps / k * 60


def analyze(path, truth):
    d = np.load(path, allow_pickle=True)
    cube = np.asarray(d["snapshots"], np.complex64)
    counts = d["counts"].astype(int); bins = d["bins"].astype(int)
    C = cube[:, :int(counts.min()), :]
    chans = demod_channels(C, bins)
    _, f0, _, _ = estimate_rr(chans, FPS)
    rr = np.array([sqi(bandpass(c, FPS, RR_LO, RR_HI), FPS, RR_LO, RR_HI) for c in chans])
    top = np.argsort(rr)[::-1][:6]
    breath = bandpass(chans[top[0]], FPS, RR_LO, RR_HI)
    trough = breath < np.percentile(breath, TROUGH_PCTL)     # exhale/baseline half

    n = int(20 * FPS); step = int(10 * FPS)
    traj = []
    for s in range(0, C.shape[1] - n + 1, step):
        hrs = []
        for i in top:
            x = bandpass(chans[i][s:s + n], FPS, LO, HI)
            hr = masked_autocorr_hr(x, trough[s:s + n], FPS)
            if hr: hrs.append(hr)
        traj.append(round(np.median(hrs)) if hrs else 0)
    nz = [x for x in traj if x]
    print(f"  {path:16s} true {truth:12s} trough-HR med={round(np.median(nz)) if nz else '--':>4} "
          f"| {traj}")


print(f"trough-gated masked-autocorr HR (trough = lowest {TROUGH_PCTL}% of breathing):\n")
for p, t in [("tachy2_cube.npz", "131->91 T"), ("sport33_cube.npz", "95->82"),
             ("tachy3_cube.npz", "84-87"), ("sit33_cube.npz", "82"),
             ("sit39_cube.npz", "81"), ("lie41_cube.npz", "77"), ("fall20_cube.npz", "80")]:
    analyze(p, t)
