"""Morphology-based per-beat detection on the PHASE residual (user: the J-K-L beat
complex repeats and is visible per-beat, even when autocorr locks a lower rhythm).
Detect each beat by its sharp deflection, mark them, and get HR from inter-beat
intervals -- not from spectrum/autocorr. Show early (true 128) and late (true 93).
"""
import numpy as np
from scipy.signal import find_peaks
import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
from bcg_vitals import bandpass, sqi, demod_channels, estimate_rr, RR_LO, RR_HI
from peak_cycle_probe import peak_cycle_subtract

FPS = 18.78

d = np.load("tachy2_cube.npz", allow_pickle=True)
cube = np.asarray(d["snapshots"], np.complex64)
counts = d["counts"].astype(int); bins = d["bins"].astype(int)
C = cube[:, :int(counts.min()), :]
chans = demod_channels(C, bins)
_, f0, _, _ = estimate_rr(chans, FPS)
rr = np.array([sqi(bandpass(c, FPS, RR_LO, RR_HI), FPS, RR_LO, RR_HI) for c in chans])
top = np.argsort(rr)[::-1][:6]
breath = bandpass(chans[top[0]], FPS, RR_LO, RR_HI)


def residual(i0, i1):
    r = np.zeros(i1 - i0)
    for i in top:
        rr_ = peak_cycle_subtract(chans[i][i0:i1], breath[i0:i1], FPS)
        if rr_ is not None:
            r = r + bandpass(rr_, FPS, 1.0, 2.8)
    return r / (r.std() + 1e-9)


def detect_beats(s, r_min=0.5):
    """Template-matched beats + PER-BEAT correlation gate (user: morphology INVARIANT
    while rate changes). Seed strong beats -> learn template -> matched-filter (catch
    weak beats) -> keep only candidates whose local waveform CORRELATES with the
    template (>= r_min) -> reject spurious noise matches. Refractory 0.4s (keep higher r)."""
    dist = int(FPS / (150 / 60))
    sp, _ = find_peaks(s, distance=dist, prominence=1.0)
    sn, _ = find_peaks(-s, distance=dist, prominence=1.0)
    sign = 1 if len(sp) >= len(sn) else -1
    ss = sign * s
    seeds = sp if sign == 1 else sn
    w = int(0.20 * FPS)
    wins = [ss[p - w:p + w] for p in seeds if p - w >= 0 and p + w < len(ss)]
    if len(wins) < 3:
        return seeds
    templ = np.mean(wins, 0); templ = templ - templ.mean()
    tn = templ / (np.linalg.norm(templ) + 1e-9)
    mf = np.correlate(ss - ss.mean(), templ, "same")
    cand, _ = find_peaks(mf, distance=int(0.30 * FPS), prominence=np.std(mf) * 0.25)
    # per-beat correlation with the template
    kept = []
    for p in cand:
        if p - w < 0 or p + w >= len(ss):
            continue
        seg = ss[p - w:p + w]; seg = seg - seg.mean()
        r = float(seg @ tn) / (np.linalg.norm(seg) + 1e-9)
        if r >= r_min:
            kept.append((p, r))
    # refractory: enforce min 0.4s, keep the higher-r beat in conflicts
    kept.sort()
    out = []
    for p, r in kept:
        if out and (p - out[-1][0]) < dist:
            if r > out[-1][1]:
                out[-1] = (p, r)
        else:
            out.append((p, r))
    return np.array([p for p, r in out])


def period_from_ibi(ibi, lo=0.42, hi=0.85):
    """True beat period = the T whose integer multiples best explain the IBIs
    (missed beats give 2T,3T; robust GCD-like estimate)."""
    if len(ibi) < 3:
        return 0
    Ts = np.linspace(lo, hi, 300)
    scores = [np.median(np.abs(ibi - np.round(ibi / T) * T)) for T in Ts]
    return 60 / Ts[int(np.argmin(scores))]


fig, ax = plt.subplots(2, 1, figsize=(15, 8))
for row, (a, b, true) in enumerate([(0, 30, 128), (95, 120, 93)]):
    i0, i1 = int(a * FPS), int(b * FPS)
    s = residual(i0, i1)
    t = np.arange(len(s)) / FPS
    pk = detect_beats(s)
    ibi = np.diff(pk) / FPS
    ibi = ibi[(ibi > 0.35) & (ibi < 1.1)]
    hr_med = 60 / np.median(ibi) if len(ibi) else 0
    hr_comb = period_from_ibi(ibi)                      # missed-beat-robust
    ax[row].plot(t, s, lw=0.8)
    ax[row].plot(pk / FPS, s[pk], "rv", ms=7,
                 label=f"beats: median HR={hr_med:.0f}, comb HR={hr_comb:.0f}")
    ax[row].set_xlim(0, 15); ax[row].set_title(f"tachy2 {a}-{b}s (true {true}) — template-matched beats")
    ax[row].set_xlabel("s"); ax[row].legend(fontsize=9)
    print(f"{a}-{b}s true {true}: {len(pk)} beats | median {hr_med:.0f} | "
          f"COMB {hr_comb:.0f}bpm | IBIs ms {[round(x*1000) for x in ibi[:12]]}")
plt.tight_layout(); plt.savefig("beat_morph.png", dpi=115)
print("saved beat_morph.png")
