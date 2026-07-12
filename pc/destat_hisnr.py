"""Quantify the lead: in the destationarized spectrogram, look ONLY at high-SNR frames
(where a peak clearly stands out) and check whether their argmax-bpm tracks the true
descending HR. If tachy2's confident frames cluster near 131->91 and S/sit39 near ~85,
the info is present and the miss is purely a tracking/gating problem (user is right)."""
import numpy as np
from bcg_vitals import bandpass, sqi, demod_channels, RR_LO, RR_HI

FPS = 18.78
LO, HI = 0.9, 2.5


def destat_frames(path):
    d = np.load(path, allow_pickle=True)
    cube = np.asarray(d["snapshots"], dtype=np.complex64)
    counts = d["counts"].astype(int); bins = d["bins"].astype(int)
    C = cube[:, :int(counts.min()), :]
    chans = demod_channels(C, bins)
    rr = np.array([sqi(bandpass(c, FPS, RR_LO, RR_HI), FPS, RR_LO, RR_HI) for c in chans])
    top = np.argsort(rr)[::-1][:5]
    n = int(8 * FPS); step = int(1 * FPS)
    f = np.fft.rfftfreq(n, 1 / FPS); m = (f >= LO) & (f <= HI); bpm = f[m] * 60
    Ss, tv = [], []
    for s in range(0, int(counts.min()) - n, step):
        P = np.zeros(m.sum())
        for i in top:
            seg = chans[i][s:s + n]
            P += np.abs(np.fft.rfft(seg - seg.mean())) ** 2 [m] if False else \
                 (np.abs(np.fft.rfft(seg - seg.mean())) ** 2)[m]
        Ss.append(P); tv.append(s / FPS)
    S = np.array(Ss).T                                   # (F,T)
    R = np.clip(S - np.median(S, axis=1, keepdims=True), 0, None)
    return R, np.array(tv), bpm


def report(path, truth_fn, label):
    R, tv, bpm = destat_frames(path)
    # per frame: peak bpm and its prominence (peak / median of column)
    peak_bpm = bpm[np.argmax(R, 0)]
    prom = R.max(0) / (np.median(R, 0) + 1e-9)
    # confident frames = top 30% prominence
    thr = np.percentile(prom, 70)
    conf = prom >= thr
    err = np.abs(peak_bpm[conf] - truth_fn(tv[conf]))
    print(f"\n{path}  ({label})")
    print(f"  confident frames: {conf.sum()}/{len(tv)}  "
          f"median |peak-truth| = {np.median(err):.0f} bpm  "
          f"(within10bpm: {np.mean(err<10):.0%})")
    # binned peak bpm in confident frames vs truth
    for a, b in [(0, 30), (30, 60), (60, 90), (90, 120)]:
        w = conf & (tv >= a) & (tv < b)
        if w.any():
            print(f"    t[{a:>3}-{b:<3}]  radar-peak median {np.median(peak_bpm[w]):.0f}  "
                  f"true {truth_fn(np.array([(a+b)/2]))[0]:.0f}  (n={w.sum()})")


report("tachy2_cube.npz", lambda t: np.where(t <= 60, 131 - 21 * t / 60, 110 - 19 * (t - 60) / 60), "Q true 131->91")
report("tachy1_cube.npz", lambda t: 120 - 25 * t / 120, "P true >110 desc")
report("tachy3_cube.npz", lambda t: np.full_like(t, 85.0), "S true 84-87 flat")
report("sit39_cube.npz", lambda t: np.full_like(t, 81.0), "true 81 flat")
