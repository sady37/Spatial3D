"""Spectrogram destationarize + DP ridge track. Implements the user's two insights:
  - harmonics are TIME-CONSTANT (horizontal lines) -> subtract each frequency row's
    time-median: stationary comb vanishes, a MOVING (descending) HR ridge survives.
  - HR trajectory is SMOOTH/descending -> Viterbi track with a continuity prior
    integrates a faint-but-consistent ridge that per-frame argmax misses.

Test: tachy2/P should yield a DESCENDING track near 131->91; S/sit39 a FLAT ~85/81.

    python destat_track.py
"""
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from bcg_vitals import bandpass, sqi, demod_channels, RR_LO, RR_HI

FPS = 18.78
LO, HI = 0.9, 2.5           # cardiac band (exclude <0.9 breathing residue)


def spectrogram(sig, fps, win_s=8, step_s=1):
    n = int(win_s * fps); step = int(step_s * fps)
    S, t = [], []
    for s in range(0, len(sig) - n, step):
        seg = sig[s:s + n]
        f = np.fft.rfftfreq(n, 1 / fps)
        P = np.abs(np.fft.rfft(seg - seg.mean())) ** 2
        S.append(P); t.append(s / fps)
    f = np.fft.rfftfreq(n, 1 / fps)
    m = (f >= LO) & (f <= HI)
    return np.array(S).T[m], np.array(t), f[m] * 60


def dp_track(R, bpm, lam=0.4):
    """Viterbi ridge: maximize sum(R) - lam*|df| over a smooth path. R:(F,T)>=0."""
    F, T = R.shape
    Rn = R / (R.max() + 1e-12)
    cost = np.full((F, T), -1e9); back = np.zeros((F, T), int)
    cost[:, 0] = Rn[:, 0]
    step = np.abs(bpm[:, None] - bpm[None, :])          # (F,F) bpm distance
    for t in range(1, T):
        prev = cost[:, t - 1][None, :] - lam * step     # (F_cur, F_prev)
        back[:, t] = np.argmax(prev, 1)
        cost[:, t] = Rn[:, t] + prev[np.arange(F), back[:, t]]
    path = np.zeros(T, int); path[-1] = np.argmax(cost[:, -1])
    for t in range(T - 1, 0, -1):
        path[t - 1] = back[path[t], t]
    return bpm[path]


def analyze(ax, path, true_desc):
    d = np.load(path, allow_pickle=True)
    cube = np.asarray(d["snapshots"], dtype=np.complex64)
    counts = d["counts"].astype(int); bins = d["bins"].astype(int)
    C = cube[:, :int(counts.min()), :]
    chans = demod_channels(C, bins)
    rr_sqi = np.array([sqi(bandpass(c, FPS, RR_LO, RR_HI), FPS, RR_LO, RR_HI) for c in chans])
    top = np.argsort(rr_sqi)[::-1][:5]

    # sum spectrograms of the top chest bins (coherent-ish energy pool)
    Ssum = None; bpm = tvec = None
    for i in top:
        S, tvec, bpm = spectrogram(chans[i], FPS)
        Ssum = S if Ssum is None else Ssum + S
    # destationarize: remove each frequency row's time-median (kills fixed comb)
    R = Ssum - np.median(Ssum, axis=1, keepdims=True)
    R = np.clip(R, 0, None)
    track = dp_track(R, bpm)
    slope = np.polyfit(tvec, track, 1)[0] * 60          # bpm change over full record
    print(f"  {path:18s} track: start {track[:8].mean():.0f} -> end {track[-8:].mean():.0f} bpm "
          f"| median {np.median(track):.0f} | slope {slope:+.0f}bpm/rec ({true_desc})")

    ax.pcolormesh(tvec, bpm, np.log1p(R), shading="auto", cmap="magma")
    ax.plot(tvec, track, color="lime", lw=1.5)
    ax.set_ylim(LO * 60, HI * 60); ax.set_title(path.replace("_cube.npz", ""))
    ax.set_xlabel("s"); ax.set_ylabel("bpm")


fig, axes = plt.subplots(1, 4, figsize=(18, 4.5))
for ax, (p, desc) in zip(axes, [("tachy2_cube.npz", "Q true 131->91 DESC"),
                                ("tachy1_cube.npz", "P true >110 DESC"),
                                ("tachy3_cube.npz", "S true 84-87 FLAT"),
                                ("sit39_cube.npz", "true 81 FLAT")]):
    analyze(ax, p, desc)
plt.suptitle("destationarized spectrogram (comb removed) + DP ridge track (green)")
plt.tight_layout(); plt.savefig("destat_track.png", dpi=115)
print("saved destat_track.png")
