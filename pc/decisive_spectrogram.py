"""Decisive visual: does the true HR descent (131->91) appear as a ridge in tachy2's
CHEST bin after breathing removal? Compares raw / spatial breath-null / breathing
template-subtraction spectrograms, with the true trajectory + harmonic comb overlaid.

Info is proven present (bin-65 tracked; resting = TI parity) -> if the chest bin shows
NO descending ridge under any breathing removal, the issue is which processing exposes
it, not SNR.

    python decisive_spectrogram.py
"""
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from scipy.signal import hilbert
from bcg_vitals import bandpass, sqi, demod_channels, estimate_rr, LAMBDA_MM, RR_LO, RR_HI
from spatial_null_probe import per_antenna_disp, breath_subspace
from synced_template_probe import template_subtract

FPS = 18.78
LO, HI = 0.7, 2.6


def spec(sig, fps, win_s=8, step_s=1):
    n = int(win_s * fps); step = int(step_s * fps)
    S, t = [], []
    for s in range(0, len(sig) - n, step):
        seg = sig[s:s + n]
        f = np.fft.rfftfreq(n, 1 / fps)
        P = np.abs(np.fft.rfft(seg - seg.mean()))
        m = (f >= LO) & (f <= HI)
        S.append(P[m]); t.append(s / fps)
    return np.array(S).T, np.array(t), f[m] * 60


d = np.load("tachy2_cube.npz", allow_pickle=True)
cube = np.asarray(d["snapshots"], dtype=np.complex64)
counts = d["counts"].astype(int); bins = d["bins"].astype(int)
C = cube[:, :int(counts.min()), :]
chans = demod_channels(C, bins)
rr_sqi = np.array([sqi(bandpass(c, FPS, RR_LO, RR_HI), FPS, RR_LO, RR_HI) for c in chans])
top = np.argsort(rr_sqi)[::-1][:5]
ci = top[0]                                  # chest bin
_, f0, _, _ = estimate_rr(chans, FPS)
breath = bandpass(chans[top[0]], FPS, RR_LO, RR_HI)
print(f"chest bin {bins[ci]} ({bins[ci]*0.0234375:.2f}m), f0={f0:.3f}Hz")

# three processings of the chest bin
raw = chans[ci]
D = per_antenna_disp(C[ci]); Ub, _, _ = breath_subspace(D, FPS, f0)
snull = (D @ (np.eye(16) - Ub @ Ub.T)).sum(1)
tsub = template_subtract(chans[ci], breath, FPS)

# true HR trajectory (NEXT.md): 0-60s 131->110 ; 60-120s 109->91  (approx linear)
def truth(t):
    return np.where(t <= 60, 131 - (131 - 110) * t / 60, 110 - (110 - 91) * (t - 60) / 60)

fig, axes = plt.subplots(1, 3, figsize=(16, 5))
for ax, sig, name in [(axes[0], raw, "raw chest"),
                      (axes[1], snull, "spatial breath-null"),
                      (axes[2], tsub, "breathing template-sub")]:
    S, t, fb = spec(sig, FPS)
    ax.pcolormesh(t, fb, np.log1p(S), shading="auto", cmap="magma")
    for n in range(2, 8):
        ax.axhline(n * f0 * 60, color="cyan", ls=":", lw=0.6, alpha=0.6)
    ax.plot(t, truth(t), color="lime", lw=2.0, label="true HR")
    ax.set_ylim(40, 155); ax.set_xlabel("s"); ax.set_ylabel("bpm")
    ax.set_title(name); ax.legend(fontsize=8, loc="upper right")
plt.suptitle("tachy2 (Q) true 131->91 | green=truth, cyan=fixed breathing comb")
plt.tight_layout(); plt.savefig("decisive_spectrogram.png", dpi=115)
print("saved decisive_spectrogram.png")
