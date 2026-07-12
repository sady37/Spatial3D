"""Time-frequency energy map (spectrogram) of HR/RR for tachy3 (~2.2m, normal, true
84-87). Left: raw spectrogram (breathing fundamental + harmonic comb + cardiac).
Middle: per-bpm-row destationarized (stationary breathing comb cancels, moving cardiac
survives). Right: whole-record spectrum with harmonics marked + true-HR band.
Overlays: cyan = breathing harmonics n*f0 ; green = true HR (85).
"""
import numpy as np
import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
from bcg_vitals import bandpass, sqi, demod_channels, estimate_rr, RR_LO, RR_HI

FPS = 18.78
LO_BPM, HI_BPM = 8, 150
PATH, TRUE = "tachy3_cube.npz", 85


def spectrogram(chans, top, win_s=10, hop_s=1):
    n = int(win_s * FPS); hop = int(hop_s * FPS)
    f = np.fft.rfftfreq(n, 1 / FPS); bpm = f * 60
    m = (bpm >= LO_BPM) & (bpm <= HI_BPM)
    S, tv = [], []
    for s in range(0, chans.shape[1] - n + 1, hop):
        P = np.zeros(m.sum())
        for i in top:
            seg = chans[i][s:s + n]
            P += (np.abs(np.fft.rfft(seg - seg.mean())) ** 2)[m]
        S.append(P); tv.append((s + n / 2) / FPS)
    return np.array(S).T, np.array(tv), bpm[m]


d = np.load(PATH, allow_pickle=True)
cube = np.asarray(d["snapshots"], np.complex64)
counts = d["counts"].astype(int); bins = d["bins"].astype(int)
C = cube[:, :int(counts.min()), :]
chans = demod_channels(C, bins)
_, f0, _, _ = estimate_rr(chans, FPS)
rr = np.array([sqi(bandpass(c, FPS, RR_LO, RR_HI), FPS, RR_LO, RR_HI) for c in chans])
top = np.argsort(rr)[::-1][:6]
S, tv, bpm = spectrogram(chans, top)
R = np.clip(S - np.median(S, axis=1, keepdims=True), 0, None)
harms = [k * f0 * 60 for k in range(1, 10) if LO_BPM <= k * f0 * 60 <= HI_BPM]
print(f"tachy3 f0={f0*60:.0f}rpm; harmonics(bpm)={[round(h) for h in harms]}; true HR={TRUE} "
      f"(=n{TRUE/(f0*60):.1f})")

fig, ax = plt.subplots(1, 3, figsize=(17, 5))
for a, (M, lab) in zip(ax[:2], [(S, "raw spectrogram"), (R, "destat (breathing removed)")]):
    a.pcolormesh(tv, bpm, np.log1p(M), shading="auto", cmap="magma")
    for h in harms:
        a.axhline(h, color="cyan", ls=":", lw=0.7, alpha=0.7)
    a.axhline(TRUE, color="lime", lw=1.8, label=f"true HR {TRUE}")
    a.set_ylim(LO_BPM, HI_BPM); a.set_xlabel("s"); a.set_ylabel("bpm"); a.set_title(lab)
    a.legend(fontsize=8, loc="upper right")
# whole-record spectrum
sig = np.zeros(C.shape[1])
for i in top:
    sig = sig + bandpass(chans[i], FPS, 0.13, 2.6)
f = np.fft.rfftfreq(len(sig), 1 / FPS); Sp = np.abs(np.fft.rfft(sig - sig.mean()))
mb = (f * 60 >= LO_BPM) & (f * 60 <= HI_BPM)
ax[2].plot(f[mb] * 60, Sp[mb], color="steelblue")
for h in harms:
    ax[2].axvline(h, color="cyan", ls=":", lw=0.8, alpha=0.7)
ax[2].axvline(TRUE, color="lime", lw=1.8, label=f"true HR {TRUE}")
ax[2].set_xlim(LO_BPM, HI_BPM); ax[2].set_xlabel("bpm"); ax[2].set_title("whole-record spectrum")
ax[2].legend(fontsize=8)
plt.suptitle(f"tachy3 (~2.2m, normal, true {TRUE}) — HR/RR energy map (f0={f0*60:.0f}rpm, cyan=harmonics)")
plt.tight_layout(); plt.savefig("energy_map.png", dpi=115)
print("saved energy_map.png")
