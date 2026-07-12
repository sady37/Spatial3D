"""Look, don't guess: is the tachy2 cardiac visible in a SHORT (quasi-stationary)
segment after breathing removal? The full-record HR sweeps 131->91 so any whole-record
autocorr smears. Per segment the HR is ~stationary and (early ~128=2.13Hz) sits in the
GAP between breathing harmonics 7 and 8. Show the residual spectrum with harmonics +
true HR marked, for an early (high) and late (low) segment.
"""
import numpy as np
import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
from bcg_vitals import bandpass, sqi, demod_channels, estimate_rr, RR_LO, RR_HI
from peak_cycle_probe import peak_cycle_subtract

FPS = 18.78


def seg_spectrum(chans, top, breath, i0, i1, f0):
    S = None; f = None
    for i in top:
        resid = peak_cycle_subtract(chans[i][i0:i1], breath[i0:i1], FPS)
        if resid is None:
            resid = chans[i][i0:i1]
        rb = bandpass(resid, FPS, 0.9, 2.6)
        f = np.fft.rfftfreq(len(rb), 1 / FPS)
        P = np.abs(np.fft.rfft(rb - rb.mean())) ** 2
        S = P if S is None else S + P
    return f, S


d = np.load("tachy2_cube.npz", allow_pickle=True)
cube = np.asarray(d["snapshots"], np.complex64)
counts = d["counts"].astype(int); bins = d["bins"].astype(int)
C = cube[:, :int(counts.min()), :]
chans = demod_channels(C, bins)
_, f0, _, _ = estimate_rr(chans, FPS)
rr = np.array([sqi(bandpass(c, FPS, RR_LO, RR_HI), FPS, RR_LO, RR_HI) for c in chans])
top = np.argsort(rr)[::-1][:6]
breath = bandpass(chans[top[0]], FPS, RR_LO, RR_HI)
print(f"f0={f0:.3f}Hz ({f0*60:.0f}rpm); harmonics(bpm): {[round(n*f0*60) for n in range(4,9)]}")

fig, axes = plt.subplots(1, 2, figsize=(14, 5))
for ax, (a, b, lab, tru) in zip(axes, [(0, 25, "early 0-25s", (125, 131)),
                                        (95, 120, "late 95-120s", (91, 96))]):
    f, S = seg_spectrum(chans, top, breath, int(a * FPS), int(b * FPS), f0)
    m = (f >= 0.9) & (f <= 2.6); fb, Sb = f[m] * 60, S[m]
    ax.plot(fb, Sb, color="steelblue")
    for n in range(4, 9):
        ax.axvline(n * f0 * 60, color="cyan", ls=":", lw=0.8, alpha=0.7)
    ax.axvspan(tru[0], tru[1], color="lime", alpha=0.25, label=f"true HR {tru[0]}-{tru[1]}")
    # top-3 peaks
    order = np.argsort(Sb)[::-1]
    picks = []
    for o in order:
        if all(abs(fb[o] - fb[p]) > 6 for p in picks):
            picks.append(o)
        if len(picks) >= 3: break
    for o in picks:
        ax.annotate(f"{fb[o]:.0f}", (fb[o], Sb[o]), fontsize=9, color="red")
    ax.set_title(f"tachy2 {lab}  (cyan=harm, green=true)"); ax.set_xlabel("bpm")
    ax.legend(fontsize=8)
    print(f"{lab}: true {tru}, top-3 residual peaks (bpm) = {[round(fb[o]) for o in picks]}")
plt.tight_layout(); plt.savefig("tachy2_separate.png", dpi=115)
print("saved tachy2_separate.png")
