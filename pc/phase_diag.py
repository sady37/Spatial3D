"""Phase-based diagnostic (the right domain): is the tachy2 cardiac PHASE-COHERENT
after breathing removal, even though its energy is ~noise-floor? Energy spectra mislead
(cardiac = tiny bump); autocorrelation of the phase residual tests COHERENCE, which
noise lacks. Show, for an early (HR~128) and late (HR~95) 25s window: (top) the
breathing-removed phase/displacement time series, (bottom) its autocorrelation with the
true cardiac period marked.
"""
import numpy as np
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

fig, ax = plt.subplots(2, 2, figsize=(15, 8))
for col, (a, b, hrtrue) in enumerate([(0, 25, 128), (95, 120, 93)]):
    i0, i1 = int(a * FPS), int(b * FPS)
    # coherent sum of breathing-removed phase residual across chest bins
    resid = np.zeros(i1 - i0)
    for i in top:
        r = peak_cycle_subtract(chans[i][i0:i1], breath[i0:i1], FPS)
        if r is not None:
            resid = resid + bandpass(r, FPS, 1.0, 2.5)
    resid = resid / (resid.std() + 1e-9)
    t = np.arange(len(resid)) / FPS
    # autocorr
    ac = np.correlate(resid, resid, "full")[len(resid) - 1:]
    ac = ac / ac[0]
    lag = np.arange(len(ac)) / FPS
    true_period = 60.0 / hrtrue

    ax[0, col].plot(t[:int(8 * FPS)], resid[:int(8 * FPS)], lw=0.8)
    ax[0, col].set_title(f"tachy2 {a}-{b}s: breathing-removed PHASE residual (true HR {hrtrue})")
    ax[0, col].set_xlabel("s"); ax[0, col].set_ylabel("norm disp")
    m = lag < 1.4
    ax[1, col].plot(lag[m], ac[m])
    ax[1, col].axvline(true_period, color="lime", lw=1.5, label=f"true period {true_period*1000:.0f}ms ({hrtrue}bpm)")
    # mark the detected autocorr peak in the cardiac lag range
    l0, l1 = int(FPS / (150 / 60)), int(FPS / (60 / 60))
    k = l0 + int(np.argmax(ac[l0:l1]))
    ax[1, col].axvline(lag[k], color="red", ls="--", label=f"peak {60/lag[k]:.0f}bpm (h={ac[k]:.2f})")
    ax[1, col].set_title("autocorrelation of phase residual"); ax[1, col].set_xlabel("lag (s)")
    ax[1, col].legend(fontsize=8)
    print(f"{a}-{b}s (true {hrtrue}): autocorr peak {60/lag[k]:.0f}bpm h={ac[k]:.2f}; "
          f"height at true period {true_period:.3f}s = {ac[int(true_period*FPS)]:.2f}")
plt.tight_layout(); plt.savefig("phase_diag.png", dpi=115)
print("saved phase_diag.png")
