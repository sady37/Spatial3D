"""After RR -> mark harmonics 1..7 (n*f0), notch them narrowly, and look for a SHARP
non-harmonic peak (= heart). Also a spectrogram with the FIXED harmonic comb overlaid:
per user's insight the comb stays put while a real HR ridge drifts ACROSS it.

    python harmonic_map.py
"""
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from bcg_vitals import bandpass, sqi, fft_peak, demod_channels, estimate_rr, RR_LO, RR_HI

CARD_LO, CARD_HI = 1.0, 2.5
NH = 7                      # harmonics 1..7
HW = 0.03                  # narrow notch half-width (Hz)


def sharpness(f, S, floc):
    """Q-like sharpness of the peak nearest floc: peak / local-median over +-0.15Hz."""
    m = np.abs(f - floc) <= 0.15
    if not m.any():
        return 0.0, floc
    seg = S[m]; ff = f[m]
    pk = seg.max(); loc = ff[np.argmax(seg)]
    med = np.median(seg) + 1e-12
    return float(pk / med), float(loc)


def analyze(ax, ax2, path, fps, true_hr):
    d = np.load(path, allow_pickle=True)
    cube = np.asarray(d["snapshots"], dtype=np.complex64)
    counts = d["counts"].astype(int); bins = d["bins"].astype(int)
    C = cube[:, :int(counts.min()), :]
    chans = demod_channels(C, bins)
    rr_sqi = np.array([sqi(bandpass(c, fps, RR_LO, RR_HI), fps, RR_LO, RR_HI)
                       for c in chans])
    top = np.argsort(rr_sqi)[::-1][:5]
    _, f0, _, _ = estimate_rr(chans, fps)          # robust median-of-top-bins f0
    sig = bandpass(chans[top[0]], fps, 0.7, 3.2)

    f = np.fft.rfftfreq(len(sig), 1 / fps)
    S = np.abs(np.fft.rfft(sig - sig.mean()))
    harms = [n * f0 for n in range(1, NH + 1)]

    # notch harmonics 1..7, inspect residual in cardiac band
    Sn = S.copy()
    for h in harms:
        Sn[np.abs(f - h) <= HW] = 0
    band = (f >= CARD_LO) & (f <= CARD_HI)
    fb, Sb = f[band], Sn[band]
    kpk = np.argmax(Sb)
    res_f, res_amp = fb[kpk], Sb[kpk]
    sharp, sloc = sharpness(f, Sn, res_f)

    print(f"\n=== {path}  true {true_hr}  f0={f0:.3f}Hz ({f0*60:.0f}rpm) ===")
    print("  harmonics n*f0 (bpm):", [round(h * 60) for h in harms])
    amps = [S[np.argmin(np.abs(f - h))] for h in harms]
    print("  harmonic amplitudes  :", [round(a, 1) for a in amps])
    print(f"  residual peak after notching 1-7: {res_f*60:.0f}bpm  "
          f"sharpness(Q)={sharp:.1f}  (sharp&non-harmonic => heart)")

    # --- plot spectrum + harmonics ---
    ax.plot(f * 60, S, lw=0.8, color="steelblue")
    for h in harms:
        ax.axvline(h * 60, color="red", ls=":", lw=0.8, alpha=0.6)
    ax.axvline(res_f * 60, color="green", lw=1.5, label=f"residual {res_f*60:.0f}")
    ax.set_xlim(30, 160); ax.set_title(f"{path}\ntrue {true_hr}, f0={f0*60:.0f}rpm")
    ax.set_xlabel("bpm"); ax.legend(fontsize=7)

    # --- spectrogram with fixed comb overlaid ---
    n = int(10 * fps); step = int(2 * fps)
    spec, times = [], []
    for s in range(0, len(sig) - n, step):
        seg = sig[s:s + n]
        ff = np.fft.rfftfreq(n, 1 / fps)
        SS = np.abs(np.fft.rfft(seg - seg.mean()))
        mm = (ff >= 0.7) & (ff <= 2.6)
        spec.append(SS[mm]); times.append(s / fps)
    spec = np.array(spec).T
    ffm = ff[mm]
    ax2.pcolormesh(times, ffm * 60, np.log1p(spec), shading="auto", cmap="magma")
    for h in harms:
        ax2.axhline(h * 60, color="cyan", ls=":", lw=0.6, alpha=0.7)
    ax2.set_ylim(40, 160); ax2.set_xlabel("s"); ax2.set_ylabel("bpm")
    ax2.set_title("spectrogram + fixed comb (cyan)")


fig, axes = plt.subplots(2, 3, figsize=(15, 8))
for j, (path, truth) in enumerate([("tachy2_cube.npz", "128->91 (Q)"),
                                   ("tachy3_cube.npz", "84-87 (S)"),
                                   ("sit39_cube.npz", "~81")]):
    analyze(axes[0, j], axes[1, j], path, 18.78, truth)
plt.tight_layout(); plt.savefig("harmonic_map.png", dpi=110)
print("\nsaved harmonic_map.png")
