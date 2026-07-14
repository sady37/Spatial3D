"""Measure the ACTUAL amplitudes (mm) on the co-located chest bin:
  - RR fundamental and its harmonics A_k at k*RR (the decay envelope)
  - HR amplitude A_HR
and answer: at the HR frequency, is the RR-harmonic level > or < HR?  (i.e. is HR
buried under a decaying harmonic, or does it poke above where harmonics have died?)

Uses demod mm displacement; spectral amplitude = 2|rfft|/N (physical mm per component).

    python3 breath_hr_amplitude.py [cube.npz] [truth_bpm] [t0] [t1]
"""
import sys
import numpy as np
from bcg_vitals import demod_channels, bandpass, sqi, beat_count, RR_LO, RR_HI


def load(path):
    d = np.load(path, allow_pickle=True)
    cube = np.asarray(d["snapshots"], np.complex64)
    counts = d["counts"].astype(int); bins = d["bins"].astype(int)
    C = cube[:, :int(counts.min()), :]
    try:
        ts = np.asarray(d["frame_ts"], float)[:int(counts.min())]
        span = ts[-1] - ts[0]
        fps = (len(ts) - 1) / (span / 1000 if span > 1e4 else span)
    except KeyError:
        fps = 18.78
    return C, bins, fps


def amp_spectrum(x, fps):
    x = x - x.mean()
    X = 2 * np.abs(np.fft.rfft(x)) / len(x)          # mm per sinusoidal component
    f = np.fft.rfftfreq(len(x), 1 / fps)
    return f, X


def peak_near(f, X, f0, df):
    m = (f >= f0 - df) & (f <= f0 + df)
    return float(X[m].max()) if m.any() else 0.0


def main():
    path = sys.argv[1] if len(sys.argv) > 1 else "sit39_cube.npz"
    truth = float(sys.argv[2]) if len(sys.argv) > 2 else 81.0
    t0 = float(sys.argv[3]) if len(sys.argv) > 3 else 0.0
    t1 = float(sys.argv[4]) if len(sys.argv) > 4 else 1e9
    C, bins, fps = load(path)
    i0, i1 = int(t0 * fps), min(C.shape[1], int(t1 * fps))
    C = C[:, i0:i1, :]
    chans = demod_channels(C, bins)
    T = chans.shape[1]

    # co-located chest bin: strong breathing (RR & HR co-located per memory)
    resp_sqi = np.array([sqi(bandpass(c, fps, RR_LO, RR_HI), fps, RR_LO, RR_HI) for c in chans])
    ci = int(np.argmax(resp_sqi))
    x = chans[ci]
    f, X = amp_spectrum(x, fps)

    # RR fundamental
    mrr = (f >= RR_LO) & (f <= RR_HI)
    f0 = f[mrr][np.argmax(X[mrr])]
    rr = f0 * 60
    df = 0.03                                        # tolerance ~2 bpm
    A = {k: peak_near(f, X, k * f0, df) for k in range(1, 9)}
    fHR = truth / 60.0
    A_HR = peak_near(f, X, fHR, 0.05)
    # HR cross-check by beat-count in a clean sub-band
    hr_bc = beat_count(bandpass(x, fps, 0.9, 2.0), fps, hi_bpm=130)

    print(f"{path} [t={t0:.0f}..{min(t1,T/fps+t0):.0f}s] chest bin {bins[ci]} "
          f"({bins[ci]*0.0234375:.2f}m), {T} frames @{fps:.2f}fps")
    print(f"  RR = {rr:.1f} rpm (f0={f0:.3f}Hz)   HR truth {truth:.0f} (beat-count {hr_bc:.0f})\n")
    print(f"  {'k':>2} {'freq(bpm)':>9} {'A_k(um)':>9} {'A_k/A_1':>8}   in cardiac band?")
    for k in range(1, 9):
        fb = k * rr
        note = ""
        if 48 <= fb <= 150:
            note = f"<-- vs A_HR={A_HR*1000:.1f}um: harmonic {'>' if A[k] > A_HR else '<'} HR"
        print(f"  {k:>2} {fb:9.0f} {A[k]*1000:9.1f} {A[k]/(A[1]+1e-12):8.2f}   {note}")

    # decay law fit (log-linear over k=1..6)
    ks = np.array([k for k in range(1, 7) if A[k] > 0])
    ak = np.array([A[k] for k in ks])
    slope = np.polyfit(ks, np.log(ak), 1)[0]
    print(f"\n  RR fundamental A_1 = {A[1]*1000:.0f} um ; HR A_HR = {A_HR*1000:.1f} um "
          f"-> RR/HR = {A[1]/(A_HR+1e-12):.0f}x")
    print(f"  harmonic decay ~ exp({slope:.2f}*k)  (each harmonic x{np.exp(slope):.2f} of previous)")
    # at HR freq, interpolate the harmonic envelope level
    env_at_HR = np.exp(np.polyval(np.polyfit(ks, np.log(ak), 1), fHR / f0))
    print(f"  harmonic ENVELOPE extrapolated to HR freq ({fHR/f0:.1f}xRR) = {env_at_HR*1000:.1f} um")
    print(f"  => at HR freq: envelope {env_at_HR*1000:.1f}um  vs  HR {A_HR*1000:.1f}um  -> "
          f"HR {'ABOVE' if A_HR > env_at_HR else 'BELOW'} the decaying harmonic floor")

    try:
        import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
        fig, ax = plt.subplots(figsize=(11, 5))
        ax.semilogy(f * 60, X * 1000, "C7", lw=.8, label="amplitude spectrum (um)")
        for k in range(1, 9):
            fb = k * rr
            if fb <= 160:
                ax.plot(fb, A[k] * 1000, "vC0", ms=9)
                ax.annotate(f"{k}xRR", (fb, A[k]*1000), fontsize=8, ha="center")
        ax.plot(truth, A_HR * 1000, "^C3", ms=13, label=f"HR {truth:.0f} ({A_HR*1000:.1f}um)")
        kk = np.linspace(1, 8, 50)
        ax.plot(kk * rr, np.exp(np.polyval(np.polyfit(ks, np.log(ak), 1), kk)) * 1000,
                "C1--", lw=1.5, label="harmonic decay envelope")
        ax.axvspan(48, 150, color="C3", alpha=.06)
        ax.set_xlim(0, 160); ax.set_xlabel("bpm"); ax.set_ylabel("amplitude (um, log)")
        ax.set_title(f"{path}: RR harmonics vs HR amplitude (co-located chest bin)")
        ax.legend(fontsize=9)
        fig.tight_layout(); fig.savefig("breath_hr_amplitude.png", dpi=115)
        print("  saved breath_hr_amplitude.png")
    except Exception as e:
        print("plot skipped:", e)


if __name__ == "__main__":
    main()
