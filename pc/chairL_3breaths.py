"""Plot 3 complete breathing cycles from chairL to SEE the waveform shape / asymmetry.
Shows: clean breathing (drift removed, cardiac removed) + the broadband (cardiac riding
on top). Marks troughs/peaks, annotates inspiration(rise) vs expiration(fall) durations.

    python3 chairL_3breaths.py
"""
import numpy as np
from scipy.signal import find_peaks
from bcg_vitals import demod_channels, bandpass, sqi, RR_LO, RR_HI

BLOCK = "chairL_20260713_183514.npz"


def load(path):
    d = np.load(path, allow_pickle=True)
    cube = np.asarray(d["snapshots"], np.complex64)
    counts = d["counts"].astype(int); bins = d["bins"].astype(int)
    C = cube[:, :int(counts.min()), :]
    ts = np.asarray(d["frame_ts"], float)[:int(counts.min())]
    span = ts[-1] - ts[0]
    if span > 1e4:
        span /= 1000.0
    return C, bins, (len(ts) - 1) / span


def main():
    C, bins, fps = load(BLOCK)
    chans = demod_channels(C, bins)
    resp_sqi = np.array([sqi(bandpass(c, fps, RR_LO, RR_HI), fps, RR_LO, RR_HI)
                         for c in chans])
    rb = int(np.argmax(resp_sqi))
    shape = bandpass(chans[rb], fps, 0.1, 0.8)      # breathing shape (up to ~4 harmonics)
    broad = bandpass(chans[rb], fps, 0.1, 2.6)      # breathing + cardiac ripple

    tr, _ = find_peaks(-shape, distance=int(fps / RR_HI))
    pk, _ = find_peaks(shape,  distance=int(fps / RR_HI))

    # find 3 consecutive, most-regular breaths (trough_i -> trough_i+3)
    best, bestvar = 0, 1e9
    for j in range(len(tr) - 3):
        seg = np.diff(tr[j:j + 4])
        v = np.std(seg) / (np.mean(seg) + 1e-9)
        if v < bestvar:
            bestvar, best = v, j
    a, b = tr[best], tr[best + 3]
    t = (np.arange(a, b) - a) / fps
    sh = shape[a:b]; br = broad[a:b]
    trs = tr[(tr >= a) & (tr <= b)]
    pks = pk[(pk >= a) & (pk <= b)]

    print(f"{BLOCK}: breathing bin {bins[rb]} ({bins[rb]*0.0234375:.2f}m), "
          f"fps={fps:.2f}, showing troughs {trs} (samples)")
    # per-cycle rise/fall
    anc = sorted([(i, 'tr') for i in trs] + [(i, 'pk') for i in pks])
    for k in range(len(anc) - 1):
        (i0, t0), (i1, t1) = anc[k], anc[k + 1]
        if t0 != t1:
            dur = (i1 - i0) / fps
            print(f"  {t0}->{t1}: {dur:.2f}s  ({'inspiration/rise' if t0=='tr' else 'expiration/fall'})")

    try:
        import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
        fig, ax = plt.subplots(2, 1, figsize=(11, 7), sharex=True)
        ax[0].plot(t, sh, "C0", lw=2, label="breathing shape (0.1-0.8Hz)")
        ax[0].plot((trs - a) / fps, shape[trs], "v", color="C3", ms=10, label="trough (start inhale)")
        ax[0].plot((pks - a) / fps, shape[pks], "^", color="C2", ms=10, label="peak (start exhale)")
        for i in trs:
            ax[0].axvline((i - a) / fps, color="grey", ls=":", lw=.8)
        # shade inspiration vs expiration
        for k in range(len(anc) - 1):
            (i0, t0), (i1, t1) = anc[k], anc[k + 1]
            if t0 != t1 and a <= i0 and i1 <= b:
                ax[0].axvspan((i0 - a) / fps, (i1 - a) / fps,
                              color=("C2" if t0 == 'tr' else "C1"), alpha=.08)
        ax[0].set_ylabel("chest displacement (mm)")
        ax[0].set_title("3 complete breaths — chairL (green=inhale/rise, orange=exhale/fall)")
        ax[0].legend(loc="upper right", fontsize=8)
        ax[1].plot(t, br, "C7", lw=1, label="broadband (breathing + cardiac 0.1-2.6Hz)")
        ax[1].plot(t, sh, "C0", lw=2, alpha=.6, label="breathing only")
        ax[1].set_ylabel("displacement (mm)")
        ax[1].legend(loc="upper right", fontsize=8)
        # cardiac = broadband - breathing shape, band-limited to cardiac
        card = bandpass(broad[a:b], fps, 0.9, 2.0)
        cpk, _ = find_peaks(card, distance=int(fps / (110 / 60)))
        bpm = len(cpk) / (t[-1] - t[0]) * 60
        ax2 = ax[1].twinx()
        ax2.plot(t, card, "C3", lw=1.2, alpha=.7)
        ax2.plot(t[cpk], card[cpk], "rx", ms=8)
        ax2.set_ylabel("cardiac ripple (mm)", color="C3")
        ax2.tick_params(axis='y', labelcolor="C3")
        ax[1].set_xlabel("time (s)")
        ax[1].set_title(f"cardiac ripple (red, right axis): {len(cpk)} beats in "
                        f"{t[-1]-t[0]:.1f}s = {bpm:.0f} bpm")
        print(f"cardiac beats counted: {len(cpk)} in {t[-1]-t[0]:.1f}s -> {bpm:.0f} bpm")
        fig.tight_layout(); fig.savefig("chairL_3breaths.png", dpi=120)
        print("saved chairL_3breaths.png")
    except Exception as e:
        print("plot skipped:", e)


if __name__ == "__main__":
    main()
