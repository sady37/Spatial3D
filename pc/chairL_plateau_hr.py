"""RESTING HR via micro-breath-hold harvesting + spectral folding (user design):
 1. SPATIAL: pool all breathing bins -> one high-SNR breathing reference (freq/phase/amp).
 2. TEMPORAL GATE: keep only the LONG SLOW edge of each breath (|velocity|<k) = per-cycle
    natural mini breath-hold where breathing is momentarily still -> cardiac unobscured.
 3. FOLD: breathing is slow -> many plateau windows; stack them (Lomb-Scargle over the
    retained samples = many windows folded into one fine spectrum) -> lift cardiac SNR.

Sidesteps RR-harmonic entanglement by SELECTING the quiet moments, not subtracting.
Target = the true RESTING HR value (chairL oximeter ~77), with empty-chair as null.

    python3 chairL_plateau_hr.py [block.npz] [truth_bpm]
"""
import sys
import numpy as np
from scipy.signal import find_peaks, lombscargle
from bcg_vitals import demod_channels, bandpass, sqi, autocorr_peak, beat_count, RR_LO, RR_HI

HR_LO, HR_HI = 0.9, 2.2          # 54-132 bpm search
K_VEL = 0.5                      # plateau = |breath velocity| < K_VEL * median|v|


def load(path):
    d = np.load(path, allow_pickle=True)
    cube = np.asarray(d["snapshots"], np.complex64)
    counts = d["counts"].astype(int); bins = d["bins"].astype(int)
    C = cube[:, :int(counts.min()), :]
    try:
        ts = np.asarray(d["frame_ts"], float)[:int(counts.min())]
        span = ts[-1] - ts[0]
        if span > 1e4:
            span /= 1000.0
        fps = (len(ts) - 1) / span
    except KeyError:
        fps = 18.78
    return C, bins, fps


def pool(chans, sqis, band, fps, topk=8):
    """SQI-weighted, sign-aligned combine of the top-SQI bins -> one channel."""
    top = np.argsort(sqis)[::-1][:topk]
    ref = bandpass(chans[top[0]], fps, *band)
    acc = np.zeros_like(ref)
    for i in top:
        b = bandpass(chans[i], fps, *band)
        s = np.sign(np.dot(b, ref) + 1e-12)
        acc += s * sqis[i] * b
    return acc / (np.sum(sqis[top]) + 1e-12), top


def main():
    block = sys.argv[1] if len(sys.argv) > 1 else "chairL_20260713_183514.npz"
    truth = float(sys.argv[2]) if len(sys.argv) > 2 else 77.0
    t0 = float(sys.argv[3]) if len(sys.argv) > 3 else 0.0
    t1 = float(sys.argv[4]) if len(sys.argv) > 4 else 1e9
    C, bins, fps = load(block)
    i0, i1 = int(t0 * fps), min(C.shape[1], int(t1 * fps))
    C = C[:, i0:i1, :]
    chans = demod_channels(C, bins)
    T = chans.shape[1]; t = np.arange(T) / fps
    print(f"[window t={t0:.0f}..{min(t1,C.shape[1]/fps+t0):.0f}s]")

    resp_sqi = np.array([sqi(bandpass(c, fps, RR_LO, RR_HI), fps, RR_LO, RR_HI) for c in chans])
    hr_sqi = np.array([sqi(bandpass(c, fps, HR_LO, HR_HI), fps, HR_LO, HR_HI) for c in chans])

    # 1. SPATIAL: pooled breathing reference + pooled cardiac channel
    breath, rtop = pool(chans, resp_sqi, (0.1, 0.8), fps)
    card,  ctop = pool(chans, hr_sqi, (HR_LO, HR_HI), fps)
    tr, _ = find_peaks(-breath, distance=int(fps / RR_HI))
    rr = len(tr) / (T / fps) * 60
    print(f"{block}: {T} frames @ {fps:.2f}fps ({T/fps:.0f}s), truth~{truth:.0f}")
    print(f" 1. SPATIAL: pooled {len(rtop)} resp bins (RR={rr:.1f} rpm) + {len(ctop)} cardiac bins")

    # 2. TEMPORAL GATE: long-slow-edge plateau (breathing momentarily still)
    v = np.gradient(breath) * fps
    thr = K_VEL * np.median(np.abs(v))
    # choose the LONG edge = the velocity sign whose low-|v| samples span longer
    quiet = np.abs(v) < thr
    long_neg = quiet & (v < 0); long_pos = quiet & (v > 0)
    plateau = long_neg if long_neg.sum() >= long_pos.sum() else long_pos
    plateau |= quiet & (np.abs(v) < 0.25 * thr)          # include near-still turning zones
    duty = plateau.mean()
    print(f" 2. GATE: plateau (long slow edge, |v|<{K_VEL}median) -> "
          f"{plateau.sum()} samples = {duty*100:.0f}% duty (~{duty*T/fps:.0f}s of {T/fps:.0f}s)")

    # 3. FOLD: average per-plateau-window periodograms (Welch/Bartlett — each window
    # analysed alone => NO cross-window gate-comb at k*RR; cardiac line builds, noise
    # averages down). Contrast: LS over ALL gated samples (has the comb artifact).
    fgrid = np.linspace(HR_LO, HR_HI, 1600)
    NFFT = int(8 * fps)                                  # zero-pad grid (fine bins)
    fpad = np.fft.rfftfreq(NFFT, 1 / fps)
    bandm = (fpad >= HR_LO) & (fpad <= HR_HI)
    # split plateau mask into contiguous segments
    segs, s = [], None
    for i in range(T):
        if plateau[i] and s is None:
            s = i
        elif not plateau[i] and s is not None:
            if i - s >= int(0.5 * fps):                  # >=0.5s usable
                segs.append((s, i))
            s = None
    Pavg = np.zeros(bandm.sum()); nseg = 0
    for (s0, s1) in segs:
        seg = card[s0:s1] - card[s0:s1].mean()
        seg = seg * np.hanning(len(seg))                 # taper -> low leakage
        S = np.abs(np.fft.rfft(seg, NFFT)) ** 2
        S = S[bandm]
        if S.max() > 0:
            Pavg += S / S.max(); nseg += 1               # normalize each -> equal vote
    Pavg /= max(nseg, 1)
    fb = fpad[bandm]
    hr_plat = fb[np.argmax(Pavg)] * 60
    # time-domain beat-count per plateau window on the SINGLE best cardiac bin
    cbest = bandpass(chans[ctop[0]], fps, 0.9, 2.0)
    bpms, wts = [], []
    for (s0, s1) in segs:
        if s1 - s0 >= int(1.5 * fps):
            bpms.append(beat_count(cbest[s0:s1], fps, hi_bpm=130))
            wts.append(s1 - s0)
    hr_bc = float(np.median(bpms)) if bpms else float("nan")
    print(f"      PLATEAU beat-count (best bin {bins[ctop[0]]}, {len(bpms)} win) = "
          f"{hr_bc:.0f} bpm  {sorted(round(b) for b in bpms)}")
    # contrast baselines
    w = 2 * np.pi * fgrid
    P_full = lombscargle(t, card - card.mean(), w, normalize=True)
    hr_full = fgrid[np.argmax(P_full)] * 60
    P_plat = np.interp(fgrid, fb, Pavg)                  # for plotting on common grid
    ac_full, ac_h = autocorr_peak(bandpass(card, fps, 1.0, 1.7), fps, 60, 102)
    print(f"      [fold = {nseg} plateau windows averaged]")

    print(f" 3. FOLD (Lomb-Scargle stack):")
    print(f"      full-signal LS peak     = {hr_full:5.0f} bpm")
    print(f"      full-signal autocorr    = {ac_full:5.0f} bpm (band-center prone)")
    print(f"      PLATEAU-stack LS peak    = {hr_plat:5.0f} bpm   <== method")
    print(f"    truth {truth:.0f} -> plateau err {hr_plat-truth:+.0f} | full-LS err {hr_full-truth:+.0f}")
    # breathing-harmonic suppression check
    print("    k*RR harmonics in band:", [f"{k}x={k*rr:.0f}" for k in range(3,9)
                                          if HR_LO<=k*rr/60<=HR_HI])

    try:
        import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
        fig, ax = plt.subplots(2, 1, figsize=(12, 8))
        t0, t1 = 0, min(T, int(30*fps))
        ax[0].plot(t[t0:t1], breath[t0:t1], "C0", lw=1.3, label="pooled breathing")
        pm = plateau[t0:t1]
        ax[0].plot(t[t0:t1][pm], breath[t0:t1][pm], ".", color="C2", ms=4,
                   label=f"plateau (kept, {duty*100:.0f}% duty)")
        ax[0].set_title(f"{block} — step2 gate: keep long-edge plateau (green) = micro breath-holds")
        ax[0].set_xlabel("s"); ax[0].set_ylabel("mm"); ax[0].legend(fontsize=8)
        fr = fgrid * 60
        ax[1].plot(fr, P_full/P_full.max(), "C7", lw=1, label=f"full-signal ({hr_full:.0f})")
        ax[1].plot(fr, P_plat/P_plat.max(), "C1", lw=1.8, label=f"PLATEAU-stack ({hr_plat:.0f})")
        ax[1].axvline(truth, color="g", ls="--", lw=1.8, label=f"truth {truth:.0f}")
        for k in range(3, 9):
            if HR_LO <= k*rr/60 <= HR_HI:
                ax[1].axvline(k*rr, color="r", ls=":", lw=.8)
        ax[1].set_title("step3 fold: plateau-stack Lomb-Scargle (red dotted = k*RR harmonics)")
        ax[1].set_xlabel("bpm"); ax[1].set_ylabel("norm power"); ax[1].legend(fontsize=8)
        fig.tight_layout(); fig.savefig("chairL_plateau_hr.png", dpi=115)
        print("saved chairL_plateau_hr.png")
    except Exception as e:
        print("plot skipped:", e)


if __name__ == "__main__":
    main()
