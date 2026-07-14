"""ChairL validation of RESPIRATORY-PHASE ENSEMBLE AVERAGING to break RR-harmonic
entanglement (the discussion: model breathing as g(theta), subtract, residual=cardiac).

Two-anchor (LEFT/RIGHT split) phase vs single-anchor, vs raw:
  - single-anchor: mark troughs only, phase 0->2pi linear per whole cycle (uniform,
    ignores inspiration/expiration timing asymmetry).
  - two-anchor:   mark troughs AND peaks, phase 0->pi trough->peak, pi->2pi
    peak->trough, each HALF warped to its own duration (absorbs I:E ratio drift).
Fold the broadband displacement on breathing phase -> g_hat(theta) captures breathing
INCLUDING its cardiac-band harmonics; residual = signal - g_hat(theta) = cardiac.

Decision: does the residual cardiac-band peak move off k*RR (fake) toward true HR ~77?

    python3 chairL_phasefold.py
"""
import sys
import numpy as np
from scipy.signal import find_peaks, hilbert
from bcg_vitals import (demod_channels, bandpass, sqi, fft_peak, autocorr_peak,
                        RR_LO, RR_HI)

BLOCK = "chairL_20260713_183514.npz"   # one full ~5-min person block (oximeter 74-81)
TRUTH = 77.0
HR_LO, HR_HI = 1.0, 1.7                 # validated resting cardiac band (60-102 bpm)


def load(path):
    d = np.load(path, allow_pickle=True)
    cube = np.asarray(d["snapshots"], np.complex64)
    counts = d["counts"].astype(int); bins = d["bins"].astype(int)
    C = cube[:, :int(counts.min()), :]
    ts = np.asarray(d["frame_ts"], float)[:int(counts.min())]
    span = ts[-1] - ts[0]
    if span > 1e4:                      # ms
        span /= 1000.0
    fps = (len(ts) - 1) / span
    return C, bins, fps


def band_peak(x, fps, lo, hi):
    f = np.fft.rfftfreq(len(x), 1 / fps)
    S = np.abs(np.fft.rfft(x - x.mean())) ** 2
    m = (f >= lo) & (f <= hi)
    ff = f[m]; SS = S[m]
    k = int(np.argmax(SS))
    return ff[k] * 60, SS[k], ff, SS


def phase_single(breath, fps):
    """One anchor per cycle (troughs); phase advances 0->2pi linearly per cycle."""
    tr, _ = find_peaks(-breath, distance=int(fps / RR_HI))
    theta = np.full(len(breath), np.nan)
    for a, b in zip(tr[:-1], tr[1:]):
        theta[a:b] = np.linspace(0, 2 * np.pi, b - a, endpoint=False)
    return theta, tr


def phase_two(breath, fps):
    """Two anchors (trough & peak); each HALF warped to its own duration:
    trough->peak = 0..pi (inspiration edge), peak->trough = pi..2pi (expiration)."""
    dist = int(fps / RR_HI)
    tr, _ = find_peaks(-breath, distance=dist)
    pk, _ = find_peaks(breath, distance=dist)
    # interleave anchors in time order, tagging trough(0)/peak(1)
    anc = sorted([(i, 0) for i in tr] + [(i, 1) for i in pk])
    theta = np.full(len(breath), np.nan)
    for (i0, t0), (i1, t1) in zip(anc[:-1], anc[1:]):
        if t0 == t1:                       # skip double trough/peak (missed anchor)
            continue
        base = 0.0 if t0 == 0 else np.pi   # trough->peak: 0..pi ; peak->trough: pi..2pi
        theta[i0:i1] = base + np.linspace(0, np.pi, i1 - i0, endpoint=False)
    return theta, tr, pk


def fold_subtract(sig, theta, nbin=72, smooth=5):
    """g_hat(theta) = mean(sig) per phase bin (cardiac averages out across breaths);
    light circular smoothing; residual = sig - g_hat(theta)."""
    valid = ~np.isnan(theta)
    idx = np.floor(theta / (2 * np.pi) * nbin).astype(int)
    idx = np.clip(idx, 0, nbin - 1)
    g = np.zeros(nbin); cnt = np.zeros(nbin)
    for i in np.where(valid)[0]:
        g[idx[i]] += sig[i]; cnt[idx[i]] += 1
    g = np.where(cnt > 0, g / np.maximum(cnt, 1), 0.0)
    # circular smoothing
    k = np.ones(smooth) / smooth
    g = np.convolve(np.r_[g[-smooth:], g, g[:smooth]], k, "same")[smooth:smooth + nbin]
    resid = sig.copy()
    resid[valid] = sig[valid] - g[idx[valid]]
    resid[~valid] = 0.0
    return resid, g


def main():
    block = sys.argv[1] if len(sys.argv) > 1 else BLOCK
    C, bins, fps = load(block)
    DR = 0.0234375
    chans = demod_channels(C, bins)                        # (nbin, T) mm disp
    T = chans.shape[1]
    print(f"{BLOCK}: {len(bins)} bins x {T} frames @ {fps:.2f} fps "
          f"({T/fps:.0f}s, truth~{TRUTH:.0f} bpm)\n")

    # --- breathing reference: best resp-SQI bin ---
    resp_sqi = np.array([sqi(bandpass(c, fps, RR_LO, RR_HI), fps, RR_LO, RR_HI)
                         for c in chans])
    rb = int(np.argmax(resp_sqi))
    breath = bandpass(chans[rb], fps, RR_LO, RR_HI)

    th1, tr = phase_single(breath, fps)
    th2, tr2, pk2 = phase_two(breath, fps)
    # RR from the ACTUAL trough cadence (fft_peak locks a sub/2nd-harmonic here)
    rr = len(tr) / (T / fps) * 60
    rr_hz = rr / 60.0
    print(f"breathing ref bin {bins[rb]} ({bins[rb]*DR:.2f}m)  RR={rr:.1f} rpm "
          f"(trough-cadence, f0={rr_hz:.3f}Hz)")
    for k in range(3, 9):
        print(f"    {k}xRR = {k*rr:.0f} bpm ({k*rr_hz:.3f}Hz)"
              + ("   <-- lands in [60,102]" if HR_LO <= k*rr_hz <= HR_HI else ""))
    print(f"\nanchors: {len(tr)} troughs (single);  {len(tr2)} troughs + {len(pk2)} peaks (two)")
    # inspiration:expiration timing asymmetry (why left/right split matters)
    anc = sorted([(i, 0) for i in tr2] + [(i, 1) for i in pk2])
    seg = [(anc[j+1][0]-anc[j][0], anc[j][1]) for j in range(len(anc)-1)
           if anc[j][1] != anc[j+1][1]]
    rise = np.median([d for d, t in seg if t == 0]) / fps   # trough->peak
    fall = np.median([d for d, t in seg if t == 1]) / fps   # peak->trough
    print(f"median rise (trough->peak)={rise:.2f}s  fall={fall:.2f}s  I:E={rise/fall:.2f}")

    # --- fold+subtract on the top HR-SQI bins; median residual spectrum ---
    hr_sqi = np.array([sqi(bandpass(c, fps, HR_LO, HR_HI), fps, HR_LO, HR_HI)
                       for c in chans])
    top = np.argsort(hr_sqi)[::-1][:8]

    lb, hb = int(HR_LO*60), int(HR_HI*60)
    def run(theta):
        raw_fft, res_fft, raw_ac, res_ac = [], [], [], []
        Sres = None; f_axis = None; resid_stack = []
        for i in top:
            sig = bandpass(chans[i], fps, 0.1, 2.5)        # breathing+harmonics+cardiac
            rawb = bandpass(sig, fps, HR_LO, HR_HI)
            rp, _, _, _ = band_peak(rawb, fps, HR_LO, HR_HI)
            resid, _ = fold_subtract(sig, theta)
            resb = bandpass(resid, fps, HR_LO, HR_HI)
            xp, _, fx, Sx = band_peak(resb, fps, HR_LO, HR_HI)
            raw_fft.append(rp); res_fft.append(xp)
            ra, _ = autocorr_peak(rawb, fps, lb, hb); raw_ac.append(ra)
            xa, _ = autocorr_peak(resb, fps, lb, hb); res_ac.append(xa)
            Sres = Sx if Sres is None else Sres + Sx; f_axis = fx
            resid_stack.append(resb / (resb.std() + 1e-9))
        avg = np.mean(resid_stack, 0)                      # coherent-avg residual
        avg_ac, _ = autocorr_peak(avg, fps, lb, hb)
        return (np.array(raw_fft), np.array(res_fft), np.array(raw_ac),
                np.array(res_ac), f_axis, Sres, avg_ac)

    r_fft, r1_fft, r_ac, r1_ac, fx, S1, avg1 = run(th1)
    _,     r2_fft, _,    r2_ac, _,  S2, avg2 = run(th2)
    p1 = fx[np.argmax(S1)] * 60; p2 = fx[np.argmax(S2)] * 60
    print(f"\n--- cardiac-band HR (bpm) over top-8 HR bins; truth~{TRUTH:.0f} ---")
    print(f"{'method':18s} {'FFTargmax med':>13s} {'AUTOCORR med':>13s}  (autocorr per-bin)")
    print(f"{'RAW (no subtract)':18s} {np.median(r_fft):13.0f} {np.median(r_ac):13.0f}"
          f"  {np.round(r_ac).astype(int)}")
    print(f"{'SINGLE-anchor':18s} {np.median(r1_fft):13.0f} {np.median(r1_ac):13.0f}"
          f"  {np.round(r1_ac).astype(int)}")
    print(f"{'TWO-anchor':18s} {np.median(r2_fft):13.0f} {np.median(r2_ac):13.0f}"
          f"  {np.round(r2_ac).astype(int)}")
    print(f"\nsummed residual spectrum peak: single={p1:.0f}  two={p2:.0f} bpm")
    print(f"coherent-avg residual autocorr: single={avg1:.0f}  two={avg2:.0f} bpm")
    print(f"\nerr vs truth {TRUTH:.0f} (autocorr median): "
          f"RAW {np.median(r_ac)-TRUTH:+.0f} | single {np.median(r1_ac)-TRUTH:+.0f} "
          f"| two {np.median(r2_ac)-TRUTH:+.0f}")
    res1, res2, raw_pk = r1_fft, r2_fft, r_fft

    # --- figure ---
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        fig, ax = plt.subplots(2, 2, figsize=(13, 8))
        t = np.arange(min(T, int(20*fps))) / fps
        ax[0,0].plot(t, breath[:len(t)], lw=.8)
        ax[0,0].plot(tr[tr < len(t)]/fps, breath[tr[tr < len(t)]], "v", ms=5, label="trough")
        ax[0,0].plot(pk2[pk2 < len(t)]/fps, breath[pk2[pk2 < len(t)]], "^", ms=5, label="peak")
        ax[0,0].set_title(f"breathing ref (RR={rr:.0f}, I:E={rise/fall:.2f})"); ax[0,0].legend(fontsize=8)
        ax[0,0].set_xlabel("s")
        # folded shapes
        _, g1 = fold_subtract(bandpass(chans[top[0]], fps, 0.1, 2.5), th1)
        _, g2 = fold_subtract(bandpass(chans[top[0]], fps, 0.1, 2.5), th2)
        ax[0,1].plot(np.linspace(0,360,len(g1)), g1, label="single-anchor")
        ax[0,1].plot(np.linspace(0,360,len(g2)), g2, label="two-anchor")
        ax[0,1].axvline(180, color="k", ls=":", lw=.7); ax[0,1].set_xlabel("breathing phase (deg)")
        ax[0,1].set_title("folded breathing shape g_hat(theta)"); ax[0,1].legend(fontsize=8)
        fr = fx * 60
        for a, S, lab, p in [(ax[1,0], S1, "single", p1), (ax[1,1], S2, "two", p2)]:
            a.plot(fr, S1/S1.max() if S is S1 else S/S.max(), color="C7", lw=.8, label="residual")
            a.axvline(TRUTH, color="g", ls="--", label=f"truth {TRUTH:.0f}")
            for k in range(3, 8):
                if HR_LO <= k*rr_hz <= HR_HI:
                    a.axvline(k*rr, color="r", ls=":", lw=.8, label=f"{k}xRR")
            a.axvline(p, color="C0", lw=1.5, alpha=.6, label=f"peak {p:.0f}")
            a.set_title(f"{lab}-anchor residual spectrum"); a.set_xlabel("bpm")
            a.legend(fontsize=7)
        plt.tight_layout(); plt.savefig("chairL_phasefold.png", dpi=110)
        print("\nsaved chairL_phasefold.png")
    except Exception as e:
        print("plot skipped:", e)


if __name__ == "__main__":
    main()
