"""DECISIVE test: does respiratory-phase ensemble subtraction (two-anchor / left-right
split) break the RR-harmonic entanglement that locks every freq tracker on tachy2?

tachy2: HR sweeps 131->91 (truth known), breathing harmonics pollute [1.3,2.4]Hz.
Baselines already failed: mean-lock FFT ~84 (r=0); excess-harmonic tracker r=0.32.

Method: build a GLOBAL two-anchor breathing phase over the whole 120s (many breaths ->
cardiac averages out of g_hat), subtract g_hat(theta) per bin, then track cardiac in
sliding windows on the RESIDUAL. Compare RAW-tracking vs RESIDUAL-tracking vs truth.

    python3 tachy2_phasefold.py
"""
import numpy as np
from bcg_vitals import (bandpass, sqi, demod_channels, estimate_rr, fft_peak,
                        autocorr_peak, RR_LO, RR_HI)
from chairL_phasefold import phase_single, phase_two, fold_subtract

FPS = 18.78
HR_LO, HR_HI = 1.3, 2.4          # 78-144 bpm, same band as the baseline tracker


def truth(t):
    return np.where(t <= 60, 131 - 21 * t / 60, 110 - 19 * (t - 60) / 60)


def track(sig_by_bin, top, win, step, N, breath_bb=None, anchor="none"):
    """sliding-window median FFT-argmax HR in the tachy band over top bins.
    anchor='two'/'single' -> LOCAL per-window respiratory-phase subtraction using
    that window's own breathing (breath_bb = broadband breathing ref bin)."""
    tt, hh = [], []
    for s in range(0, N - win + 1, step):
        pk = []
        theta = None
        if anchor != "none" and breath_bb is not None:
            br = bandpass(breath_bb[s:s + win], FPS, RR_LO, RR_HI)
            theta = (phase_two(br, FPS)[0] if anchor == "two"
                     else phase_single(br, FPS)[0])
        for i in top:
            seg = sig_by_bin[i][s:s + win]
            if theta is not None:
                seg = fold_subtract(seg, theta)[0]
            ff = fft_peak(bandpass(seg, FPS, HR_LO, HR_HI), FPS, HR_LO, HR_HI)
            if ff:
                pk.append(ff * 60)
        tt.append((s + win / 2) / FPS)
        hh.append(np.median(pk) if pk else np.nan)
    tt = np.array(tt); hh = np.array(hh)
    ok = ~np.isnan(hh)
    if ok.any():
        hh = np.interp(tt, tt[ok], hh[ok])
    return tt, hh


def main():
    d = np.load("tachy2_cube.npz", allow_pickle=True)
    cube = np.asarray(d["snapshots"], np.complex64)
    counts = d["counts"].astype(int); bins = d["bins"].astype(int)
    C = cube[:, :int(counts.min()), :]
    chans = demod_channels(C, bins)
    N = C.shape[1]; DR = 0.0234375

    # breathing ref + global phase (two-anchor)
    resp_sqi = np.array([sqi(bandpass(c, FPS, RR_LO, RR_HI), FPS, RR_LO, RR_HI) for c in chans])
    rb = int(np.argmax(resp_sqi))
    breath = bandpass(chans[rb], FPS, RR_LO, RR_HI)
    th1, tr_a = phase_single(breath, FPS)
    th2, tr2, pk2 = phase_two(breath, FPS)
    rr = len(tr_a) / (N / FPS) * 60
    print("tachy2: %d bins x %d (%.0fs @%.2ffps).  breathing ref bin %d (%.2fm) RR=%.1f rpm"
          % (C.shape[0], N, N / FPS, FPS, bins[rb], bins[rb] * DR, rr))
    for k in range(4, 9):
        f = k * rr / 60
        print("    %dxRR=%3.0f bpm (%.2fHz)%s" % (k, k * rr, f,
              "  <-- in [78,144]" if HR_LO <= f <= HR_HI else ""))

    # broadband per-bin (drift removed, breathing+harmonics+cardiac kept)
    bb = np.array([bandpass(chans[i], FPS, 0.1, 2.6) for i in range(len(bins))])
    # GLOBAL breathing subtraction (single vs two anchor)
    res1 = np.array([fold_subtract(bb[i], th1)[0] for i in range(len(bins))])
    res2 = np.array([fold_subtract(bb[i], th2)[0] for i in range(len(bins))])

    # top HR-SQI bins on the tachy band
    hr_sqi = np.array([sqi(bandpass(c, FPS, HR_LO, HR_HI), FPS, HR_LO, HR_HI) for c in chans])
    top = np.argsort(hr_sqi)[::-1][:6]

    win = int(20 * FPS); step = int(3 * FPS)          # 20s -> ~6 breaths/window
    breath_bb = bandpass(chans[rb], FPS, 0.1, 2.6)
    tt, h_raw = track(bb, top, win, step, N)
    _,  h_g2 = track(res2, top, win, step, N)                          # global two-anchor
    _,  h_l1 = track(bb, top, win, step, N, breath_bb, "single")        # local single
    _,  h_l2 = track(bb, top, win, step, N, breath_bb, "two")           # local two-anchor
    tr = truth(tt)

    def score(pred, name):
        mae = np.mean(np.abs(pred - tr))
        r = 0.0 if np.std(pred) < 1e-6 else np.corrcoef(pred, tr)[0, 1]
        # late-segment (t>40s) where entanglement bites: truth 105->91
        late = tt > 40
        mae_l = np.mean(np.abs(pred[late] - tr[late]))
        r_l = np.corrcoef(pred[late], tr[late])[0, 1] if np.std(pred[late]) > 1e-6 else 0.0
        print("  %-24s MAE=%5.1f r=%+.2f | late(t>40) MAE=%5.1f r=%+.2f"
              % (name, mae, r, mae_l, r_l))
        return mae, r
    print("\ntruth 131->91 over t=%.0f..%.0fs, std=%.1f bpm (win=20s)" % (tt[0], tt[-1], np.std(tr)))
    score(h_raw, "RAW (no subtract)")
    score(h_g2, "GLOBAL two-anchor")
    score(h_l1, "LOCAL single-anchor")
    score(h_l2, "LOCAL two-anchor")
    h_s1, h_s2 = h_l1, h_l2                            # reuse for plot
    print("\n>>> win = tracking r jumps positive AND MAE drops = phase-fold breaks entanglement")

    try:
        import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
        fig, ax = plt.subplots(figsize=(11, 5))
        ax.plot(tt, tr, "k-", lw=2.4, label="truth 131->91")
        ax.plot(tt, h_raw, "C7.-", lw=1, ms=3, alpha=.7,
                label="RAW (r=%+.2f)" % np.corrcoef(h_raw, tr)[0, 1])
        ax.plot(tt, h_s1, "C0.-", lw=1.2, ms=3,
                label="single-anchor (r=%+.2f)" % np.corrcoef(h_s1, tr)[0, 1])
        ax.plot(tt, h_s2, "C1.-", lw=1.6, ms=4,
                label="TWO-anchor (r=%+.2f)" % np.corrcoef(h_s2, tr)[0, 1])
        for k in range(4, 9):
            f = k * rr
            if HR_LO * 60 <= f <= HR_HI * 60:
                ax.axhline(f, color="r", ls=":", lw=.7, alpha=.5)
        ax.set_xlabel("t (s)"); ax.set_ylabel("HR bpm"); ax.set_ylim(75, 140)
        ax.legend(loc="upper right", fontsize=9)
        ax.set_title("tachy2: two-anchor respiratory-phase subtraction vs entanglement "
                     "(red dotted = k*RR harmonics)")
        fig.tight_layout(); fig.savefig("tachy2_phasefold.png", dpi=110)
        print("saved tachy2_phasefold.png")
    except Exception as e:
        print("plot skipped:", e)


if __name__ == "__main__":
    main()
