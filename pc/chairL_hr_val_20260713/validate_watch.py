"""Final validation: blind radar chest-bin HR vs Apple Watch HR, per 30s window,
on the sit and lie resting segments (wall-clock gated, walk excluded).

Self-contained in this directory (reads the npz + watch CSV next to it, imports
bcg_vitals from the parent pc/). Radar HR = SUB-LAG interpolated autocorr per bin,
then a ROBUST FUSION over the top-K cardiac-SNR chest bins: trim 1/4 off each end
and mean the middle half (25%-trimmed mean). This removes single-bin fragility —
at sit (marginal cardiac SNR ~2-4) one bin can lock a spurious low value; trimming
both ends drops that low outlier and any high outlier. Independent beat-count
(same fusion) as a cross-check. Writes results_per_window.csv + validate_watch.png
(SIT top, LIE bottom).

    .venv/bin/python3 validate_watch.py
"""
import os, sys, time, csv
import numpy as np
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))                 # parent pc/ -> bcg_vitals
from bcg_vitals import demod_channels, bandpass, autocorr_peak, beat_count, RR_LO, RR_HI

WATCH = os.path.join(HERE, "watch_hr_0713.csv")
WIN, STEP = 30.0, 10.0
KBINS = 6                                                 # top cardiac-SNR chest bins fused
SEGS = [
    ("chairL_sit_20260713_225001.npz", (22, 51, 0), (22, 54, 0), "SIT"),
    ("chairL_sit_20260713_225502.npz", (22, 56, 20), (22, 59, 0), "LIE"),
]


def trimmed(vals):
    """Robust center: drop >=1 off each end (median for tiny clusters), mean the rest."""
    s = sorted(v for v in vals if v)
    if not s:
        return None
    k = len(s); lo = max(1, k // 4) if k >= 3 else 0
    mid = s[lo:k - lo] if k - 2 * lo >= 1 else s
    return float(np.mean(mid))


def hr_cluster(disp, fps, f0):
    """Physically-grounded chest cluster (no scatter/硬凑): find the CONTIGUOUS body
    range span (breathing present), then take the cardiac hotspot = max-cSNR bin
    (chest end, low breathing) +/-2 CLAMPED to the body span. HR fuses only these.
    Abdomen (high-breathing end) is deliberately excluded (it pulls HR to k*RR)."""
    N = disp.shape[1]; f = np.fft.rfftfreq(N, 1 / fps)
    rr = np.array([bandpass(d, fps, RR_LO, 0.6).std() for d in disp])
    above = np.where(rr > 0.15 * rr.max())[0]
    lo_b, hi_b = int(above.min()), int(above.max())      # contiguous body span

    def cs(i):
        d = disp[i]; X = 2 * np.abs(np.fft.rfft(d - d.mean())) / N
        band = (f >= 1.0) & (f <= 1.7); nh = np.zeros_like(f, bool)
        for k in range(1, 12):
            nh |= np.abs(f - k * f0) <= 0.035
        c = band & ~nh
        return X[np.where(c)[0][np.argmax(X[c])]] / (np.median(X[band & ~nh]) + 1e-9)
    csn = {i: cs(i) for i in range(lo_b, hi_b + 1)}
    hot = max(csn, key=csn.get)                           # cardiac hotspot = chest
    return [i for i in range(hot - 2, hot + 3) if lo_b <= i <= hi_b]


def analyse(path, s0, s1):
    d = np.load(os.path.join(HERE, path), allow_pickle=True)
    n = int(d["counts"].astype(int).min())
    ts = np.asarray(d["frame_ts"], float)[:n]; t0 = float(d["block_start_epoch"])
    base = list(time.localtime(t0))
    def ep(hms):
        b = base[:]; b[3], b[4], b[5] = hms; return time.mktime(time.struct_time(b))
    idx = np.where((ts >= ep(s0)) & (ts <= ep(s1)))[0]
    C = np.asarray(d["snapshots"], np.complex64)[:, idx[0]:idx[-1] + 1, :]
    bins = d["bins"].astype(int); tw = ts[idx[0]:idx[-1] + 1]
    fps = (len(tw) - 1) / (tw[-1] - tw[0])
    disp = demod_channels(C, bins); N = disp.shape[1]; f = np.fft.rfftfreq(N, 1 / fps)
    rr_amp = [bandpass(x, fps, RR_LO, 0.6).std() for x in disp]
    Xr = np.abs(np.fft.rfft(bandpass(disp[int(np.argmax(rr_amp))], fps, RR_LO, RR_HI)))
    m = (f >= RR_LO) & (f <= RR_HI); f0 = f[m][np.argmax(Xr[m])]
    chidx = hr_cluster(disp, fps, f0)
    return disp, fps, tw, N, chidx, [int(bins[c]) for c in chidx], f0


def main():
    a = np.loadtxt(WATCH, delimiter=",", skiprows=1); wep, whr = a[:, 0], a[:, 1]
    rows = [("segment", "wall", "radar_fused", "beat_fused", "watch", "d")]
    panels, all_r, all_w = [], [], []
    print("blind radar HR = contiguous chest cluster (cardiac hotspot +/-2, trimmed) "
          "vs Apple Watch\n")
    for path, s0, s1, lab in SEGS:
        disp, fps, tw, N, chidx, chbins, f0 = analyse(path, s0, s1)
        sigs = [bandpass(disp[c], fps, 1.0, 1.7) for c in chidx]
        cbcs = [bandpass(disp[c], fps, 0.9, 2.0) for c in chidx]
        W = int(WIN * fps)
        print(f"=== {lab}  chest bins {sorted(chbins)}  RR={f0*60:.0f} ===")
        print(f"  {'wall':>8} {'radar':>6} {'beat':>5} {'watch':>6} {'d':>5}")
        T, R, B, Wv = [], [], [], []
        s = 0
        while s + W <= N:
            hrv = [autocorr_peak(sg[s:s + W], fps, 60, 102, interp=True)[0] for sg in sigs]
            hr = trimmed(hrv)
            bc = trimmed([beat_count(cb[s:s + W], fps, hi_bpm=110) for cb in cbcs])
            tc = tw[s + W // 2]
            wm = (wep >= tc - WIN / 2) & (wep <= tc + WIN / 2)
            wv = float(np.mean(whr[wm])) if wm.any() else float(np.interp(tc, wep, whr))
            if hr:
                T.append(tc); R.append(hr); B.append(bc); Wv.append(wv)
                wall = time.strftime("%H:%M:%S", time.localtime(tc))
                print(f"  {wall:>8} {hr:6.1f} {bc:5.0f} {wv:6.0f} {hr-wv:+5.1f}")
                rows.append((lab, wall, f"{hr:.1f}", f"{bc:.0f}", f"{wv:.0f}", f"{hr-wv:+.1f}"))
            s += int(STEP * fps)
        R, B, Wv = np.array(R), np.array(B), np.array(Wv)
        mae = np.mean(np.abs(R - Wv))
        print(f"  -> {lab}: radar median {np.median(R):.1f}, watch median {np.median(Wv):.0f}, "
              f"MAE={mae:.1f}, bias={np.mean(R-Wv):+.1f}\n")
        panels.append((lab, sorted(chbins), np.array(T), R, B, Wv, mae))
        all_r += list(R); all_w += list(Wv)
    all_r, all_w = np.array(all_r), np.array(all_w)
    print(f"OVERALL: MAE={np.mean(np.abs(all_r-all_w)):.1f} bpm, bias={np.mean(all_r-all_w):+.1f} "
          f"(n={len(all_r)} windows)")
    with open(os.path.join(HERE, "results_per_window.csv"), "w", newline="") as fh:
        csv.writer(fh).writerows(rows)
    print("saved results_per_window.csv")

    try:
        import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
        fig, ax = plt.subplots(2, 1, figsize=(11, 7.5))
        for k, (lab, chbins, T, R, B, Wv, mae) in enumerate(panels):
            t = T - T[0]
            ax[k].plot(t, Wv, "k-o", lw=2, ms=4, label="Apple Watch")
            ax[k].plot(t, R, "C1.-", lw=1.6, ms=7, label=f"radar chest cluster {chbins}")
            ax[k].plot(t, B, "C0x--", lw=.8, ms=6, alpha=.7, label="radar beat-count (x-check)")
            for yy in (70.4, 75.1, 80.5):
                ax[k].axhline(yy, color="grey", ls=":", lw=.6, alpha=.6)
            ax[k].set_title(f"{lab} — MAE {mae:.1f} bpm  (bins {chbins[0]}-{chbins[-1]})")
            ax[k].set_ylabel("HR (bpm)"); ax[k].set_ylim(66, 96)
            ax[k].legend(fontsize=8, loc="upper right"); ax[k].set_xlabel("time in segment (s)")
        fig.suptitle("blind radar HR vs Apple Watch — RR-anchored chest-end cluster "
                     f"(overall MAE {np.mean(np.abs(all_r-all_w)):.1f} bpm)", y=1.0)
        fig.tight_layout(); fig.savefig(os.path.join(HERE, "validate_watch.png"), dpi=120)
        print("saved validate_watch.png")
    except Exception as e:
        print("plot skipped:", e)


if __name__ == "__main__":
    main()
