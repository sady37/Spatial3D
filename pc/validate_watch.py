"""Final validation: blind radar chest-bin HR vs Apple Watch HR, per 30s window,
on the sit and lie resting segments (wall-clock gated, walk excluded).

    .venv/bin/python3 validate_watch.py
"""
import time
import numpy as np
from bcg_vitals import demod_channels, bandpass, autocorr_peak, RR_LO, RR_HI

WATCH = "watch_hr_0713.csv"
WIN, STEP = 30.0, 10.0
SEGS = [  # (file, (h,m,s) start, (h,m,s) end, label)
    ("chairL_sit_20260713_225001.npz", (22, 51, 0), (22, 54, 0), "SIT"),
    ("chairL_sit_20260713_225502.npz", (22, 56, 20), (22, 59, 0), "LIE"),
]


def watch_load():
    a = np.loadtxt(WATCH, delimiter=",", skiprows=1)
    return a[:, 0], a[:, 1]


def blind_chest(disp, fps, f0):
    N = disp.shape[1]; f = np.fft.rfftfreq(N, 1 / fps)
    rr = np.array([bandpass(d, fps, RR_LO, 0.6).std() for d in disp])
    body = rr > 0.12 * rr.max()

    def cs(d):
        X = 2 * np.abs(np.fft.rfft(d - d.mean())) / N
        band = (f >= 1.0) & (f <= 1.7); nh = np.zeros_like(f, bool)
        for k in range(1, 12):
            nh |= np.abs(f - k * f0) <= 0.035
        c = band & ~nh
        return X[np.where(c)[0][np.argmax(X[c])]] / (np.median(X[band & ~nh]) + 1e-9)
    csn = np.array([cs(disp[i]) if body[i] else 0 for i in range(disp.shape[0])])
    return int(np.argmax(csn))


def main():
    wep, whr = watch_load()
    print(f"blind radar chest-bin HR  vs  Apple Watch  (30s windows)\n")
    all_r, all_w = [], []
    for path, s0, s1, lab in SEGS:
        d = np.load(path, allow_pickle=True); n = int(d["counts"].astype(int).min())
        ts = np.asarray(d["frame_ts"], float)[:n]; t0 = float(d["block_start_epoch"])
        base = list(time.localtime(t0))
        def ep(hms):
            b = base[:]; b[3], b[4], b[5] = hms; return time.mktime(time.struct_time(b))
        w0, w1 = ep(s0), ep(s1)
        idx = np.where((ts >= w0) & (ts <= w1))[0]
        C = np.asarray(d["snapshots"], np.complex64)[:, idx[0]:idx[-1] + 1, :]
        bins = d["bins"].astype(int); tw = ts[idx[0]:idx[-1] + 1]
        fps = (len(tw) - 1) / (tw[-1] - tw[0])
        disp = demod_channels(C, bins); N = disp.shape[1]; f = np.fft.rfftfreq(N, 1 / fps)
        Xr = np.abs(np.fft.rfft(bandpass(disp[np.argmax([bandpass(x,fps,RR_LO,0.6).std() for x in disp])], fps, RR_LO, RR_HI)))
        m = (f >= RR_LO) & (f <= RR_HI); f0 = f[m][np.argmax(Xr[m])]
        ch = blind_chest(disp, fps, f0)
        sig = bandpass(disp[ch], fps, 1.0, 1.7)

        print(f"=== {lab}  {path}  chest bin {bins[ch]} ({bins[ch]*0.0234:.2f}m)  RR={f0*60:.0f} ===")
        print(f"  {'wall':>8} {'radar':>6} {'watch':>6} {'d':>5}")
        rr_l, wr_l = [], []
        s = 0
        while s + int(WIN * fps) <= N:
            seg = sig[s:s + int(WIN * fps)]
            hr, _ = autocorr_peak(seg, fps, 60, 102)
            tc = tw[s + int(WIN * fps) // 2]
            wm = (wep >= tc - WIN / 2) & (wep <= tc + WIN / 2)
            wv = np.mean(whr[wm]) if wm.any() else np.interp(tc, wep, whr)
            if hr:
                rr_l.append(hr); wr_l.append(wv)
                print(f"  {time.strftime('%H:%M:%S',time.localtime(tc)):>8} {hr:6.0f} {wv:6.0f} {hr-wv:+5.0f}")
            s += int(STEP * fps)
        rr_l, wr_l = np.array(rr_l), np.array(wr_l)
        mae = np.mean(np.abs(rr_l - wr_l)); bias = np.mean(rr_l - wr_l)
        r = np.corrcoef(rr_l, wr_l)[0, 1] if np.std(rr_l) > 1e-6 and np.std(wr_l) > 1e-6 else float('nan')
        print(f"  -> {lab}: radar median {np.median(rr_l):.0f}, watch median {np.median(wr_l):.0f}, "
              f"MAE={mae:.1f}, bias={bias:+.1f}, r={r:+.2f}\n")
        all_r += list(rr_l); all_w += list(wr_l)
    all_r, all_w = np.array(all_r), np.array(all_w)
    print(f"OVERALL: MAE={np.mean(np.abs(all_r-all_w)):.1f} bpm, bias={np.mean(all_r-all_w):+.1f}, "
          f"r={np.corrcoef(all_r,all_w)[0,1]:+.2f}  (n={len(all_r)} windows)")


if __name__ == "__main__":
    main()
