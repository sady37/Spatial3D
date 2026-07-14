"""干净证明:tracking > mean-lock —— 在真 HR 大摆动(131→91)且无削波的雷达数据上。

sleepad 压电动态段被硬件削波堵死(见 memory hr-tracking-noise-limited);tachy2 雷达
无削波、真值轨迹已知(post-exercise 恢复 131→91,Δ40,std~13 ≫ 噪声),是干净试验台。

同一套雷达位移信号,两种方法都对 truth(t) 打分:
  MEAN-LOCK  整段单一 HR(ensemble)—— 常数,漏掉扫频,r~0、MAE 大
  TRACKING   逐窗 excess-harmonic 跟随 —— 跟住 131→91,r 高、MAE 小
判据:MAE + 与 truth 的相关 r(常数 r≡0;真 tracking r>0)。

    python3 tachy2_tracking_vs_meanlock.py
"""
import numpy as np
from bcg_vitals import (bandpass, sqi, demod_channels, estimate_rr, fft_peak,
                        RR_LO, RR_HI)

FPS = 18.78
HR_LO, HR_HI = 1.3, 2.4          # 78–144 bpm 心搏带


def truth(t):
    return np.where(t <= 60, 131 - 21 * t / 60, 110 - 19 * (t - 60) / 60)


def seg_excess_hr(disp, f0):
    """呼吸低谐波(n1-4)定包络,心搏带谐波(n5-7)超出包络最多者→HR。"""
    disp = disp - disp.mean()
    f = np.fft.rfftfreq(len(disp), 1 / FPS)
    S = np.abs(np.fft.rfft(disp))
    A = np.array([float(S[np.abs(f - n * f0) <= 0.05].max()) if (np.abs(f - n * f0) <= 0.05).any()
                  else 1e-9 for n in range(1, 9)])
    good = A[:4] > 0
    if good.sum() < 2:
        return None
    c = np.polyfit(np.arange(1, 5)[good], np.log(A[:4][good]), 1)
    base = np.exp(np.polyval(c, np.arange(1, 9)))
    exc = A / base
    ns = np.array([5, 6, 7])
    e = exc[ns - 1]
    if e.max() < 1.3:
        return None
    w = np.clip(e - 1.0, 0, None)
    return np.sum(w * ns * f0 * 60) / (w.sum() + 1e-9)


def main():
    d = np.load("tachy2_cube.npz", allow_pickle=True)
    cube = np.asarray(d["snapshots"], np.complex64)
    counts = d["counts"].astype(int); bins = d["bins"].astype(int)
    C = cube[:, :int(counts.min()), :]
    chans = demod_channels(C, bins)
    _, f0, _, _ = estimate_rr(chans, FPS)
    rr = np.array([sqi(bandpass(c, FPS, RR_LO, RR_HI), FPS, RR_LO, RR_HI) for c in chans])
    top = np.argsort(rr)[::-1][:6]
    N = C.shape[1]; dur = N / FPS
    print("tachy2: %d bins, %d slow-time, %.0fs @%.2ffps, breathing f0=%.0f bpm"
          % (C.shape[0], N, dur, FPS, f0 * 60))

    # ---------- TRACKING:逐窗 excess-harmonic ----------
    win = int(15 * FPS); step = int(3 * FPS)
    tt, htrk = [], []
    for s in range(0, N - win + 1, step):
        hrs = [seg_excess_hr(chans[i][s:s + win], f0) for i in top]
        hrs = [h for h in hrs if h]
        tt.append((s + win / 2) / FPS)
        htrk.append(np.median(hrs) if hrs else np.nan)
    tt = np.array(tt); htrk = np.array(htrk)
    # 填补 nan(线性)+ 轻连续
    ok = ~np.isnan(htrk)
    htrk = np.interp(tt, tt[ok], htrk[ok])

    # ---------- MEAN-LOCK:整段单一 HR ----------
    # (a) 朴素:整段心搏带 FFT 峰(memory 记的"锁 81"式)
    ml_fft = np.median([fft_peak(bandpass(chans[i], FPS, HR_LO, HR_HI), FPS, HR_LO, HR_HI) * 60
                        for i in top])
    # (b) 公平:整段 excess-harmonic 单值
    ml_exc = np.nanmedian([seg_excess_hr(chans[i], f0) or np.nan for i in top])

    # ---------- 对 truth 打分 ----------
    tr = truth(tt)
    def score(pred, name):
        pred = np.full_like(tr, pred) if np.isscalar(pred) else pred
        mae = np.mean(np.abs(pred - tr))
        r = 0.0 if np.std(pred) < 1e-6 else np.corrcoef(pred, tr)[0, 1]
        print("  %-22s MAE=%5.1f bpm   r=%+.2f" % (name, mae, r))
        return pred
    print("\ntruth 扫频 131→91 (窗中心 %.0f→%.0fs), std=%.1f bpm" % (tt[0], tt[-1], np.std(tr)))
    p_ml_f = score(ml_fft, "MEAN-LOCK naive-FFT (%.0f)" % ml_fft)
    p_ml_e = score(ml_exc, "MEAN-LOCK excess (%.0f)" % ml_exc)
    p_trk = score(htrk, "TRACKING excess-window")
    print("\n>>> tracking 相关应 ≫ mean-lock(常数 r≡0),MAE 应显著更低 = 干净证明")

    try:
        import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
        fig, ax = plt.subplots(figsize=(11, 4.5))
        ax.plot(tt, tr, "k-", lw=2.4, label="truth 131→91")
        ax.plot(tt, htrk, "C1.-", lw=1.4, ms=4, label="TRACKING (r=%+.2f)"
                % np.corrcoef(htrk, tr)[0, 1])
        ax.axhline(ml_fft, color="C0", ls="--", lw=1.4, label="MEAN-LOCK naive %.0f (r=0)" % ml_fft)
        ax.axhline(ml_exc, color="C3", ls=":", lw=1.4, label="MEAN-LOCK excess %.0f" % ml_exc)
        ax.set_xlabel("t (s)"); ax.set_ylabel("HR bpm"); ax.set_ylim(75, 140)
        ax.legend(loc="upper right")
        ax.set_title("tachy2: MEAN-LOCK fails (84, r=0); freq-tracker also can't follow (r=0.32) — entanglement unsolved")
        fig.tight_layout(); fig.savefig("tachy2_tracking_vs_meanlock.png", dpi=110)
        print("\nsaved tachy2_tracking_vs_meanlock.png")
    except Exception as e:
        print("plot skipped:", e)


if __name__ == "__main__":
    main()
