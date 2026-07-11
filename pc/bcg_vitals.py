"""Radar vitals via BCG-style pipeline (ref: sleep_pad_algorithm sleep算法.md).

Adapts the load-cell BCG algorithm to the radar cube. Each range bin's coherent
slow-time projection = one "channel" (44 bins ~ the pad's 4 channels). Per channel:
  1. bandpass to remove baseline drift (the <0.05Hz drift that swamped raw FFT)
  2. SQI = E_band / (E_total - E_band)  -> weight/select best bins
  3. HR: FFT peak + autocorrelation (robust when FFT peak is weak), consensus
  4. RR: low-freq band peak
SQI-weighted fusion across the top bins.

    python bcg_vitals.py fall20_cube.npz --fps 18.78 --t0 41 --t1 112
"""
import argparse
import numpy as np

HR_LO, HR_HI = 0.7, 2.5     # 42-150 bpm
RR_LO, RR_HI = 0.12, 0.6    # 7-36 rpm
DR = 0.0234375


def bandpass(x, fps, lo, hi):
    x = x - x.mean()
    f = np.fft.rfftfreq(len(x), 1 / fps)
    X = np.fft.rfft(x)
    X[(f < lo) | (f > hi)] = 0
    return np.fft.irfft(X, n=len(x))


def sqi(x, fps, lo, hi):
    f = np.fft.rfftfreq(len(x), 1 / fps)
    S = np.abs(np.fft.rfft(x - x.mean())) ** 2
    band = (f >= lo) & (f <= hi)
    Eb = S[band].sum()
    return Eb / (S.sum() - Eb + 1e-12)


def fft_peak(x, fps, lo, hi):
    f = np.fft.rfftfreq(len(x), 1 / fps)
    S = np.abs(np.fft.rfft(x - x.mean())) ** 2
    m = (f >= lo) & (f <= hi)
    return f[m][np.argmax(S[m])] if m.any() else None


def autocorr_peak(x, fps, lo, hi):
    x = bandpass(x, fps, lo, hi)
    ac = np.correlate(x, x, "full")[len(x) - 1:]
    if ac[0] <= 0:
        return None
    ac = ac / ac[0]
    lag_lo, lag_hi = int(fps / hi), int(fps / lo)
    if lag_hi <= lag_lo + 1 or lag_hi >= len(ac):
        return None
    seg = ac[lag_lo:lag_hi]
    return fps / (lag_lo + int(np.argmax(seg)))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("path")
    ap.add_argument("--fps", type=float, required=True)
    ap.add_argument("--t0", type=float, default=0.0)
    ap.add_argument("--t1", type=float, default=1e9)
    ap.add_argument("--topk", type=int, default=8)
    a = ap.parse_args()

    d = np.load(a.path, allow_pickle=True)
    cube = np.asarray(d["snapshots"], dtype=np.complex64)
    counts = d["counts"].astype(int); bins = d["bins"].astype(int)
    K = int(counts.min())
    i0, i1 = int(a.t0 * a.fps), min(K, int(a.t1 * a.fps))
    C = cube[:, i0:i1, :]

    # per-bin channel = PHASE of the coherently-combined complex (arctangent
    # demodulation). Phase carries mm/sub-mm displacement (Δφ=4π·Δr/λ ≈ 2.5rad/mm
    # @λ=5mm) — the amplitude/energy view threw that away. This is the standard
    # radar vital-signs channel and gives HR/tremor 10-100× the amplitude SNR.
    LAMBDA_MM = 5.0
    chans = []
    for i in range(len(bins)):
        m = C[i].mean(0); m = m / (np.linalg.norm(m) + 1e-9)
        z = C[i] @ m.conj()                       # complex per frame
        phi = np.unwrap(np.angle(z))              # rad
        disp_mm = -LAMBDA_MM / (4 * np.pi) * (phi - phi.mean())
        chans.append(disp_mm)                     # mm displacement time series
    chans = np.array(chans)                       # (nbin, T)

    # --- HR: SQI per bin (after HR bandpass), pick top-k, FFT+autocorr consensus
    hr_bp = np.array([bandpass(c, a.fps, HR_LO, HR_HI) for c in chans])
    hr_sqi = np.array([sqi(c, a.fps, HR_LO, HR_HI) for c in hr_bp])
    hr_top = np.argsort(hr_sqi)[::-1][:a.topk]
    print(f"HR top bins by SQI: "
          + ", ".join(f"{bins[i]}({bins[i]*DR:.2f}m,SQI{hr_sqi[i]:.2f})" for i in hr_top))
    hr_f, hr_a, wts = [], [], []
    for i in hr_top:
        ff = fft_peak(chans[i], a.fps, HR_LO, HR_HI)
        aa = autocorr_peak(chans[i], a.fps, HR_LO, HR_HI)
        if ff: hr_f.append(ff * 60); wts.append(hr_sqi[i])
        if aa: hr_a.append(aa * 60)
    wts = np.array(wts)
    hr_fft = np.average(hr_f, weights=wts) if hr_f else None
    hr_ac = np.median(hr_a) if hr_a else None
    print(f"  HR  FFT(SQI-weighted) = {hr_fft:.0f} bpm | autocorr(median) = {hr_ac:.0f} bpm")
    print(f"      per-bin FFT bpm: {[round(v) for v in hr_f]}")
    print(f"      per-bin AC  bpm: {[round(v) for v in hr_a]}")
    agree = hr_fft and hr_ac and abs(hr_fft - hr_ac) <= 8
    print(f"  -> HR = {(hr_fft+hr_ac)/2:.0f} bpm  [{'CONSENSUS' if agree else 'LOW-CONF: FFT/AC disagree'}]"
          if (hr_fft and hr_ac) else "  -> HR: insufficient")

    # --- RR: low-freq band, SQI, SQI-weighted FFT peak
    rr_bp = np.array([bandpass(c, a.fps, RR_LO, RR_HI) for c in chans])
    rr_sqi = np.array([sqi(c, a.fps, RR_LO, RR_HI) for c in rr_bp])
    rr_top = np.argsort(rr_sqi)[::-1][:a.topk]
    rr_f, rw = [], []
    for i in rr_top:
        ff = fft_peak(chans[i], a.fps, RR_LO, RR_HI)
        if ff: rr_f.append(ff * 60); rw.append(rr_sqi[i])
    rr = np.average(rr_f, weights=rw) if rr_f else None
    print(f"RR top bins by SQI: "
          + ", ".join(f"{bins[i]}({bins[i]*DR:.2f}m,SQI{rr_sqi[i]:.2f})" for i in rr_top))
    print(f"  RR (SQI-weighted) = {rr:.0f} rpm   per-bin: {[round(v) for v in rr_f]}")


if __name__ == "__main__":
    main()
