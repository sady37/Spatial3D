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
try:
    from scipy.signal import find_peaks
    _HAVE_SCIPY = True
except Exception:
    _HAVE_SCIPY = False

HR_HI = 2.5                 # 150 bpm ceiling
RR_LO, RR_HI = 0.12, 0.6    # 7-36 rpm
DR = 0.0234375


def bandpass(x, fps, lo, hi, notch_f0=None, notch_hw=0.022, notch_n=25):
    x = x - x.mean()
    f = np.fft.rfftfreq(len(x), 1 / fps)
    X = np.fft.rfft(x)
    X[(f < lo) | (f > hi)] = 0
    if notch_f0:                       # narrow-notch the breathing harmonic comb
        for n in range(1, notch_n):
            X[(f >= n * notch_f0 - notch_hw) & (f <= n * notch_f0 + notch_hw)] = 0
    return np.fft.irfft(X, n=len(x))


def beat_count(sig, fps, hi_bpm=150, height=0.25):
    """Time-domain heartbeat count (BCG J-peak method) -> bpm. Robust to the
    smooth breathing-harmonic residue that fools FFT argmax (sharp beats survive
    thresholding; the harmonic doesn't)."""
    s = sig / (sig.std() + 1e-9)
    dist = max(1, int(fps / (hi_bpm / 60)))
    if _HAVE_SCIPY:
        pk, _ = find_peaks(s, distance=dist, height=height)
        n = len(pk)
    else:
        n = sum(1 for i in range(1, len(s) - 1)
                if s[i] > s[i - 1] and s[i] > s[i + 1] and s[i] > height)
    return n / (len(sig) / fps) * 60


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


def autocorr_bpm(sig, fps, lo_bpm=48, hi_bpm=150):
    """First autocorrelation peak within a bpm range -> bpm (already-filtered sig)."""
    ac = np.correlate(sig, sig, "full")[len(sig) - 1:]
    if ac[0] <= 0:
        return None
    ac = ac / ac[0]
    l0, l1 = int(fps / (hi_bpm / 60)), int(fps / (lo_bpm / 60))
    if l1 <= l0 + 1 or l1 >= len(ac):
        return None
    return fps / (l0 + int(np.argmax(ac[l0:l1]))) * 60


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

    # --- RR first: low-freq band, SQI-weighted FFT peak. Also gives f0 for HR. ---
    rr_sqi = np.array([sqi(bandpass(c, a.fps, RR_LO, RR_HI), a.fps, RR_LO, RR_HI)
                       for c in chans])
    rr_top = np.argsort(rr_sqi)[::-1][:a.topk]
    rr_f, rw = [], []
    for i in rr_top:
        ff = fft_peak(chans[i], a.fps, RR_LO, RR_HI)
        if ff: rr_f.append(ff * 60); rw.append(rr_sqi[i])
    rr = np.average(rr_f, weights=rw) if rr_f else None
    f0 = rr / 60.0 if rr else 0.25
    print(f"RR (SQI-weighted) = {rr:.0f} rpm  (f0={f0:.3f}Hz, 5th harm={5*f0*60:.0f}bpm)  "
          f"per-bin: {[round(v) for v in rr_f]}")

    # --- HR: the FFT argmax over the whole HR band is fooled by breathing-
    # harmonic residue (halving error). Fixes: (1) start the band ABOVE the dense
    # RR harmonics (~3.5·f0), (2) narrow-notch the RR harmonic comb, (3) use
    # TIME-DOMAIN beat-count (primary; sharp beats survive, smooth harmonic
    # doesn't) + autocorrelation (cross-check), not FFT argmax. ---
    hr_lo = max(0.9, 3.5 * f0)
    hr_sqi = np.array([sqi(bandpass(c, a.fps, hr_lo, HR_HI, notch_f0=f0),
                           a.fps, hr_lo, HR_HI) for c in chans])
    hr_top = np.argsort(hr_sqi)[::-1][:a.topk]
    print(f"HR band {hr_lo:.2f}-{HR_HI:.1f}Hz (RR-adaptive + notched). top bins: "
          + ", ".join(f"{bins[i]}({bins[i]*DR:.2f}m)" for i in hr_top))
    bc, ac = [], []
    for i in hr_top:
        sig = bandpass(chans[i], a.fps, hr_lo, HR_HI, notch_f0=f0)
        bc.append(beat_count(sig, a.fps))
        aa = autocorr_bpm(sig, a.fps, int(hr_lo * 60), 150)
        if aa: ac.append(aa)
    hr = float(np.median(bc)) if bc else None            # PRIMARY = beat-count
    hr_ac = float(np.median(ac)) if ac else None
    # confidence from inter-bin beat-count agreement (a real HR is consistent
    # across bins; harmonic artifacts scatter). autocorr is only a soft x-check —
    # it locks onto the smooth breathing residue when that outpowers the beats.
    spread = float(np.std(bc)) if len(bc) > 1 else 99.0
    conf = "HIGH" if spread < 4 else ("MED" if spread < 8 else "LOW")
    print(f"  beat-count (PRIMARY) = {hr:.0f} bpm  [{conf}, bin-spread {spread:.1f}]  "
          f"{[round(v) for v in bc]}")
    print(f"  autocorr   (x-check) = {hr_ac:.0f} bpm   {[round(v) for v in ac]}"
          f"{'  (lagging — breathing residue)' if hr and hr_ac and hr - hr_ac > 8 else ''}")
    print(f"  -> HR = {hr:.0f} bpm")


if __name__ == "__main__":
    main()
