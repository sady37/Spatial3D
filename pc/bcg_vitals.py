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
    # median (robust) not SQI-weighted: a couple of near bins carry low-freq
    # artifact energy -> high SQI but a wrong (too-low) RR that skews the average.
    rr = float(np.median(rr_f)) if rr_f else None
    f0 = rr / 60.0 if rr else 0.25
    rr_spread = float(np.std(rr_f)) if len(rr_f) > 1 else 99.0
    rr_conf = "HIGH" if rr_spread < 2 else ("MED" if rr_spread < 4 else "LOW")
    print(f"RR (median) = {rr:.0f} rpm  [{rr_conf}, bin-spread {rr_spread:.1f}]  "
          f"(f0={f0:.3f}Hz, 5th harm={5*f0*60:.0f}bpm)  per-bin: {[round(v) for v in rr_f]}")

    # --- HR: at ~4m the cardiac phase is WEAKER than the breathing-harmonic
    # residue, so any "find the strongest peak" (FFT argmax, harmonic-sum, or
    # coprime/CRT folding) locks onto the residue -> halving. No transform lifts a
    # sub-noise signal; the lever is a PHYSIOLOGICAL BAND PRIOR that excludes the
    # low-freq residue. AUTOCORRELATION in [1.0-1.7Hz] (60-102bpm, resting-elderly
    # prior) is SNR-robust (responds to the period, not a single peak) and matched
    # Apple Watch across seated/side/lying (all ~81). Beat-count is a strong-signal
    # cross-check. Widen HR_PHYS_HI only if tachycardia must be caught. ---
    HR_PHYS_LO, HR_PHYS_HI = 1.0, 1.7
    hr_sqi = np.array([sqi(bandpass(c, a.fps, HR_PHYS_LO, HR_PHYS_HI, notch_f0=f0),
                           a.fps, HR_PHYS_LO, HR_PHYS_HI) for c in chans])
    hr_top = np.argsort(hr_sqi)[::-1][:a.topk]
    print(f"HR band {HR_PHYS_LO}-{HR_PHYS_HI}Hz (physiological prior + RR-notch). "
          f"top bins: " + ", ".join(f"{bins[i]}({bins[i]*DR:.2f}m)" for i in hr_top))
    ac, bc = [], []
    for i in hr_top:
        sig = bandpass(chans[i], a.fps, HR_PHYS_LO, HR_PHYS_HI, notch_f0=f0)
        aa = autocorr_bpm(sig, a.fps, int(HR_PHYS_LO * 60), int(HR_PHYS_HI * 60))
        if aa: ac.append(aa)
        bc.append(beat_count(sig, a.fps, hi_bpm=int(HR_PHYS_HI * 60)))
    hr = float(np.median(ac)) if ac else None            # PRIMARY = autocorr@band
    hr_bc = float(np.median(bc)) if bc else None
    spread = float(np.std(ac)) if len(ac) > 1 else 99.0
    conf = "HIGH" if spread < 3 else ("MED" if spread < 6 else "LOW")
    print(f"  autocorr@band (PRIMARY) = {hr:.0f} bpm  [{conf}, bin-spread {spread:.1f}]  "
          f"{[round(v) for v in ac]}")
    print(f"  beat-count    (x-check) = {hr_bc:.0f} bpm   {[round(v) for v in bc]}")
    print(f"  -> HR = {hr:.0f} bpm")


if __name__ == "__main__":
    main()
