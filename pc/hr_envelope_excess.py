"""HR via HARMONIC-ENVELOPE EXCESS (user design): RR is slow/stable so the k*RR
harmonic FREQUENCIES are known and the harmonics DECAY with k. Fit that decay
envelope, extrapolate into the cardiac band, and HR = the spectral peak that
EXCEEDS the envelope (i.e. sits BETWEEN harmonics, above the decayed floor).
Since RR is known we PREDICT collision (HR near k*RR) and flag low confidence.

Does NOT model the per-breath waveform (that failed) — only the stable decay envelope.

    python3 hr_envelope_excess.py [cube.npz] [truth] [t0] [t1]
"""
import sys
import numpy as np
from bcg_vitals import demod_channels, bandpass, sqi, RR_LO, RR_HI

HR_LO_BPM, HR_HI_BPM = 50, 130


def load(path):
    d = np.load(path, allow_pickle=True)
    cube = np.asarray(d["snapshots"], np.complex64)
    counts = d["counts"].astype(int); bins = d["bins"].astype(int)
    C = cube[:, :int(counts.min()), :]
    try:
        ts = np.asarray(d["frame_ts"], float)[:int(counts.min())]
        span = ts[-1] - ts[0]
        fps = (len(ts) - 1) / (span / 1000 if span > 1e4 else span)
    except KeyError:
        fps = 18.78
    return C, bins, fps


def main():
    path = sys.argv[1] if len(sys.argv) > 1 else "lie41_cube.npz"
    truth = float(sys.argv[2]) if len(sys.argv) > 2 else 0.0
    t0 = float(sys.argv[3]) if len(sys.argv) > 3 else 0.0
    t1 = float(sys.argv[4]) if len(sys.argv) > 4 else 1e9
    C, bins, fps = load(path)
    i0, i1 = int(t0 * fps), min(C.shape[1], int(t1 * fps))
    C = C[:, i0:i1, :]
    chans = demod_channels(C, bins)
    T = chans.shape[1]

    resp_sqi = np.array([sqi(bandpass(c, fps, RR_LO, RR_HI), fps, RR_LO, RR_HI) for c in chans])
    hr_sqi = np.array([sqi(bandpass(c, fps, HR_LO_BPM/60, HR_HI_BPM/60), fps,
                           HR_LO_BPM/60, HR_HI_BPM/60) for c in chans])
    # pool top HR-SQI bins (favor cardiac visibility); sign-align
    top = np.argsort(hr_sqi)[::-1][:6]
    ref = bandpass(chans[top[0]], fps, 0.1, 2.6)
    x = np.zeros_like(ref)
    for i in top:
        b = bandpass(chans[i], fps, 0.1, 2.6)
        x += np.sign(np.dot(b, ref) + 1e-9) * b
    # amplitude spectrum
    X = 2 * np.abs(np.fft.rfft(x - x.mean())) / len(x)
    f = np.fft.rfftfreq(len(x), 1 / fps)

    # RR from best resp bin
    rb = int(np.argmax(resp_sqi))
    xr = chans[rb]
    Xr = np.abs(np.fft.rfft(xr - xr.mean()))
    mrr = (f >= RR_LO) & (f <= RR_HI)
    f0 = f[mrr][np.argmax(Xr[mrr])]
    rr = f0 * 60

    # harmonic amplitudes on the pooled spectrum, fit log-linear decay envelope
    def peak_near(f0h, df=0.03):
        m = (f >= f0h - df) & (f <= f0h + df)
        return X[m].max() if m.any() else 0.0
    ks = np.arange(1, int((HR_HI_BPM/60) / f0) + 2)
    Ak = np.array([peak_near(k * f0) for k in ks])
    good = Ak > 0
    c = np.polyfit(ks[good], np.log(Ak[good]), 1)
    env = lambda ff: np.exp(np.polyval(c, ff / f0))   # envelope amplitude at freq ff

    # noise floor = median X in non-harmonic parts of the band
    band = (f >= HR_LO_BPM/60) & (f <= HR_HI_BPM/60)
    near_harm = np.zeros_like(f, bool)
    for k in ks:
        near_harm |= np.abs(f - k * f0) <= 0.035
    floor = np.median(X[band & ~near_harm])
    # HR = tallest NON-harmonic peak in the resting-physiological prior [55,105] that
    # clears BOTH the noise floor and the extrapolated harmonic envelope (absolute mm).
    prior = (f >= 55/60) & (f <= 105/60)
    cand = prior & ~near_harm & (X > 1.8 * floor) & (X > env(f))
    if cand.any():
        j = np.where(cand)[0][np.argmax(X[cand])]
        hr = f[j] * 60
        hr_excess = X[j] / (env(f[j]) + 1e-9)
        hr_snr = X[j] / (floor + 1e-9)
    else:
        hr, hr_excess, hr_snr = float("nan"), 0.0, 0.0

    # collision prediction: is the found HR (or truth prior) within df of a harmonic?
    kfrac = (hr / rr)
    coll = abs(kfrac - round(kfrac)) < 0.12
    conf = "LOW(collision risk)" if coll or hr_excess < 1.5 else "OK"

    print(f"{path} [t={t0:.0f}..{min(t1,T/fps+t0):.0f}s]  RR={rr:.1f}rpm  truth={truth:.0f}")
    print(f"  harmonic decay: each x{np.exp(c[0]):.2f}; harmonics at "
          f"{[f'{round(k*rr)}' for k in ks if k*rr<=HR_HI_BPM]} bpm")
    print(f"  -> HR = {hr:.0f} bpm  (at {kfrac:.1f}xRR, x{hr_excess:.1f} over envelope, "
          f"SNR x{hr_snr:.1f} over floor)  [{conf}]" +
          (f"   err {hr-truth:+.0f}" if truth else ""))


if __name__ == "__main__":
    main()
