"""Decisive test: is true tachy (1.83-2.18Hz) PRESENT & PERSISTENT in tachy2 at all?

If a real 110-131bpm cardiac rhythm exists, a short-time spectrum shows a STABLE
RIDGE in [1.83,2.18]Hz across the recording. A breathing PM-harmonic that merely
wanders through that band jitters frame-to-frame. This test is estimator-independent:
it asks the physics question directly, and compares tachy2 (TRUE 110-131) against
sit39 (TRUE ~81, must NOT show a tachy ridge).

For each cube we take the single best cardiac bin and, per STFT frame, record where
the spectral argmax lands in [1.0-2.5]Hz. Metric = fraction of frames whose peak sits
in the tachy sub-band [1.83-2.18]. tachy2 >> sit39 => signal present, keep engineering.
tachy2 ~ sit39 => genuinely inseparable in this array/geometry; report that verdict.

    python tachy_existence_probe.py
"""
import numpy as np
from bcg_vitals import (bandpass, sqi, fft_peak, demod_channels, LAMBDA_MM,
                        RR_LO, RR_HI)

TACHY_LO, TACHY_HI = 1.83, 2.18       # 110-131 bpm
CARD_LO, CARD_HI = 1.0, 2.5


def stft_peaks(sig, fps, win_s=8, step_s=1):
    """Per-frame spectral argmax within [CARD_LO,CARD_HI] Hz -> list of peak freqs."""
    n = int(win_s * fps); step = int(step_s * fps)
    peaks = []
    for s in range(0, len(sig) - n, step):
        seg = sig[s:s + n]
        f = np.fft.rfftfreq(n, 1 / fps)
        S = np.abs(np.fft.rfft(seg - seg.mean())) ** 2
        m = (f >= CARD_LO) & (f <= CARD_HI)
        peaks.append(f[m][np.argmax(S[m])])
    return np.array(peaks)


def analyze(path, fps, true_hr):
    d = np.load(path, allow_pickle=True)
    cube = np.asarray(d["snapshots"], dtype=np.complex64)
    counts = d["counts"].astype(int); bins = d["bins"].astype(int)
    C = cube[:, :int(counts.min()), :]
    chans = demod_channels(C, bins)

    # cardiac-band SQI to pick the best cardiac bin (broad band, not breathing)
    card_sqi = np.array([sqi(bandpass(c, fps, CARD_LO, CARD_HI), fps, CARD_LO, CARD_HI)
                         for c in chans])
    best = int(np.argmax(card_sqi))
    sig = bandpass(chans[best], fps, CARD_LO, CARD_HI)
    peaks = stft_peaks(sig, fps)
    in_tachy = np.mean((peaks >= TACHY_LO) & (peaks <= TACHY_HI))

    # also: whole-record spectrum, top-3 peaks in the cardiac band
    f = np.fft.rfftfreq(len(sig), 1 / fps)
    S = np.abs(np.fft.rfft(sig - sig.mean())) ** 2
    m = (f >= CARD_LO) & (f <= CARD_HI)
    fb, Sb = f[m], S[m]
    order = np.argsort(Sb)[::-1][:3]
    tops = [(round(fb[o] * 60), round(Sb[o] / Sb.max(), 2)) for o in order]

    print(f"\n=== {path}  (TRUE {true_hr}) best cardiac bin {bins[best]} "
          f"({bins[best]*0.0234375:.2f}m) ===")
    print(f"  STFT frames with peak in TACHY band [110-131]: {in_tachy:6.1%}  "
          f"(n={len(peaks)} frames)")
    print(f"  peak-freq spread: median {np.median(peaks)*60:.0f}bpm  "
          f"IQR {np.percentile(peaks,25)*60:.0f}-{np.percentile(peaks,75)*60:.0f}  "
          f"std {np.std(peaks)*60:.0f}bpm  (low std = stable ridge)")
    print(f"  whole-record top-3 cardiac-band peaks (bpm,rel): {tops}")


if __name__ == "__main__":
    analyze("tachy2_cube.npz", 18.78, "110-131")
    analyze("sit39_cube.npz", 18.78, "~81")
    analyze("tachy1_cube.npz", 18.78, "far, >110")
