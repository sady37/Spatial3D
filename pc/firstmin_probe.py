"""First-minute vs second-minute cardiac-band probe on tachy2.

Question under test: the whole-record blind HR reads ~87bpm (the recovery tail).
If that read is just "tail overwhelming head", then restricting to the FIRST 60s
-- where the true HR is still 100+bpm post-exercise -- should surface a ~130bpm
peak in the head. Does it? Or does the head-segment cardiac band collapse below
even the tail's line?
"""
import numpy as np
from bcg_vitals import (demod_channels, estimate_rr, bandpass, sqi,
                        fft_peak, RR_LO, RR_HI)

FPS = 18.78
HP_LO, HP_HI = 0.10, 3.2
CARD_LO, CARD_HI = 1.2, 2.6      # 72..156 bpm search band

d = np.load("tachy2_cube.npz", allow_pickle=True)
cube = np.asarray(d["snapshots"], np.complex64)
counts = d["counts"].astype(int); bins = d["bins"].astype(int)
C = cube[:, :int(counts.min()), :]
chans = demod_channels(C, bins)          # (nbin, T)
T = chans.shape[1]
dur = T / FPS
print(f"T={T} snapshots, {dur:.1f}s total, {chans.shape[0]} bins")

# chest bin = top resp-SQI (same rule as load_bin)
_, f0, _, _ = estimate_rr(chans, FPS)
rr = np.array([sqi(bandpass(c, FPS, RR_LO, RR_HI), FPS, RR_LO, RR_HI) for c in chans])
idx = int(np.argsort(rr)[::-1][0])
print(f"chest bin idx={idx} bin#={bins[idx]}  f0(RR)={f0*60:.1f} bpm")

y_full = bandpass(chans[idx], FPS, HP_LO, HP_HI)

def card_spectrum(y, fps, lo, hi, f0):
    """band-limited cardiac spectrum with breathing comb notched at f0 (same as EKF
    seed_fh0 preprocessing intent). Returns (freqs_bpm, mag, peak_bpm, peak_mag,
    band_snr = peak / median-in-band)."""
    yb = bandpass(y, fps, lo, hi, notch_f0=f0)
    n = len(yb)
    w = np.hanning(n)
    Y = np.abs(np.fft.rfft(yb * w))
    fr = np.fft.rfftfreq(n, 1/fps)
    m = (fr >= lo) & (fr <= hi)
    frb, Yb = fr[m], Y[m]
    k = int(np.argmax(Yb))
    snr = Yb[k] / (np.median(Yb) + 1e-9)
    return frb*60, Yb, frb[k]*60, Yb[k], snr

nhalf = T // 2
segs = {
    "full   [0-120s]": y_full,
    "1st min[0-60s ]": y_full[:nhalf],
    "2nd min[60-120s]": y_full[nhalf:],
    "head   [0-40s ]": y_full[:int(40*FPS)],
    "tail   [80-120s]": y_full[int(80*FPS):],
}

print("\n%-18s %8s %10s %8s" % ("segment", "peakBPM", "peakMag", "bandSNR"))
print("-"*48)
res = {}
for name, seg in segs.items():
    _, _, pk, pm, snr = card_spectrum(seg, FPS, CARD_LO, CARD_HI, f0)
    res[name] = (pk, pm, snr)
    print("%-18s %8.1f %10.3f %8.2f" % (name, pk, pm, snr))

# Also: sweep sub-windows to see WHERE in time a 100+ peak, if any, lives
print("\nsliding 20s windows (hop 10s): peak bpm / bandSNR")
win = int(20*FPS); hop = int(10*FPS)
s = 0
while s + win <= T:
    seg = y_full[s:s+win]
    _, _, pk, pm, snr = card_spectrum(seg, FPS, CARD_LO, CARD_HI, f0)
    t0, t1 = s/FPS, (s+win)/FPS
    tag = "  <-- >100bpm" if pk > 100 else ""
    print("  [%3.0f-%3.0fs] peak=%6.1f bpm  SNR=%5.2f%s" % (t0, t1, pk, snr, tag))
    s += hop
