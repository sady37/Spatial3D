"""Scan every range bin (and a coherent combo) for a JKL-fingerprint cardiac line
in tachy2's early window. Two questions:
  1. Does ANY bin lift the tachy line (102-144bpm) above its own low-band clutter?
  2. In which bin does the JKL 2f/3f/4f magnitude fingerprint actually appear?
"""
import numpy as np
from bcg_vitals import demod_channels, bandpass, sqi, RR_LO, RR_HI

FPS = 18.78
TP = np.array([-1.209, -0.176, -0.306, -0.534])
TQ = np.array([-0.068,  0.283, -0.167,  0.124])
TMAG = np.sqrt(TP**2 + TQ**2)            # per-harmonic magnitude fingerprint
TMAG_n = TMAG / np.linalg.norm(TMAG)     # unit vector [h1..h4]
print("JKL fingerprint |h1..h4| =", np.round(TMAG, 3),
      " normalized ->", np.round(TMAG_n, 3))

d = np.load("tachy2_cube.npz", allow_pickle=True)
cube = np.asarray(d["snapshots"], np.complex64)
counts = d["counts"].astype(int); bins = d["bins"].astype(int)
C = cube[:, :int(counts.min()), :]
chans = demod_channels(C, bins)

T0, T1 = int(0*FPS), int(30*FPS)         # early window: HR ~131->120, tight band
FH_LO, FH_HI = 1.70, 2.40                # 102 .. 144 bpm candidate fundamentals

def spec(seg):
    n = len(seg); w = np.hanning(n)
    Y = np.abs(np.fft.rfft(seg * w)); fr = np.fft.rfftfreq(n, 1/FPS)
    return fr, Y

def mag_near(fr, Y, fc, tol=0.04):
    m = (fr >= fc-tol) & (fr <= fc+tol)
    return Y[m].max() if m.any() else 0.0

def fingerprint_scan(seg):
    """best cardiac fundamental in [FH_LO,FH_HI] by harmonic-template match.
    Returns (fH_bpm, corr, absenergy, high_peak_mag, low_clutter_mag)."""
    yb = bandpass(seg, FPS, 1.2, 9.0)     # up to 4th harmonic of ~2.2Hz
    fr, Y = spec(yb)
    best = (0, -1, 0)
    for fH in np.arange(FH_LO, FH_HI, 0.01):
        mags = np.array([mag_near(fr, Y, k*fH) for k in range(1, 5)])
        nrm = np.linalg.norm(mags)
        if nrm < 1e-9:
            continue
        corr = float(np.dot(mags/nrm, TMAG_n))   # shape match 0..1
        eng = float(nrm)
        if corr > best[1]:
            best = (fH*60, corr, eng)
    # clutter references in the SAME early window
    frb, Yb = spec(bandpass(seg, FPS, 1.2, 2.6))
    hi = Yb[(frb >= 1.75) & (frb <= 2.5)].max()
    lo = Yb[(frb >= 1.2) & (frb <= 1.75)].max()
    return best[0], best[1], best[2], hi, lo

print("\nbin  #   respSQI | fpFH(bpm) fpCorr fpEng | hiPk  loPk  hi>lo")
print("-"*66)
rows = []
for i in range(len(bins)):
    seg = chans[i, T0:T1]
    rs = sqi(bandpass(chans[i], FPS, RR_LO, RR_HI), FPS, RR_LO, RR_HI)
    fh, corr, eng, hi, lo = fingerprint_scan(seg)
    rows.append((i, bins[i], rs, fh, corr, eng, hi, lo))
    flag = "HI" if hi > lo else "  "
    print("%3d %4d  %6.2f | %7.1f  %5.2f %6.1f | %5.1f %5.1f  %s"
          % (i, bins[i], rs, fh, corr, eng, hi, lo, flag))

# rank by fingerprint quality (corr * energy) among physically-plausible bins
print("\n== top-6 bins by fingerprint score (corr*energy) ==")
rows.sort(key=lambda r: r[4]*r[5], reverse=True)
for r in rows[:6]:
    print("  bin#%-4d respSQI=%.2f  fpFH=%.1fbpm corr=%.2f eng=%.1f  hiPk=%.1f loPk=%.1f"
          % (r[1], r[2], r[3], r[4], r[5], r[6], r[7]))

# coherent multi-bin combine over the chest cluster (top-respSQI +/-2 bins)
resp = np.array([sqi(bandpass(chans[i], FPS, RR_LO, RR_HI), FPS, RR_LO, RR_HI)
                 for i in range(len(bins))])
c0 = int(np.argmax(resp))
grp = range(max(0, c0-2), min(len(bins), c0+3))
# phase-align by cross-correlation to chest bin, then sum
ref = chans[c0, T0:T1]
combo = np.zeros_like(ref)
for i in grp:
    s = chans[i, T0:T1]
    lag = np.argmax(np.correlate(s - s.mean(), ref - ref.mean(), 'full')) - (len(ref)-1)
    combo += np.roll(s, -lag)
fh, corr, eng, hi, lo = fingerprint_scan(combo)
print("\n== coherent combo of chest bins %s ==" % list(grp))
print("  fpFH=%.1fbpm corr=%.2f eng=%.1f | hiPk=%.1f loPk=%.1f  %s"
      % (fh, corr, eng, hi, lo, "HI>lo" if hi > lo else "lo>=hi"))
