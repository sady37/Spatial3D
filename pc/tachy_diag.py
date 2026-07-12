"""Diagnose the post-exercise HR MISS on tachy1 (AppleWatch 135->93, radar flat
81). Hypothesis: elevated RR (19rpm, f0=0.31Hz) makes the breathing-harmonic
comb dense (4x=75,5x=94,6x=113,7x=131bpm) and the RR-notch removes the true HR
where it lands ON a harmonic. Compare cardiac-band spectrum + autocorr WITH vs
WITHOUT the notch, in an early window (HR~130 per watch) and a late one (HR~95),
over the SQI-top cardiac bins. If the true-HR line is present without the notch
and gone with it -> notch is the culprit."""
import numpy as np
from bcg_vitals import demod_channels, bandpass, sqi, fft_peak, autocorr_peak, estimate_rr

import sys
FPS = 18.8
PATH = sys.argv[1] if len(sys.argv) > 1 else "tachy1_cube.npz"
d = np.load(PATH, allow_pickle=True)
cube = np.asarray(d["snapshots"], dtype=np.complex64)
K = int(d["counts"].astype(int).min()); bins = d["bins"].astype(int)
cube = cube[:, :K, :]
W = int(15 * FPS)

for tag, t0 in [("EARLY t~5-20s (watch ~128)", int(5 * FPS)),
                ("LATE  t~95-110s (watch ~95)", int(95 * FPS))]:
    C = cube[:, t0:t0 + W, :]
    chans = demod_channels(C, bins)
    rr, f0, sp, rrf = estimate_rr(chans, FPS)
    print(f"\n===== {tag} =====")
    print(f"RR={rr:.0f}rpm f0={f0:.3f}Hz  breathing harmonics(bpm): "
          f"{[round(f0*60*n) for n in range(3, 9)]}")
    # top cardiac bins by full-band [1.0-2.4] SQI (no notch, to not bias)
    lo, hi = 1.0, 2.4
    s = np.array([sqi(bandpass(c, FPS, lo, hi), FPS, lo, hi) for c in chans])
    top = np.argsort(s)[::-1][:8]
    # per-bin, SQI-RANKED: does the highest-SNR bin carry the true (~110-135) HR?
    # also test de-alias: autocorr height at the FUNDAMENTAL vs at 2x-rate (half lag).
    print(f"  SQI-rank bin   ac_bpm ac_h   |  h@2x-rate  fft_bpm   (true HR 110-135)")
    for r, i in enumerate(top):
        sig = bandpass(chans[i], FPS, lo, hi)
        bpm, h = autocorr_peak(sig, FPS, int(lo * 60), int(hi * 60))
        # height at double rate (half period) — if the true rate is 2x bpm
        h2 = 0.0
        if bpm:
            ac = np.correlate(sig, sig, "full")[len(sig) - 1:]
            ac = ac / (ac[0] + 1e-9)
            lag2 = int(round(FPS / ((2 * bpm) / 60)))
            if 0 < lag2 < len(ac):
                h2 = float(ac[lag2])
        ff = fft_peak(chans[i], FPS, lo, hi)
        print(f"  #{r} bin{int(bins[i]):3d}({bins[i]*0.0234:.2f}m) "
              f"{bpm and round(bpm):>4}bpm h={h:.2f}   h@{bpm and round(2*bpm)}={h2:+.2f}   "
              f"fft={ff and round(ff*60)}bpm")
