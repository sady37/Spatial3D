"""Resting/static case: is HR reproducibly computable? Not via the (non-reproducible)
fingerprint -- via the marginal FUNDAMENTAL made robust by long stationary integration.
Test: band-limited autocorrelation HR on the chest bin, split into full/halves/thirds
and across top SQI bins. Report RATE spread (reproducibility) + does the fundamental
finally beat the low-band clutter (unlike tachy).
"""
import numpy as np
from bcg_vitals import demod_channels, bandpass, sqi, estimate_rr, RR_LO, RR_HI

def acf_hr(x, fps, lo=1.0, hi=2.2):
    """HR (bpm) = dominant lag in band-limited autocorrelation. Robust at low SNR:
    integrates every cycle instead of needing one FFT bin to clear."""
    xb = bandpass(x, fps, lo, hi)
    xb = xb - xb.mean()
    ac = np.correlate(xb, xb, 'full')[len(xb)-1:]
    ac /= (ac[0] + 1e-12)
    lo_lag = int(fps/hi); hi_lag = int(fps/lo)
    seg = ac[lo_lag:hi_lag]
    if len(seg) == 0: return 0.0, 0.0
    k = int(np.argmax(seg)) + lo_lag
    return 60.0*fps/k, float(ac[k])          # bpm, periodicity strength (0..1)

def band_clutter(x, fps, f0):
    """peak in HR band (1.2-2.0) vs low clutter band, after breathing-comb notch."""
    xb = bandpass(x, fps, 1.0, 2.6, notch_f0=f0)
    n=len(xb); w=np.hanning(n); Y=np.abs(np.fft.rfft(xb*w)); fr=np.fft.rfftfreq(n,1/fps)
    card = Y[(fr>=1.2)&(fr<=2.0)].max()
    clut = Y[(fr>=0.7)&(fr<=1.2)].max()
    return card, clut

def run(path, fps, expect_bpm, tag):
    d = np.load(path, allow_pickle=True)
    cube = np.asarray(d["snapshots"], np.complex64)
    counts=d["counts"].astype(int); bins=d["bins"].astype(int)
    C=cube[:,:int(counts.min()),:]; chans=demod_channels(C,bins)
    _,f0,_,_=estimate_rr(chans,fps)
    rr=np.array([sqi(bandpass(c,fps,RR_LO,RR_HI),fps,RR_LO,RR_HI) for c in chans])
    order=np.argsort(rr)[::-1]
    ch=chans[order[0]]; T=len(ch); dur=T/fps
    print("\n== %s ==  %.0fs @%.2ffps  RR=%.1f/min  expectHR~%s" % (tag,dur,fps,f0*60,expect_bpm))

    full,st = acf_hr(ch,fps)
    h1,_=acf_hr(ch[:T//2],fps); h2,_=acf_hr(ch[T//2:],fps)
    L=T//3; t1,_=acf_hr(ch[:L],fps); t2,_=acf_hr(ch[L:2*L],fps); t3,_=acf_hr(ch[2*L:],fps)
    print("  chest ACF HR: full=%.1f (strength=%.2f) | halves %.1f/%.1f | thirds %.1f/%.1f/%.1f"
          % (full,st,h1,h2,t1,t2,t3))
    print("  RATE spread: halves d=%.1f  thirds std=%.1f bpm" % (abs(h1-h2), np.std([t1,t2,t3])))

    # across top-6 bins, full-record
    hrs=[acf_hr(chans[order[i]],fps)[0] for i in range(6)]
    print("  top-6 bins HR = %s  -> median %.1f  inter-bin std %.1f"
          % (np.round(hrs,1), np.median(hrs), np.std(hrs)))

    card,clut=band_clutter(ch,fps,f0)
    print("  fundamental vs clutter: cardPk=%.1f clutPk=%.1f  -> %s"
          % (card,clut,"CARD WINS" if card>clut else "clutter wins"))

for path,fps,exp,tag in [
    ("sit33_cube.npz",20.0,"85","sit33 resting (template src)"),
    ("sit39_cube.npz",18.78,"87","sit39 resting"),
    ("fall20_cube.npz",18.78,"82","fall20 resting-after"),
    ("tachy3_cube.npz",18.78,"86","tachy3 recovered-resting"),
    ("sidesit_cube.npz",18.78,"?","sidesit side-on"),
]:
    try: run(path,fps,exp,tag)
    except Exception as e: print("\n== %s == ERROR %s" % (tag,e))
