"""Does the in-band signal have a SUSTAINED periodic comb (real heartbeat) or a
single damped bump (bandpass ringing on noise)? Extend the autocorr to ~4 cycles
and measure the 2nd and 3rd period-multiple peaks relative to the 1st.

A genuine oscillator: ac(2T)~ac(T), ac(3T) still clearly >0  (comb persists).
Filtered noise: ac(2T) collapses toward 0 (one bump, then decorrelated).

Empty room (occupancy-gated out) is the negative control; the 4 resting cubes
should show a comb the empty room does not IF the mainline truly sees a heart.
"""
import numpy as np
from bcg_vitals import demod_channels, bandpass, sqi

FPS = 18.78
LO, HI = 1.0, 1.7

def load(path):
    d = np.load(path, allow_pickle=True)
    cube = np.asarray(d["snapshots"], dtype=np.complex64)
    bins = d["bins"].astype(int)
    return demod_channels(cube, bins), bins

def comb(sig, fps, maxcyc=4):
    ac = np.correlate(sig, sig, "full")[len(sig)-1:]
    ac = ac / (ac[0] + 1e-12)
    # locate 1st peak in [1.0,1.7]Hz
    l0, l1 = int(fps/(HI)), int(fps/(LO))
    k1 = l0 + int(np.argmax(ac[l0:l1+1]))
    peaks = []
    for n in range(1, maxcyc+1):
        c = n*k1
        if c+2 >= len(ac): break
        # local max within +-2 lags of the n-th multiple
        w = ac[c-2:c+3]
        peaks.append(float(w.max()))
    return k1, peaks

def analyze(path, label):
    chans, bins = load(path)
    hr_sqi = np.array([sqi(bandpass(c, FPS, LO, HI), FPS, LO, HI) for c in chans])
    top = np.argsort(hr_sqi)[::-1][:8]
    combs = []
    for i in top:
        sig = bandpass(chans[i], FPS, LO, HI)
        k1, peaks = comb(sig, FPS)
        combs.append(peaks)
    # align to 4 entries
    combs = [p+[np.nan]*(4-len(p)) for p in combs]
    med = np.nanmedian(np.array(combs), axis=0)
    print(f"{label:34s} median comb ac(1T..4T) = "
          + "  ".join(f"{v:+.2f}" for v in med)
          + f"   [decay 2T/1T={med[1]/med[0]:.2f}, 3T/1T={med[2]/med[0]:.2f}]")

print("period-multiple autocorr comb (1T=one heartbeat period):\n")
analyze("sit33_cube.npz",  "sit33  (person, truth~82)")
analyze("sit39_cube.npz",  "sit39  (person, truth~81)")
analyze("lie41_cube.npz",  "lie41  (person, truth~77)")
analyze("fall20_cube.npz", "fall20 (person, truth~80)")
analyze("emptyL_cube.npz", "emptyL (NO person, control)")
