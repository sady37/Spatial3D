"""Test a HARMONIC-NUMBER-SCALED breathing notch to separate real tachy from
resting spurious high-freq peaks. Hypothesis: resting cubes' high-freq FFT
clusters (e.g. lie41 120/135) are HIGH-ORDER breathing harmonics (8x/9x of RR);
the fixed 0.022Hz notch misses them because harmonic n has frequency error ~n*df0.
Widening the notch as n grows should delete those harmonic false-clusters while
sparing a true HR that does NOT sit on n*f0 (tachy2's ~120).

Compare per-bin FFT peaks in [1.0-2.4Hz] with fixed vs scaled-width notch."""
import numpy as np
from bcg_vitals import demod_channels, estimate_rr, sqi, fft_peak, occupancy

FPS = 18.8
LO, HI, SPLIT = 1.0, 2.4, 1.7
CASES = [("tachy2(HR110-131)", "tachy2_cube.npz"),
         ("tachy3(HR84-87)", "tachy3_cube.npz"),
         ("rest_near(rest)", "rest_near_cube.npz"),
         ("sit39(rest81)", "sit39_cube.npz"),
         ("sidesit(rest81)", "sidesit_cube.npz"),
         ("lie41(rest87)", "lie41_cube.npz"),
         ("fall20(rest81)", "fall20_cube.npz")]


def bandpass_scaled_notch(x, fps, lo, hi, f0, hw0=0.022, kscale=1.0, nmax=25):
    """bandpass [lo,hi] with a breathing-harmonic notch whose half-width GROWS
    with harmonic number n: hw_n = hw0*(1 + kscale*(n-1)). kscale=0 == fixed."""
    x = x - x.mean()
    f = np.fft.rfftfreq(len(x), 1 / fps)
    X = np.fft.rfft(x)
    X[(f < lo) | (f > hi)] = 0
    if f0:
        for n in range(1, nmax):
            hw = hw0 * (1 + kscale * (n - 1))
            X[(f >= n * f0 - hw) & (f <= n * f0 + hw)] = 0
    return np.fft.irfft(X, n=len(x))


def peaks(chans, fps, f0, kscale, topk=8):
    ff = []
    for c in chans:
        sig = bandpass_scaled_notch(c, fps, LO, HI, f0, kscale=kscale)
        F = np.fft.rfftfreq(len(sig), 1 / fps)
        S = np.abs(np.fft.rfft(sig)) ** 2
        m = (F >= LO) & (F <= HI)
        if m.any() and S[m].sum() > 0:
            ff.append(F[m][np.argmax(S[m])] * 60)
    # SQI-top selection uses the same scaled notch
    sqis = [sqi(bandpass_scaled_notch(c, fps, LO, HI, f0, kscale=kscale), fps, LO, HI)
            for c in chans]
    top = np.argsort(sqis)[::-1][:topk]
    ftop = []
    for i in top:
        sig = bandpass_scaled_notch(chans[i], fps, LO, HI, f0, kscale=kscale)
        F = np.fft.rfftfreq(len(sig), 1 / fps)
        S = np.abs(np.fft.rfft(sig)) ** 2
        m = (F >= LO) & (F <= HI)
        if m.any() and S[m].sum() > 0:
            ftop.append(F[m][np.argmax(S[m])] * 60)
    ftop = np.array(ftop)
    frac = float(np.mean(ftop > SPLIT * 60)) if len(ftop) else 0.0
    hi = ftop[ftop > SPLIT * 60]
    return frac, (float(np.median(hi)) if len(hi) else None), np.sort(ftop)


print(f"{'case':20} {'RR':>4}   fixed-notch          scaled-notch(k=1.5)")
print(f"{'':20} {'':>4}   frac  hi_med  peaks   frac  hi_med  peaks")
for name, path in CASES:
    d = np.load(path, allow_pickle=True)
    cube = np.asarray(d["snapshots"], dtype=np.complex64)
    K = int(d["counts"].astype(int).min()); bins = d["bins"].astype(int)
    W = min(K, int(20 * FPS))
    chans = demod_channels(cube[:, :W, :], bins)
    _, f0, _, _ = estimate_rr(chans, FPS)
    f_fix, m_fix, p_fix = peaks(chans, FPS, f0, 0.0)
    f_sc, m_sc, p_sc = peaks(chans, FPS, f0, 1.5)
    print(f"{name:20} {f0*60:4.0f}   {f_fix:4.0%} {str(m_fix):>6}  "
          f"{np.round(p_fix).astype(int).tolist()}")
    print(f"{'':20} {'':>4}                          {f_sc:4.0%} {str(m_sc):>6}  "
          f"{np.round(p_sc).astype(int).tolist()}")
