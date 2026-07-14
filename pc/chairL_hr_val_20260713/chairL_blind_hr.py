"""BLIND resting-HR validation of the chest-bin approach (user's insight: breathing is
abdominal/large; heart is chest/small -> pick the low-breathing chest bin where the
heartbeat is un-buried). NOTHING below uses the truth: bin selection and HR estimation
are truth-free; truth (oximeter ~77) is used ONLY to print the final error.

Pipeline per recording:
  1. demod all range bins -> mm displacement
  2. RR fundamental f0 from the max-breathing (abdomen) bin
  3. BLIND chest bin = body bin maximizing cardiac-SNR = (tallest NON-harmonic peak in
     [1.0,1.7Hz]) / (noise floor). Naturally prefers low-breathing bins with a clean beat.
  4. HR = autocorr in [1.0,1.7Hz] on that bin (validated estimator) + envelope-excess x-check
  5. (optional) Wiener-cancel the abdomen (pure-RR reference) from the chest, re-estimate
Reports HR + error vs oximeter; empty chair must NOT yield a confident, body-anchored HR.

    python3 chairL_blind_hr.py
"""
import numpy as np
from scipy.signal import csd, welch
from bcg_vitals import demod_channels, bandpass, autocorr_peak, RR_LO, RR_HI

TRUTH = 77.0                                   # oximeter 74-81, avg ~77 (REPORT ONLY)
HRLO, HRHI = 1.0, 1.7                          # validated resting cardiac band
BLOCKS = ["chairL_20260713_183013.npz", "chairL_20260713_183514.npz",
          "chairL_20260713_184015.npz", "chairL_20260713_184515.npz"]
T0, T1 = 60.0, 300.0                           # skip the just-sat-down transient


def load(path):
    d = np.load(path, allow_pickle=True)
    cube = np.asarray(d["snapshots"], np.complex64)
    counts = d["counts"].astype(int); bins = d["bins"].astype(int)
    C = cube[:, :int(counts.min()), :]
    dr = float(d["dr_m"]) if "dr_m" in d.files else 0.0234
    try:
        ts = np.asarray(d["frame_ts"], float)[:int(counts.min())]
        s = ts[-1] - ts[0]; fps = (len(ts) - 1) / (s / 1000 if s > 1e4 else s)
    except KeyError:
        fps = 18.78
    return C, bins, fps, dr


def cardiac_snr(disp, fps, f0):
    """tallest NON-harmonic peak in [HRLO,HRHI] over the local noise floor + its freq."""
    N = len(disp); f = np.fft.rfftfreq(N, 1 / fps)
    X = 2 * np.abs(np.fft.rfft(disp - disp.mean())) / N
    band = (f >= HRLO) & (f <= HRHI)
    nharm = np.zeros_like(f, bool)
    for k in range(1, 12):
        nharm |= np.abs(f - k * f0) <= 0.035
    cand = band & ~nharm
    if not cand.any():
        return 0.0, None
    floor = np.median(X[band & ~nharm])
    j = np.where(cand)[0][np.argmax(X[cand])]
    return X[j] / (floor + 1e-9), f[j] * 60


def analyse(path, tw=(T0, T1)):
    C, bins, fps, dr = load(path)
    i0, i1 = int(tw[0] * fps), min(C.shape[1], int(tw[1] * fps))
    C = C[:, i0:i1, :]
    disp = demod_channels(C, bins)
    N = disp.shape[1]

    # breathing amplitude per bin; abdomen = max; f0 from abdomen
    rr_amp = np.array([bandpass(d, fps, RR_LO, 0.6).std() for d in disp])
    ab = int(np.argmax(rr_amp))
    f = np.fft.rfftfreq(N, 1 / fps)
    Xr = np.abs(np.fft.rfft(bandpass(disp[ab], fps, RR_LO, RR_HI)))
    m = (f >= RR_LO) & (f <= RR_HI); f0 = f[m][np.argmax(Xr[m])]

    # BLIND chest bin: body bins (breathing present) ranked by cardiac-SNR
    body = rr_amp > 0.12 * rr_amp.max()
    csnr = np.array([cardiac_snr(disp[i], fps, f0)[0] if body[i] else 0.0
                     for i in range(len(bins))])
    ch = int(np.argmax(csnr))

    def hr_autocorr(d):
        v, h = autocorr_peak(bandpass(d, fps, HRLO, HRHI), fps, 60, 102)
        return v, h
    hr_ab, _ = hr_autocorr(disp[ab])
    hr_ch, hh = hr_autocorr(disp[ch])
    _, hr_ex = cardiac_snr(disp[ch], fps, f0)

    # Wiener cancel abdomen (pure-RR ref) from chest
    nper = int(min(40 * fps, N // 3))
    fw, Paa = welch(disp[ab], fps, nperseg=nper)
    _, Pca = csd(disp[ab], disp[ch], fps, nperseg=nper)
    H = Pca / (Paa + 1e-12)
    Hf = np.interp(f, fw, H.real) + 1j * np.interp(f, fw, H.imag)
    r = np.fft.irfft(np.fft.rfft(disp[ch]) - Hf * np.fft.rfft(disp[ab]), N)
    hr_w, _ = hr_autocorr(r)

    return dict(bins=bins, dr=dr, ab=ab, ch=ch, f0=f0, csnr=csnr[ch],
                rr_amp=rr_amp, hr_ab=hr_ab, hr_ch=hr_ch, hr_ex=hr_ex,
                hr_w=hr_w, hh=hh)


def main():
    print(f"BLIND chest-bin resting HR (truth-free selection+estimation; oximeter {TRUTH:.0f} "
          f"= error only)\n  window t={T0:.0f}-{T1:.0f}s\n")
    print(f"  {'block':26} {'abdo(m)':8} {'chest(m)':9} {'RR':>4} {'HRabdo':>7} "
          f"{'HRchest':>8} {'HRwiener':>9} {'excess':>7} {'cSNR':>5}")
    errs = []
    for b in BLOCKS:
        try:
            r = analyse(b)
        except FileNotFoundError:
            print(f"  {b:26} (missing)"); continue
        errs.append(r["hr_ch"] - TRUTH)
        print(f"  {b.replace('chairL_','').replace('.npz',''):26} "
              f"{r['bins'][r['ab']]*r['dr']:8.2f} {r['bins'][r['ch']]*r['dr']:9.2f} "
              f"{r['f0']*60:4.0f} {r['hr_ab']:7.0f} {r['hr_ch']:8.0f} {r['hr_w']:9.0f} "
              f"{r['hr_ex']:7.0f} {r['csnr']:5.1f}   err(chest) {r['hr_ch']-TRUTH:+.0f}")
    if errs:
        print(f"\n  chest-bin HR: mean|err|={np.mean(np.abs(errs)):.1f}  "
              f"bias={np.mean(errs):+.1f}  vs oximeter {TRUTH:.0f}")

    # NULL: empty chair — should have NO body / no confident anchored HR
    try:
        r = analyse("emptychair_20260713_192151.npz", (0, 300))
        print(f"\n  NULL emptychair: chest bin {r['bins'][r['ch']]*r['dr']:.2f}m "
              f"cSNR={r['csnr']:.1f} HR={r['hr_ch']:.0f} (no oximeter; cSNR/HR should look "
              f"weak/unstable vs person)")
    except FileNotFoundError:
        pass


if __name__ == "__main__":
    main()
