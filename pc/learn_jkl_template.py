"""M2 step 1 — learn a PHASE-NORMALIZED JKL cardiac template g_JKL(phi) from clean
beats and Fourier-expand to fixed coeffs {p_m,q_m} for the M2 EKF cardiac term.

Two fixes over beat_morph.py's template:
  1. PHASE-normalized averaging. beat_morph averages beats in a FIXED 0.40s TIME
     window; at tachy the IBI changes (0.47s early -> 0.65s late), so the same window
     smears the very harmonic ratios that ARE the JKL fingerprint. Here each inter-beat
     interval is resampled to N bins over phi in [0,2pi) -> rate-invariant shape.
  2. WIDE band for shape, NARROW band for detection. The JKL sharp deflection puts
     energy in 2f_H,3f_H; beat_morph's residual bandpass[1.0,2.8] CUTS those (at tachy
     2.18Hz, 2f=4.4/3f=6.5Hz are above 2.8) -> the learned shape collapses to a sine
     with NO fingerprint. So detect beats on the narrow-band residual (good SNR) but
     LEARN the shape on a wide-band residual (keeps the harmonics).

Output: g_JKL(phi), coeffs {p_m,q_m}_1..M (unit-RMS), the fingerprint magnitude ratios
|h2|/|h1|,|h3|/|h1|, and a plot. These coeffs are frozen into joint_ekf.py's cardiac
term in M2 step 2:  y_H = a_H * sum_m [p_m cos(m phi_H) + q_m sin(m phi_H)].

    python learn_jkl_template.py                       # tachy3 whole (clean, flat 85)
    python learn_jkl_template.py tachy2_cube.npz 0 30   # tachy2 early strong beats
"""
import argparse
import numpy as np
from scipy.signal import find_peaks
import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
from bcg_vitals import bandpass, sqi, demod_channels, estimate_rr, RR_LO, RR_HI
from peak_cycle_probe import peak_cycle_subtract

FPS = 18.78
DET_LO, DET_HI = 1.0, 2.8        # narrow band: detect beats (SNR on the fundamental)
SHAPE_LO = 1.0                    # wide band low; high = keep 2f,3f (set per Nyquist)


def load_residuals(path, i0, i1, ntop=6):
    """Return (s_shape, f0, shape_hi): the cardiac residual over frames [i0,i1) for shape
    learning. Breathing is removed by PRECISE brick-wall NOTCHING of the whole k*f0 comb
    (bcg_vitals.bandpass notch_f0) on the chest bin -- this collapsed the folding 'noise
    forest' (residual breathing) that peak_cycle_subtract left, sharpening the tachy3
    cardiac peak from max/med 38.8 -> 85.6 (user: RR is fully known, subtract it exactly
    first, THEN the JKL fingerprint emerges). Wide band [SHAPE_LO, ~7Hz] keeps 2f,3f."""
    d = np.load(path, allow_pickle=True)
    cube = np.asarray(d["snapshots"], np.complex64)
    counts = d["counts"].astype(int); bins = d["bins"].astype(int)
    C = cube[:, :int(counts.min()), :]
    chans = demod_channels(C, bins)
    _, f0, _, _ = estimate_rr(chans, FPS)
    rr = np.array([sqi(bandpass(c, FPS, RR_LO, RR_HI), FPS, RR_LO, RR_HI) for c in chans])
    top = np.argsort(rr)[::-1][:ntop]
    shape_hi = min(7.0, FPS / 2 - 0.5)
    chest = chans[top[0]][i0:i1]
    s_shape = bandpass(chest, FPS, SHAPE_LO, shape_hi, notch_f0=f0)  # notch k*f0 comb
    return s_shape / (s_shape.std() + 1e-9), f0, shape_hi


def _profile(s_shape, f, t, N):
    """Fold s_shape at frequency f: average by phase phi=2pi f t (mod 2pi) into N bins."""
    ph = (f * t) % 1.0
    idx = np.minimum((ph * N).astype(int), N - 1)
    prof = np.zeros(N); cnt = np.zeros(N)
    np.add.at(prof, idx, s_shape); np.add.at(cnt, idx, 1)
    prof = prof / np.maximum(cnt, 1)
    return prof - prof.mean()


def learn_template(s_shape, f_lo, f_hi, N=64, M=4, nf=800):
    """EPOCH FOLDING (no per-beat detection). Search cardiac f in [f_lo,f_hi] maximizing
    the folded-profile variance (phase-coherent power = epoch-folding periodogram), then
    fold the WIDE-band residual at that f into g(phi). Robust at far range / low SNR where
    peak detection over-counts. f_lo/f_hi are a TIGHT prior around the labeled clean
    segment's known rate (legit: template is learned once from a labeled segment, frozen).
    Fourier-expand g -> {p_m,q_m}_1..M (unit-RMS)."""
    t = np.arange(len(s_shape)) / FPS
    fs = np.linspace(f_lo, f_hi, nf)
    var = np.array([np.var(_profile(s_shape, f, t, N)) for f in fs])
    f = fs[int(np.argmax(var))]
    g = _profile(s_shape, f, t, N)
    # Fourier: g(phi) ~ sum_m p_m cos(m phi) + q_m sin(m phi)
    phi = np.arange(N) / N * 2 * np.pi
    p = np.array([2.0 / N * np.sum(g * np.cos(m * phi)) for m in range(1, M + 1)])
    q = np.array([2.0 / N * np.sum(g * np.sin(m * phi)) for m in range(1, M + 1)])
    rms = np.sqrt(0.5 * np.sum(p ** 2 + q ** 2))      # RMS of the M-harmonic template
    p, q = p / rms, q / rms
    ncyc = (t[-1] * f)                                # cardiac cycles folded
    return g, p, q, f, int(ncyc), fs, var


def recon(p, q, phi):
    return sum(p[m] * np.cos((m + 1) * phi) + q[m] * np.sin((m + 1) * phi)
              for m in range(len(p)))


def main():
    global FPS
    ap = argparse.ArgumentParser()
    ap.add_argument("path", nargs="?", default="tachy3_cube.npz")
    ap.add_argument("t0", nargs="?", type=float, default=0.0)
    ap.add_argument("t1", nargs="?", type=float, default=None)
    ap.add_argument("--flo", type=float, default=1.30, help="cardiac f search low Hz")
    ap.add_argument("--fhi", type=float, default=1.55, help="cardiac f search high Hz")
    ap.add_argument("--fps", type=float, default=FPS, help="frame rate (sit33 3.3m=20fps)")
    ap.add_argument("-M", type=int, default=4, help="template harmonics")
    ap.add_argument("-N", type=int, default=64, help="phase bins per cycle")
    a = ap.parse_args()
    FPS = a.fps

    d = np.load(a.path, allow_pickle=True)
    T_all = int(d["counts"].astype(int).min())
    i0 = int(a.t0 * FPS)
    i1 = int(a.t1 * FPS) if a.t1 is not None else T_all
    i1 = min(i1, T_all)

    s_shape, f0, shape_hi = load_residuals(a.path, i0, i1)
    print(f"{a.path}  {a.t0:.0f}-{i1/FPS:.0f}s  RR f0={f0*60:.1f}rpm  "
          f"shape band [{SHAPE_LO},{shape_hi:.1f}]Hz  fold search [{a.flo},{a.fhi}]Hz")

    g, p, q, f, ncyc, fs, var = learn_template(s_shape, a.flo, a.fhi, N=a.N, M=a.M)
    mag = np.hypot(p, q)                               # |h_m|
    print(f"folded f={f:.3f}Hz ({f*60:.0f}bpm)  over {ncyc} cardiac cycles")
    print("  m :   p_m      q_m     |h_m|   |h_m|/|h1|")
    for m in range(a.M):
        print(f"  {m+1} : {p[m]:+7.3f}  {q[m]:+7.3f}  {mag[m]:6.3f}   {mag[m]/mag[0]:6.3f}")
    print(f"  fingerprint: |h2|/|h1|={mag[1]/mag[0]:.3f}  |h3|/|h1|={mag[2]/mag[0]:.3f}"
          + (f"  |h4|/|h1|={mag[3]/mag[0]:.3f}" if a.M >= 4 else ""))
    print("  p_m =", np.array2string(p, precision=4, separator=", "))
    print("  q_m =", np.array2string(q, precision=4, separator=", "))

    # --- plot: folded template + M-harmonic recon | harmonic bars | folding periodogram
    phi = np.arange(a.N) / a.N * 2 * np.pi
    gr = recon(p, q, phi)
    gr = gr / (np.sqrt(np.mean(gr ** 2)) + 1e-9) * (np.sqrt(np.mean(g ** 2)))
    fig, ax = plt.subplots(1, 3, figsize=(16, 4.2))
    ax[0].plot(phi, g / (g.std() + 1e-9), "k-", lw=1.6, label="folded g_JKL(phi)")
    ax[0].plot(phi, gr / (g.std() + 1e-9), "r--", lw=1.6, label=f"{a.M}-harmonic recon")
    ax[0].axhline(0, color="0.7", lw=0.6)
    ax[0].set_xlabel("phi (rad)"); ax[0].set_ylabel("norm. amplitude")
    ax[0].set_title(f"{a.path} phase-folded JKL template ({ncyc} cycles, {f*60:.0f}bpm)")
    ax[0].legend(fontsize=9)
    ax[1].bar(np.arange(1, a.M + 1), mag / mag[0], color="steelblue", width=0.6)
    ax[1].set_xlabel("harmonic m"); ax[1].set_ylabel("|h_m| / |h1|")
    ax[1].set_title("harmonic fingerprint (2f,3f excess = JKL sharpness)")
    ax[1].set_xticks(np.arange(1, a.M + 1))
    ax[2].plot(fs * 60, var, "b-", lw=1.0)
    ax[2].axvline(f * 60, color="r", ls="--", lw=1.2, label=f"peak {f*60:.0f}bpm")
    ax[2].set_xlabel("folding freq (bpm)"); ax[2].set_ylabel("folded-profile variance")
    ax[2].set_title("epoch-folding periodogram"); ax[2].legend(fontsize=9)
    plt.tight_layout(); plt.savefig("learn_jkl_template.png", dpi=115)
    print("saved learn_jkl_template.png")


if __name__ == "__main__":
    main()
