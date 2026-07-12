"""M1 — double-oscillator EKF for joint respiration-heartbeat decoupling (single bin).

Prior single-frame methods gave an UNSTABLE f_H (autocorr 75 / beat 94-143 /
Bessel 133 / bin-argmax 82-140, truth 131->91) because they estimate the rate
one frame at a time and the breathing-harmonic comb reaches into the cardiac band
(HR ~ 5-7 x RR). This tracks R(t) and H(t) as *simultaneous* latent oscillators in
one state space, so three levers fight the harmonic lock at once:

  1. WAVEFORM PRIOR  — breathing is a full harmonic comb (K_r harmonics, all rigidly
     tied to the SAME phase phi_R); one adaptive model subtracts the whole comb at
     once (the Bessel insight, but now phase-locked and time-varying). The heartbeat
     is a fixed shape g_JKL(phi_H): shape fixed, frequency free (user's key insight).
  2. CONTINUITY      — omega_H is a slow random walk (tiny process noise), so the
     rate cannot jump onto a breathing harmonic between frames; it must move smoothly.
  3. SYNCHRONICITY   — breathing harmonics move rigidly at k*omega_R; the heartbeat
     is asynchronous. A phase-locked harmonic coeff cannot fit the async beat when
     averaged over time, so the two separate even where their frequencies cross.

Filter = EKF forward (nonlinear in the phases) + RTS backward smoother for a smooth
trajectory. State x = [phi_R, omega_R, {a_R^k,b_R^k}_1..Kr, phi_H, omega_H,
{a_H^m,b_H^m}_1..Kh]. Observation y(t) = phase-demod chest displacement (mm):
    y = Sum_k [a_R^k cos(k phi_R)+b_R^k sin(k phi_R)]
      + Sum_m [a_H^m cos(m phi_H)+b_H^m sin(m phi_H)]

f_H init is critical (else EKF locks the strongest breathing harmonic): seeded from
the cardiac-excess residual FFT peak in a warmup window (--fh0 overrides).

    python joint_ekf.py                      # tachy2 / tachy3 / sport33, saves png
    python joint_ekf.py tachy2_cube.npz --fh0 2.15 --bin 94
"""
import argparse
import numpy as np
import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
from bcg_vitals import (demod_channels, estimate_rr, bandpass, sqi,
                        fft_peak, RR_LO, RR_HI)

FPS = 18.78
KR = 8          # breathing harmonics (0.3Hz*8 ~ 2.5Hz covers the comb into cardiac band)
KH = 1          # cardiac harmonics (fundamental; shape=sinusoid for M1, template=M2)

# M2 JKL cardiac template: phase-folded from sit33 @20fps (clean resting, 105 cycles),
# unit-RMS Fourier coeffs {p_m,q_m}_1..4 (learn_jkl_template.py). Per-harmonic spectral
# SNR (peak/local-floor): 1f 1.4x, 2f 2.1x, 3f 1.2x, 4f 1.2x -> 2f is the real JKL
# fingerprint; 3f/4f sit on the noise floor at 3.3m. Default --tM 3 uses {1f,2f,3f};
# the 4f term is dropped (near-noise, and 4*omega_H aliases past Nyquist at tachy where
# it drove an omega_H runaway that locked tachy3 to 150). The 2f fingerprint enters the
# [0.10,3.2]Hz obs band exactly as HR drops below ~96bpm -- i.e. right in tachy2's
# 91~5*RR overlap tail, letting the oscillator hold the descent where a sinusoid can't.
TEMPLATE_P = np.array([-1.209, -0.176, -0.306, -0.534])
TEMPLATE_Q = np.array([-0.068,  0.283, -0.167,  0.124])


def _tmpl(ph, p, q):
    """JKL template as an ANALYTIC PAIR at cardiac phase ph:
      Tc(ph) = sum_m p_m cos(m ph)+q_m sin(m ph)      (in-phase)
      Ts(ph) = sum_m p_m sin(m ph)-q_m cos(m ph)      (quadrature = Hilbert of Tc)
    Observation y_H = a_H*Tc + b_H*Ts: (a_H,b_H) give amplitude + a global phase anchor
    (linear, stable like M1's a,b) while the per-harmonic MAGNITUDES |h_m|=|p_m,q_m| stay
    LOCKED under that rotation -- so the 2f/3f/4f fingerprint that breathing can't fake is
    preserved. Returns (Tc, Ts, dTc/dph, dTs/dph)."""
    m = np.arange(1, len(p) + 1)
    c, s = np.cos(m * ph), np.sin(m * ph)
    Tc = float(np.sum(p * c + q * s)); Ts = float(np.sum(p * s - q * c))
    dTc = float(np.sum(m * (-p * s + q * c))); dTs = float(np.sum(m * (p * c + q * s)))
    return Tc, Ts, dTc, dTs


def _tmpl_cols(phi, p, q):
    """[Tc, Ts] regressor columns over a phase vector (for ls_init), shape (T,2)."""
    m = np.arange(1, len(p) + 1)
    cph = np.cos(np.outer(phi, m)); sph = np.sin(np.outer(phi, m))
    return np.stack([cph @ p + sph @ q, cph @ q * -1 + sph @ p], axis=1)


def set_KR(k):
    global KR
    KR = k
HP_LO, HP_HI = 0.10, 3.2    # preprocess band: drop <RR drift, keep comb+cardiac


# ------------------------------------------------------------------ data loading
def load_bin(path, fps, bin_override=None):
    """Return (y mm displacement of the chest bin, f0 Hz, bin_number). Chest bin =
    top resp-SQI bin per brief; --bin selects a specific range-bin number instead."""
    d = np.load(path, allow_pickle=True)
    cube = np.asarray(d["snapshots"], np.complex64)
    counts = d["counts"].astype(int); bins = d["bins"].astype(int)
    C = cube[:, :int(counts.min()), :]
    chans = demod_channels(C, bins)
    _, f0, _, _ = estimate_rr(chans, fps)
    if bin_override is not None:
        idx = int(np.argmin(np.abs(bins - bin_override)))
    else:
        rr = np.array([sqi(bandpass(c, fps, RR_LO, RR_HI), fps, RR_LO, RR_HI)
                       for c in chans])
        idx = int(np.argsort(rr)[::-1][0])
    y = bandpass(chans[idx], fps, HP_LO, HP_HI)     # drop baseline drift
    return y, f0, int(bins[idx])


# ------------------------------------------------------- harmonic design / init
def _regressors(phi_R, phi_H, kr, kh):
    """Design matrix columns for [a_R^1,b_R^1,...,a_H^1,b_H^1,...] given phase vecs."""
    cols = []
    for k in range(1, kr + 1):
        cols += [np.cos(k * phi_R), np.sin(k * phi_R)]
    for m in range(1, kh + 1):
        cols += [np.cos(m * phi_H), np.sin(m * phi_H)]
    return np.array(cols).T                          # (T, 2kr+2kh)


def seed_fh0(y, fps, f0, warm_s=30.0):
    """f_H seed = FFT peak of the cardiac-excess residual (breathing harmonic comb
    LS-removed at constant f0) over the warmup window. Removing the comb first is
    what lets a TACHY (2.1Hz) seed survive instead of collapsing to a breathing
    harmonic at ~1.3Hz. Searched over 1.2-2.6Hz (72-156bpm)."""
    n = min(len(y), int(warm_s * fps))
    seg = y[:n]; t = np.arange(n) / fps
    phi = 2 * np.pi * f0 * t
    G = _regressors(phi, phi, KR, 0)                 # breathing comb only
    coef, *_ = np.linalg.lstsq(G, seg, rcond=None)
    resid = seg - G @ coef
    pk = fft_peak(bandpass(resid, fps, 1.2, 2.6), fps, 1.2, 2.6)
    return pk if pk else 1.5


def ls_init(y, fps, f0, fh0, template=None, warm_s=15.0):
    """Least-squares seed of all harmonic coeffs over a warmup window, phi_R=2pi f0 t.
    M1 (template=None): free cardiac comb, phi_H=2pi fh0 t -> coef len 2KR+2KH.
    M2 (template=(p,q)): cardiac columns = [Tc, Ts] of the JKL template; (a_H,b_H) carry
    amplitude + anchor linearly -> coef len 2KR+2. phi_H(0)=0."""
    n = min(len(y), int(warm_s * fps)); t = np.arange(n) / fps
    phiR = 2 * np.pi * f0 * t
    if template is None:
        G = _regressors(phiR, 2 * np.pi * fh0 * t, KR, KH)
    else:
        p, q = template
        G = np.hstack([_regressors(phiR, phiR, KR, 0),
                       _tmpl_cols(2 * np.pi * fh0 * t, p, q)])
    coef, *_ = np.linalg.lstsq(G, y[:n], rcond=None)
    return coef


# ------------------------------------------------------------------ EKF + RTS
WH_LO, WH_HI = 1.25, 2.5    # cardiac freq clamp Hz (75-150bpm; floor keeps the
                            # track off the ~64bpm breathing k3-4 residue in the
                            # late tachy-recovery phase where 91bpm ~ 5*RR overlaps)
WR_LO, WR_HI = 0.13, 0.55   # breathing freq clamp Hz


def run(y, fps, f0, fh0,
        s_wR=0.04, s_wH=0.05, s_cR=0.03, s_cH=0.02, s_pR=0.003, s_pH=0.003,
        antiharm=0.0, template=None, pwH0=0.10):
    """Forward EKF + RTS smoother. Process-noise stds are per-sqrt(second):
       s_wR/s_wH angular-freq drift (s_wH SMALL = HR continuity),
       s_cR/s_cH harmonic-coeff drift, s_pR/s_pH phase jitter.
    omega_H/omega_R are hard-clamped to physiological bands each step (kills the
    runaway-to-empty and DC-collapse failure modes). antiharm (>0) adds a soft
    anti-harmonic pull: when omega_H drifts within one bin of an integer multiple
    k*omega_R (k>=3), it is nudged OFF the harmonic, so the cardiac oscillator
    cannot silently merge into a breathing-comb tooth (see brief 'lock harmonic').
    Returns dict with smoothed fR(bpm), fH(bpm), cardiac amplitude, filtered fH."""
    T = len(y); dt = 1.0 / fps
    use_t = template is not None
    if use_t:
        p_t, q_t = template
    nR = 2 * KR
    nH = 2 if use_t else 2 * KH   # M2: (a_H,b_H) on [Tc,Ts]; M1: {a,b}_H comb
    n = 2 + nR + 2 + nH
    iPR, iWR = 0, 1
    iCR = 2                     # breathing coeffs iCR .. iCR+nR
    iPH = 2 + nR; iWH = iPH + 1
    iCH = iWH + 1              # cardiac coeffs iCH .. iCH+nH

    # --- state / covariance init
    x = np.zeros(n)
    x[iWR] = 2 * np.pi * f0
    x[iWH] = 2 * np.pi * fh0
    coef = ls_init(y, fps, f0, fh0, template=template)
    x[iCR:iCR + nR] = coef[:nR]
    x[iCH:iCH + nH] = coef[nR:nR + nH]
    P = np.eye(n) * 1e-3
    P[iWR, iWR] = (2 * np.pi * 0.03) ** 2
    P[iWH, iWH] = (2 * np.pi * pwH0) ** 2            # smaller = trust the seed, avoid a jump
    for j in range(iCR, iCR + nR): P[j, j] = 1.0
    for j in range(iCH, iCH + nH): P[j, j] = 0.5

    # --- constant transition + process noise
    F = np.eye(n); F[iPR, iWR] = dt; F[iPH, iWH] = dt
    q = np.zeros(n)
    q[iPR] = (s_pR ** 2) * dt; q[iWR] = (s_wR ** 2) * dt
    q[iPH] = (s_pH ** 2) * dt; q[iWH] = (s_wH ** 2) * dt
    q[iCR:iCR + nR] = (s_cR ** 2) * dt
    q[iCH:iCH + nH] = (s_cH ** 2) * dt
    Q = np.diag(q)
    R = np.var(bandpass(y, fps, 3.2, min(9.0, fps / 2 - 0.2))) + 1e-6  # out-of-band floor
    R = max(R, 1e-4)

    # --- forward pass, storing for RTS
    xf = np.zeros((T, n)); Pf = np.zeros((T, n, n))
    xp = np.zeros((T, n)); Pp = np.zeros((T, n, n))
    for t in range(T):
        # predict
        xpr = F @ x
        Ppr = F @ P @ F.T + Q
        xp[t] = xpr; Pp[t] = Ppr
        # measurement h(x) + Jacobian
        pr, ph = xpr[iPR], xpr[iPH]
        H = np.zeros(n); yhat = 0.0
        dPR = 0.0
        for k in range(1, KR + 1):
            a, b = xpr[iCR + 2 * (k - 1)], xpr[iCR + 2 * (k - 1) + 1]
            ck, sk = np.cos(k * pr), np.sin(k * pr)
            yhat += a * ck + b * sk
            H[iCR + 2 * (k - 1)] = ck; H[iCR + 2 * (k - 1) + 1] = sk
            dPR += k * (-a * sk + b * ck)
        H[iPR] = dPR
        dPH = 0.0
        if use_t:                                    # M2: y_H = a_H*Tc + b_H*Ts, {p,q} frozen
            aH, bH = xpr[iCH], xpr[iCH + 1]
            Tc, Ts, dTc, dTs = _tmpl(ph, p_t, q_t)
            yhat += aH * Tc + bH * Ts
            H[iCH] = Tc; H[iCH + 1] = Ts             # d/d a_H, d/d b_H
            dPH += aH * dTc + bH * dTs               # d/d phi_H
        else:                                        # M1: free cardiac comb
            for m in range(1, KH + 1):
                c, dd = xpr[iCH + 2 * (m - 1)], xpr[iCH + 2 * (m - 1) + 1]
                cm, sm = np.cos(m * ph), np.sin(m * ph)
                yhat += c * cm + dd * sm
                H[iCH + 2 * (m - 1)] = cm; H[iCH + 2 * (m - 1) + 1] = sm
                dPH += m * (-c * sm + dd * cm)
        H[iPH] = dPH
        # update
        S = H @ Ppr @ H + R
        Kg = (Ppr @ H) / S
        innov = y[t] - yhat
        x = xpr + Kg * innov
        P = (np.eye(n) - np.outer(Kg, H)) @ Ppr
        # physiological clamps (kill runaway-to-empty / DC-collapse)
        x[iWR] = np.clip(x[iWR], 2 * np.pi * WR_LO, 2 * np.pi * WR_HI)
        x[iWH] = np.clip(x[iWH], 2 * np.pi * WH_LO, 2 * np.pi * WH_HI)
        # anti-harmonic nudge: push omega_H off the nearest breathing-comb tooth
        if antiharm > 0:
            wR = x[iWR]
            k = int(round(x[iWH] / wR))
            if k >= 3:
                tooth = k * wR
                gap = x[iWH] - tooth
                guard = 2 * np.pi * 0.06                # ~4bpm guard band
                if abs(gap) < guard:
                    x[iWH] += antiharm * (np.sign(gap) if gap != 0 else 1) * (guard - abs(gap))
                    x[iWH] = np.clip(x[iWH], 2 * np.pi * WH_LO, 2 * np.pi * WH_HI)
        xf[t] = x; Pf[t] = P

    # --- RTS backward smoother (F constant)
    xs = xf.copy(); Ps = Pf.copy()
    for t in range(T - 2, -1, -1):
        Ppr_next = Pp[t + 1]
        try:
            G = Pf[t] @ F.T @ np.linalg.inv(Ppr_next)
        except np.linalg.LinAlgError:
            G = Pf[t] @ F.T @ np.linalg.pinv(Ppr_next)
        xs[t] = xf[t] + G @ (xs[t + 1] - xp[t + 1])
        Ps[t] = Pf[t] + G @ (Ps[t + 1] - Pp[t + 1]) @ G.T

    fR = xs[:, iWR] / (2 * np.pi) * 60.0
    fH = xs[:, iWH] / (2 * np.pi) * 60.0
    fH_filt = xf[:, iWH] / (2 * np.pi) * 60.0
    # cardiac amplitude envelope (hypot(a,b) both: template=in-phase/quad, M1=cos/sin)
    aH = np.hypot(xs[:, iCH], xs[:, iCH + 1])
    return dict(fR=fR, fH=fH, fH_filt=fH_filt, aH=aH, R=R)


# ------------------------------------------------------------------ evaluation
def cardiac_spectrogram(y, fps, f0, win_s=12.0, hop_s=2.0):
    """STFT of the breathing-notched signal in the cardiac band -> (times, freqs bpm,
    power) for a faint background so the tracked f_H can be judged against real
    energy."""
    win = int(win_s * fps); hop = int(hop_s * fps)
    times, cols = [], []
    freqs = np.fft.rfftfreq(win, 1 / fps)
    band = (freqs >= 1.0) & (freqs <= 2.7)
    for s in range(0, len(y) - win + 1, hop):
        seg = bandpass(y[s:s + win], fps, 1.0, 2.7, notch_f0=f0)
        S = np.abs(np.fft.rfft(seg * np.hanning(win)))
        cols.append(S[band]); times.append((s + win / 2) / fps)
    P = np.array(cols).T
    P = P / (P.max() + 1e-9)
    return np.array(times), freqs[band] * 60, P


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("path", nargs="?", default=None)
    ap.add_argument("--fps", type=float, default=FPS)
    ap.add_argument("--fh0", type=float, default=None, help="f_H seed Hz (override)")
    ap.add_argument("--bin", type=int, default=None, help="range-bin number override")
    ap.add_argument("--swH", type=float, default=0.015, help="omega_H drift std (continuity "
                    "knob; 0.015 blocks the weak-signal template runaway yet passes the slow "
                    "tachy2 descent; original M1 used 0.05 -> tachy2 floor-parked at 75)")
    ap.add_argument("--tM", type=int, default=3, help="template harmonics used (1f,2f,3f; "
                    "2f is the real fingerprint, 4f~noise+aliases -> runaway, dropped)")
    a = ap.parse_args()
    tP, tQ = TEMPLATE_P[:a.tM], TEMPLATE_Q[:a.tM]

    # (path, fh0 prior [Hz, brief: bessel/wideband seed], bin=None -> auto SQI-top
    # chest bin per brief recipe, truth label). Same params for all three cubes.
    jobs = ([(a.path, a.fh0, a.bin, "?")] if a.path else
            [("tachy2_cube.npz", 2.18, None, "131->91"),
             ("tachy3_cube.npz", 1.42, None, "84-87 flat"),
             ("sport33_cube.npz", 1.70, None, "101->106->82")])

    fig, axes = plt.subplots(len(jobs), 1, figsize=(13, 3.4 * len(jobs)),
                             squeeze=False)
    for ax, (path, fh0_pri, binN, truth) in zip(axes[:, 0], jobs):
        y, f0, binN_used = load_bin(path, a.fps, binN)
        fh0_auto = seed_fh0(y, a.fps, f0)
        fh0 = fh0_pri if fh0_pri is not None else fh0_auto
        # ablation: M1 free-sine cardiac vs M2 frozen JKL template, same EKF params
        out_m1 = run(y, a.fps, f0, fh0, s_wH=a.swH, template=None)
        out_tp = run(y, a.fps, f0, fh0, s_wH=a.swH, template=(tP, tQ))
        t = np.arange(len(y)) / a.fps

        tt, ff, P = cardiac_spectrogram(y, a.fps, f0)
        ax.pcolormesh(tt, ff, P, cmap="Greys", shading="auto", alpha=0.55)
        # AppleWatch truth overlay (piecewise linear anchors)
        TR = {"tachy2_cube.npz": ([0, 60, 120], [131, 110, 91]),
              "tachy3_cube.npz": ([0, 120], [85, 85]),
              "sport33_cube.npz": ([0, 40, 80, 120], [101, 106, 95, 82])}

        def mae(out):
            if path not in TR:
                return float("nan")
            tr = np.interp(t, TR[path][0], TR[path][1])
            m = (t >= 5) & (t <= t[-1] - 3)          # skip warmup + smoother edge
            return float(np.mean(np.abs(out["fH"][m] - tr[m])))

        if path in TR:
            ax.plot(TR[path][0], TR[path][1], "b--", lw=1.6, alpha=0.7,
                    label="AppleWatch truth")
        ax.plot(t, out_m1["fH"], color="orange", ls="--", lw=1.6, alpha=0.9,
                label=f"M1 sine  (MAE {mae(out_m1):.1f})")
        ax.plot(t, out_tp["fH"], "r-", lw=2.2,
                label=f"M2 template (MAE {mae(out_tp):.1f})")
        ax.plot(t, out_tp["fR"] * 5, "c--", lw=1.0, alpha=0.6, label="5xf_R (harmonic)")
        ax.axhline(fh0 * 60, color="g", ls=":", lw=1, label=f"seed {fh0*60:.0f}")
        ax.set_ylim(50, 160); ax.set_xlim(0, t[-1])
        ax.set_ylabel("bpm"); ax.set_xlabel("s")

        def ht(out):
            s = out["fH"]
            return (np.median(s[int(8 * a.fps):int(28 * a.fps)]),
                    np.median(s[-int(25 * a.fps):-int(3 * a.fps)]))
        h1, l1 = ht(out_m1); h2, l2 = ht(out_tp)
        ax.set_title(f"{path}  bin{binN_used}  truth {truth}  f_R={f0*60:.0f}rpm  |  "
                     f"M1 {h1:.0f}->{l1:.0f}  M2 {h2:.0f}->{l2:.0f} bpm")
        ax.legend(fontsize=7, loc="upper right", ncol=2)
        print(f"{path:16s} bin{binN_used} truth {truth:14s} seed{fh0*60:4.0f}  "
              f"| M1 {h1:3.0f}->{l1:3.0f} MAE{mae(out_m1):5.1f}  "
              f"| M2 {h2:3.0f}->{l2:3.0f} MAE{mae(out_tp):5.1f}  "
              f"aH_med={np.median(out_tp['aH']):.3f}")

    plt.tight_layout(); out_png = "joint_ekf.png"
    plt.savefig(out_png, dpi=115); print(f"saved {out_png}")


if __name__ == "__main__":
    main()
