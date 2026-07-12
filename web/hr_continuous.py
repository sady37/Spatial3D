"""Real-time continuous HR + tachycardia layer on top of the validated
BCG-style radar vitals core (bcg_vitals.py).

This module adds ONLY a temporal layer; it does NOT touch the validated
phase-demodulation + band-prior + autocorr@band core. It imports those
functions verbatim from bcg_vitals and wraps them in:

  Task 1 (continuous HR):
    * sliding window (default 15s / 1.5s step) -> instantaneous HR per window
    * Kalman smoother  (state=HR, random walk, adaptive R by confidence)
    * continuity validator (last-5 history, +-10% vs median / +-20% vs prev)
    * backup (on suspicion: re-estimate over a longer 30s window)

  Task 2 (tachycardia):
    * adaptive HR ceiling. Default resting band [1.0,1.7]Hz (validated).
      A probe band [1.0,2.2]Hz is also run; the high estimate is only
      ACCEPTED IF its autocorr peak is stronger AND more bin-consistent than
      the resting-band peak (band-prior arbitration -- never drops the low
      edge below 1.0Hz, so the 0.7-1.0Hz breathing-harmonic trap stays out).

Usage:
    python hr_continuous.py sit39_cube.npz --fps 18.78
    python hr_continuous.py sit39_cube.npz --fps 18.78 --tachy   # allow >102bpm
"""
import argparse
import numpy as np

# ---- validated core: import verbatim, do not reimplement --------------------
from bcg_vitals import (bandpass, sqi, fft_peak, autocorr_bpm, beat_count,
                        RR_LO, RR_HI, DR)

LAMBDA_MM = 5.0
HR_PHYS_LO, HR_PHYS_HI = 1.0, 1.7        # validated resting prior
HR_TACHY_HI = 2.2                        # 132 bpm probe ceiling (Task 2)


# ============================================================================
# refinement (on top of core, not a replacement): sub-sample autocorr peak
# ============================================================================
def autocorr_peak_interp(sig, fps, lo_bpm, hi_bpm):
    """Same first-autocorr-peak-in-band as bcg_vitals.autocorr_bpm, but with
    parabolic interpolation of the peak lag -> smooth (non-quantized) bpm and a
    normalized peak strength in [0,1]. Returns (bpm, strength) or (None, 0).

    The integer-lag autocorr quantizes bpm in ~6bpm steps near 80bpm; that is
    fine for a single 240s call but makes a continuous track look steppy. The
    interpolation refines the SAME peak the validated method already picks."""
    ac = np.correlate(sig, sig, "full")[len(sig) - 1:]
    if ac[0] <= 0:
        return None, 0.0
    ac = ac / ac[0]
    l0, l1 = int(fps / (hi_bpm / 60)), int(fps / (lo_bpm / 60))
    if l1 <= l0 + 1 or l1 >= len(ac):
        return None, 0.0
    k = l0 + int(np.argmax(ac[l0:l1]))
    strength = float(ac[k])
    # parabolic interpolation on lag (guard the edges)
    if 0 < k < len(ac) - 1:
        y0, y1, y2 = ac[k - 1], ac[k], ac[k + 1]
        denom = (y0 - 2 * y1 + y2)
        delta = 0.5 * (y0 - y2) / denom if abs(denom) > 1e-9 else 0.0
        delta = float(np.clip(delta, -0.5, 0.5))
    else:
        delta = 0.0
    lag = k + delta
    return fps / lag * 60.0, strength


# ============================================================================
# phase demodulation -- extracted verbatim from bcg_vitals.main (unchanged math)
# ============================================================================
def phase_channels(cube_slice):
    """cube_slice: (nbin, T, nchan) complex -> (nbin, T) mm-displacement."""
    chans = []
    for i in range(cube_slice.shape[0]):
        Ci = cube_slice[i]
        m = Ci.mean(0)
        m = m / (np.linalg.norm(m) + 1e-9)
        z = Ci @ m.conj()
        phi = np.unwrap(np.angle(z))
        disp_mm = -LAMBDA_MM / (4 * np.pi) * (phi - phi.mean())
        chans.append(disp_mm)
    return np.array(chans)


def estimate_rr_f0(chans, fps, topk=8):
    """RR (median FFT peak, SQI-selected) -> f0 for the HR notch. Verbatim recipe."""
    rr_sqi = np.array([sqi(bandpass(c, fps, RR_LO, RR_HI), fps, RR_LO, RR_HI)
                       for c in chans])
    rr_top = np.argsort(rr_sqi)[::-1][:topk]
    rr_f = []
    for i in rr_top:
        ff = fft_peak(chans[i], fps, RR_LO, RR_HI)
        if ff:
            rr_f.append(ff * 60)
    rr = float(np.median(rr_f)) if rr_f else None
    f0 = rr / 60.0 if rr else 0.25
    return rr, f0


def _band_estimate(chans, fps, f0, lo, hi, topk=8):
    """autocorr@band over the SQI-top bins. Exactly the validated HR recipe,
    parameterized by band + sub-sample interp. Returns
    (median_bpm, spread, median_strength, vals[], strs[]) so the caller can do
    cluster voting across bins (needed for robust tachycardia arbitration)."""
    hr_sqi = np.array([sqi(bandpass(c, fps, lo, hi, notch_f0=f0), fps, lo, hi)
                       for c in chans])
    top = np.argsort(hr_sqi)[::-1][:topk]
    vals, strs = [], []
    for i in top:
        sig = bandpass(chans[i], fps, lo, hi, notch_f0=f0)
        bpm, st = autocorr_peak_interp(sig, fps, int(lo * 60), int(hi * 60))
        if bpm:
            vals.append(bpm)
            strs.append(st)
    if not vals:
        return None, 99.0, 0.0, [], []
    return (float(np.median(vals)), float(np.std(vals)),
            float(np.median(strs)), vals, strs)


def estimate_window(chans, fps, tachy=False, topk=8):
    """One window -> instantaneous HR estimate dict.

    Task 2 arbitration: resting band is primary. If tachy=True we also probe
    [1.0,2.2]. The high answer is trusted ONLY when it lands above the resting
    ceiling AND its peak is stronger and more bin-consistent than resting's."""
    rr, f0 = estimate_rr_f0(chans, fps, topk)
    hr_lo, spread_lo, str_lo, _, _ = _band_estimate(chans, fps, f0,
                                                   HR_PHYS_LO, HR_PHYS_HI, topk)
    src = "resting[1.0-1.7]"
    hr, spread, strength = hr_lo, spread_lo, str_lo

    if tachy and hr_lo is not None:
        _, _, _, vals_hi, strs_hi = _band_estimate(chans, fps, f0,
                                                  HR_PHYS_LO, HR_TACHY_HI, topk)
        # CLUSTER VOTE (robust to a bimodal bin population): count the quality
        # bins that resolve as tachycardic (> resting ceiling). Promote only if a
        # MAJORITY agree AND that cluster's periodicity is genuinely strong. This
        # is the band-prior "trust the high band iff it has a strong CONSISTENT
        # peak" rule -- and it never lowers the 1.0Hz edge, so the 0.7-1.0Hz
        # breathing-harmonic trap stays excluded.
        ceil_bpm = HR_PHYS_HI * 60 + 3            # 105 bpm
        hi_idx = [k for k, v in enumerate(vals_hi) if v > ceil_bpm]
        if vals_hi and len(hi_idx) >= max(2, int(np.ceil(0.5 * len(vals_hi)))):
            cluster = [vals_hi[k] for k in hi_idx]
            cl_str = float(np.median([strs_hi[k] for k in hi_idx]))
            if cl_str >= 0.5:                     # absolute clean-periodicity floor
                hr = float(np.median(cluster))
                spread = float(np.std(cluster))
                strength = cl_str
                src = "tachy[1.0-2.2]"

    # confidence from bin-spread (matches core's HIGH/MED/LOW thresholds)
    if spread < 3:
        conf = "HIGH"
    elif spread < 6:
        conf = "MED"
    else:
        conf = "LOW"
    return dict(hr=hr, rr=rr, spread=spread, strength=strength,
                conf=conf, src=src, f0=f0)


# ============================================================================
# AF (atrial fibrillation) suspicion  -- rhythm-regularity, not rate
# ============================================================================
# AF risk-stratifies stroke: it is the source of ~10% of ischemic strokes
# (cardioembolic). Its signature is IRREGULARITY, not speed. Two radar-visible
# marks: (1) irregular R-R timing, (2) irregular per-beat amplitude (variable
# diastolic filling -> variable stroke volume -> variable chest displacement).
#
# HARD FINDING (measured on sit39 @3.5m): the native cardiac SNR is too low to
# judge regularity -- real seated sinus autocorr-strength ~0.26, while a clean
# regular beat reads ~0.95. The RATE survives (autocorr argmax locks the period
# even in noise) but rhythm MORPHOLOGY does not. So AF classification MUST be
# gated on signal quality: below the gate we return 'indeterminate' rather than
# a false flag. The gate lets a strong regular beat (0.95) and a strong
# irregular one (~0.52) through to be classified, and abstains on weak signals.
#
# THRESHOLDS BELOW ARE PROVISIONAL -- calibrated on one synthetic AF + one real
# sinus file. They need a real AF capture (closer range / stronger coupling) to
# finalize. Treat the numbers as a scaffold, not validated cut-points.
# Gate on cardiac PRESENCE (scale-invariant), NOT on regularity: cardiac-band
# energy / respiratory-band energy. Breathing is always strongly present, so it
# is a stable per-capture reference and the ratio is invariant to displacement
# gain. This does NOT penalize AF for being irregular (the earlier autocorr-
# strength gate did). Measured: real seated sinus ~0.42 (too weak to judge),
# injected cardiac 1.3-16 (strong enough). Below the gate => abstain.
AF_CARD_GATE = 0.90    # cardiac/respiratory energy ratio floor
AF_CONC_SINUS = 0.60   # fused-spectrum peak concentration: >= => regular
AF_ENT_SINUS = 0.45    # normalized spectral entropy: <= => regular
AF_CONC_AF = 0.50      # < => spread (irregular)
AF_ENT_AF = 0.70       # > => spread (irregular)


def _fused_hr_spectrum(chans, fps, f0, top, lo, hi):
    """SQI-weighted sum of the top bins' HR-band power spectra (feature-level
    fusion, ref sleep算法.md option B). Real beats are coherent across torso
    bins and add up; noise does not -- this sharpens a genuine rhythm peak."""
    Sf, f, = None, None
    for i in top:
        sig = bandpass(chans[i], fps, lo, hi, notch_f0=f0)
        w = sqi(sig, fps, lo, hi)
        f = np.fft.rfftfreq(len(sig), 1 / fps)
        S = np.abs(np.fft.rfft(sig - sig.mean())) ** 2
        Sf = w * S if Sf is None else Sf + w * S
    m = (f >= lo) & (f <= hi)
    fb, Sb = f[m], Sf[m]
    if Sb.sum() <= 0:
        return 0.0, 1.0
    p = fb[np.argmax(Sb)]
    conc = float(Sb[(fb >= p - 0.15) & (fb <= p + 0.15)].sum() / Sb.sum())
    Pn = Sb / Sb.sum()
    ent = float(-(Pn * np.log(Pn + 1e-12)).sum() / np.log(len(Pn)))
    return conc, ent


def af_metrics(chans, fps, f0, topk=8):
    """Per-window AF suspicion. Returns dict(state, concentration, entropy,
    ac_strength, rr_cv). state in {sinus, af_suspected, uncertain, indeterminate}.

    RATE band-prior stays intact: we analyze the same [1.0-2.2]Hz band the
    tachy probe uses (so AF-with-RVR up to 132bpm is in view)."""
    lo, hi = HR_PHYS_LO, HR_TACHY_HI
    hr_sqi = np.array([sqi(bandpass(c, fps, lo, hi, notch_f0=f0), fps, lo, hi)
                       for c in chans])
    top = np.argsort(hr_sqi)[::-1][:topk]

    conc, ent = _fused_hr_spectrum(chans, fps, f0, top, lo, hi)

    # cardiac PRESENCE gate: cardiac-band / respiratory-band energy (per raw bin)
    card = []
    for i in top:
        c = chans[i]
        f = np.fft.rfftfreq(len(c), 1 / fps)
        S = np.abs(np.fft.rfft(c - c.mean())) ** 2
        Ehr = S[(f >= lo) & (f <= hi)].sum()
        Err = S[(f >= RR_LO) & (f <= RR_HI)].sum()
        card.append(Ehr / (Err + 1e-12))
    card_ratio = float(np.median(card))

    # autocorr strength (diagnostic) + time-domain R-R CV, per bin
    strs, rr_cvs = [], []
    for i in top:
        sig = bandpass(chans[i], fps, lo, hi, notch_f0=f0)
        _, st = autocorr_peak_interp(sig, fps, int(lo * 60), int(hi * 60))
        strs.append(st)
        s = sig / (sig.std() + 1e-9)
        dist = max(1, int(fps / (180 / 60)))
        try:
            from scipy.signal import find_peaks
            pk, _ = find_peaks(s, distance=dist, height=0.25)
        except Exception:
            pk = np.array([k for k in range(1, len(s) - 1)
                           if s[k] > s[k - 1] and s[k] > s[k + 1] and s[k] > 0.25])
        if len(pk) >= 5:
            rr = np.diff(pk) / fps
            rr_cvs.append(float(rr.std() / (rr.mean() + 1e-9)))
    ac_strength = float(np.median(strs)) if strs else 0.0
    rr_cv = float(np.median(rr_cvs)) if rr_cvs else 99.0

    # gate: is the cardiac signal strong enough (present) to judge REGULARITY?
    if card_ratio < AF_CARD_GATE:
        state = "indeterminate"
    elif conc >= AF_CONC_SINUS and ent <= AF_ENT_SINUS:
        state = "sinus"
    elif conc < AF_CONC_AF and ent > AF_ENT_AF:
        state = "af_suspected"
    else:
        state = "uncertain"
    return dict(state=state, concentration=conc, entropy=ent,
                ac_strength=ac_strength, rr_cv=rr_cv, card_ratio=card_ratio)


# ============================================================================
# Kalman smoother  (scalar random-walk state = HR)
# ============================================================================
class KalmanSmoother:
    def __init__(self, q=2.0):
        self.x = None        # state (bpm)
        self.P = 100.0       # state variance
        self.Q = q           # process noise (bpm^2 per step): allows real drift

    def update(self, z, R):
        if self.x is None:                 # initialize on first valid measurement
            self.x, self.P = z, R
            return self.x
        # predict (random walk)
        self.P += self.Q
        # update
        K = self.P / (self.P + R)
        self.x = self.x + K * (z - self.x)
        self.P = (1 - K) * self.P
        return self.x


# ============================================================================
# Continuity validator  (ref: sleep算法.md core-innovation section)
# ============================================================================
class ContinuityValidator:
    def __init__(self, hist_size=5, tol_avg=0.10, tol_prev=0.20):
        self.hist = []            # last N ACCEPTED HRs
        self.N = hist_size
        self.tol_avg = tol_avg
        self.tol_prev = tol_prev

    def validate(self, hr):
        """Return (is_valid, reason). Valid if within +-10% of history median
        OR within +-20% of the previous accepted value."""
        if not self.hist:
            return True, "seed"
        med = float(np.median(self.hist))
        prev = self.hist[-1]
        ok_avg = abs(hr - med) <= self.tol_avg * med
        ok_prev = abs(hr - prev) <= self.tol_prev * prev
        if ok_avg or ok_prev:
            return True, "ok"
        return False, f"outlier (med {med:.0f}, prev {prev:.0f})"

    def accept(self, hr):
        self.hist.append(hr)
        if len(self.hist) > self.N:
            self.hist.pop(0)


# ============================================================================
# main sliding-window loop
# ============================================================================
def run_continuous(path, fps, win_s=15.0, step_s=1.5, backup_s=30.0,
                   tachy=False, topk=8, verbose=True):
    d = np.load(path, allow_pickle=True)
    cube = np.asarray(d["snapshots"], dtype=np.complex64)
    K = int(d["counts"].astype(int).min())
    cube = cube[:, :K, :]

    W = int(win_s * fps)
    S = int(step_s * fps)
    WB = int(backup_s * fps)
    starts = list(range(0, K - W + 1, S))

    kf = KalmanSmoother()
    cv = ContinuityValidator()
    fail_streak = 0
    af_hist = []                       # rolling af states for a sustained alert

    t_out, hr_raw, hr_smooth, flags = [], [], [], []
    af_states, af_conc = [], []
    for s0 in starts:
        t_center = (s0 + W / 2) / fps
        chans = phase_channels(cube[:, s0:s0 + W, :])
        est = estimate_window(chans, fps, tachy=tachy, topk=topk)
        z = est["hr"]
        if z is None:
            continue

        # AF suspicion (rhythm regularity, gated on cardiac SNR)
        af = af_metrics(chans, fps, est["f0"], topk=topk)
        af_hist.append(af["state"])
        if len(af_hist) > 7:
            af_hist.pop(0)
        n_af = sum(1 for s in af_hist if s == "af_suspected")
        af_alert = n_af >= 4          # sustained majority over ~10s of windows

        is_valid, reason = cv.validate(z)
        used_backup = False
        if not is_valid:
            fail_streak += 1
            # backup: longer 30s window ending at this window's end -> trend value
            b0 = max(0, s0 + W - WB)
            bchans = phase_channels(cube[:, b0:s0 + W, :])
            best = estimate_window(bchans, fps, tachy=tachy, topk=topk)
            if best["hr"] is not None:
                z = best["hr"]                 # use robust trend estimate
                used_backup = True
            # sustained failure => genuine regime change (e.g. post-movement
            # HR jump). Trust the backup trend and re-seed history around it.
            if fail_streak >= 3:
                cv.hist = [z]
                fail_streak = 0
        else:
            fail_streak = 0
            cv.accept(z)

        # adaptive measurement noise R: trust clean/consistent windows,
        # distrust suspicious/backup ones (high SQI->fast, low SQI->stable).
        R = {"HIGH": 4.0, "MED": 16.0, "LOW": 64.0}[est["conf"]]
        if used_backup:
            R = 36.0
        xs = kf.update(z, R)

        t_out.append(t_center)
        hr_raw.append(est["hr"])
        hr_smooth.append(xs)
        af_states.append(af["state"])
        af_conc.append(af["concentration"])
        flag = est["src"][:5]
        if not is_valid:
            flag += "|BK" if used_backup else "|susp"
        flags.append(flag)
        if verbose:
            alert = "  <<< AF ALERT" if af_alert else ""
            print(f"t={t_center:6.1f}s  inst={est['hr']:5.1f}  "
                  f"smooth={xs:5.1f}  RR={est['rr'] or 0:4.0f}  "
                  f"[{est['conf']:4}] {est['src']:14} | "
                  f"AF:{af['state']:12} (conc={af['concentration']:.2f} "
                  f"ent={af['entropy']:.2f} str={af['ac_strength']:.2f}){alert}")

    return dict(t=np.array(t_out), raw=np.array(hr_raw),
                smooth=np.array(hr_smooth), flags=flags,
                af_states=af_states, af_conc=np.array(af_conc))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("path")
    ap.add_argument("--fps", type=float, required=True)
    ap.add_argument("--win", type=float, default=15.0)
    ap.add_argument("--step", type=float, default=1.5)
    ap.add_argument("--tachy", action="store_true", help="allow HR>102bpm")
    ap.add_argument("--plot", default=None)
    a = ap.parse_args()
    res = run_continuous(a.path, a.fps, win_s=a.win, step_s=a.step, tachy=a.tachy)
    print(f"\nsummary: n={len(res['t'])}  "
          f"smooth HR mean={np.mean(res['smooth']):.1f}  "
          f"min={np.min(res['smooth']):.1f}  max={np.max(res['smooth']):.1f}")
    if a.plot:
        _plot(res, a.path, a.plot)


def _plot(res, title, out):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig, ax = plt.subplots(figsize=(11, 4.5))
    ax.plot(res["t"], res["raw"], ".", ms=4, color="#bbb",
            label="instantaneous (per 15s window)")
    ax.plot(res["t"], res["smooth"], "-", lw=2, color="#c0392b",
            label="Kalman-smoothed + continuity")
    ax.set_xlabel("time (s)")
    ax.set_ylabel("HR (bpm)")
    ax.set_title(f"Continuous HR — {title}")
    ax.grid(alpha=0.3)
    ax.legend(loc="upper right")
    fig.tight_layout()
    fig.savefig(out, dpi=130)
    print(f"saved {out}")


if __name__ == "__main__":
    main()
