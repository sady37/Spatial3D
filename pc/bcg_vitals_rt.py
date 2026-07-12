"""Continuous (sliding-window) HR from the validated BCG cube pipeline.

The single-shot recipe in bcg_vitals.py (phase demod -> SQI chest bins ->
autocorr @ physiological band, tachy arbitration) is UNCHANGED and reused
verbatim here — this file only adds the *temporal layer* the sleep-pad algo
prescribes on top of the per-window instantaneous estimate:

  1. sliding window (15s, ~1.5s hop) -> one instantaneous HR + bin-spread/band
  2. Kalman smoothing        (scalar random-walk; measurement noise scales with
                              the window's inter-bin spread -> low-confidence
                              windows move the state less)
  3. continuity validation   (current vs recent-5 history; ok if within +-10%
                              of the mean OR +-20% of the previous; else the raw
                              value is NOT emitted, the tracker coasts)
  4. backup / re-acquire     (after K consecutive continuity failures -> a
                              longer-window autocorr trend re-anchors the tracker;
                              this is what recovers after a turn / large motion)

    python bcg_vitals_rt.py sit39_cube.npz --fps 18.8
    python bcg_vitals_rt.py sit39_cube.npz --fps 18.8 --tachy 2.2 --plot hr.png
"""
import argparse
from collections import deque
import numpy as np
from bcg_vitals import (demod_channels, estimate_rr, estimate_hr, hr_band_search,
                        occupancy, bandpass, sqi, fft_peak, autocorr_peak,
                        beat_count, RR_LO, RR_HI)
try:
    from scipy.stats import theilslopes
    _HAVE_THEIL = True
except Exception:
    _HAVE_THEIL = False


# ============================================================================
# ELEVATED/recovering HR (post-exercise tachycardia) — segment-slope detector.
# ============================================================================
# HARD FINDING (2026-07-11, see pc/NEXT.md): the per-window HR MISSES post-exercise
# tachycardia because a SWEEPING rate (e.g. 131->91bpm as the heart recovers) cannot
# be band-integrated the way a stationary resting HR is — full-record autocorr smears
# it to ~70-81. But the DESCENT itself is a robust, clinically meaningful signal ("HR
# elevated and recovering"). So: cut the record into ~15s quasi-stationary segments,
# take a WIDE-band [1.0-2.5] autocorr HR per segment (no resting cap), and test for a
# significant DOWNWARD trend (Theil-Sen slope + permutation p). Absolute per-segment
# bpm is noisy (+-31) so ONLY the slope sign is trusted. Validated on the cubes:
# fires on near-range post-exercise (tachy2: slope -17bpm/rec, p=0.06), SILENT on 5
# resting cubes (sit39/lie41/sidesit/fall20/tachy3 -> 0 false alarms). LIMITATION:
# misses FAR-range tachy (tachy1 @3.9m, slope only -7) — per-segment SNR falls with
# range, so this is a NEAR-range (<~2.5m) capability. Thresholds are a scaffold from
# ONE near-range positive (tachy2, borderline p) and need more near post-exercise
# captures to anchor. Pure post-hoc add-on; does not touch the validated HR core.
ELEV_SEG_S, ELEV_STEP_S = 15.0, 5.0
ELEV_LO, ELEV_HI = 1.0, 2.5
ELEV_SPREAD_MAX = 20.0            # drop noisy segments (inter-bin std) before the fit
ELEV_SLOPE_MIN = -8.0            # bpm over whole record: more negative => descending
ELEV_P_MAX = 0.10               # permutation p that the slope is <= observed


def _perm_p_slope(t, hr, obs_slope, n=400):
    """Permutation p-value that a descending slope this steep arises by chance:
    fraction of time-shuffled series whose Theil-Sen slope <= obs_slope. Uses a
    deterministic roll/reverse family (no RNG -> reproducible)."""
    if not _HAVE_THEIL or len(hr) < 5:
        return 1.0
    cnt = tot = 0
    for k in range(n):
        h = np.roll(hr, k % len(hr))
        if (k // len(hr)) % 2:
            h = h[::-1]
        s = theilslopes(h, t)[0]
        cnt += (s <= obs_slope); tot += 1
    return cnt / max(1, tot)


def elevated_hr_trend(cube, bins, fps, seg_s=ELEV_SEG_S, step_s=ELEV_STEP_S):
    """Post-hoc 'HR elevated & recovering' detector (see block comment). Returns
    dict(elevated, slope_rec, p, n_seg, series=[(t,hr,spread)])."""
    K = cube.shape[1]; n = int(seg_s * fps); step = int(step_s * fps)
    # single full-record f0 for the RR-notch (a per-segment f0 on 15s is noisy and
    # shifts the notch each segment, which flattens the very descent we're detecting)
    _, f0, _, _ = estimate_rr(demod_channels(cube, bins), fps)
    t, hr, sp = [], [], []
    for s in range(0, K - n + 1, step):
        ch = demod_channels(cube[:, s:s + n, :], bins)
        if not occupancy(ch, fps)["present"]:          # only trend a present person
            continue
        r = hr_band_search(ch, fps, f0, ELEV_LO, ELEV_HI, topk=8)
        if r["hr"]:
            t.append(s / fps + seg_s / 2); hr.append(r["hr"]); sp.append(r["spread"])
    t, hr, sp = np.array(t), np.array(hr), np.array(sp)
    out = dict(elevated=False, slope_rec=0.0, p=1.0, n_seg=len(hr),
               series=list(zip(t.tolist(), hr.tolist(), sp.tolist())))
    if len(hr) < 5 or not _HAVE_THEIL:
        return out
    keep = sp <= ELEV_SPREAD_MAX
    tk, hk = (t[keep], hr[keep]) if keep.sum() >= 5 else (t, hr)
    slope = theilslopes(hk, tk)[0]
    rec = float(t[-1] - t[0]) if len(t) > 1 else 0.0
    out["slope_rec"] = slope * rec
    out["p"] = _perm_p_slope(tk, hk, slope)
    out["elevated"] = (out["slope_rec"] < ELEV_SLOPE_MIN) and (out["p"] < ELEV_P_MAX)
    return out


# ============================================================================
# AF (atrial fibrillation) suspicion — rhythm REGULARITY, not rate. Pure add-on
# on the same phase-demod channels; does not touch the validated HR core.
# ============================================================================
# AF is the source of ~10% of ischemic (cardioembolic) strokes; its radar-visible
# signature is IRREGULARITY, not speed. HARD FINDING (measured on sit39 @3.5m):
# native cardiac SNR is too low to judge regularity — real seated sinus reads as
# spread as synthetic AF, so we MUST gate on cardiac PRESENCE and otherwise abstain
# ('indeterminate') rather than emit a false flag. The gate is scale-invariant
# (cardiac-band / respiratory-band energy; breathing is always present so the ratio
# is invariant to displacement gain) and does NOT penalize irregularity. THRESHOLDS
# BELOW ARE PROVISIONAL — a scaffold from one synthetic AF + real sinus; they need a
# real AF (or close-range strong sinus) capture to finalize.
AF_CARD_GATE = 0.90      # cardiac/respiratory energy ratio floor to even classify
AF_CONC_SINUS = 0.60     # fused-spectrum peak concentration: >= => regular
AF_ENT_SINUS = 0.45      # normalized spectral entropy:        <= => regular
AF_CONC_AF = 0.50        # <  => spread (irregular)
AF_ENT_AF = 0.70         # >  => spread (irregular)
AF_BAND_LO, AF_BAND_HI = 1.0, 2.2   # analyze same band the tachy probe uses


def _fused_hr_spectrum(chans, fps, f0, top, lo, hi):
    """SQI-weighted sum of the top bins' HR-band power spectra (feature-level
    fusion, ref sleep算法.md option B): real beats are coherent across torso bins
    and add; noise does not -> sharpens a genuine rhythm peak. Returns
    (concentration, normalized_entropy)."""
    Sf, f = None, None
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
    ac_strength, rr_cv, card_ratio); state in
    {sinus, af_suspected, uncertain, indeterminate}. Gated on cardiac PRESENCE:
    below the gate -> indeterminate (abstain), never a false flag."""
    lo, hi = AF_BAND_LO, AF_BAND_HI
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
        _, st = autocorr_peak(sig, fps, int(lo * 60), int(hi * 60), interp=True)
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


class HRKalman:
    """Scalar random-walk Kalman on HR (bpm). Process noise q keeps the state
    slow (HR drifts, doesn't jump); measurement noise R grows with the window's
    inter-bin spread so noisy windows barely nudge the estimate."""
    def __init__(self, q=0.8, r_base=3.0):
        self.x = None; self.P = 1e3; self.q = q; self.r_base = r_base

    def _R(self, spread):
        return self.r_base ** 2 + float(spread) ** 2

    def update(self, z, spread):
        R = self._R(spread)
        if self.x is None:
            self.x = float(z); self.P = R; return self.x
        P = self.P + self.q ** 2
        K = P / (P + R)
        self.x += K * (float(z) - self.x)
        self.P = (1 - K) * P
        return self.x

    def coast(self):
        """No trusted measurement this step: hold state, grow uncertainty."""
        self.P += self.q ** 2
        return self.x

    def reanchor(self, z, P=25.0):
        """Hard re-set after backup re-acquire (e.g. post-turn)."""
        self.x = float(z); self.P = P; return self.x


class Continuity:
    """Sleep-pad continuity check over the recent-N accepted HR values."""
    def __init__(self, n=5, tol_avg=0.10, tol_prev=0.20):
        self.hist = deque(maxlen=n); self.tol_avg = tol_avg
        self.tol_prev = tol_prev; self.fails = 0

    def check(self, hr):
        if not self.hist:
            return True
        avg = float(np.mean(self.hist)); prev = self.hist[-1]
        return (abs(hr - avg) <= self.tol_avg * avg or
                abs(hr - prev) <= self.tol_prev * prev)

    def push(self, hr):
        self.hist.append(float(hr)); self.fails = 0

    def fail(self):
        self.fails += 1

    def reset(self, hr):
        self.hist.clear(); self.hist.append(float(hr)); self.fails = 0


def backup_estimate(cube, bins, fps, center, half_w, tachy_hi):
    """Longer-window (2x) trend re-acquire at `center` sample. Same validated
    estimator, wider window -> more stable, higher latency (sleep-pad 备份机制)."""
    i0 = max(0, center - half_w); i1 = min(cube.shape[1], center + half_w)
    C = cube[:, i0:i1, :]
    chans = demod_channels(C, bins)
    _, f0, _, _ = estimate_rr(chans, fps)
    res = estimate_hr(chans, fps, f0, tachy_hi=tachy_hi, interp=True)
    return res["hr"], res["spread"], res["band"]


def run(cube, bins, fps, win_s=15.0, hop_s=1.5, tachy_hi=None,
        backup_after=3, backup_win_s=30.0, q_min=0.40):
    """Slide the validated estimator and apply the temporal layer. A window is
    TRUSTED only if it passes BOTH gates: (a) signal quality — resting-band
    autocorr height (SQI proxy) >= q_min, which is the only reliable dropout
    detector (the HR VALUE itself is band-limited to 60-102bpm so it stays
    plausible even on pure noise); (b) continuity — value within +-10% of the
    recent mean or +-20% of the previous. Untrusted windows coast the Kalman;
    `backup_after` consecutive untrusted windows trigger a longer-window
    re-acquire (post-turn recovery)."""
    w = int(win_s * fps); hop = int(hop_s * fps)
    bw = int(backup_win_s * fps / 2)
    T = cube.shape[1]
    kf = HRKalman(); cont = Continuity()
    af_hist = deque(maxlen=7)                       # rolling AF states -> alert
    rows = []
    for i in range(0, max(1, T - w + 1), hop):
        C = cube[:, i:i + w, :]
        chans = demod_channels(C, bins)
        t = (i + w / 2) / fps
        # OCCUPANCY gate first: no person -> suppress vitals, coast, no AF/tachy.
        occ = occupancy(chans, fps)
        if not occ["present"]:
            af_hist.append("no_person")
            rows.append((t, np.nan, kf.coast(), "noperson", 99.0, "LOW", 0.0,
                         "no_person", 0.0, False))
            continue
        _, f0, _, _ = estimate_rr(chans, fps)
        res = estimate_hr(chans, fps, f0, tachy_hi=tachy_hi, interp=True)
        hr_meas, spread, band = res["hr"], res["spread"], res["band"]
        quality = res["low"]["strength"]           # SQI proxy (autocorr height)
        af = af_metrics(chans, fps, f0)            # rhythm regularity (gated)
        af_hist.append(af["state"])
        af_alert = sum(s == "af_suspected" for s in af_hist) >= 4  # sustained >~10s
        if hr_meas is None:
            rows.append((t, np.nan, kf.coast(), "none", 99.0, band, quality,
                         af["state"], af["concentration"], af_alert)); continue

        conf_ok = (band == "HIGH") or (quality >= q_min)   # tachy vote self-gates
        val_ok = cont.check(hr_meas)
        if conf_ok and val_ok:
            cont.push(hr_meas)
            hr_out = kf.update(hr_meas, spread)
            src = "track"
        else:
            cont.fail()
            if cont.fails >= backup_after:
                hr_b, sp_b, band_b = backup_estimate(
                    cube, bins, fps, i + w // 2, bw, tachy_hi)
                if hr_b is not None:
                    cont.reset(hr_b); kf.reanchor(hr_b)
                    hr_out = hr_b; src = "backup"; band = band_b; spread = sp_b
                else:
                    hr_out = kf.coast(); src = "coast"
            else:
                hr_out = kf.coast()
                src = "lowconf" if not conf_ok else "suspect"  # raw NOT emitted
        rows.append((t, hr_meas, hr_out, src, spread, band, quality,
                     af["state"], af["concentration"], af_alert))
    return rows


def plot(rows, path, title):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    t = np.array([r[0] for r in rows])
    meas = np.array([r[1] for r in rows], dtype=float)
    smooth = np.array([r[2] if r[2] is not None else np.nan for r in rows], dtype=float)
    src = [r[3] for r in rows]; band = [r[5] for r in rows]
    fig, ax = plt.subplots(figsize=(12, 5))
    # raw window measurements, colored by band
    lo = np.array([b == "LOW" for b in band])
    ax.scatter(t[lo], meas[lo], s=16, c="#9db8d2", label="window HR (resting band)", zorder=2)
    if (~lo).any():
        ax.scatter(t[~lo], meas[~lo], s=22, c="#d98c8c", label="window HR (tachy band)", zorder=2)
    # smoothed Kalman track
    ax.plot(t, smooth, "-", c="#1f4e79", lw=2.0, label="Kalman HR", zorder=4)
    # annotate non-track sources
    for tag, col, mk, lab in [("backup", "#2e8b57", "X", "backup re-acquire"),
                              ("suspect", "#e08a1e", "v", "suspect (coasting)"),
                              ("lowconf", "#c94f4f", "v", "low-confidence (coasting)"),
                              ("coast", "#b0b0b0", "v", "coast")]:
        m = np.array([s == tag for s in src])
        if m.any():
            ax.scatter(t[m], smooth[m], s=55, c=col, marker=mk, label=lab, zorder=5)
    good = smooth[np.isfinite(smooth)]
    if len(good):
        ax.axhline(np.median(good), ls="--", c="#888", lw=1,
                   label=f"median {np.median(good):.0f} bpm")
    ax.set_xlabel("time (s)"); ax.set_ylabel("HR (bpm)")
    ax.set_title(title); ax.grid(alpha=0.3); ax.legend(loc="upper right", fontsize=8, ncol=2)
    fig.tight_layout(); fig.savefig(path, dpi=110)
    print(f"saved plot -> {path}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("path")
    ap.add_argument("--fps", type=float, required=True)
    ap.add_argument("--win", type=float, default=15.0, help="window seconds")
    ap.add_argument("--hop", type=float, default=1.5, help="step seconds")
    ap.add_argument("--tachy", type=float, default=0.0,
                    help="widen HR ceiling to this Hz (0=disabled)")
    ap.add_argument("--plot", default="", help="save HR-vs-time PNG to this path")
    a = ap.parse_args()

    d = np.load(a.path, allow_pickle=True)
    cube = np.asarray(d["snapshots"], dtype=np.complex64)
    counts = d["counts"].astype(int); bins = d["bins"].astype(int)
    K = int(counts.min()); cube = cube[:, :K, :]
    tachy_hi = a.tachy if a.tachy else None

    rows = run(cube, bins, a.fps, a.win, a.hop, tachy_hi=tachy_hi)
    print(f"{a.path}: {len(rows)} windows ({a.win:.0f}s/{a.hop:.1f}s hop), "
          f"{K/a.fps:.0f}s @ {a.fps}fps"
          + (f", tachy ceiling {a.tachy}Hz" if tachy_hi else ""))
    print(f"{'t(s)':>6} {'meas':>6} {'HR':>6}  {'src':<8} {'spread':>6} {'qual':>5} "
          f"band  {'AF':<13} conc")
    for t, meas, hr, src, spread, band, qual, af_st, af_conc, af_alert in rows:
        ms = f"{meas:6.0f}" if np.isfinite(meas) else "   -- "
        hs = f"{hr:6.1f}" if hr is not None else "   -- "
        al = "  <<< AF ALERT" if af_alert else ""
        print(f"{t:6.1f} {ms} {hs}  {src:<8} {spread:6.1f} {qual:5.2f} "
              f"{band:<5} {af_st:<13} {af_conc:.2f}{al}")
    smooth = np.array([r[2] for r in rows if r[2] is not None], dtype=float)
    if len(smooth):
        print(f"\nKalman HR: median {np.median(smooth):.1f}, "
              f"range {np.nanmin(smooth):.0f}-{np.nanmax(smooth):.0f} bpm")
    src_ct = {}
    for r in rows: src_ct[r[3]] = src_ct.get(r[3], 0) + 1
    print("sources:", src_ct)
    af_ct = {}
    for r in rows: af_ct[r[7]] = af_ct.get(r[7], 0) + 1
    n_alert = sum(1 for r in rows if r[9])
    print("AF states:", af_ct, f"| sustained-alert windows: {n_alert}")

    # ELEVATED/recovering HR (post-exercise) — segment-slope, post-hoc add-on
    ev = elevated_hr_trend(cube, bins, a.fps)
    tag = ("  <<< HR ELEVATED / RECOVERING (post-exercise)" if ev["elevated"]
           else "(no significant descent)")
    print(f"elevated-trend: {ev['n_seg']} seg, slope={ev['slope_rec']:+.0f}bpm/rec "
          f"p(desc)={ev['p']:.2f} -> {tag}")
    if a.plot:
        plot(rows, a.plot, f"Continuous HR — {a.path}  (win {a.win:.0f}s, hop {a.hop:.1f}s)")


if __name__ == "__main__":
    main()
