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
from bcg_vitals import demod_channels, estimate_rr, estimate_hr, hr_band_search


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
    res = estimate_hr(chans, fps, f0, tachy_hi=tachy_hi)
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
    rows = []
    for i in range(0, max(1, T - w + 1), hop):
        C = cube[:, i:i + w, :]
        chans = demod_channels(C, bins)
        _, f0, _, _ = estimate_rr(chans, fps)
        res = estimate_hr(chans, fps, f0, tachy_hi=tachy_hi)
        hr_meas, spread, band = res["hr"], res["spread"], res["band"]
        quality = res["low"]["strength"]           # SQI proxy (autocorr height)
        t = (i + w / 2) / fps
        if hr_meas is None:
            rows.append((t, np.nan, kf.coast(), "none", 99.0, band, quality)); continue

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
        rows.append((t, hr_meas, hr_out, src, spread, band, quality))
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
    print(f"{'t(s)':>6} {'meas':>6} {'HR':>6}  {'src':<8} {'spread':>6} {'qual':>5} band")
    for t, meas, hr, src, spread, band, qual in rows:
        ms = f"{meas:6.0f}" if np.isfinite(meas) else "   -- "
        hs = f"{hr:6.1f}" if hr is not None else "   -- "
        print(f"{t:6.1f} {ms} {hs}  {src:<8} {spread:6.1f} {qual:5.2f} {band}")
    smooth = np.array([r[2] for r in rows if r[2] is not None], dtype=float)
    if len(smooth):
        print(f"\nKalman HR: median {np.median(smooth):.1f}, "
              f"range {np.nanmin(smooth):.0f}-{np.nanmax(smooth):.0f} bpm")
    src_ct = {}
    for r in rows: src_ct[r[3]] = src_ct.get(r[3], 0) + 1
    print("sources:", src_ct)
    if a.plot:
        plot(rows, a.plot, f"Continuous HR — {a.path}  (win {a.win:.0f}s, hop {a.hop:.1f}s)")


if __name__ == "__main__":
    main()
