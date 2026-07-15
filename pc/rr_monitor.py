"""Continuous RR tracking + breathing-PATTERN classification (regular / erratic / apnea).

RR is the radar's most reliable vital (breathing 0.15-0.5Hz, peak/median ~1000x); this is
the refocus after HR was finalized as a geometry-gated conditional (see memory
vitals-priority-pivot). Design grounded in the actual signal (rr_monitor calibration
2026-07-14):
  - windowed FFT-RR is reliable; per-breath PEAK counting is NOT at 4m (noise/harmonics
    create false peaks: sit39 peak-rate 25 vs true 14). So the RR TRACK uses windowed FFT.
  - breath TIMING/regularity uses ANALYTIC-PHASE cycle counting (unwrap Hilbert phase of the
    tight-band respiration; each 2*pi = one breath) — robust where peak-detection fails.
  - IBI regularity separates breathing (CV ~0.3) from noise (empty CV ~1.9).
  - apnea/hypopnea = RELATIVE amplitude drop vs the person's own baseline (not an absolute
    floor — per-person depth varies 0.03-0.56mm), sustained >= APNEA_MIN_S.

    python rr_monitor.py case/lie41_cube.npz --fps 18.78
"""
import argparse
import numpy as np
import bcg_vitals as bv
import living_gate as lg

APNEA_MIN_S = 10.0          # sustained amplitude cessation to call apnea
APNEA_FRAC = 0.30           # amplitude below this fraction of baseline = cessation
ERRATIC_CV = 0.55           # IBI coefficient-of-variation above this = erratic breathing
# Presence uses the validated SCENE-INVARIANT living_gate (RR-band spatial concentration +
# chest-sized cluster), NOT an absolute displacement floor: a noisy empty room varies by
# environment (2um one session, 12um another) and defeats any fixed floor — a real empty
# capture (12um clutter @seated-4m) fooled the old floor into 'REGULAR breathing @20rpm'.
# living_gate rejects it (concentration/cluster are scene-invariant). See memory
# vitals-occupancy-gate.


def _analytic(x):
    n = len(x); X = np.fft.fft(x); H = np.zeros(n)
    if n % 2 == 0: H[0] = H[n // 2] = 1; H[1:n // 2] = 2
    else:          H[0] = 1; H[1:(n + 1) // 2] = 2
    return np.fft.ifft(X * H)


def respiration_channel(cube, bins, fps):
    """Strongest-breathing (abdomen) bin -> respiration displacement + its RR f0."""
    chans = bv.demod_channels(cube, bins)
    Pb = np.array([np.mean(bv.bandpass(c, fps, bv.RR_LO, bv.RR_HI) ** 2) for c in chans])
    ab = int(np.argmax(Pb))
    resp = bv.bandpass(chans[ab], fps, bv.RR_LO, bv.RR_HI)
    rr, f0, spread, _ = bv.estimate_rr(chans, fps)   # validated median RR (stable)
    return resp, chans[ab], rr, f0, spread, int(bins[ab]) if bins is not None else ab


def continuous_rr(resp, fps, win=20.0, hop=2.0):
    """Sliding-window FFT-RR track. Returns (t_centers, rr_rpm)."""
    w, h = int(win * fps), int(hop * fps); N = len(resp)
    t, rr = [], []
    for s in range(0, max(1, N - w), h):
        ff = bv.fft_peak(resp[s:s + w], fps, bv.RR_LO, bv.RR_HI)
        if ff:
            t.append((s + w / 2) / fps); rr.append(ff * 60)
    return np.array(t), np.array(rr)


def breath_cycles(resp, fps, f0):
    """Analytic-phase cycle counting -> breath times (s) and inter-breath intervals (s)."""
    tight = bv.bandpass(resp, fps, max(bv.RR_LO, f0 - 0.08), f0 + 0.08)
    phase = np.unwrap(np.angle(_analytic(tight)))
    phase = phase - phase[0]
    n = int(phase[-1] // (2 * np.pi))
    idx = np.arange(len(phase))
    times = [np.interp(2 * np.pi * k, phase, idx) / fps for k in range(1, n + 1)]
    times = np.array(times)
    ibi = np.diff(times) if len(times) > 1 else np.array([])
    return times, ibi


def amplitude_track(disp_raw, fps, win=5.0, hop=1.0):
    """TIME-LOCAL breathing depth: per-window detrended std of the RAW displacement (mm).
    Uses per-window DETREND (not a global bandpass) so a real cessation goes to ~0 locally —
    a global FFT bandpass bleeds surrounding breathing into a flat gap and hides apnea.
    Breathing (~100um) dominates the 5s-window AC over cardiac (~um) and drift (detrended)."""
    w, h = int(win * fps), int(hop * fps); N = len(disp_raw)
    x = np.arange(w); A = np.vstack([x, np.ones_like(x)]).T
    t, amp = [], []
    for s in range(0, max(1, N - w), h):
        seg = disp_raw[s:s + w]
        m, b = np.linalg.lstsq(A, seg, rcond=None)[0]
        t.append((s + w / 2) / fps); amp.append(float(np.std(seg - (m * x + b))))
    return np.array(t), np.array(amp)


def reflection_track(cube_ab, fps, win=5.0, hop=1.0):
    """Static body-reflection strength |z| per window at the abdomen bin. A breath-holding
    person still reflects radar (reflection retained); an empty chair does not — this is
    what separates APNEA (cessation, person present) from ABSENCE (person left), since the
    breathing amplitude drops to ~0 in BOTH."""
    mag = np.abs(cube_ab).mean(1)
    w, h = int(win * fps), int(hop * fps)
    t, r = [], []
    for s in range(0, max(1, len(mag) - w), h):
        t.append((s + w / 2) / fps); r.append(float(mag[s:s + w].mean()))
    return np.array(t), np.array(r)


REFL_PRESENT = 0.5          # reflection >= this fraction of baseline => person still present


def find_apnea(t_amp, amp, baseline, refl, refl_base, fps):
    """Sustained (>= APNEA_MIN_S) breathing cessation, split by body reflection into APNEA
    (reflection retained -> person present holding breath) vs ABSENCE (reflection lost ->
    person left). Returns (apnea_events, absence_events), each (start_s, end_s, dur_s)."""
    low = amp < APNEA_FRAC * baseline
    apnea, absence, i, n = [], [], 0, len(low)
    while i < n:
        if low[i]:
            j = i
            while j < n and low[j]:
                j += 1
            dur = t_amp[j - 1] - t_amp[i]
            if dur >= APNEA_MIN_S:
                ev = (round(float(t_amp[i]), 1), round(float(t_amp[j - 1]), 1), round(float(dur), 1))
                present = np.mean(refl[i:j]) >= REFL_PRESENT * refl_base
                (apnea if present else absence).append(ev)
            i = j
        else:
            i += 1
    return apnea, absence


def monitor(cube, bins, fps, dr=None):
    """Full RR monitor. Returns dict with the RR track, regularity, depth, pattern, apnea.

    PATTERN precedence: NO_BREATHING (presence gate) > APNEA (sustained cessation) >
    ERRATIC (IBI-CV high) > REGULAR. Presence is gated FIRST via the validated occupancy
    check (empty-room noise otherwise reads as 'regular breathing' because analytic-phase
    cycle-counting makes any narrowband noise look periodic — only the displacement floor
    separates them). Regularity is judged on IBI-CV (breath-to-breath), NOT the windowed
    FFT-RR track std, which carries estimator noise (harmonic hopping) not real irregularity.
    """
    chans = bv.demod_channels(cube, bins)
    dr = float(dr) if dr is not None else bv.DR
    occ = lg.living_present(chans, bins, dr, fps)     # scene-invariant living-person gate
    Pb = np.array([np.mean(bv.bandpass(c, fps, bv.RR_LO, bv.RR_HI) ** 2) for c in chans])
    ab_idx = int(np.argmax(Pb))
    resp = bv.bandpass(chans[ab_idx], fps, bv.RR_LO, bv.RR_HI)
    disp_raw = chans[ab_idx]
    rr_est, f0, spread, _ = bv.estimate_rr(chans, fps)
    ab = int(bins[ab_idx]) if bins is not None else ab_idx

    t_rr, rr = continuous_rr(resp, fps)
    times, ibi = breath_cycles(resp, fps, f0)
    t_amp, amp = amplitude_track(disp_raw, fps)
    t_ref, refl = reflection_track(cube[ab_idx], fps)
    baseline = float(np.median(amp)) if len(amp) else 0.0
    refl_base = float(np.median(refl)) if len(refl) else 0.0
    present = bool(occ["present"])                     # scene-invariant living_gate verdict
    apnea, absence = (find_apnea(t_amp, amp, baseline, refl, refl_base, fps)
                      if (present and baseline > 0) else ([], []))

    rr_med = float(rr_est) if rr_est else (float(np.median(rr)) if len(rr) else None)
    rr_track_mad = float(np.median(np.abs(rr - rr_med))) if len(rr) > 1 else 0.0  # robust
    ibi_cv = float(np.std(ibi) / np.mean(ibi)) if len(ibi) > 1 else 99.0
    depth_cv = float(np.std(amp) / (np.mean(amp) + 1e-12)) if len(amp) > 1 else 0.0

    if not present:
        pattern, rr_med = "NO_BREATHING", None
    elif apnea:
        pattern = "APNEA"
    elif absence:
        pattern = "LEFT"          # person departed mid-record (cessation + reflection lost)
    elif ibi_cv > ERRATIC_CV:
        pattern = "ERRATIC"
    else:
        pattern = "REGULAR"
    rr_conf = "HIGH" if spread < 2 else ("MED" if spread < 4 else "LOW")
    return dict(rr_rpm=rr_med, rr_track_mad=rr_track_mad, rr_conf=rr_conf, f0=f0, abd_bin=ab,
                present=present, present_frac=occ.get("present_frac", 0.0),
                ibi_cv=ibi_cv, n_breaths=len(times), depth_mm=baseline, depth_cv=depth_cv,
                pattern=pattern, apnea=apnea, absence=absence,
                t_rr=t_rr, rr_track=rr, t_amp=t_amp, amp=amp)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("path"); ap.add_argument("--fps", type=float, default=18.78)
    ap.add_argument("--plot", default=None)
    a = ap.parse_args()
    d = np.load(a.path, allow_pickle=True)
    bins = d["bins"].astype(int); K = int(d["counts"].astype(int).min())
    cube = np.asarray(d["snapshots"], np.complex64)[:, :K, :]
    dr = float(d["dr_m"]) if "dr_m" in d.files else None
    r = monitor(cube, bins, a.fps, dr=dr)
    if r["rr_rpm"] is not None:
        print(f"RR = {r['rr_rpm']:.0f} rpm  [{r['rr_conf']}, track-MAD {r['rr_track_mad']:.1f}]  "
              f"(abd bin{r['abd_bin']}, {r['n_breaths']} breaths)")
    else:
        print(f"RR = --  (no living person; living_gate present_frac {r['present_frac']:.0%})")
    print(f"regularity: IBI-CV {r['ibi_cv']:.2f}   depth {r['depth_mm']*1000:.0f}um "
          f"(CV {r['depth_cv']:.2f})   present={r['present']}")
    tail = ""
    if r["apnea"]: tail += f"   apnea {r['apnea']}"
    if r.get("absence"): tail += f"   absence/left {r['absence']}"
    print(f"PATTERN: {r['pattern']}{tail}")
    if a.plot:
        import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
        fig, (a1, a2) = plt.subplots(2, 1, figsize=(11, 5), sharex=True)
        a1.plot(r["t_rr"], r["rr_track"], ".-"); a1.set_ylabel("RR rpm"); a1.grid(alpha=.3)
        a1.set_title(f"{a.path.split('/')[-1]}  RR={r['rr_rpm']:.0f}  {r['pattern']}")
        a2.plot(r["t_amp"], r["amp"] * 1000); a2.axhline(APNEA_FRAC * r['depth_mm'] * 1000, color='r', ls=':')
        a2.set_ylabel("depth um"); a2.set_xlabel("s"); a2.grid(alpha=.3)
        fig.tight_layout(); fig.savefig(a.plot, dpi=90); print(f"saved {a.plot}")


if __name__ == "__main__":
    main()
