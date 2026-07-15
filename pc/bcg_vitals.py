"""Radar vitals via BCG-style pipeline (ref: sleep_pad_algorithm sleep算法.md).

Adapts the load-cell BCG algorithm to the radar cube. Each range bin's coherent
slow-time projection = one "channel" (44 bins ~ the pad's 4 channels). Per channel:
  1. bandpass to remove baseline drift (the <0.05Hz drift that swamped raw FFT)
  2. SQI = E_band / (E_total - E_band)  -> weight/select best bins
  3. HR: FFT peak + autocorrelation (robust when FFT peak is weak), consensus
  4. RR: low-freq band peak
SQI-weighted fusion across the top bins.

    python bcg_vitals.py fall20_cube.npz --fps 18.78 --t0 41 --t1 112
"""
import argparse
import numpy as np
try:
    from scipy.signal import find_peaks
    _HAVE_SCIPY = True
except Exception:
    _HAVE_SCIPY = False

HR_HI = 2.5                 # 150 bpm ceiling
RR_LO, RR_HI = 0.12, 0.6    # 7-36 rpm
DR = 0.0234375
LAMBDA_MM = 5.0             # radar wavelength (λ=5mm) for phase->displacement
# Occupancy gate — measured empty vs 4 occupied at 15/30/45s windows
# (occupancy_probe.py / occ_window_probe.py). resp-band chest displacement RMS is
# the clean discriminator at EVERY window scale: empty <=1.4um vs occupied >=6um
# (>=8x gap). Inter-bin RR spread does NOT separate at window scale (empty
# 5-11rpm vs occupied 4-10rpm overlap) so it is diagnostic only, NOT gated on.
# Present iff displacement clears the floor. See memory vitals-occupancy-gate.
BREATH_DISP_MIN = 0.004     # mm, RMS resp-band displacement floor (empty ~0.001,
                            # occupied@15s >=0.010; 4um -> empty clean, big margin)


def bandpass(x, fps, lo, hi, notch_f0=None, notch_hw=0.022, notch_n=25):
    x = x - x.mean()
    f = np.fft.rfftfreq(len(x), 1 / fps)
    X = np.fft.rfft(x)
    X[(f < lo) | (f > hi)] = 0
    if notch_f0:                       # narrow-notch the breathing harmonic comb
        for n in range(1, notch_n):
            X[(f >= n * notch_f0 - notch_hw) & (f <= n * notch_f0 + notch_hw)] = 0
    return np.fft.irfft(X, n=len(x))


def beat_count(sig, fps, hi_bpm=150, height=0.25):
    """Time-domain heartbeat count (BCG J-peak method) -> bpm. Robust to the
    smooth breathing-harmonic residue that fools FFT argmax (sharp beats survive
    thresholding; the harmonic doesn't)."""
    s = sig / (sig.std() + 1e-9)
    dist = max(1, int(fps / (hi_bpm / 60)))
    if _HAVE_SCIPY:
        pk, _ = find_peaks(s, distance=dist, height=height)
        n = len(pk)
    else:
        n = sum(1 for i in range(1, len(s) - 1)
                if s[i] > s[i - 1] and s[i] > s[i + 1] and s[i] > height)
    return n / (len(sig) / fps) * 60


def sqi(x, fps, lo, hi):
    f = np.fft.rfftfreq(len(x), 1 / fps)
    S = np.abs(np.fft.rfft(x - x.mean())) ** 2
    band = (f >= lo) & (f <= hi)
    Eb = S[band].sum()
    return Eb / (S.sum() - Eb + 1e-12)


def fft_peak(x, fps, lo, hi):
    f = np.fft.rfftfreq(len(x), 1 / fps)
    S = np.abs(np.fft.rfft(x - x.mean())) ** 2
    m = (f >= lo) & (f <= hi)
    return f[m][np.argmax(S[m])] if m.any() else None


def autocorr_peak(sig, fps, lo_bpm=48, hi_bpm=150, interp=False):
    """Largest autocorrelation peak in a bpm range -> (bpm, normalized_height).
    Height (0-1) measures periodicity strength: used to arbitrate low vs high band.
    interp=True adds parabolic sub-lag interpolation of the SAME peak -> a smooth
    (non-quantized) bpm; the integer-lag grid steps ~6bpm near 80bpm, which makes a
    continuous track look steppy. Default False keeps the validated integer-lag
    value byte-identical (main path never sets interp)."""
    ac = np.correlate(sig, sig, "full")[len(sig) - 1:]
    if ac[0] <= 0:
        return None, 0.0
    ac = ac / ac[0]
    l0, l1 = int(fps / (hi_bpm / 60)), int(fps / (lo_bpm / 60))
    if l1 <= l0 + 1 or l1 >= len(ac):
        return None, 0.0
    k = l0 + int(np.argmax(ac[l0:l1]))
    lag = float(k)
    if interp and 0 < k < len(ac) - 1:                # refine the same peak lag
        y0, y1, y2 = ac[k - 1], ac[k], ac[k + 1]
        denom = y0 - 2 * y1 + y2
        if abs(denom) > 1e-9:
            lag = k + float(np.clip(0.5 * (y0 - y2) / denom, -0.5, 0.5))
    return fps / lag * 60, float(ac[k])


def autocorr_bpm(sig, fps, lo_bpm=48, hi_bpm=150):
    """First autocorrelation peak within a bpm range -> bpm (already-filtered sig)."""
    return autocorr_peak(sig, fps, lo_bpm, hi_bpm)[0]


# --- Extracted core (reused by bcg_vitals_rt.py for sliding-window HR). The
# phase-demod + band-prior recipe below is the VALIDATED core — do not change its
# math; the RT layer only wraps it in a sliding window + Kalman/continuity. ---

def demod_channels(C, bins):
    """Per-bin PHASE demodulation -> mm displacement time series (nbin, T).
    z = coherently-combined complex; disp = -λ/4π·unwrap(angle(z)). This is the
    validated cardiac channel — mm motion lives in phase, not amplitude."""
    chans = []
    for i in range(len(bins)):
        m = C[i].mean(0); m = m / (np.linalg.norm(m) + 1e-9)
        z = C[i] @ m.conj()                       # complex per frame
        phi = np.unwrap(np.angle(z))              # rad
        disp_mm = -LAMBDA_MM / (4 * np.pi) * (phi - phi.mean())
        chans.append(disp_mm)                     # mm displacement time series
    return np.array(chans)                        # (nbin, T)


def estimate_rr(chans, fps, topk=8):
    """RR via low-freq SQI-top bins, MEDIAN fft peak. Returns (rr_rpm, f0_hz,
    bin_spread, per_bin_list). f0 also drives the HR-band RR-harmonic notch."""
    rr_sqi = np.array([sqi(bandpass(c, fps, RR_LO, RR_HI), fps, RR_LO, RR_HI)
                       for c in chans])
    rr_top = np.argsort(rr_sqi)[::-1][:topk]
    rr_f = []
    for i in rr_top:
        ff = fft_peak(chans[i], fps, RR_LO, RR_HI)
        if ff: rr_f.append(ff * 60)
    rr = float(np.median(rr_f)) if rr_f else None
    f0 = rr / 60.0 if rr else 0.25
    spread = float(np.std(rr_f)) if len(rr_f) > 1 else 99.0
    return rr, f0, spread, rr_f


def occupancy(chans, fps, topk=8):
    """Presence gate from BREATHING COHERENCE — the upstream 'is a person here?'
    check that HR/tachy/AF all depend on. A stationary person has coherent
    breathing (0.12-0.6Hz): real mm-scale chest displacement AND the same rate
    across chest bins. Empty-room noise has ~1um displacement and scattered
    per-bin rates, which otherwise fools every estimator into false vitals
    (empty room -> false 120bpm tachycardia + sustained AF alert). Returns
    dict(present, disp_rms, rr_spread, rr_med). present iff BOTH the displacement
    floor and the inter-bin agreement ceiling say a person is there."""
    resp_sqi = np.array([sqi(bandpass(c, fps, RR_LO, RR_HI), fps, RR_LO, RR_HI)
                         for c in chans])
    top = np.argsort(resp_sqi)[::-1][:topk]
    rr_f, rms = [], []
    for i in top:
        b = bandpass(chans[i], fps, RR_LO, RR_HI)
        ff = fft_peak(chans[i], fps, RR_LO, RR_HI)
        if ff:
            rr_f.append(ff * 60)
        rms.append(float(np.sqrt(np.mean(b ** 2))))          # mm displacement RMS
    disp_rms = float(np.median(rms)) if rms else 0.0
    rr_spread = float(np.std(rr_f)) if len(rr_f) > 1 else 99.0   # diagnostic only
    rr_med = float(np.median(rr_f)) if rr_f else 0.0
    present = disp_rms >= BREATH_DISP_MIN
    return dict(present=present, disp_rms=disp_rms, rr_spread=rr_spread,
                rr_med=rr_med)


def hr_band_search(chans, fps, f0, lo, hi, topk=8, interp=False):
    """Autocorr-in-band HR over the SQI-top bins of ONE band. Returns dict with
    hr (median bpm), spread (inter-bin std), strength (median autocorr height =
    periodicity confidence), hr_bc (beat-count x-check), ac list, top bins.
    interp only refines the per-bin bpm (see autocorr_peak); default off."""
    hr_sqi = np.array([sqi(bandpass(c, fps, lo, hi, notch_f0=f0), fps, lo, hi)
                       for c in chans])
    top = np.argsort(hr_sqi)[::-1][:topk]
    ac, heights, bc = [], [], []
    for i in top:
        sig = bandpass(chans[i], fps, lo, hi, notch_f0=f0)
        bpm, h = autocorr_peak(sig, fps, int(round(lo * 60)), int(round(hi * 60)),
                               interp=interp)
        if bpm: ac.append(bpm); heights.append(h)
        bc.append(beat_count(sig, fps, hi_bpm=int(round(hi * 60))))
    return dict(
        hr=float(np.median(ac)) if ac else None,
        spread=float(np.std(ac)) if len(ac) > 1 else 99.0,
        strength=float(np.median(heights)) if heights else 0.0,
        hr_bc=float(np.median(bc)) if bc else None,
        ac=ac, top=top)


def hr_region_vote(chans, fps, f0, lo, split, hi, topk=8):
    """Region classifier for tachycardia. Runs autocorr over the FULL [lo,hi]
    band (single consistent filter) per SQI-top bin and asks: does the cardiac
    PERIOD land above `split`? Returns (median_bpm, frac_above_split). Fair
    across the resting/tachy boundary because it compares one wide-band period
    estimate, not intra-band peakiness (which is biased toward narrow high bands).
    Note: raw autocorr height and narrow-band spectral prominence do NOT
    discriminate — both saturate on band-limited residue; only the wide-band
    PERIOD does (verified on the 4 resting cubes: frac_above stays < 0.5)."""
    s = np.array([sqi(bandpass(c, fps, lo, hi, notch_f0=f0), fps, lo, hi)
                  for c in chans])
    top = np.argsort(s)[::-1][:topk]
    cand = []
    for i in top:
        sig = bandpass(chans[i], fps, lo, hi, notch_f0=f0)
        bpm, _ = autocorr_peak(sig, fps, int(round(lo * 60)), int(round(hi * 60)))
        if bpm: cand.append(bpm)
    if not cand:
        return None, 0.0
    frac = float(np.mean([c > split * 60 for c in cand]))
    return float(np.median(cand)), frac


def hr_fft_value(chans, fps, f0, lo, hi, topk=8):
    """HR from median FFT peak in a band. Used for the TACHY value: once the band
    is confirmed to START above the breathing-harmonic residue (>=1.7Hz), FFT
    argmax is clean (no lower residue to halve into) and gives far finer
    resolution than autocorr, whose integer-lag grid is coarse up here
    (only ~3 lags span 102-132bpm @19fps)."""
    s = np.array([sqi(bandpass(c, fps, lo, hi, notch_f0=f0), fps, lo, hi)
                  for c in chans])
    top = np.argsort(s)[::-1][:topk]
    pk = []
    for i in top:
        ff = fft_peak(chans[i], fps, lo, hi)
        if ff: pk.append(ff * 60)
    hr = float(np.median(pk)) if pk else None
    spread = float(np.std(pk)) if len(pk) > 1 else 99.0
    return hr, spread, pk


def estimate_hr(chans, fps, f0, topk=8, lo=1.0, hi=1.7,
                tachy_hi=None, vote_frac=0.5, interp=False):
    """Arbitrated HR. Resting band [lo,hi]=[1.0,1.7] is the VALIDATED default and
    is always what's returned unless tachy_hi is set AND a majority of SQI-top
    bins show the cardiac period sitting above `hi` (region vote). The low edge
    stays at `lo` in every path, so the 0.7-1.0Hz breathing-harmonic residue is
    never re-admitted. interp only sub-lag-refines the resting-band bpm for a
    smoother continuous track (off in the validated main path). Returns a result
    dict; band='HIGH' => tachycardia."""
    low = hr_band_search(chans, fps, f0, lo, hi, topk, interp=interp)
    out = dict(low=low, high=None, region=None, band="LOW", hr=low["hr"],
               spread=low["spread"], strength=low["strength"], hr_bc=low["hr_bc"])
    if tachy_hi and tachy_hi > hi:
        med, frac = hr_region_vote(chans, fps, f0, lo, hi, tachy_hi, topk)
        out["region"] = dict(median=med, frac_above=frac)
        if med is not None and med > hi * 60 and frac >= vote_frac:
            hr, spread, pk = hr_fft_value(chans, fps, f0, hi, tachy_hi, topk)
            if hr is not None:
                out.update(band="HIGH", hr=hr, spread=spread, strength=frac,
                           high=dict(hr=hr, spread=spread, pk=pk))
    return out


def chest_decoupled_hr(chans, fps, bins=None, lo=0.9, hi=2.3,
                       min_offset=3, breath_max_frac=0.5, q_conf=0.25):
    """Range-decoupled chest HR — the SPATIAL escape from RR-harmonic entanglement.

    When HR == k*RR the cardiac line is frequency-coincident with a breathing
    harmonic and NO frequency method (FFT/autocorr/notch) can split them. But the
    chest (heart) and abdomen (breathing) sit in DIFFERENT range bins on radial /
    long-axis geometry, so read HR at the CHEST CLUSTER — bins range-separated from
    the abdomen where breathing (and thus its harmonic) is suppressed, so the
    co-frequent cardiac pokes out. CRITICAL: do NOT harmonic-mask here (space has
    already suppressed breathing; masking by frequency would kill the on-harmonic
    cardiac). Validated 2026-07-14: chairL 76 (truth ~80), lie_long 68 (truth 71,
    HR==8xRR) — both where the stock estimate_hr locks the 80.6 harmonic artifact.

    REQUIRES chest/abdomen in different range bins: face-on/seated colocated
    geometry has no chest bin -> returns decoupled=False. Still MARGINAL (cardiac
    coherence q~0.4 at the noise floor, ~±2-5 bpm). ⚠️ v1 GATING IS UNRELIABLE (TODO):
    `confident` (median q>=q_conf) is NOT trustworthy — validated chairL reads q0.22
    (flagged low yet correct) while a colocated seated capture can read q>=0.25 and be
    confidently WRONG; and `decoupled=True` false-fires on SECONDARY-REFLECTION bins
    that sit >=min_offset from the abdomen but are not the real chest. Robust gating
    (chest must belong to the SAME contiguous breathing body cluster as the abdomen;
    reject far reflection clusters) is the open problem. Reliable today ONLY on clean
    radial/long-axis captures (validated lie_long). Do not blind-trust `confident`.

    Returns dict(hr, decoupled, confident, abdomen, chest, q, per_bin, reason).
    """
    nbin = len(chans)
    def _bin(i):
        return int(bins[i]) if bins is not None else i
    Pb = np.array([(np.abs(np.fft.rfft(bandpass(c, fps, RR_LO, RR_HI))) ** 2).sum()
                   for c in chans])
    ab = int(np.argmax(Pb))
    # chest candidates: range-separated from the abdomen, breathing suppressed,
    # but still on the body (breathing above the noise floor, not an empty bin)
    cand = [i for i in range(nbin)
            if 0.01 * Pb[ab] < Pb[i] < breath_max_frac * Pb[ab]
            and abs(_bin(i) - _bin(ab)) >= min_offset]
    if not cand:
        return dict(hr=None, decoupled=False, confident=False, abdomen=_bin(ab),
                    chest=[], q=0.0, per_bin=[], reason="colocated (no chest bin)")
    rows = []
    for i in cand:
        x = bandpass(chans[i], fps, lo, hi)
        f = np.fft.rfftfreq(len(x), 1 / fps)
        S = np.abs(np.fft.rfft(x - x.mean())) ** 2
        m = (f >= lo) & (f <= hi)
        fftpk = f[m][np.argmax(S[m])] * 60
        ac, q = autocorr_peak(x, fps, int(lo * 60), int(hi * 60))
        rows.append((i, fftpk, ac, q))
    rows.sort(key=lambda r: -r[3])                       # by cardiac coherence q
    top = rows[:max(3, len(rows) // 2)]                  # high-coherence half
    # per bin: trust FFT&autoc mean where they agree (<=8bpm), else the sharper FFT
    vals = [(fp + ac) / 2 if abs(fp - ac) <= 8 else fp for (_, fp, ac, q) in top]
    hr = float(np.median(vals))
    qm = float(np.median([r[3] for r in top]))
    return dict(hr=hr, decoupled=True, confident=qm >= q_conf, abdomen=_bin(ab),
                chest=[_bin(r[0]) for r in top], q=qm,
                per_bin=[(_bin(r[0]), round(r[1], 1), round(r[2], 1), round(r[3], 2))
                         for r in top], reason="")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("path")
    ap.add_argument("--fps", type=float, required=True)
    ap.add_argument("--t0", type=float, default=0.0)
    ap.add_argument("--t1", type=float, default=1e9)
    ap.add_argument("--topk", type=int, default=8)
    ap.add_argument("--tachy", type=float, default=0.0,
                    help="widen HR ceiling to this Hz (e.g. 2.2 = 132bpm) with "
                         "high-band arbitration; 0 = disabled (validated resting)")
    a = ap.parse_args()

    d = np.load(a.path, allow_pickle=True)
    cube = np.asarray(d["snapshots"], dtype=np.complex64)
    counts = d["counts"].astype(int); bins = d["bins"].astype(int)
    K = int(counts.min())
    i0, i1 = int(a.t0 * a.fps), min(K, int(a.t1 * a.fps))
    C = cube[:, i0:i1, :]

    # per-bin PHASE demod -> mm displacement (see demod_channels). Phase carries
    # mm motion (Δφ=4π·Δr/λ ≈ 2.5rad/mm @λ=5mm); the amplitude view threw it away.
    chans = demod_channels(C, bins)               # (nbin, T)

    # --- OCCUPANCY gate FIRST: no coherent breathing => no person => suppress all
    # vitals (else empty-room noise reads as false 120bpm tachycardia + AF). ---
    occ = occupancy(chans, a.fps, a.topk)
    print(f"occupancy: disp_rms={occ['disp_rms']*1000:.2f}um "
          f"rr_spread={occ['rr_spread']:.1f}rpm -> "
          f"{'PERSON' if occ['present'] else 'NO PERSON'}")
    if not occ["present"]:
        print("  -> NO PERSON: vitals suppressed (no reliable HR/RR/AF).")
        return

    # --- RR first: low-freq band, MEDIAN fft peak. Also gives f0 for HR notch. ---
    rr, f0, rr_spread, rr_f = estimate_rr(chans, a.fps, a.topk)
    rr_conf = "HIGH" if rr_spread < 2 else ("MED" if rr_spread < 4 else "LOW")
    print(f"RR (median) = {rr:.0f} rpm  [{rr_conf}, bin-spread {rr_spread:.1f}]  "
          f"(f0={f0:.3f}Hz, 5th harm={5*f0*60:.0f}bpm)  per-bin: {[round(v) for v in rr_f]}")

    # --- HR: at ~4m the cardiac phase is WEAKER than the breathing-harmonic
    # residue, so any "find the strongest peak" (FFT argmax, harmonic-sum, or
    # coprime/CRT folding) locks onto the residue -> halving. No transform lifts a
    # sub-noise signal; the lever is a PHYSIOLOGICAL BAND PRIOR that excludes the
    # low-freq residue. AUTOCORRELATION in [1.0-1.7Hz] (60-102bpm, resting-elderly
    # prior) is SNR-robust (responds to the period, not a single peak) and matched
    # Apple Watch across seated/side/lying (all ~81). Beat-count is a strong-signal
    # cross-check. --tachy widens the ceiling with high-band arbitration. ---
    HR_PHYS_LO, HR_PHYS_HI = 1.0, 1.7
    tachy_hi = a.tachy if a.tachy else None
    res = estimate_hr(chans, a.fps, f0, a.topk, HR_PHYS_LO, HR_PHYS_HI, tachy_hi=tachy_hi)
    lo_r = res["low"]
    print(f"HR band {HR_PHYS_LO}-{HR_PHYS_HI}Hz (physiological prior + RR-notch). "
          f"top bins: " + ", ".join(f"{bins[i]}({bins[i]*DR:.2f}m)" for i in lo_r["top"]))
    conf = "HIGH" if lo_r["spread"] < 3 else ("MED" if lo_r["spread"] < 6 else "LOW")
    print(f"  autocorr@band (PRIMARY) = {lo_r['hr']:.0f} bpm  [{conf}, bin-spread "
          f"{lo_r['spread']:.1f}]  {[round(v) for v in lo_r['ac']]}")
    print(f"  beat-count    (x-check) = {lo_r['hr_bc']:.0f} bpm")
    if res["region"] is not None:
        rg = res["region"]
        print(f"  tachy vote {HR_PHYS_HI}-{tachy_hi}Hz: wide-period median="
              f"{rg['median'] and round(rg['median'])}bpm frac>{HR_PHYS_HI}Hz={rg['frac_above']:.0%} "
              f"-> {res['band']} band" + (f" (FFT {res['hr']:.0f}bpm)" if res['high'] else ""))
    print(f"  -> HR = {res['hr']:.0f} bpm  [{res['band']}]")


if __name__ == "__main__":
    main()
