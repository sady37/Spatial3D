"""Fast lying/person detection by RANGE-intensity change vs an empty baseline.

Rationale (validated 2026-07-21): differencing the coherent X/Y grid vs empty
FAILS (multipath drift swamps it), but the RANGE axis is the radar's reliable
axis -- a person raises the power of the range bins they occupy. A LYING body is
a big, low reflector SPREAD over a contiguous RANGE SPAN (body length), so it
shows a wide band of elevated range bins; a standing/seated person is compact in
range. So: per-bin power(range) minus the empty baseline; contiguous bins with
+Delta above the multipath floor = a person; a long span = lying.

This is the floor-band (z~0-30cm) occlusion idea done on the reliable axis --
true elevation/z is rank-collapsed here (verified), so we use range not z.
"""
from __future__ import annotations
import numpy as np

from .build_static_scene import real_array, AZ, AZG, to_ground, NEAR_GATE


def range_power(npz):
    """Per-bin (range, power_dB) and the raw covariances/bins for a capture."""
    d = np.load(npz, allow_pickle=True)
    bins = d["bins"].astype(int); cov = d["covariances"]; dr = float(d["dr_m"])
    pw = np.array([np.real(np.trace(cov[i])) for i in range(len(bins))])
    return dict(bins=bins, range_m=bins * dr, power_db=10 * np.log10(pw + 1e-12),
                cov=cov, dr=dr)


def detect_lying_vs_baseline(base_npz, live_npz, thr_db=5.0, floor_db=2.0):
    """Compare a live capture's range-power to the empty baseline.

    thr_db  : a bin counts as 'person' if its power rose > thr_db over baseline
              (must clear the ~floor_db inter-capture multipath drift).
    Returns range, delta_db, person_bins, the longest contiguous elevated SPAN
    (m) + its excess + az-derived (x,y), and a lying_score = span_m * mean_excess.
    """
    b = range_power(base_npz); L = range_power(live_npz)
    # align by BIN NUMBER, not index: a 32-bin (bins 32-63) sit/stand capture must line
    # up with the 63-bin (bins 1-63) empty baseline by PHYSICAL RANGE, else [:n] subtracts
    # different ranges (garbage). Compare only the common bins.
    common = np.intersect1d(b["bins"], L["bins"])
    iB = {int(v): i for i, v in enumerate(b["bins"])}; iL = {int(v): i for i, v in enumerate(L["bins"])}
    baseI = np.array([iB[int(v)] for v in common]); liveI = np.array([iL[int(v)] for v in common])
    rng = L["range_m"][liveI]
    # normalise each profile to its OWN full-profile noise floor (median over ALL its bins)
    # so absolute gain / AGC differences cancel; the PERSON is a local bump.
    bp = b["power_db"][baseI] - np.median(b["power_db"])
    lp = L["power_db"][liveI] - np.median(L["power_db"])
    delta = lp - bp
    keep = rng >= NEAR_GATE
    person = keep & (delta > thr_db)
    # longest contiguous run of person bins (by range extent) = the body span;
    # a lying body spans MORE range than a seated/standing one.
    idx = np.where(person)[0]
    runs = np.split(idx, np.where(np.diff(idx) > 1)[0] + 1) if len(idx) else []
    span_bins = max(runs, key=lambda run: rng[run[-1]] - rng[run[0]],
                    default=np.array([], int)) if runs else np.array([], int)
    span_m = float(rng[span_bins[-1]] - rng[span_bins[0]]) if len(span_bins) > 1 else 0.0
    excess = float(delta[span_bins].mean()) if len(span_bins) else 0.0
    # locate: az-peak of the DIFFERENCE spectrum at the strongest person bin
    loc = None
    if len(idx):
        i0 = idx[np.argmax(delta[idx])]
        arr = real_array()
        def spec(R):
            return np.array([np.real(a.conj() @ R @ a) for a in
                             (arr.steering_vector(az, 0.0) for az in AZ)])
        Pd = spec(L["cov"][liveI[i0]]) - spec(b["cov"][baseI[i0]])
        az = AZ[int(np.argmax(Pd))]
        x, y = to_ground(rng[i0], az)
        loc = (round(float(x), 2), round(float(y), 2), round(float(rng[i0]), 2))
    return dict(range_m=rng, delta_db=delta, person_bins=person,
                span_m=round(span_m, 2), excess_db=round(excess, 1),
                n_person=int(person.sum()), location=loc,
                lying_score=round(span_m * max(excess, 0), 1))


def _motion_profile(npz):
    """Per-range MOTION energy = trace of the temporal fluctuation covariance
    fl = <x x^H> - <x><x>^H (from raw slow-time SNAPSHOTS -- the stored
    'mean'+cov are broken). The living/breathing person shows here; static
    clutter and walls ~0. Also returns the per-bin motion az-spectrum."""
    d = np.load(npz, allow_pickle=True)
    b = d["bins"].astype(int); dr = float(d["dr_m"])
    if "snapshots" not in d:
        raise ValueError(f"{npz} has no 'snapshots' (needed for fluctuation)")
    S = np.asarray(d["snapshots"]); arr = real_array()
    rng = b * dr
    mot = np.zeros(len(b)); motaz = np.zeros((len(b), len(AZ)))
    for i in range(len(b)):
        X = S[i]; X = X[np.any(X != 0, axis=1)]
        if len(X) < 8:
            continue
        R = (X.conj().T @ X) / len(X); m = X.mean(0)
        fl = R - np.outer(m, np.conj(m))
        mot[i] = np.real(np.trace(fl))
        motaz[i] = np.array([max(np.real(a.conj() @ fl @ a), 0.0) for a in
                             (arr.steering_vector(az, 0.0) for az in AZ)])
    return b, rng, mot, motaz


def detect_lying(npz, baseline="empty_20260721", thr=15.0, near_m=0.9):
    """⭐ Motion-CHANGE-per-range detector (validated 2026-07-21).

    Detect the CHANGE (user): per-range motion(now) vs motion(baseline empty),
    in dB. The living person lights up +30 dB at their range; near-field DC
    (high intensity but STABLE) CANCELS in the difference; each range is judged
    on its OWN scale (near-high / far-low) -- never a global threshold. RANGE
    localises to ~1 bin (ChairR 3.39 vs true 3.28, ChairL 4.13 vs 4.20); azimuth
    stays coarse (29deg beam) so the lateral x is only a rough bearing. A LYING
    body = the motion-change spread over a contiguous RANGE SPAN (body length).
    """
    HERE = __file__.rsplit("/", 1)[0]
    bL, rngL, mot, motaz = _motion_profile(npz)
    base = baseline if "/" in baseline else f"{HERE}/../case/{baseline}.npz"
    bB, _, motB, _ = _motion_profile(base)
    # align by BIN NUMBER (32-bin sit/stand vs 63-bin empty must match by range, not index)
    common = np.intersect1d(bL, bB)
    iL = {int(v): i for i, v in enumerate(bL)}; iB = {int(v): i for i, v in enumerate(bB)}
    liveI = np.array([iL[int(v)] for v in common]); baseI = np.array([iB[int(v)] for v in common])
    rng = rngL[liveI]; motaz = motaz[liveI]
    change = 10*np.log10(mot[liveI] + 1e-9) - 10*np.log10(motB[baseI] + 1e-9)  # per-range CHANGE
    change = np.where(rng >= near_m, change, -99.0)
    person = change > thr
    rows = np.where(person)[0]
    span_m = float(rng[rows[-1]] - rng[rows[0]]) if len(rows) > 1 else 0.0
    ip = int(np.argmax(change))
    ia = int(np.argmax(motaz[ip]))
    # azimuth-spread proxy: # of AZ steps above half-max at the peak range. A TANGENTIAL
    # body spreads WIDE in azimuth (range-span's complement); radial/compact is narrow.
    _azpk = motaz[ip]; az_halfmax = int((_azpk > 0.5 * _azpk.max()).sum()) if _azpk.max() > 0 else 0
    x, y = to_ground(rng[ip], AZ[ia])
    # DYNAMIC BASELINE — decision 2026-07-21: LOG-ONLY, do NOT update. When no
    # person is present this frame COULD be blended into the baseline (slow EMA,
    # motion-gated, event-frozen — see memory), but we deliberately DON'T yet;
    # we only flag it so a shadow log can accumulate for later policy tuning.
    update_candidate = bool(person.sum() == 0)          # empty/no-life -> refreshable
    return dict(range_m=rng, change_db=change,
                peak_db=round(float(change[ip]), 1), peak_range=round(float(rng[ip]), 2),
                n_person_bins=int(person.sum()), span_m=round(span_m, 2), az_halfmax=az_halfmax,
                update_candidate=update_candidate,      # LOG-ONLY (baseline stays fixed)
                location=(round(float(x), 2), round(float(y), 2), round(float(rng[ip]), 2)))


if __name__ == "__main__":
    import sys
    HERE = __file__.rsplit("/", 1)[0]
    # usage: python -m spatial3d.lying_detect [BASE_label CASE1 CASE2 ...]
    # default = the 0714 narrow-window demo; for a full-room lying capture run
    #   python -m spatial3d.lying_detect empty_20260721 lie_floor_20260721
    # usage: python -m spatial3d.lying_detect [BASELINE CASE1 CASE2 ...]
    args = sys.argv[1:]
    if len(args) >= 2:
        base_label, cases = args[0], args[1:]
    else:
        base_label = "empty_20260721"
        cases = ["empty_20260721", "ChairR_20260721", "ChairL_20260721", "standR_20260721"]
    print(f"baseline (still-reference): {base_label}   [baseline is FIXED — dynamic update LOG-ONLY]\n")
    print(f"{'case':20} {'peak(dB)':>8} {'@range':>7} {'#bins':>5} {'span(m)':>8} {'loc(x,y,r)':>18} {'bl-update?':>10}   true")
    print("-" * 108)
    for c in cases:
        r = detect_lying(f"{HERE}/../case/{c}.npz", baseline=base_label)
        d = np.load(f"{HERE}/../case/{c}.npz", allow_pickle=True)
        t = d["person_xyz"] if "person_xyz" in d else None
        tru = f"({-t[0]:+.2f},{t[1]:.2f})" if t is not None and np.isfinite(t[0]) else "EMPTY"
        # LOG-ONLY: 'candidate(not applied)' if empty/no-life; baseline NOT updated.
        blu = "cand(log)" if r["update_candidate"] else "frozen"
        print(f"{c:20} {r['peak_db']:>8} {r['peak_range']:>7} {r['n_person_bins']:>5} "
              f"{r['span_m']:>8} {str(r['location']):>18} {blu:>10}   {tru}")
