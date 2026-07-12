"""Continuity-based precordium-bin selector on the breath-nulled residual.

Crux from tachy_verdict.py: high-freq-fraction / energy heuristics pick NOISE bins
(empty room fabricates a persistent high ridge at 56%). But the ORACLE-correct bin
(tachy2 bin 65) had a distinguishing property the noise bins lacked: its ridge was
TEMPORALLY CONTINUOUS -- it tracked a smooth 131->110->91 trajectory, whereas noise
ridges jump erratically frame to frame.

So select the bin by ridge CONTINUITY (not high-freq fraction), weighted by residual
cardiac energy. Then read HR from the selected bin -- the VALUE (not the selection)
decides tachy vs resting. A resting bin should be selected too (smooth ~80 ridge) and
simply read ~80; a tachy bin reads ~113; noise should lose on continuity.

Continuity score per bin:
  - longest run of frames staying within +-RUN_TOL bpm of a local median (real HR
    holds a rate for many seconds; noise does not) -> run_frac in [0,1]
  - median frame-to-frame |jump| (bpm); real HR drifts slowly
  score = run_frac / (1 + med_jump/10) * sqrt(energy_share)

PASS = tachy cubes -> selected bin reads >102bpm; resting -> ~60-100; and the noise/
empty selection has low continuity (or is moot behind the occupancy gate).

    python continuity_selector.py
"""
import numpy as np
from bcg_vitals import (bandpass, sqi, fft_peak, demod_channels, RR_LO, RR_HI)
from spatial_null_probe import per_antenna_disp, breath_subspace
from tachy_existence_probe import stft_peaks, CARD_LO, CARD_HI

RUN_TOL = 8.0          # bpm: a "held rate" tolerance
HIGH_BPM = 102.0       # tachy boundary


def longest_run_frac(ridge, tol=RUN_TOL):
    """Longest run of consecutive frames within tol of the run's running median."""
    if len(ridge) < 2:
        return 0.0
    best = cur = 1
    anchor = ridge[0]
    for t in range(1, len(ridge)):
        if abs(ridge[t] - anchor) <= tol:
            cur += 1
            anchor = 0.5 * anchor + 0.5 * ridge[t]     # slow-drift anchor
        else:
            cur = 1
            anchor = ridge[t]
        best = max(best, cur)
    return best / len(ridge)


def analyze(path, fps):
    d = np.load(path, allow_pickle=True)
    cube = np.asarray(d["snapshots"], dtype=np.complex64)
    counts = d["counts"].astype(int); bins = d["bins"].astype(int)
    C = cube[:, :int(counts.min()), :]
    chans = demod_channels(C, bins)
    rr_sqi = np.array([sqi(bandpass(c, fps, RR_LO, RR_HI), fps, RR_LO, RR_HI)
                       for c in chans])
    rtop = np.argsort(rr_sqi)[::-1][:8]
    f0s = [x for x in (fft_peak(chans[i], fps, RR_LO, RR_HI) for i in rtop) if x]
    f0 = float(np.median(f0s)) if f0s else 0.3

    scored = []
    energies = []
    ridges = []
    for i in range(len(bins)):
        D = per_antenna_disp(C[i])
        Ub, _, _ = breath_subspace(D, fps, f0)
        Dr = D @ (np.eye(16) - Ub @ Ub.T)
        Dh = np.column_stack([bandpass(Dr[:, a], fps, CARD_LO, CARD_HI) for a in range(16)])
        e = (Dh ** 2).sum(0); z = Dh @ np.sqrt(e / (e.sum() + 1e-12))
        ridges.append(stft_peaks(z, fps) * 60)
        energies.append(float((z ** 2).sum()))
    energies = np.array(energies); eshare = energies / (energies.sum() + 1e-12)

    for i in range(len(bins)):
        r = ridges[i]
        if len(r) < 4:
            scored.append((-1, i, 0, 0, 0)); continue
        run = longest_run_frac(r)
        jump = float(np.median(np.abs(np.diff(r))))
        score = run / (1 + jump / 10) * np.sqrt(eshare[i])
        scored.append((score, i, float(np.median(r)), run, jump))
    scored.sort(reverse=True)
    top = scored[:3]
    sc, i, med, run, jump = top[0]
    verdict = "TACHY" if med > HIGH_BPM else "resting/normal"
    print(f"  {path:20s} sel bin {bins[i]:>3} ({bins[i]*0.0234375:.2f}m)  "
          f"med={med:>5.0f}bpm  run={run:.2f} jump={jump:>4.0f}  -> {verdict}")
    for sc, i, med, run, jump in top[1:]:
        print(f"  {'':20s}   alt bin {bins[i]:>3}          "
              f"med={med:>5.0f}bpm  run={run:.2f} jump={jump:>4.0f}")


CUBES = [("tachy2_cube.npz", "TACHY 110-131"), ("tachy1_cube.npz", "TACHY far"),
         ("sit39_cube.npz", "resting 81"), ("lie41_cube.npz", "resting 77"),
         ("sidesit_cube.npz", "resting 78"), ("fall20_cube.npz", "resting 80"),
         ("emptyT_cube.npz", "EMPTY")]
print("continuity-selected precordium bin (breath-nulled residual):\n")
for path, truth in CUBES:
    try:
        print(f"[{truth}]")
        analyze(path, 18.78)
    except Exception as ex:
        print(f"  {path}: ERROR {ex}")
