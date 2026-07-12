"""Make-or-break: answer-independent tachy discriminator via spatial breath-null.

Selection rule uses NO ground truth. For each range bin:
  1. per-antenna phase demod -> breath-subspace null -> cardiac-band residual z
  2. STFT ridge (peak in [1.0-2.5]Hz per 8s frame)
  3. high_frac = fraction of frames with ridge > 1.7Hz (102bpm)
     stability = 1 - MAD(ridge)/median(ridge)   (temporal coherence of the ridge)
     evidence  = high_frac * max(stability,0) * sqrt(residual cardiac energy share)
Pick the bin with max evidence = the putative precordium point-source. Classify
TACHY if that bin's median ridge > 102bpm AND high_frac >= 0.35.

PASS = tachy cubes -> TACHY with correct bpm; resting + empty -> not TACHY.

    python tachy_verdict.py
"""
import numpy as np
from bcg_vitals import (bandpass, sqi, fft_peak, demod_channels, RR_LO, RR_HI)
from spatial_null_probe import per_antenna_disp, breath_subspace
from tachy_existence_probe import stft_peaks, CARD_LO, CARD_HI

HIGH_HZ = 1.7          # 102 bpm boundary
HIGH_FRAC_MIN = 0.35


def select_and_classify(path, fps):
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

    # residual cardiac energy per bin (for the energy-share weight)
    zs, energies, ridges = [], [], []
    for i in range(len(bins)):
        D = per_antenna_disp(C[i])
        Ub, _, _ = breath_subspace(D, fps, f0)
        Dr = D @ (np.eye(16) - Ub @ Ub.T)
        Dh = np.column_stack([bandpass(Dr[:, a], fps, CARD_LO, CARD_HI) for a in range(16)])
        e = (Dh ** 2).sum(0); z = Dh @ np.sqrt(e / (e.sum() + 1e-12))
        ridges.append(stft_peaks(z, fps) * 60)
        energies.append(float((z ** 2).sum()))
    energies = np.array(energies); eshare = energies / (energies.sum() + 1e-12)

    best = (-1.0, None, 0.0, 0.0)
    for i in range(len(bins)):
        r = ridges[i]
        if len(r) < 4:
            continue
        high_frac = float(np.mean(r > HIGH_HZ * 60))
        med = float(np.median(r))
        mad = float(np.median(np.abs(r - med)))
        stability = max(0.0, 1 - mad / (med + 1e-9))
        evidence = high_frac * stability * np.sqrt(eshare[i])
        if evidence > best[0]:
            best = (evidence, bins[i], med, high_frac)
    _, bsel, med, hf = best
    tachy = (med > HIGH_HZ * 60) and (hf >= HIGH_FRAC_MIN)
    return bsel, med, hf, tachy


CUBES = [("tachy2_cube.npz", "TACHY 110-131"),
         ("tachy1_cube.npz", "TACHY far >110"),
         ("sit39_cube.npz",  "resting ~81"),
         ("lie41_cube.npz",  "resting ~77"),
         ("sidesit_cube.npz","resting ~78"),
         ("fall20_cube.npz", "resting ~80"),
         ("emptyT_cube.npz", "EMPTY (no person)")]

print("answer-independent spatial-null tachy discriminator:\n")
print(f"  {'cube':20s} {'truth':18s} {'bin':>4} {'medbpm':>7} {'hi_frac':>7}  verdict")
for path, truth in CUBES:
    try:
        b, med, hf, tachy = select_and_classify(path, 18.78)
        v = "TACHY" if tachy else "not-tachy"
        print(f"  {path:20s} {truth:18s} {b:>4} {med:>7.0f} {hf:>7.0%}  {v}")
    except Exception as ex:
        print(f"  {path:20s} {truth:18s}  ERROR {ex}")
