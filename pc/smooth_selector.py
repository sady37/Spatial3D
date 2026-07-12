"""Fix the continuity metric: SMOOTH TRAJECTORY (not held-rate), non-overlapping frames.

continuity_selector.py penalized tachy because tachy2's true HR SWEEPS 131->91 -- a
"held-constant" metric prefers resting-steady bins. Correct notion: a real HR ridge
(steady OR sweeping) is a SMOOTH trajectory = low 2nd-difference; erratic noise is not.
Use non-overlapping 6s frames so jumps are real.

Decisive evidence: for tachy2, RANK all 44 bins by this smoothness score and show
where the oracle bin 65 (the only bin that tracked the true descent) lands. If bin 65
does NOT rank near the top, NO intrinsic selector finds it -> selection is exhausted.

    python smooth_selector.py
"""
import numpy as np
from bcg_vitals import (bandpass, sqi, fft_peak, demod_channels, RR_LO, RR_HI)
from spatial_null_probe import per_antenna_disp, breath_subspace
from tachy_existence_probe import CARD_LO, CARD_HI

HIGH_BPM = 102.0


def stft_peaks_ns(sig, fps, win_s=6, step_s=6):
    """NON-overlapping STFT ridge (bpm per frame)."""
    n = int(win_s * fps); step = int(step_s * fps)
    out = []
    for s in range(0, len(sig) - n + 1, step):
        seg = sig[s:s + n]
        f = np.fft.rfftfreq(n, 1 / fps)
        S = np.abs(np.fft.rfft(seg - seg.mean())) ** 2
        m = (f >= CARD_LO) & (f <= CARD_HI)
        out.append(f[m][np.argmax(S[m])] * 60)
    return np.array(out)


def bin_scores(path, fps):
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

    energies, ridges = [], []
    for i in range(len(bins)):
        D = per_antenna_disp(C[i])
        Ub, _, _ = breath_subspace(D, fps, f0)
        Dr = D @ (np.eye(16) - Ub @ Ub.T)
        Dh = np.column_stack([bandpass(Dr[:, a], fps, CARD_LO, CARD_HI) for a in range(16)])
        e = (Dh ** 2).sum(0); z = Dh @ np.sqrt(e / (e.sum() + 1e-12))
        ridges.append(stft_peaks_ns(z, fps))
        energies.append(float((z ** 2).sum()))
    energies = np.array(energies); eshare = energies / (energies.sum() + 1e-12)

    rows = []
    for i in range(len(bins)):
        r = ridges[i]
        if len(r) < 4:
            rows.append((bins[i], -1, 0, 99)); continue
        d2 = np.diff(r, 2)                                   # 2nd difference (bpm)
        smooth = 1.0 / (1.0 + np.std(d2) / 10.0)            # low 2nd-diff = smooth
        score = smooth * np.sqrt(eshare[i])
        rows.append((bins[i], score, float(np.median(r)), float(np.std(d2))))
    return rows


print("smooth-trajectory selector (non-overlap 6s frames):\n")
CUBES = [("tachy2_cube.npz", "TACHY 110-131"), ("tachy1_cube.npz", "TACHY far"),
         ("sit39_cube.npz", "resting 81"), ("lie41_cube.npz", "resting 77"),
         ("sidesit_cube.npz", "resting 78"), ("fall20_cube.npz", "resting 80"),
         ("emptyT_cube.npz", "EMPTY")]
for path, truth in CUBES:
    rows = bin_scores(path, 18.78)
    rows_s = sorted(rows, key=lambda t: -t[1])
    b, sc, med, sd = rows_s[0]
    v = "TACHY" if med > HIGH_BPM else "resting"
    line = f"[{truth:14s}] sel bin {b:>3} med={med:>5.0f}bpm 2nd-diff-std={sd:>4.0f} -> {v}"
    if path == "tachy2_cube.npz":
        rank = [i for i, r in enumerate(rows_s) if r[0] == 65][0] + 1
        b65 = [r for r in rows if r[0] == 65][0]
        line += f"   || oracle bin65 rank {rank}/44, med={b65[2]:.0f}bpm"
    print(line)
