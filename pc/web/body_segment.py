"""Body-model segmentation per range bin from a 2-D feature (user's idea):
  R = static reflection  |mean_t(z)|      (rigid strong reflector)
  B = breathing motion   std(bandpass RR) (respiratory surface excursion)
  C = cardiac motion     bandpass-HR energy over floor (precordial pulsation)

Signature (user):
  head    : R high,  B ~0,   C ~0      (rigid, no respiration/cardiac surface motion)
  chest   : R high,  B low,  C high     (heart underneath)
  abdomen : R low,   B high, C low      (soft, largest breathing excursion)
Even back-facing, the head (strong R, zero B) stays distinct from the torso.

Offline test on the recorded case cubes (sensor-free).  python3 body_segment.py
"""
import os, sys
import numpy as np
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from bcg_vitals import bandpass, RR_LO, RR_HI

HRLO, HRHI = 1.0, 1.7


def features(cube, bins, fps):
    """Per-bin (R, B, C). z = coherent slow-time; disp = phase displacement."""
    R, B, C = [], [], []
    for i in range(len(bins)):
        X = cube[i]                                   # (T,16)
        m = X.mean(0); m = m / (np.linalg.norm(m) + 1e-9)
        z = X @ m.conj()                              # (T,) complex
        R.append(np.abs(z.mean()))                    # static reflection
        phi = np.unwrap(np.angle(z))
        disp = phi - phi.mean()                       # rad displacement
        B.append(np.std(bandpass(disp, fps, RR_LO, RR_HI)))
        hb = bandpass(disp, fps, HRLO, HRHI)
        C.append(np.std(hb))
    return np.array(R), np.array(B), np.array(C)


def segment(cube, bins, fps, dr):
    R, B, C = features(cube, bins, fps)
    Rn = R / (R.max() + 1e-12)                        # normalize each feature 0..1
    Bn = B / (B.max() + 1e-12)
    Cn = C / (C.max() + 1e-12)
    labels = []
    for i in range(len(bins)):
        r, b, c = Rn[i], Bn[i], Cn[i]
        if r < 0.15 and b < 0.15:
            labels.append("·")                        # empty/air
        elif b >= 0.45:
            labels.append("腹ABD")                    # strong breathing
        elif c >= 0.40 and r >= 0.30:
            labels.append("胸CHE")                    # strong cardiac + solid reflector
        elif r >= 0.40 and b < 0.25 and c < 0.30:
            labels.append("头HEAD")                   # rigid strong reflector, no motion
        else:
            labels.append("body")
    return Rn, Bn, Cn, labels


def run(path, fps):
    d = np.load(path, allow_pickle=True)
    cube = np.asarray(d["snapshots"], np.complex64)
    counts = d["counts"].astype(int); bins = d["bins"].astype(int)
    dr = float(d["dr_m"]) if "dr_m" in d.files else 0.0234
    cube = cube[:, :int(counts.min()), :]
    Rn, Bn, Cn, lab = segment(cube, bins, fps, dr)
    print(f"\n=== {os.path.basename(path)}  ({len(bins)} bins @ {fps:.1f}fps) ===")
    print(f"{'bin':>4} {'range':>6} | {'R refl':>6} {'B resp':>6} {'C card':>6} | segment")
    for i in range(len(bins)):
        bar = "#" * int(Bn[i] * 10)
        print(f"{int(bins[i]):>4} {int(bins[i])*dr:>5.2f}m | {Rn[i]:>6.2f} {Bn[i]:>6.2f} "
              f"{Cn[i]:>6.2f} | {lab[i]:<6} {bar}")


if __name__ == "__main__":
    HERE = os.path.dirname(os.path.abspath(__file__))
    CASE = os.path.join(os.path.dirname(HERE), "case")
    for name, fps in [("lie41_cube.npz", 18.8), ("chairL_sit_20260713_225001.npz", 18.78),
                      ("sit39_cube.npz", 18.8)]:
        p = os.path.join(CASE, name)
        if os.path.exists(p):
            run(p, fps)
