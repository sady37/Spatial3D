"""Synthetic tachycardia validation of the adaptive-ceiling LOGIC.

We cannot fabricate a real post-exercise radar return, so this injects a KNOWN
fast cardiac phase modulation into the REAL sit39 cube (real noise / breathing /
multipath) and checks that:
  * the fixed [1.0,1.7]Hz ceiling clips/aliases the fast beat, while
  * the adaptive [1.0,2.2]Hz arbitration recovers it.

This validates the mechanism only. Real Apple-Watch-referenced validation still
requires a genuine post-exercise capture.
"""
import numpy as np
from hr_continuous import (phase_channels, estimate_window, HR_PHYS_HI)

LAMBDA_MM = 5.0
FPS = 18.78


def inject(cube, bins, body_bins, f_hr, amp_mm, fps):
    """Multiply the body-bin channels by a common-mode phase = a d(t) heartbeat
    displacement at f_hr. phi_add = -4pi/lambda * d(t) (matches demod sign)."""
    out = cube.copy()
    T = cube.shape[1]
    t = np.arange(T) / fps
    d = amp_mm * np.sin(2 * np.pi * f_hr * t)         # mm displacement
    phi_add = -4 * np.pi / LAMBDA_MM * d               # rad
    ph = np.exp(1j * phi_add).astype(np.complex64)
    for b in body_bins:
        idx = np.where(bins == b)[0]
        if len(idx):
            out[idx[0]] = out[idx[0]] * ph[None, :].T
    return out


def main():
    d = np.load("sit39_cube.npz", allow_pickle=True)
    cube = np.asarray(d["snapshots"], dtype=np.complex64)
    bins = d["bins"].astype(int)
    K = int(d["counts"].astype(int).min())
    cube = cube[:, :K, :]

    # body bins = the whole torso range the person occupies (~3.5-3.9m). A real
    # post-exercise beat modulates the whole body coherently, so the majority of
    # quality bins carry the fast rate (that is what the cluster vote checks).
    body_bins = list(range(150, 166))

    print(f"{'true HR':>8} | {'fixed[1.0-1.7]':>16} | {'adaptive[1.0-2.2]':>18} | {'src':>16}")
    print("-" * 70)
    for bpm_true in [78, 108, 120, 132]:
        f_hr = bpm_true / 60.0
        # amplitude ~0.35mm (post-exercise beats are stronger than resting @3.5m)
        cj = inject(cube, bins, body_bins, f_hr, amp_mm=0.35, fps=FPS)
        # single 30s window for a clean read
        W = int(30 * FPS)
        chans = phase_channels(cj[:, :W, :])
        fixed = estimate_window(chans, FPS, tachy=False)
        adapt = estimate_window(chans, FPS, tachy=True)
        print(f"{bpm_true:6d}   | {fixed['hr']:14.1f}   | "
              f"{adapt['hr']:16.1f}   | {adapt['src']:>16}")

    print("\nNote: 78bpm row is below the ceiling -> both should agree (~78) and")
    print("adaptive must NOT promote to the high band (band-prior safety).")


if __name__ == "__main__":
    main()
