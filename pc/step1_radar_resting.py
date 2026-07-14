"""STEP 1: port current algorithm to RADAR, validate RESTING RR + HR.
Before attacking the dynamic RR-harmonic entanglement (tachy2), confirm the
freq/envelope mainline works on the radar's easy (static, non-resonant) case.

Truths (bpm) from pc/tachy2_excess_track.py: sit33~82, sit39~81, lie41~77, fall20~80.
Uses bcg_vitals: demod_channels -> occupancy -> estimate_rr -> estimate_hr (validated).

    python3 step1_radar_resting.py
"""
import numpy as np
from bcg_vitals import demod_channels, estimate_rr, estimate_hr, occupancy

FPS = 18.78
CUBES = [("sit33_cube.npz", 82), ("sit39_cube.npz", 81),
         ("lie41_cube.npz", 77), ("fall20_cube.npz", 80),
         ("sidesit_cube.npz", None)]
EMPTY = [("emptyL_cube.npz", None), ("base_cube.npz", None)]


def run(path):
    d = np.load(path, allow_pickle=True)
    cube = np.asarray(d["snapshots"], np.complex64)
    counts = d["counts"].astype(int); bins = d["bins"].astype(int)
    C = cube[:, :int(counts.min()), :]
    chans = demod_channels(C, bins)
    occ = occupancy(chans, FPS)
    rr, f0, spread, _ = estimate_rr(chans, FPS)
    hr = estimate_hr(chans, FPS, f0)
    return occ, rr, hr, C.shape


print("STEP 1 — radar resting RR/HR (current algorithm)\n")
print("%-18s %-8s | RR   | HR(bpm) band  strength | truth  err" % ("cube", "present"))
for path, truth in CUBES:
    try:
        occ, rr, hr, shp = run(path)
    except FileNotFoundError:
        print("  %-18s (missing)" % path); continue
    err = "" if truth is None or hr["hr"] is None else "%+.0f" % (hr["hr"] - truth)
    print("  %-16s pres=%-3s | %4s | %5s  %-4s  s=%.2f | %-5s  %s" % (
        path.replace("_cube.npz", ""), "Y" if occ["present"] else "N",
        "%.0f" % rr if rr else "--",
        "%.0f" % hr["hr"] if hr["hr"] else "--", hr["band"], hr["strength"],
        truth if truth else "--", err))

print("\nempty-room null (should read present=N):")
for path, _ in EMPTY:
    try:
        occ, rr, hr, _ = run(path)
        print("  %-16s present=%-3s disp_rms=%.3f  (rr=%s hr=%s)" % (
            path.replace("_cube.npz", ""), "Y" if occ["present"] else "N",
            occ["disp_rms"], "%.0f" % rr if rr else "--",
            "%.0f" % hr["hr"] if hr["hr"] else "--"))
    except FileNotFoundError:
        print("  %-16s (missing)" % path)
