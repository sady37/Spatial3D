"""Capture a raw slow-time CUBE (snapshots) for breathing-FFT / RR.

Unlike cap_K.py (cov/mean only, k=100=20s), this keeps every frame's per-bin
16-antenna snapshot and runs for DURATION seconds so breath_fft has enough
slow-time samples (RR wants ~40s @5fps). Saves via cube.save_cube ->
'snapshots'+'counts'+'covariances', so breath_fft.py and compare_captures.py
both read it.

    python cap_cube.py evK_cube.npz 55      # out-file, seconds
"""
import sys, os, time, numpy as np
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from spatial3d.range_music import N_VIRT_ANT, BinAccumulator
from spatial3d.uart_reader import RadarSession
from spatial3d.cube import pack_snapshots, save_cube

OUT = sys.argv[1] if len(sys.argv) > 1 else "evK_cube.npz"
DUR = float(sys.argv[2]) if len(sys.argv) > 2 else 55.0
CLI = "/dev/cu.usbmodem0000RA441"; DATA = "/dev/cu.usbmodem0000RA444"
CFG = "/Users/sady3721/project/TI/Tiinstall/profile_music_5fps_fullroom.cfg"

s = RadarSession(CLI, DATA); s.start_drain()
live = 0; t0 = time.time()
while time.time() - t0 < 6 and live < 5:
    f = s.get_frame(timeout=1.0)
    if f is not None and f.range_antenna() is not None: live += 1
if live >= 5:
    print("attached, settle 15s", flush=True); t0 = time.time()
    while time.time() - t0 < 15: s.get_frame(timeout=0.5)
else:
    print("cfg+preheat 120s", flush=True); s.send_cfg(CFG, echo=False); t0 = time.time()
    while time.time() - t0 < 120: s.get_frame(timeout=0.5)

acc = BinAccumulator(k=100000, n_ant=N_VIRT_ANT); bins = range(87, 271)
print(f"capturing CUBE {DUR:.0f}s (lie still)...", flush=True); t0 = time.time(); n = 0
while time.time() - t0 < DUR:
    f = s.get_frame(timeout=1.0)
    if f is None: continue
    ra = f.range_antenna()
    if ra is not None: acc.add(ra); n += 1
s.close()
# Full superset save: cube(snapshots)+counts+covariances+dr_m via save_cube,
# PLUS mean (per kept bin) so cov/mean consumers keep working. Nothing discarded.
binsA, cube, counts = pack_snapshots(acc, bins, min_snapshots=20)
mean = np.stack([cube[i, :int(counts[i])].mean(0)
                 for i in range(len(binsA))]).astype(np.complex64)
save_cube(OUT, acc, bins, min_snapshots=20, mean=mean)
print(f"SAVED {OUT}: {len(binsA)} bins, {n} frames, K/bin~{int(np.median(counts))} "
      f"({n/5.0:.0f}s @5fps)  keys=snapshots+counts+covariances+mean+dr_m", flush=True)
