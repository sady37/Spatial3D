"""Capture a raw slow-time CUBE (snapshots) for breathing / HR / tremor FFT.

Keeps every frame's per-bin 16-antenna snapshot and runs for DURATION seconds so
FFT has enough slow-time samples. Saves via cube.save_cube ->
'snapshots'+'counts'+'covariances'+'mean'+'dr_m' (a strict superset — MUSIC and
compare_captures read 'covariances', breath_fft reads 'snapshots'). Never drops
the cube like the old cov-only cap_J/cap_K.

    python cap_cube.py evK_cube.npz 120                          # current boot cfg
    # to change fps/window: POWER-CYCLE first, then send new cfg as first boot cfg:
    python cap_cube.py fall20_cube.npz 120 --cfg <20fps.cfg>

NOTE: this stock demo does NOT service CLI while streaming (mid-stream sensorStop
-> [NO-Done], re-sent cfg ignored, verified 2026-07-10). So you CANNOT switch fps
or the range window on a running sensor. The only way is a physical power-cycle,
after which cap_cube's cold path (no live stream detected) sends --cfg as the
first boot config. --reconfig is a best-effort stop+resend that does NOT work
against this demo mid-stream; kept only for a possible future runtime-writable
firmware.
"""
import argparse, os, sys, time
import numpy as np
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from spatial3d.range_music import N_VIRT_ANT, BinAccumulator
from spatial3d.uart_reader import RadarSession
from spatial3d.cube import pack_snapshots, save_cube

CLI = "/dev/cu.usbmodem0000RA441"; DATA = "/dev/cu.usbmodem0000RA444"
CFG_DEFAULT = "/Users/sady3721/project/TI/Tiinstall/profile_music_5fps_fullroom.cfg"

ap = argparse.ArgumentParser()
ap.add_argument("out")
ap.add_argument("dur", type=float, nargs="?", default=55.0, help="capture seconds")
ap.add_argument("--cfg", default=CFG_DEFAULT)
ap.add_argument("--reconfig", action="store_true",
                help="force sensorStop->send cfg->sensorStart (needed to change fps)")
a = ap.parse_args()

s = RadarSession(CLI, DATA); s.start_drain()
live = 0; t0 = time.time()
while time.time() - t0 < 6 and live < 5:
    f = s.get_frame(timeout=1.0)
    if f is not None and f.range_antenna() is not None: live += 1

if a.reconfig or live < 5:
    if live >= 5:                       # streaming -> stop cleanly before reconfig
        print("reconfig: sensorStop + drain 2s", flush=True)
        try: s.send_cli("sensorStop 0", wait=2.0, echo=False)
        except Exception: pass
        t0 = time.time()
        while time.time() - t0 < 2: s.get_frame(timeout=0.3)
    print(f"sending cfg {os.path.basename(a.cfg)} ...", flush=True)
    s.send_cfg(a.cfg, echo=False)
    warm = live >= 5                    # RF already warm if it was streaming
    pre = 30 if warm else 120
    print(f"{'warm' if warm else 'cold'} preheat {pre}s", flush=True)
    t0 = time.time()
    while time.time() - t0 < pre: s.get_frame(timeout=0.5)
else:
    print("attached, settle 15s", flush=True); t0 = time.time()
    while time.time() - t0 < 15: s.get_frame(timeout=0.5)

acc = BinAccumulator(k=100000, n_ant=N_VIRT_ANT); bins = range(87, 271)
print(f"capturing CUBE {a.dur:.0f}s (hold still)...", flush=True); t0 = time.time(); n = 0
while time.time() - t0 < a.dur:
    f = s.get_frame(timeout=1.0)
    if f is None: continue
    ra = f.range_antenna()
    if ra is not None: acc.add(ra); n += 1
s.close()

binsA, cube, counts = pack_snapshots(acc, bins, min_snapshots=20)
if len(binsA) == 0:
    print("NO usable bins captured — check stream/cfg", flush=True); sys.exit(1)
mean = np.stack([cube[i, :int(counts[i])].mean(0)
                 for i in range(len(binsA))]).astype(np.complex64)
save_cube(a.out, acc, bins, min_snapshots=20, mean=mean)
fps_est = n / a.dur
print(f"SAVED {a.out}: {len(binsA)} bins {int(binsA[0])}-{int(binsA[-1])}, "
      f"{n} frames, K/bin~{int(np.median(counts))} (~{fps_est:.1f}fps)  "
      f"keys=snapshots+counts+covariances+mean+dr_m", flush=True)
