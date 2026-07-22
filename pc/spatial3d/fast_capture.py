#!/usr/bin/env python3
"""1-minute cube capture for the 5-pose lying test (2026-07-21).

Same cube primitives as room_capture.py but tuned for a ~1-min hold:
  - 3 s 3001 warmup (grabs any track + resets the cubeGuard budget), THEN cube
  - 3 whole-room cube rounds (bins 1-64), short gaps -> ~50 s total (< 1-min hold)
  - explicit "CATCH SUCCESS" / "CATCH FAILED" final line so each pose is confirmed
    saved BEFORE we ever analyze (user protocol: capture all 5 first, analyze later)

ATTACHES to the already-streaming pose65s firmware -- never resends cfg. Close the
live display/server first so the DATA port is free.

Usage (invoke ~3 s after the subject is in pose):
    python -m spatial3d.fast_capture test1     # test2 | test3 | test4 | test5
"""
import os, sys, time
import numpy as np

PC = "/Users/sady3721/project/owl/Spatial3D/pc"
sys.path.insert(0, PC)
from spatial3d.uart_reader import RadarSession
from spatial3d.range_music import N_VIRT_ANT, BinAccumulator
from spatial3d.cube import save_cube

CLI = "/dev/cu.usbmodem0000RA441"
DATA = "/dev/cu.usbmodem0000RA444"
SHOTS = [(20, 19), (48, 16)]     # bins 1-39 and 32-64 (whole room, == room_capture/empty baseline)
ROUNDS = 3                       # 3x2x20 = 120 cube-frames < cubeGuard 300 budget
NFRAMES = 20
ROUND_GAP_S = 4.0                # short (fits 1-min hold); 3001 warmup already reset budget
WARMUP_S = 3.0                   # user protocol: "3s后开始cube" — also resets cube budget
DR = 0.106                       # pose65s
MOUNT, TILT = 2.0, 25.0

label = sys.argv[1] if len(sys.argv) > 1 else "test"
OUT = os.path.join(PC, "case", f"{label}_20260721.npz")

s = RadarSession(CLI, DATA); s.start_drain()
print(f"[{label}] attaching to live stream (no cfg resend) ...", flush=True)
t0 = time.time(); nf = 0
while time.time() - t0 < 6 and nf < 5:
    if s.get_frame(timeout=1.0) is not None:
        nf += 1
if nf < 1:
    print("CATCH FAILED: NO STREAM. Ensure pose65s is streaming and the DATA port is "
          "free (close the live display/server). NOT resending cfg.", flush=True)
    sys.exit(1)

# ---------------- 3 s warmup: grab any track + reset cube budget ----------------
print(f"[{label}] {WARMUP_S:.0f}s warmup (hold pose) ...", flush=True)
track_log = []; pc_all = []
t0 = time.time()
while time.time() - t0 < WARMUP_S:
    fr = s.get_frame(timeout=1.0)
    if fr is None:
        continue
    for tg in fr.targets():
        track_log.append((tg.tid, tg.x, tg.y, tg.z))
    pc = fr.point_cloud()
    if pc is not None and getattr(pc, "xyz", None) is not None and len(pc.xyz):
        for p in pc.xyz:
            pc_all.append((float(p[0]), float(p[1]), float(p[2])))
person = np.median(np.array([[t[1], t[2], t[3]] for t in track_log], float), axis=0) \
    if track_log else np.array([np.nan, np.nan, np.nan])
print(f"[{label}] warmup: {len(track_log)} track samples, person≈{person}, {len(pc_all)} pc pts", flush=True)

# ---------------- cube sweep ----------------
print(f"[{label}] cube {ROUNDS} rounds, gap {ROUND_GAP_S:.0f}s ...", flush=True)
acc = BinAccumulator(k=100000, n_ant=N_VIRT_ANT)

def _drain():
    n = 0
    while s.get_frame(timeout=0.02) is not None and n < 5000:
        n += 1

t_all = time.time()
for rnd in range(ROUNDS):
    for (cbin, hw) in SHOTS:
        _drain()
        try:
            s.send_cli(f"cubeQuery {cbin} {hw} {NFRAMES}", wait=0.3, echo=False)
        except Exception as e:
            print(f"  send_cli err: {e}", flush=True)
        got = 0; t0 = time.time()
        while time.time() - t0 < NFRAMES / 10.0 + 4.0:
            fr = s.get_frame(timeout=1.0)
            if fr is None:
                continue
            try:
                tbc = fr.track_bin_cube()
            except Exception:
                tbc = None
            if tbc is None:
                continue
            for e in tbc.entries:
                acc.snaps.setdefault(int(e.range_bin), []).append(e.vec)
                got += 1
        nbins = sum(1 for b in acc.snaps if acc.snaps[b])
        print(f"  round {rnd} bin={cbin} hw={hw}: +{got} | bins={nbins}", flush=True)
    if rnd < ROUNDS - 1:
        time.sleep(ROUND_GAP_S)

allbins = sorted(b for b in acc.snaps if len(acc.snaps[b]) >= 8)
if len(allbins) < 20:
    # too few bins = wedged/starved firmware -> DO NOT report success (user must re-capture)
    print(f"CATCH FAILED: only {len(allbins)} cube bins >=8 snaps (expected ~50+). "
          f"Firmware may be wedged — power-cycle + reload pose65s, then retry {label}.", flush=True)
    sys.exit(2)

save_cube(OUT, acc, allbins, dr=DR, min_snapshots=8)
d = dict(np.load(OUT, allow_pickle=True))
d["track_xyz"] = np.array([[t[1], t[2], t[3]] for t in track_log], float)
d["person_xyz"] = person
d["pc_points"] = np.array(pc_all, float)
d["mount_m"] = MOUNT; d["tilt_deg"] = TILT
np.savez(OUT, **d)

binsA = np.array(allbins)
print(f"\nCATCH SUCCESS: {OUT}", flush=True)
print(f"  cube: {len(binsA)} bins {int(binsA.min())}-{int(binsA.max())} "
      f"({binsA.min()*DR:.2f}-{binsA.max()*DR:.2f}m)  |  person_xyz={person}  |  "
      f"{time.time()-t_all:.0f}s cube (+{WARMUP_S:.0f}s warmup)", flush=True)
