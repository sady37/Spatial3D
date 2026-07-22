#!/usr/bin/env python3
"""Two-phase room capture for coordinate calibration + static scene.

Phase 1 (30 s): record the live point cloud + TRACKS. A seated person is a
tracked target -> fr.targets() gives their (x,y,z) in the firmware's WORLD frame
= the chair's ground-truth coordinate (no tape measure needed). For 'empty'
there are simply no tracks.

Phase 2 (cube): the SAME cubeQuery 2-shot whole-room sweep as cube_sweep, 5
rounds, SHORT gaps -> per-bin zero-Doppler covariances (MUSIC-ready, same format
as static_empty). Recording 3001 first (no cube budget spent) gives the cube
phase a fresh cubeGuard budget.

ATTACHES to the already-streaming pose65s firmware -- never resends cfg.

Usage:
    python -m spatial3d.room_capture ChairR      # ChairL | empty
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
SHOTS = [(20, 19), (48, 16)]     # bins 1-39 and 32-64 (whole room)
ROUNDS = 5
NFRAMES = 20                     # 5x2x20 = 200 cube-frames < cubeGuard 300 budget
ROUND_GAP_S = 10.0               # shortened (was 60); phase-1 already reset budget
PC_SECONDS = 30.0                # phase-1 point-cloud/track recording
DR = 0.106                       # pose65s
MOUNT, TILT = 2.0, 25.0

label = sys.argv[1] if len(sys.argv) > 1 else "room"
OUT = os.path.join(PC, "case", f"{label}_20260721.npz")

s = RadarSession(CLI, DATA); s.start_drain()
print("attaching to live stream (no cfg resend) ...", flush=True)
t0 = time.time(); nf = 0
while time.time() - t0 < 6 and nf < 5:
    if s.get_frame(timeout=1.0) is not None:
        nf += 1
if nf < 1:
    print("NO STREAM. Ensure pose65s is streaming and the DATA port is free "
          "(close the live display). NOT resending cfg. Aborting.", flush=True)
    sys.exit(1)

# ---------------- Phase 1: 30 s point cloud + tracks ----------------
print(f"\n[Phase 1] recording {PC_SECONDS:.0f}s of 3001/track -- SIT STILL ...", flush=True)
track_log = []      # (tid, x, y, z)
pc_all = []         # accumulated 3001 Cartesian points (x,y,z,snr)
t0 = time.time(); last = 0
while time.time() - t0 < PC_SECONDS:
    fr = s.get_frame(timeout=1.0)
    if fr is None:
        continue
    for tg in fr.targets():
        track_log.append((tg.tid, tg.x, tg.y, tg.z))
    pc = fr.point_cloud()
    if pc is not None and getattr(pc, "xyz", None) is not None and len(pc.xyz):
        for p in pc.xyz:
            pc_all.append((float(p[0]), float(p[1]), float(p[2])))
    el = int(time.time() - t0)
    if el != last:
        last = el
        if track_log:
            tx = np.median([t[1] for t in track_log]); ty = np.median([t[2] for t in track_log])
            tz = np.median([t[3] for t in track_log])
            print(f"  {el:2d}s  tracks={len(track_log)}  person≈({tx:+.2f},{ty:.2f},{tz:+.2f})", flush=True)
        else:
            print(f"  {el:2d}s  no track yet ({len(pc_all)} pc pts)", flush=True)

if track_log:
    T = np.array([[t[1], t[2], t[3]] for t in track_log], float)
    person = np.median(T, axis=0)
    print(f"[Phase 1] person (median track) = ({person[0]:+.2f}, {person[1]:.2f}, "
          f"{person[2]:+.2f}) m  from {len(track_log)} samples", flush=True)
else:
    person = np.array([np.nan, np.nan, np.nan])
    print(f"[Phase 1] NO tracks (expected for empty room); {len(pc_all)} pc pts", flush=True)

# ---------------- Phase 2: cube sweep ----------------
print(f"\n[Phase 2] cube sweep {ROUNDS} rounds, gap {ROUND_GAP_S:.0f}s ...", flush=True)
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
if not allbins:
    print("NO cube bins >=8 snaps. Saving phase-1 track only.", flush=True)
    np.savez(OUT, track_xyz=np.array([[t[1], t[2], t[3]] for t in track_log], float),
             person_xyz=person, pc_points=np.array(pc_all, float),
             mount_m=MOUNT, tilt_deg=TILT, dr_m=DR)
    sys.exit(0)

save_cube(OUT, acc, allbins, dr=DR, min_snapshots=8)
d = dict(np.load(OUT, allow_pickle=True))
d["track_xyz"] = np.array([[t[1], t[2], t[3]] for t in track_log], float)
d["person_xyz"] = person
d["pc_points"] = np.array(pc_all, float)
d["mount_m"] = MOUNT; d["tilt_deg"] = TILT
np.savez(OUT, **d)

binsA = np.array(allbins)
print(f"\nSAVED {OUT}", flush=True)
print(f"  cube: {len(binsA)} bins {int(binsA.min())}-{int(binsA.max())} "
      f"({binsA.min()*DR:.2f}-{binsA.max()*DR:.2f}m)", flush=True)
print(f"  person_xyz = {person}", flush=True)
print(f"  total {time.time()-t_all:.0f}s (+30s phase1)", flush=True)
