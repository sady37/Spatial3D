#!/usr/bin/env python3
"""Sitting-person capture for coordinate calibration (chairR / chairL ground truth).

Triggered on the user's "go": waits 3 s (settle), then runs the SAME cubeQuery
2-shot sweep as cube_sweep.py (track-independent 320 burst) while a person sits
still (breathing). Per-bin 16-antenna zero-Doppler covariances -> save_cube npz,
SAME format as static_empty_20260721.npz so build_static_scene / differencing
work directly. The breathing person is a strong new reflector at a KNOWN chair
position -> ground truth to calibrate to_ground (H_EFF / spacing / aspect) + X_SIGN.

ATTACHES to the already-streaming pose65s firmware -- never resends cfg (won't
disturb the live display). Aborts if the stream isn't up or the DATA port is
held by another reader.

Usage:
    python -m spatial3d.sit_capture chairR      # or chairL
"""
import os, sys, time

PC = "/Users/sady3721/project/owl/Spatial3D/pc"
sys.path.insert(0, PC)
from spatial3d.uart_reader import RadarSession
from spatial3d.range_music import N_VIRT_ANT, BinAccumulator
from spatial3d.cube import save_cube

CLI = "/dev/cu.usbmodem0000RA441"
DATA = "/dev/cu.usbmodem0000RA444"
SHOTS = [(20, 19), (48, 16)]     # bins 1-39 and 32-64 (whole room)
ROUNDS = 3
NFRAMES = 30                     # ~3 s of frames per shot per round
ROUND_GAP_S = 12.0               # anti-wedge (< cube_sweep's 60s; person capture is short)
SETTLE_S = 3.0                   # user says "go" -> settle before capturing
DR = 0.106                       # pose65s

label = sys.argv[1] if len(sys.argv) > 1 else "chair"
OUT = os.path.join(PC, "case", f"sit_{label}_20260721.npz")

acc = BinAccumulator(k=100000, n_ant=N_VIRT_ANT)
s = RadarSession(CLI, DATA); s.start_drain()

# --- attach-only: confirm the firmware is already streaming (NO cfg resend) ---
print("attaching to live stream (no cfg resend) ...", flush=True)
t0 = time.time(); nf = 0
while time.time() - t0 < 6 and nf < 5:
    if s.get_frame(timeout=1.0) is not None:
        nf += 1
if nf < 1:
    print("NO STREAM (0 frames). Ensure pose65s firmware is streaming and the "
          "DATA port isn't held by another reader (close the live display). "
          "NOT resending cfg. Aborting.", flush=True)
    sys.exit(1)
print(f"stream OK ({nf} frames). Settling {SETTLE_S:.0f}s -- sit still ...", flush=True)
time.sleep(SETTLE_S)
t_all = time.time()


def _drain_queue():
    n = 0
    while s.get_frame(timeout=0.02) is not None and n < 5000:
        n += 1
    return n


for rnd in range(ROUNDS):
    for (cbin, hw) in SHOTS:
        _drain_queue()
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
        print(f"round {rnd} shot bin={cbin} hw={hw}: +{got} | bins={nbins}", flush=True)
    if rnd < ROUNDS - 1:
        print(f"  round {rnd} done ({time.time()-t_all:.0f}s), gap {ROUND_GAP_S:.0f}s ...", flush=True)
        time.sleep(ROUND_GAP_S)

allbins = sorted(b for b in acc.snaps if len(acc.snaps[b]) >= 8)
if not allbins:
    print("NO bins with >=8 snapshots. Check cubeQuery / stream.", flush=True)
    sys.exit(1)
binsA, cube, counts = save_cube(OUT, acc, allbins, dr=DR, min_snapshots=8)
import numpy as np
print(f"\nSAVED {OUT}", flush=True)
print(f"  {len(binsA)} bins {int(binsA.min())}-{int(binsA.max())} "
      f"({binsA.min()*DR:.2f}-{binsA.max()*DR:.2f}m)  "
      f"snaps/bin med={int(np.median(counts))}  total {time.time()-t_all:.0f}s", flush=True)
