#!/usr/bin/env python3
"""Static empty-room CUBE sweep on the live pose65s (people_tracking 6.5m) firmware.

Uses cubeQuery (TRACK-INDEPENDENT forced 320 burst, confirmed in mmw_cli.c:MmwDemo_CLICubeQuery)
so it works in an EMPTY room with no track. 2 shots cover the room:
  shot1  cubeQuery 20 19 N  -> bins 1-39   (half_win capped at TBC_MAX_ENTRIES/2=19)
  shot2  cubeQuery 48 16 N  -> bins 32-64   (overlap 32-39)
5 rounds, 60 s between rounds (avoid the 320-flood wedge; firmware cubeGuardCfg 300/300/3000 also
caps it). Attaches to the ALREADY-STREAMING firmware -- never resends cfg.

Per-bin 16-antenna zero-Doppler vectors accumulate -> covariances -> save_cube npz (MUSIC-ready).
"""
import os, sys, time
PC = "/Users/sady3721/project/owl/Spatial3D/pc"
sys.path.insert(0, PC)
from spatial3d.uart_reader import RadarSession
from spatial3d.range_music import N_VIRT_ANT, BinAccumulator
from spatial3d.cube import save_cube

CLI = "/dev/cu.usbmodem0000RA441"; DATA = "/dev/cu.usbmodem0000RA444"
OUT = os.path.join(PC, "record", "static_empty_20260721.npz")
SHOTS = [(20, 19), (48, 16)]     # (center_bin, half_win) -> bins 1-39 and 32-64
ROUNDS = 5
NFRAMES = 25                     # per shot per round; 10 queries x 25 = 250 < firmware budget 300
ROUND_GAP_S = 60.0
DR = 0.106                       # pose65s range resolution (128-samp 6.5m cfg)

POSE65S = "/Users/sady3721/project/TI/Tiinstall/sbr_3dpt_6p5m_pose_128.cfg"
acc = BinAccumulator(k=100000, n_ant=N_VIRT_ANT)
s = RadarSession(CLI, DATA); s.start_drain()
# sensor was NOT streaming (0 frames at attach) -> send pose65s cfg (sensorStop->cfg->sensorStart).
print("sending pose65s cfg to START streaming ...", flush=True)
s.send_cfg(POSE65S, echo=False)
# wait for the stream to come up (factoryCalibCfg can take 10-60s)
t0 = time.time(); nf = 0
while time.time() - t0 < 75 and nf < 5:
    if s.get_frame(timeout=1.0) is not None:
        nf += 1
print(f"stream up: {nf} frames seen in {time.time()-t0:.0f}s", flush=True)
if nf < 1:
    print("STREAM DID NOT START -- aborting (check firmware/power-cycle).", flush=True)
    sys.exit(1)
t_all = time.time()

def _drain_queue():
    """Pop stale frames buffered during the sleep so the read loop reaches the fresh burst."""
    n = 0
    while s.get_frame(timeout=0.02) is not None and n < 5000:
        n += 1
    return n

for rnd in range(ROUNDS):
    for (cbin, hw) in SHOTS:
        _drain_queue()                                   # clear stale before arming the burst
        try:
            s.send_cli(f"cubeQuery {cbin} {hw} {NFRAMES}", wait=0.3, echo=False)
        except Exception as e:
            print(f"  send_cli err: {e}", flush=True)
        got = 0; t0 = time.time()
        while time.time() - t0 < NFRAMES / 10.0 + 4.0:      # burst duration + buffer
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
        print(f"round {rnd} shot bin={cbin} hw={hw}: +{got} entries | bins so far={nbins}", flush=True)
    if rnd < ROUNDS - 1:
        print(f"  round {rnd} done ({time.time()-t_all:.0f}s), sleep {ROUND_GAP_S}s (anti-wedge)...", flush=True)
        time.sleep(ROUND_GAP_S)

allbins = sorted(b for b in acc.snaps if len(acc.snaps[b]) >= 8)
if not allbins:
    print("NO bins with >=8 snapshots -- got nothing. Check firmware cubeQuery / stream.", flush=True)
    sys.exit(1)
binsA, cube, counts = save_cube(OUT, acc, allbins, dr=DR, min_snapshots=8)
import numpy as np
print(f"\nSAVED {OUT}", flush=True)
print(f"  {len(binsA)} bins {int(binsA.min())}-{int(binsA.max())} "
      f"(range {binsA.min()*DR:.2f}-{binsA.max()*DR:.2f}m)", flush=True)
print(f"  snaps/bin min={int(counts.min())} med={int(np.median(counts))} max={int(counts.max())}", flush=True)
print(f"  total {time.time()-t_all:.0f}s", flush=True)
