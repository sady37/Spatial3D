"""Capture the tracker-driven per-bin zero-Doppler cube (TLV 320) from the
People_Tracking + trackBinCube firmware. Bring-up + slow-time-phase capture.

Unlike cap_cube.py (which reads the old range_antenna / type-8 TLV), this reads
TLV 320: for each STILL/fallen track, the zero-Doppler 16-antenna vector at the
track's range bin +- halfWin. Saves a FLAT record layout that regroups offline
by track (server-side MUSIC) or by bin over time (breathing / HR slow-time FFT).

    # probe first (short, prints what TLVs arrive, confirms 320 is emitting):
    python cap_320.py --probe

    # real capture (person sits/lies STILL in front of radar, 0.25-5 m):
    python cap_320.py evK_320.npz 60

If a live stream is already running (e.g. the Visualizer left it streaming) this
ATTACHES without reconfiguring. If nothing is streaming it sends --cfg as the
cold boot config. Same power-cycle caveat as cap_cube.py: this firmware latches
config only at sensorStart, so to change fps/window you must power-cycle first.

npz schema:
  ts        (F,)            wall-clock rx timestamp per captured frame
  n_points  (F,)            detected-point count per frame (TLV point cloud)
  n_ant     scalar          num virtual antennas in each 320 vector (16)
  # flat 320 records (one row per (frame, track, bin) entry):
  e_frame   (E,) int32      index into ts of the frame this entry came from
  e_tid     (E,) int32      track id
  e_bin     (E,) int32      range bin
  e_vel     (E,) int16      track |velocity| mm/s (diagnostic)
  e_range   (E,) float32    range_bin * rangeStep, metres
  e_vec     (E, n_ant) complex64   zero-Doppler antenna vector
"""
import argparse, os, sys, time, collections
import numpy as np
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from spatial3d.uart_reader import RadarSession

CLI = "/dev/cu.usbmodem0000RA441"
DATA = "/dev/cu.usbmodem0000RA444"
CFG_DEFAULT = "/Users/sady3721/project/TI/Tiinstall/sbr_3dpt_5m_trackcube.cfg"

ap = argparse.ArgumentParser()
ap.add_argument("out", nargs="?", default=None, help="output .npz (omit with --probe)")
ap.add_argument("dur", type=float, nargs="?", default=60.0, help="capture seconds")
ap.add_argument("--cfg", default=CFG_DEFAULT)
ap.add_argument("--probe", action="store_true",
                help="15s diagnostic: print TLV histogram + 320 counts, save nothing")
ap.add_argument("--cold", action="store_true",
                help="force full-cfg send (use right after a power-cycle, e.g. to "
                     "apply an edited cfg); default tries a bare sensorStart resume first")
ap.add_argument("--settle", type=float, default=None,
                help="seconds to settle before capturing (default 15 attach / 25 cold)")
a = ap.parse_args()

s = RadarSession(CLI, DATA); s.start_drain()

# Detect a live stream (Visualizer may have left the sensor running).
live = 0; t0 = time.time()
while time.time() - t0 < 6 and live < 5:
    f = s.get_frame(timeout=1.0)
    if f is not None: live += 1
print(f"live frames in 6s: {live}", flush=True)

if live >= 5:
    print("attached to running stream (no reconfig)", flush=True)
    settle = a.settle if a.settle is not None else 15
elif a.cold:
    print(f"--cold -> sending full cfg {os.path.basename(a.cfg)} (needs power-cycle first)",
          flush=True)
    s.send_cfg(a.cfg, echo=True)
    settle = a.settle if a.settle is not None else 25
else:
    # No stream. The demo is usually configured-but-stopped from a prior run
    # (its close() sent sensorStop, but mmwave stays OPEN). Re-sending the FULL
    # cfg here would fail "mmWave open failed" (mmwave already open) and need a
    # power-cycle. So first try a BARE sensorStart to resume the loaded config;
    # only fall back to a cold full-cfg send (use --cold after a power-cycle) if
    # resume yields no frames. NOTE: bare resume reuses the LAST-sent cfg, so to
    # apply an EDITED cfg (e.g. new velThr) you must power-cycle + run with --cold.
    print("no stream -> bare sensorStart (resume loaded cfg) ...", flush=True)
    s.send_cli("sensorStart 0 0 0 0", wait=5.0, echo=True)
    t0 = time.time(); got = 0
    while time.time() - t0 < 4 and got < 3:
        if s.get_frame(timeout=1.0) is not None: got += 1
    if got >= 3:
        print("resumed (was configured-but-stopped)", flush=True)
        settle = a.settle if a.settle is not None else 10
    else:
        print(f"resume failed -> cold full cfg {os.path.basename(a.cfg)} "
              f"(POWER-CYCLE the EVM first if this errors 'mmWave open failed')",
              flush=True)
        s.send_cfg(a.cfg, echo=True)
        settle = a.settle if a.settle is not None else 25
print(f"settle {settle:.0f}s ...", flush=True)
t0 = time.time()
while time.time() - t0 < settle:
    s.get_frame(timeout=0.5)

dur = 15.0 if a.probe else a.dur
print(f"{'PROBE' if a.probe else 'CAPTURE'} {dur:.0f}s (hold STILL)...", flush=True)

tlv_hist = collections.Counter()
ts = []; n_points = []
e_frame = []; e_tid = []; e_bin = []; e_vel = []; e_range = []; e_vec = []
n_ant = 0; nframes = 0
t0 = time.time()
while time.time() - t0 < dur:
    f = s.get_frame(timeout=1.0)
    if f is None: continue
    fi = len(ts)
    ts.append(getattr(f, "rx_ts", time.time()))
    n_points.append(len(f.detected_points()))
    for t in f.tlvs: tlv_hist[t.type] += 1
    tbc = f.track_bin_cube()
    if tbc is not None and tbc.entries:
        n_ant = tbc.num_virt_ant
        for e in tbc.entries:
            e_frame.append(fi); e_tid.append(e.tid); e_bin.append(e.range_bin)
            e_vel.append(e.vel_mmps); e_range.append(e.range_m); e_vec.append(e.vec)
    nframes += 1
# Leave the sensor STREAMING (do NOT sensorStop): this demo cannot re-open mmwave
# without a power-cycle, so keeping it running lets the next run just attach.
s.close(stop_sensor=False)

E = len(e_frame)
fps = nframes / dur if dur else 0
n320_frames = len(set(e_frame))
print(f"\nframes:{nframes} (~{fps:.1f}fps)  points_total:{sum(n_points)}", flush=True)
print(f"TLV types seen (type:count): {dict(sorted(tlv_hist.items()))}", flush=True)
print(f"TLV320: {n320_frames} frames carried it, {E} total entries, n_ant={n_ant}", flush=True)
if E:
    tids = sorted(set(e_tid))
    print(f"  track ids: {tids}", flush=True)
    i = 0
    print(f"  first entry: tid={e_tid[i]} bin={e_bin[i]} vel={e_vel[i]}mm/s "
          f"range={e_range[i]:.3f}m vec[{n_ant}]", flush=True)
else:
    print("  !! NO 320 entries -- need a STILL track in 0.25-5 m "
          "(walking/empty room emits none; check someone is seated in FOV)", flush=True)

if a.probe:
    print("\n(probe: nothing saved)", flush=True)
    sys.exit(0 if E else 2)

if not a.out:
    print("no output path given; not saving", flush=True); sys.exit(1)
np.savez_compressed(
    a.out,
    ts=np.asarray(ts, np.float64),
    n_points=np.asarray(n_points, np.int32),
    n_ant=np.int32(n_ant),
    e_frame=np.asarray(e_frame, np.int32),
    e_tid=np.asarray(e_tid, np.int32),
    e_bin=np.asarray(e_bin, np.int32),
    e_vel=np.asarray(e_vel, np.int16),
    e_range=np.asarray(e_range, np.float32),
    e_vec=(np.stack(e_vec).astype(np.complex64) if E else
           np.empty((0, n_ant), np.complex64)),
    cfg=os.path.basename(a.cfg),
)
print(f"SAVED {a.out}: {nframes} frames, {E} entries, keys=ts/n_points/e_*", flush=True)
