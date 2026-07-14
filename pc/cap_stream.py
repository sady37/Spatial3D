"""Continuous radar capture, DECOUPLED from analysis.

Warms/attaches ONCE, then streams forever, rotating to a new cube file every
--block seconds (default 600 = 10 min), timestamped. Never misses a window:
record continuously, then pick the file(s) covering the time of interest (align
to your watch-HR timeline) and analyze offline. Each file is the same format as
cap_cube (snapshots+counts+covariances+mean+dr_m), so every analysis tool reads it.

    python3 cap_stream.py chairL              # 10-min rolling files, current boot cfg
    python3 cap_stream.py chairL --block 300  # 5-min files
    python3 cap_stream.py chairL --cfg <18fps.cfg>   # cold start -> sends cfg + preheat once
Stop with Ctrl-C (flushes the in-progress block).

WHY 10-min files: HR needs ~20-30s windows; a 10-min file holds plenty and the
analysis slides/concatenates within/across files. Filename = <prefix>_<YYYYmmdd_HHMMSS>.npz
where the timestamp is the block START (local time) — align to watch HR by clock.
"""
import argparse, os, sys, time
import numpy as np
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from spatial3d.range_music import N_VIRT_ANT, BinAccumulator
from spatial3d.uart_reader import RadarSession
from spatial3d.cube import pack_snapshots, save_cube

CLI = "/dev/cu.usbmodem0000RA441"; DATA = "/dev/cu.usbmodem0000RA444"

ap = argparse.ArgumentParser()
ap.add_argument("prefix")
ap.add_argument("--block", type=float, default=600.0, help="seconds per file (default 600=10min)")
a = ap.parse_args()

# ---- READ-ONLY attach: never send cfg/sensorStop (cfg is fixed & lives in the device).
#      Recording only READS the DATA stream, so it can never stop or wipe the sensor.
#      If not streaming, start the sensor ONCE with radar_start.py — don't touch it here.
s = RadarSession(CLI, DATA); s.start_drain()
live = 0; t0 = time.time()
while time.time() - t0 < 6 and live < 5:
    f = s.get_frame(timeout=1.0)
    if f is not None and f.range_antenna() is not None: live += 1

if live < 5:
    print("NOT streaming (sensor stopped/de-configured). cap_stream is READ-ONLY — it will NOT "
          "send cfg.\nStart the sensor once:  .venv/bin/python3 radar_start.py   then re-run.",
          flush=True)
    s.close(); sys.exit(1)
print("attached (streaming) — READ-ONLY capture, no cfg/command sent", flush=True)

# ---- rolling blocks forever ----
bins = range(60, 271)
block = 0
print(f"STREAMING — rotating every {a.block:.0f}s. Ctrl-C flushes the current (partial) block.", flush=True)


def flush(acc, frame_ts, t0, stamp, out):
    """Save whatever is captured so far (full OR partial). Returns True if saved."""
    binsA, cube, counts = pack_snapshots(acc, bins, min_snapshots=20)
    if len(binsA) == 0:
        print(f"[{stamp}] no usable bins (0 frames) — nothing saved", flush=True)
        return False
    mean = np.stack([cube[i, :int(counts[i])].mean(0) for i in range(len(binsA))]).astype(np.complex64)
    save_cube(out, acc, bins, min_snapshots=20, mean=mean,
              frame_ts=np.array(frame_ts, dtype=np.float64), block_start_epoch=np.float64(t0))
    dur = (frame_ts[-1] - frame_ts[0]) if len(frame_ts) > 1 else 0.0
    aligned = "OK" if int(np.median(counts)) == len(frame_ts) else "WARN counts!=frames"
    print(f"[{stamp}] SAVED {out}: {len(binsA)} bins, {len(frame_ts)} frames, {dur:.0f}s "
          f"(~{len(frame_ts) / max(dur, 1e-9):.1f}fps), t0={t0:.3f} ts-align={aligned}", flush=True)
    return True


try:
    while True:
        acc = BinAccumulator(k=200000, n_ant=N_VIRT_ANT)
        stamp = time.strftime("%Y%m%d_%H%M%S")
        out = f"{a.prefix}_{stamp}.npz"
        t0 = time.time(); frame_ts = []
        try:
            while time.time() - t0 < a.block:
                f = s.get_frame(timeout=1.0)
                if f is None: continue
                ra = f.range_antenna()
                if ra is not None:
                    acc.add(ra)
                    frame_ts.append(getattr(f, "rx_ts", time.time()))   # ms wall-clock of this snapshot
        except KeyboardInterrupt:
            print(f"\n[{stamp}] Ctrl-C — flushing current partial block ...", flush=True)
            flush(acc, frame_ts, t0, stamp, out)
            break
        if flush(acc, frame_ts, t0, stamp, out):
            block += 1
finally:
    s.close()
