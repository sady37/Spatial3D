"""Real-time People_Tracking monitor — attach READ-ONLY to the streaming sensor
and print a refreshing status line so you can watch, live, whether a target is
detected / a track forms (TLV 308) / the track-bin cube fires (TLV 320) while you
move around in front of the radar. No need for blind 15s probes.

    python live_monitor.py

Does NOT reconfigure or stop the sensor (attaches to the already-running stream;
run cap_320.py --probe --cold once after a power-cycle to get it streaming first).
Ctrl-C to quit — the sensor keeps running.
"""
import os, sys, time, collections
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from spatial3d.uart_reader import RadarSession
from spatial3d.tlv import (TLV_DETECTED_POINTS, TLV_TRACK_BIN_CUBE)

CLI = "/dev/cu.usbmodem0000RA441"
DATA = "/dev/cu.usbmodem0000RA444"
TLV_TARGET_LIST = 308        # tracker output present => at least one track
TLV_TARGET_INDEX = 309
TLV_POINT_CLOUD = 3001
TLV_STATS = 6

UPDATE_S = 0.5

s = RadarSession(CLI, DATA)
s.start_drain()
print("attached read-only; move in front of the radar (0.25-5 m). Ctrl-C to quit.\n",
      flush=True)

t_start = time.time()
last = time.time()
nfr = 0
pts1 = 0            # type-1 detected points this window
seen3001 = False
track_frames = 0   # frames carrying TLV 308 this window
tbc_entries = 0    # TLV 320 entries this window
tbc_detail = ""
ever_track = False
ever_320 = False

try:
    while True:
        f = s.get_frame(timeout=1.0)
        now = time.time()
        if f is not None:
            nfr += 1
            types = {t.type for t in f.tlvs}
            pts1 += len(f.detected_points())
            if TLV_POINT_CLOUD in types:
                seen3001 = True
            if TLV_TARGET_LIST in types:
                track_frames += 1
                ever_track = True
            tbc = f.track_bin_cube()
            if tbc is not None and tbc.entries:
                tbc_entries += len(tbc.entries)
                ever_320 = True
                e = tbc.entries[0]
                tbc_detail = (f"tid{e.tid} bin{e.range_bin} "
                              f"{e.range_m:.2f}m vel{e.vel_mmps}")

        if now - last >= UPDATE_S:
            dt = now - last
            fps = nfr / dt if dt else 0
            trk = "YES" if track_frames else " no"
            c320 = f"{tbc_entries:2d} {tbc_detail}" if tbc_entries else " 0"
            line = (f"[{now - t_start:5.1f}s] fps{fps:4.1f} | "
                    f"pts(t1){pts1:3d} 3001:{'Y' if seen3001 else '-'} | "
                    f"TRACK(308):{trk} | 320:{c320}")
            print("\r" + line.ljust(96), end="", flush=True)
            last = now
            nfr = 0; pts1 = 0; seen3001 = False; track_frames = 0
            tbc_entries = 0; tbc_detail = ""
except KeyboardInterrupt:
    pass
finally:
    s.close(stop_sensor=False)   # READ-ONLY: leave the sensor streaming
    tag = []
    tag.append("track SEEN" if ever_track else "NO track ever")
    tag.append("320 SEEN" if ever_320 else "NO 320 ever")
    print(f"\n\nsession: {' | '.join(tag)}", flush=True)
