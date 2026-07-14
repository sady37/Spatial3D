"""Start the radar ONCE (send the fixed cfg = what TI Visualizer does at boot).
Recording (cap_stream) is READ-ONLY and never sends cfg — so it can't stop/wipe the
sensor. Run this once per power-up (or after the RAM config was cleared); then record.

    .venv/bin/python3 radar_start.py
"""
import os, sys, time
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from spatial3d.uart_reader import RadarSession

CLI = "/dev/cu.usbmodem0000RA441"; DATA = "/dev/cu.usbmodem0000RA444"
CFG = "/Users/sady3721/project/TI/Tiinstall/profile_fall_20fps_gaze.cfg"   # 20fps

s = RadarSession(CLI, DATA); s.start_drain()
print(f"sending cfg once: {os.path.basename(CFG)} ...", flush=True)
s.send_cfg(CFG, echo=False)
time.sleep(1.0)
live = 0; t = time.time()
while time.time() - t < 6:
    f = s.get_frame(timeout=1.0)
    if f is not None and f.range_antenna() is not None: live += 1
s.close()
print(f"{'STREAMING — ready to record (run cap_stream)' if live >= 5 else 'FAILED to start — power-cycle and retry'}"
      f"  ({live} frames/6s)", flush=True)
