"""Start the radar ONCE (send the fixed cfg = what TI Visualizer does at boot).
The web server (web/radar_server.py live) is READ-ONLY and never sends cfg — so it
can't stop/wipe the sensor. Run this once per power-up (or after the RAM config was
cleared); then start the display module / recorder.

    .venv/bin/python3 radar_start.py                 # default: trackcube (vitals+scene)
    .venv/bin/python3 radar_start.py fall            # fall state-machine cfg
    .venv/bin/python3 radar_start.py 10m             # 10m tracking, no TLV 320
    .venv/bin/python3 radar_start.py /abs/path.cfg   # any explicit cfg

Cfgs live in TI/Tiinstall/ and match the people_tracking_6844_FALLSM firmware
(they carry `trackBinCubeCfg` -> TLV 320). profile_fall_20fps_gaze.cfg was the OLD
vitals/gaze firmware — do NOT use it with FALLSM.
"""
import os, sys, time
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from spatial3d.uart_reader import RadarSession

CLI = "/dev/cu.usbmodem0000RA441"; DATA = "/dev/cu.usbmodem0000RA444"
CFG_DIR = "/Users/sady3721/project/TI/Tiinstall"
# shortname -> cfg file. trackcube (enable 2) fires TLV 320 on EVERY still track so
# breathing/RR/HR populate continuously = the display bring-up default. fall (enable 1)
# only bursts 320 on fall candidates -> vitals blank while seated; use it to test falls.
CFGS = {
    "pose":      "sbr_3dpt_5m_pose.cfg",         # Phase2+3 firmware: MLP + window fall legs (TLV 321)
    "trackcube": "sbr_3dpt_5m_trackcube.cfg",   # scene + vitals always on
    "fall":      "sbr_3dpt_5m_fall.cfg",         # fall ARM/CONFIRM/BURST state machine
    "5m":        "sbr_3dpt_5m.cfg",              # tracking only (no 320 -> no vitals)
    "10m":       "sbr_3dpt_10m.cfg",             # 10m range, tracking only
}
# Default is `pose` now that the people_tracking_6844_POSE firmware is the current
# flash: it enables both per-track fall legs; cube-RR for the red-Fall second-check
# still comes from the server's on-demand `cubeQuery` (Phase 1, in the image).
arg = sys.argv[1] if len(sys.argv) > 1 else "pose"
CFG = arg if os.path.sep in arg else os.path.join(CFG_DIR, CFGS.get(arg, arg))
if not os.path.exists(CFG):
    sys.exit(f"cfg not found: {CFG}\n  known shortnames: {', '.join(CFGS)}")

s = RadarSession(CLI, DATA); s.start_drain()
print(f"sending cfg once: {os.path.basename(CFG)} ...", flush=True)
print("  (echo on: each CLI line -> [OK]/[ERR]/[NO-Done]. A long silence right after\n"
      "   factoryCalibCfg is NORMAL — on-chip calibration; wait ~10-60s.)", flush=True)
s.send_cfg(CFG, echo=True)              # echo so a stall is visible, not silent
time.sleep(1.0)
# People_Tracking/FALLSM firmware does NOT emit range_antenna — count ANY parsed frame
# off the DATA port as proof the stream is alive (old range_antenna check false-failed).
live = 0; t = time.time()
while time.time() - t < 6:
    f = s.get_frame(timeout=1.0)
    if f is not None: live += 1
s.close(stop_sensor=False)   # LEAVE the sensor streaming for the read-only display; default close() sends sensorStop and would wedge web/radar_server.py with 0 frames
print(f"{'STREAMING — ready (start web/radar_server.py live)' if live >= 5 else 'FAILED to start — power-cycle and retry'}"
      f"  ({live} frames/6s)", flush=True)
