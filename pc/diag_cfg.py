"""Diagnostic: send a .cfg line-by-line and print the RAW device reply for each,
so we can see the actual error text (not just OK/ERR/NO-Done) and detect any
reply desync (reply shifted by one command). Does NOT stop the sensor on exit.

    python diag_cfg.py                                   # default trackcube cfg
    python diag_cfg.py /path/to/other.cfg
"""
import sys, os, time
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from spatial3d.uart_reader import RadarSession

CLI = "/dev/cu.usbmodem0000RA441"
DATA = "/dev/cu.usbmodem0000RA444"
CFG = sys.argv[1] if len(sys.argv) > 1 else \
    "/Users/sady3721/project/TI/Tiinstall/sbr_3dpt_5m_trackcube.cfg"

s = RadarSession(CLI, DATA)
s.start_drain()
print("banner:", repr(s.read_banner(0.8)), flush=True)

with open(CFG) as f:
    for raw in f:
        line = raw.strip()
        if not line or line.startswith("%"):
            continue
        wait = 8.0 if line.startswith(("sensorStart", "factoryCalibCfg")) else 2.5
        buf = s.send_cli(line, wait=wait, echo=False)
        flag = "ERR" if "Error" in buf else ("OK" if "Done" in buf else "NO-Done")
        short = line[:38]
        print(f"> {short:38s} [{flag}]  raw={buf!r}", flush=True)
        time.sleep(0.08)

s.close(stop_sensor=False)   # leave sensor as-is
print("done (sensor left running if it started)", flush=True)
