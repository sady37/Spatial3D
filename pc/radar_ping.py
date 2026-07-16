"""Passive port ping: read-only byte counts on the CLI + DATA UARTs for ~2 s.
Sends NOTHING — safe to run any time (won't stop/wedge the sensor). Use after a
power-cycle to decide the next step:

    .venv/bin/python3 radar_ping.py

  DATA has bytes  -> firmware auto-streams from flash; just run web/radar_server.py live
  CLI has a banner, DATA silent -> needs a cfg; run radar_start.py
  BOTH 0 bytes    -> demo not running: SOP in config/flashing mode, or bad flash.
                     Set SOP to Functional/flash-boot and FULLY power-cycle.
"""
import serial, time

PORTS = [("/dev/cu.usbmodem0000RA441", 115200, "CLI  RA441"),
         ("/dev/cu.usbmodem0000RA444", 1250000, "DATA RA444")]

for port, baud, tag in PORTS:
    try:
        s = serial.Serial(port, baud, timeout=0.3)
        n = 0; head = b""; t = time.time()
        while time.time() - t < 2.0:
            b = s.read(4096)
            if b:
                n += len(b)
                if len(head) < 32:
                    head += b[:32 - len(head)]
        s.close()
        txt = head.decode("ascii", "replace").replace("\n", " ").replace("\r", " ")
        print(f"{tag}  baud{baud:>8}: {n:6d} bytes/2s   head={head[:16].hex(' ')}"
              + (f'  ascii={txt.strip()!r}' if n else ''))
    except Exception as e:
        print(f"{tag}: OPEN-FAIL {e}")
