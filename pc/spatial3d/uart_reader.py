"""UART link to the TI AWRL6844 radar (PC side).

Two ports on the EVM (via XDS110):
  - CLI  port @115200 : send the .cfg profile to start sensing
  - DATA port @921600 : receive TLV frames (see tlv.py)

`send_config` streams a .cfg line-by-line (the same file the mmWave demo/Visualizer
uses). `iter_frames` yields parsed TLV frames from the data port.
"""

from __future__ import annotations

import time
from typing import Iterator

from .tlv import Frame, read_frame

CLI_BAUD = 115200
DATA_BAUD = 921600


def open_serial(port: str, baudrate: int, timeout: float = 1.0):
    """Open a pyserial port (imported lazily so the module loads without hardware)."""
    import serial

    return serial.Serial(port=port, baudrate=baudrate, timeout=timeout)


def send_config(cli_port: str, cfg_path: str, echo: bool = True) -> None:
    """Send a radar .cfg to the CLI UART, one line at a time.

    Blank lines and `%` comments are skipped. sensorStop/flushCfg at the top of
    most profiles reset the sensor, so re-sending a cfg is safe.
    """
    ser = open_serial(cli_port, CLI_BAUD, timeout=1.0)
    try:
        with open(cfg_path, "r") as f:
            for raw in f:
                line = raw.strip()
                if not line or line.startswith("%"):
                    continue
                ser.write((line + "\n").encode())
                time.sleep(0.02)
                resp = ser.read(256).decode(errors="replace")
                if echo:
                    print(f"> {line}\n{resp.strip()}")
    finally:
        ser.close()


def iter_frames(data_port: str, baudrate: int = DATA_BAUD) -> Iterator[Frame]:
    """Yield parsed TLV frames from the data port forever (until stream ends)."""
    ser = open_serial(data_port, baudrate, timeout=2.0)
    try:
        while True:
            yield read_frame(ser)
    finally:
        ser.close()
