"""UART link to the TI AWRL6844 radar (PC side).

Two ports on the EVM (via XDS110):
  - CLI  port @115200 : send the .cfg profile to start sensing
  - DATA port @921600 : receive TLV frames (see tlv.py)

`send_config` streams a .cfg line-by-line (the same file the mmWave demo/Visualizer
uses). `iter_frames` yields parsed TLV frames from the data port.
"""

from __future__ import annotations

import queue
import threading
import time
from typing import Callable, Iterator

from .tlv import Frame, read_frame

CLI_BAUD = 115200
DATA_BAUD = 1250000  # confirmed for xWRL6844 mmw_demo DATA UART (RA444)


def open_serial(port: str, baudrate: int, timeout: float = 1.0,
                write_timeout: float | None = None):
    """Open a pyserial port (imported lazily so the module loads without hardware).

    ``write_timeout=None`` = blocking writes, matching the proven music_clean.py
    template. This matters because ``factoryCalibCfg`` triggers an on-chip
    calibration that can take *minutes*, during which the demo stops draining
    its CLI RX; a blocking write simply waits it out and succeeds. Any finite
    timeout shorter than the calibration spuriously fails the next write (the
    sensor then never starts). The earlier 10-min hang was NOT config — it was
    live ``rangeAntennaOutput`` re-send mid-stream, which is no longer used
    (layer switching is now sensorStop/reconfig/sensorStart).
    """
    import serial

    return serial.Serial(port=port, baudrate=baudrate, timeout=timeout,
                         write_timeout=write_timeout)


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


class RadarSession:
    """Live session that drains DATA in a thread while CLI commands are sent.

    This is the pattern that keeps the mmw demo from hanging: the demo blocks
    on its DATA UART write once a frame is ready, so if the PC stops reading
    DATA while it fiddles with CLI (e.g. to roll range-antenna layers), the
    demo wedges and only an S2 reset recovers it (see music_clean.py). Here a
    background thread reads DATA continuously into a bounded queue, and the
    main thread is free to send CLI reconfig commands at any time.

    Typical use::

        with RadarSession(CLI, DATA) as s:
            s.send_cfg("profile_4T4R_music.cfg")   # includes sensorStart
            s.set_layer(87, 82)                     # roll to a layer, live
            for frame in s.frames(timeout=1.0):
                ...                                 # consume range-antenna TLVs
    """

    def __init__(
        self,
        cli_port: str,
        data_port: str,
        cli_baud: int = CLI_BAUD,
        data_baud: int = DATA_BAUD,
        queue_size: int = 512,
    ) -> None:
        self.cli = open_serial(cli_port, cli_baud, timeout=0.5)
        self.data = open_serial(data_port, data_baud, timeout=1.0)
        self._q: "queue.Queue[Frame]" = queue.Queue(maxsize=queue_size)
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self.frames_read = 0
        self.frames_dropped = 0
        self.cli_errors: list[str] = []

    # -- lifecycle -------------------------------------------------------
    def __enter__(self) -> "RadarSession":
        self.start_drain()
        return self

    def __exit__(self, *exc) -> None:
        self.close()

    def start_drain(self) -> None:
        """Spawn the background DATA-drain thread."""
        if self._thread is not None:
            return
        self._thread = threading.Thread(target=self._drain, daemon=True)
        self._thread.start()

    def _drain(self) -> None:
        while not self._stop.is_set():
            try:
                frame = read_frame(self.data)
            except Exception:
                # Transient parse/timeout; back off briefly so a persistently
                # bad stream does not spin the CPU.
                time.sleep(0.005)
                continue
            self.frames_read += 1
            try:
                self._q.put_nowait(frame)
            except queue.Full:
                # Consumer is behind; drop the oldest so we keep the newest.
                self.frames_dropped += 1
                try:
                    self._q.get_nowait()
                    self._q.put_nowait(frame)
                except (queue.Empty, queue.Full):
                    pass

    # -- CLI -------------------------------------------------------------
    def send_cli(self, line: str, wait: float = 1.5, echo: bool = True) -> str:
        """Send one CLI line and wait for the demo's Done/Error reply.

        Returns the raw reply text. Records CLI errors on self.cli_errors.
        Safe to call while the drain thread is running (separate port).
        """
        self.cli.reset_input_buffer()
        try:
            self.cli.write((line + "\n").encode())
        except Exception as e:
            # write_timeout / wedged demo: record and bail instead of hanging.
            self.cli_errors.append(f"{line} (write failed: {e})")
            if echo:
                print(f"> {line:38s} [WRITE-FAIL]")
            return ""
        t0 = time.time()
        buf = ""
        while time.time() - t0 < wait:
            chunk = self.cli.read(512).decode(errors="replace")
            if chunk:
                buf += chunk
            if "Done" in buf or "Error" in buf:
                break
        if "Error" in buf:
            self.cli_errors.append(line)
            flag = "ERR"
        elif "Done" in buf:
            flag = "OK"
        else:
            flag = "NO-Done"
        if echo:
            print(f"> {line:38s} [{flag}]")
        return buf

    def read_banner(self, wait: float = 0.4) -> str:
        """Drain and return any boot banner sitting on the CLI (e.g. 'MMW Demo').

        Reading it first (as music_clean.py does) syncs to a clean prompt and
        confirms the demo is alive after an S2 reset.
        """
        time.sleep(wait)
        try:
            return self.cli.read(4096).decode(errors="replace")
        except Exception:
            return ""

    def send_cfg(self, cfg_path: str, echo: bool = True,
                 layer: tuple[int, int] | None = None) -> None:
        """Stream a .cfg over CLI, line by line, with the drain thread live.

        Reads the boot banner first. Writes are blocking (see open_serial), so a
        calibration command that makes the demo pause simply blocks the next
        write until it finishes — no timeout tuning needed. Mirrors the proven
        music_clean.py template. ``sensorStart`` gets a longer wait.

        If *layer* = (start_bin, num_bins) is given, the ``rangeAntennaOutput``
        line is rewritten to that window. The firmware only honours
        ``rangeAntennaOutput`` inside a full config parse (an isolated re-send is
        ignored), so switching range windows means resending the whole cfg.
        """
        banner = self.read_banner()
        if echo:
            tag = "MMW" if "mmw" in banner.lower() else "?"
            print(f"  CLI banner [{tag}]: {banner.strip()[:60]!r}")
        with open(cfg_path, "r") as f:
            for raw in f:
                line = raw.strip()
                if not line or line.startswith("%"):
                    continue
                if layer is not None and line.startswith("rangeAntennaOutput"):
                    line = f"rangeAntennaOutput {layer[0]} {layer[1]} 1"
                wait = 5.0 if line.startswith("sensorStart") else 1.5
                self.send_cli(line, wait=wait, echo=echo)

    def set_layer(self, start_bin: int, num_bins: int, wait: float = 1.5,
                  echo: bool = True) -> str:
        """Roll the range-antenna window live (no stop/start).

        WARNING: on the current AWRL6844 firmware a live ``rangeAntennaOutput``
        re-send while streaming WEDGES the demo (requires an S2 reset). Use
        :meth:`restart_layer` instead unless you have verified live reconfig
        works on your firmware.
        """
        return self.send_cli(f"rangeAntennaOutput {start_bin} {num_bins} 1",
                             wait=wait, echo=echo)

    def restart_layer(self, start_bin: int, num_bins: int,
                      echo: bool = True) -> None:
        """Switch the range-antenna window via sensorStop -> reconfig -> sensorStart.

        This is the safe layer-roll mechanism (matches the gaze-mode design in
        AWRL6844.md 5.4): the demo only latches ``rangeAntennaOutput`` at
        sensorStart, and reconfiguring while streaming wedges it. Stale frames
        buffered from the previous layer are flushed after restart.
        """
        self.send_cli("sensorStop 0", wait=2.0, echo=echo)
        self.send_cli(f"rangeAntennaOutput {start_bin} {num_bins} 1",
                      wait=1.5, echo=echo)
        self.send_cli("sensorStart 0 0 0 0", wait=3.0, echo=echo)
        self.flush_frames()

    def flush_frames(self) -> None:
        """Drop any frames currently queued (e.g. stale from the previous layer)."""
        import queue as _q
        while True:
            try:
                self._q.get_nowait()
            except _q.Empty:
                break

    # -- DATA ------------------------------------------------------------
    def frames(self, timeout: float = 1.0) -> Iterator[Frame]:
        """Yield frames from the drain queue until stopped (blocks up to timeout)."""
        while not self._stop.is_set():
            try:
                yield self._q.get(timeout=timeout)
            except queue.Empty:
                continue

    def get_frame(self, timeout: float = 1.0) -> Frame | None:
        """Pop a single frame, or None if none arrived within *timeout*."""
        try:
            return self._q.get(timeout=timeout)
        except queue.Empty:
            return None

    # -- teardown --------------------------------------------------------
    def stop_sensor(self) -> None:
        """Tell the demo to stop streaming (best effort)."""
        try:
            self.cli.reset_input_buffer()
            self.cli.write(b"sensorStop 0\n")
            time.sleep(0.3)
        except Exception:
            pass

    def close(self, stop_sensor: bool = True) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=2.0)
            self._thread = None
        if stop_sensor:
            self.stop_sensor()
        for port in (self.data, self.cli):
            try:
                port.close()
            except Exception:
                pass
