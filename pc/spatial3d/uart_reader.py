"""UART voxel-map synchronization (PC side).

The DSP periodically emits the full 9000-voxel map framed as:

    magic (4B) 'VXL0' | length (uint32 LE) | payload (9000 * 8B) | crc32 (uint32 LE)

`read_grid` blocks until one complete, CRC-valid frame is received and returns
a VoxelGrid. `open_serial` is a thin wrapper so callers/tests can inject a fake
stream (any object with .read(n) -> bytes).
"""

from __future__ import annotations

import zlib
from typing import Protocol

from .voxel import VOXEL_COUNT, WIRE_SIZE, VoxelGrid

MAGIC = b"VXL0"
PAYLOAD_SIZE = VOXEL_COUNT * WIRE_SIZE  # 72000


class ByteStream(Protocol):
    def read(self, n: int) -> bytes: ...


def open_serial(port: str, baudrate: int = 921600, timeout: float = 5.0):
    """Open a pyserial port. Imported lazily so the module loads without hardware."""
    import serial  # pyserial

    return serial.Serial(port=port, baudrate=baudrate, timeout=timeout)


def _read_exact(stream: ByteStream, n: int) -> bytes:
    buf = bytearray()
    while len(buf) < n:
        chunk = stream.read(n - len(buf))
        if not chunk:
            raise EOFError("stream closed before frame completed")
        buf.extend(chunk)
    return bytes(buf)


def frame(grid: VoxelGrid) -> bytes:
    """Encode a grid into a wire frame (used by tests / the DSP simulator)."""
    payload = grid.to_bytes()
    header = MAGIC + len(payload).to_bytes(4, "little")
    crc = zlib.crc32(payload).to_bytes(4, "little")
    return header + payload + crc


def read_grid(stream: ByteStream) -> VoxelGrid:
    """Read one framed voxel map from `stream`. Resyncs on magic mismatch."""
    # Sync to magic.
    window = _read_exact(stream, len(MAGIC))
    while window != MAGIC:
        window = window[1:] + _read_exact(stream, 1)

    length = int.from_bytes(_read_exact(stream, 4), "little")
    if length != PAYLOAD_SIZE:
        raise ValueError(f"unexpected payload length {length}, want {PAYLOAD_SIZE}")

    payload = _read_exact(stream, length)
    crc = int.from_bytes(_read_exact(stream, 4), "little")
    if zlib.crc32(payload) != crc:
        raise ValueError("CRC mismatch on voxel frame")

    return VoxelGrid.from_bytes(payload)
