"""TI mmWave TLV output parser (PC side).

Parses the UART output stream produced by a TI mmWave demo running on the
AWRL6844. The frame layout below is the *standard* TI mmWave demo format; the
exact TLV type IDs and point struct can vary between demos, so verify against
the L-SDK demo's `<demo>_output.h` once the radar is streaming (use the `dump`
tool to capture real bytes).

Frame layout (little-endian):
    magic[8]        = 02 01 04 03 06 05 08 07
    version         uint32
    totalPacketLen  uint32   (whole frame incl. this header)
    platform        uint32
    frameNumber     uint32
    timeCpuCycles   uint32
    numDetectedObj  uint32
    numTLVs         uint32
    subFrameNumber  uint32
    -- then numTLVs of: [type uint32][length uint32][payload length bytes]
"""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

MAGIC = bytes([0x02, 0x01, 0x04, 0x03, 0x06, 0x05, 0x08, 0x07])

import struct

_HDR = struct.Struct("<8s8I")   # magic + 8 uint32
HEADER_SIZE = _HDR.size          # 40
_TLV_HDR = struct.Struct("<2I")  # type, length

# Common TLV type IDs (standard mmWave demo). Confirm against L-SDK.
TLV_DETECTED_POINTS = 1
TLV_RANGE_PROFILE = 2
TLV_NOISE_PROFILE = 3
TLV_SIDE_INFO = 7

# Custom TLV type for per-antenna complex data (requires firmware mod)
# Format per detection: 16 complex float32 pairs (real, imag) = 128 bytes
# Total payload: N_detections * 128 bytes
TLV_ANTENNA_COMPLEX = 8  # custom type ID, verify against firmware

_COMPLEX_PER_DET = 16  # 4TX * 4RX virtual antennas
_COMPLEX_BYTES_PER_DET = _COMPLEX_PER_DET * 8  # 16 * (float32_real + float32_imag) = 128 bytes

# Detected-point struct: x, y, z, doppler  (float32 each) = 16 bytes.
_POINT = struct.Struct("<4f")


@dataclass
class FrameHeader:
    version: int
    total_packet_len: int
    platform: int
    frame_number: int
    time_cpu_cycles: int
    num_detected_obj: int
    num_tlvs: int
    sub_frame_number: int


@dataclass
class Tlv:
    type: int
    payload: bytes


@dataclass
class Frame:
    header: FrameHeader
    tlvs: list[Tlv] = field(default_factory=list)

    def detected_points(self) -> np.ndarray:
        """(N, 4) array [x, y, z, doppler] in meters / m/s. Empty if none."""
        for t in self.tlvs:
            if t.type == TLV_DETECTED_POINTS:
                return parse_detected_points(t.payload)
        return np.empty((0, 4), dtype=np.float32)

    def antenna_complex(self) -> np.ndarray | None:
        """Per-antenna complex data. Shape (N_detections, 16) complex64. None if TLV not present."""
        for t in self.tlvs:
            if t.type == TLV_ANTENNA_COMPLEX:
                return parse_antenna_complex(t.payload)
        return None


def parse_detected_points(payload: bytes) -> np.ndarray:
    n = len(payload) // _POINT.size
    if n == 0:
        return np.empty((0, 4), dtype=np.float32)
    arr = np.frombuffer(payload[: n * _POINT.size], dtype="<f4")
    return arr.reshape(n, 4).copy()


def parse_antenna_complex(payload: bytes) -> np.ndarray:
    """Parse complex antenna data TLV. Returns (N, 16) complex64 array."""
    n = len(payload) // _COMPLEX_BYTES_PER_DET
    if n == 0:
        return np.empty((0, _COMPLEX_PER_DET), dtype=np.complex64)
    raw = np.frombuffer(payload[:n * _COMPLEX_BYTES_PER_DET], dtype='<f4')
    raw = raw.reshape(n, _COMPLEX_PER_DET, 2)  # (N, 16, 2) -- real, imag pairs
    return (raw[..., 0] + 1j * raw[..., 1]).astype(np.complex64)


def parse_frame(buf: bytes) -> Frame:
    """Parse a complete frame buffer that starts at the magic word."""
    if buf[:8] != MAGIC:
        raise ValueError("buffer does not start with TI magic word")
    fields = _HDR.unpack(buf[:HEADER_SIZE])
    header = FrameHeader(*fields[1:])  # skip magic

    tlvs: list[Tlv] = []
    off = HEADER_SIZE
    for _ in range(header.num_tlvs):
        if off + _TLV_HDR.size > len(buf):
            break
        ttype, tlen = _TLV_HDR.unpack(buf[off : off + _TLV_HDR.size])
        off += _TLV_HDR.size
        payload = buf[off : off + tlen]
        tlvs.append(Tlv(type=ttype, payload=payload))
        off += tlen
    return Frame(header=header, tlvs=tlvs)


def _read_exact(stream, n: int) -> bytes:
    buf = bytearray()
    while len(buf) < n:
        chunk = stream.read(n - len(buf))
        if not chunk:
            raise EOFError("stream closed mid-frame")
        buf.extend(chunk)
    return bytes(buf)


def read_frame(stream) -> Frame:
    """Sync to the magic word, then read one full frame by totalPacketLen."""
    window = _read_exact(stream, len(MAGIC))
    while window != MAGIC:
        window = window[1:] + _read_exact(stream, 1)

    rest_hdr = _read_exact(stream, HEADER_SIZE - len(MAGIC))
    header_buf = MAGIC + rest_hdr
    total_len = _HDR.unpack(header_buf)[2]  # totalPacketLen
    if total_len < HEADER_SIZE or total_len > 0x100000:
        raise ValueError(f"implausible totalPacketLen={total_len}")

    body = _read_exact(stream, total_len - HEADER_SIZE)
    return parse_frame(header_buf + body)


def build_frame(points: np.ndarray, frame_number: int = 1,
                antenna_complex: np.ndarray | None = None) -> bytes:
    """Encode points into a TI frame (for tests / offline replay).

    If *antenna_complex* is provided, it must be (N, 16) complex64 where N
    matches the number of points. A TLV_ANTENNA_COMPLEX block is appended.
    """
    pts = np.asarray(points, dtype="<f4").reshape(-1, 4)
    tlv_payload = pts.tobytes()
    tlv = _TLV_HDR.pack(TLV_DETECTED_POINTS, len(tlv_payload)) + tlv_payload
    num_tlvs = 1

    if antenna_complex is not None:
        cx = np.asarray(antenna_complex, dtype=np.complex64).reshape(-1, _COMPLEX_PER_DET)
        # Interleave real/imag as float32 pairs
        pairs = np.stack([cx.real, cx.imag], axis=-1).astype("<f4")
        cx_payload = pairs.tobytes()
        tlv += _TLV_HDR.pack(TLV_ANTENNA_COMPLEX, len(cx_payload)) + cx_payload
        num_tlvs += 1

    total = HEADER_SIZE + len(tlv)
    header = _HDR.pack(MAGIC, 1, total, 0, frame_number, 0, len(pts), num_tlvs, 0)
    return header + tlv
