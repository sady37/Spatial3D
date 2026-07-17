"""TI mmWave TLV output parser (PC side).

Parses the UART output stream produced by a TI mmWave demo running on the
AWRL6844. The frame layout below is the *standard* TI mmWave demo format; the
exact TLV type IDs and point struct can vary between demos, so verify against
the L-SDK demo's `<demo>_output.h` once the radar is streaming (use the `dump`
tool to capture real bytes).

Frame layout (little-endian) — People_Tracking `MmwDemo_output_message_headerID`,
44 bytes (NOTE: the SBR/People_Tracking demo splits detected-obj into Major+Minor,
so this header is 44B / 9 uint32, not the classic 40B / 8 uint32 mmw-demo header):
    magic[8]            = 02 01 04 03 06 05 08 07
    version             uint32
    totalPacketLen      uint32   (whole frame incl. this header, padded to SEGMENT_LEN=32)
    platform            uint32
    frameNumber         uint32
    timeCpuCycles       uint32
    numDetectedObjMajor uint32
    numDetectedObjMinor uint32
    numTLVs             uint32
    subFrameNumber      uint32
    -- then numTLVs of: [type uint32][length uint32][payload length bytes]
"""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

MAGIC = bytes([0x02, 0x01, 0x04, 0x03, 0x06, 0x05, 0x08, 0x07])

import struct

_HDR = struct.Struct("<8s9I")   # magic + 9 uint32 (People_Tracking headerID, 44B)
HEADER_SIZE = _HDR.size          # 44
_TLV_HDR = struct.Struct("<2I")  # type, length

# Common TLV type IDs (standard mmWave demo). Confirm against L-SDK.
TLV_DETECTED_POINTS = 1
TLV_RANGE_PROFILE = 2
TLV_NOISE_PROFILE = 3
TLV_SIDE_INFO = 7

# Range-antenna zero-Doppler complex TLV (Spatial3D firmware mod, AWRL6844.md 5.5).
# Reuses the demo enum value 8 (MMWDEMO_OUTPUT_MSG_AZIMUT_ELEVATION_STATIC_HEAT_MAP).
# Payload layout (little-endian):
#     uint16 start_bin
#     uint16 num_bins
#     then num_bins * 16 * cmplx16ImRe_t  (int16 imag, int16 real)  -- imag FIRST
# i.e. per range bin, the zero-Doppler (coherent mean over chirps) 16-virtual-antenna
# vector. NOT indexed by detection -- the server maps detections->bins by range.
TLV_RANGE_ANTENNA = 8
TLV_ANTENNA_COMPLEX = TLV_RANGE_ANTENNA  # backward-compat alias

_NUM_VIRT_ANT = 16  # 4TX * 4RX virtual antennas
_RA_SUBHDR = struct.Struct("<2H")           # start_bin, num_bins
_BYTES_PER_ANT = 4                          # cmplx16ImRe_t: int16 imag + int16 real

# Detected-point struct: x, y, z, doppler  (float32 each) = 16 bytes.
_POINT = struct.Struct("<4f")

# --- Tracker-driven per-bin zero-Doppler cube (Spatial3D firmware feature) ---
# MMWDEMO_OUTPUT_EXT_MSG_TRACK_BIN_CUBE. Emitted by the trackBinCubeCfg-enabled
# People_Tracking build: for each STILL/fallen track, the zero-Doppler antenna
# vector at the track's range bin +- halfWin.
#   [u16 num_entries][u16 num_virt_ant]
#   per entry: [u32 tid][u16 range_bin][i16 vel_mmps][f32 range_m]
#              then num_virt_ant * cmplx16ImRe_t (int16 imag, int16 real -- imag FIRST)
TLV_TRACK_BIN_CUBE = 320
_TBC_SUBHDR = struct.Struct("<2H")   # num_entries, num_virt_ant
_TBC_ENTRY = struct.Struct("<IHhf")  # tid, range_bin, vel_mmps(int16), range_m(float32) = 12 B

# People_Tracking demo TLV ids (People_Tracking mmwave_demo_mss.h enum).
TLV_TARGET_LIST = 308        # trackerProc_Target array (GTRACK_3D, 112 B each)
TLV_TARGET_INDEX = 309
TLV_POSE = 321               # Spatial3D per-track pose MLP (Stood/Sat/Lying/Falling)
# TLV 321 layout (little-endian): uint16 numResults; uint16 reserved; then
# numResults * PoseResult (12 B each). Carries BOTH on-chip fall legs per track
# (server OR-fuses via falldet/clean.py):
#   uint32 tid; uint8 pose; uint8 fallingProb; uint8 valid;      # MLP leg
#   uint8 winDown; int16 winHsCm; uint8 winLowRun; uint8 winValid # window leg
# pose: 0=Stood 1=Sat 2=Lying 3=Falling, 0xFF=unknown.
_POSE_HDR = struct.Struct("<HH")             # numResults, reserved
_POSE_ENTRY = struct.Struct("<IBBBBhBB")     # tid,pose,fp,valid,down,hsCm,lowRun,winValid
_POSE_ENTRY_SIZE = 12
POSE_LABELS = {0: "Stood", 1: "Sat", 2: "Lying", 3: "Falling", 0xFF: "Unknown"}
TLV_STATS = 6
TLV_POINT_CLOUD = 3001       # minor-motion spherical compressed point cloud
# trackerProc_Target GTRACK_3D: tid, pos[3], vel[3], acc[3], ec[16], g, conf = 112 B.
# We decode only the leading tid + pos + vel (first 40 B) and stride the full 112.
_TARGET = struct.Struct("<I9f")      # tid, posX,posY,posZ, velX,velY,velZ, accX,accY,accZ
_TARGET_SIZE = 112
# TLV 3001 minor-motion point cloud: a unit (scale) header then quantized spherical
# points (MmwDemo_output_message_UARTpointCloud).
_PC_UNIT = struct.Struct("<5f")      # elevationUnit, azimuthUnit, dopplerUnit, rangeUnit, snrUnit
_PC_DT = np.dtype([("el", "<i1"), ("az", "<i1"), ("dop", "<i2"),
                   ("rng", "<u2"), ("snr", "<u2")])   # 8 B/point


@dataclass
class FrameHeader:
    version: int
    total_packet_len: int
    platform: int
    frame_number: int
    time_cpu_cycles: int
    num_detected_obj: int          # numDetectedObjMajor
    num_detected_obj_minor: int
    num_tlvs: int
    sub_frame_number: int


@dataclass
class Tlv:
    type: int
    payload: bytes


@dataclass
class RangeAntenna:
    """Zero-Doppler antenna vectors for a contiguous range-bin window."""
    start_bin: int
    data: np.ndarray  # (num_bins, 16) complex64

    @property
    def num_bins(self) -> int:
        return self.data.shape[0]


@dataclass
class TrackBinEntry:
    """One (track, range-bin) zero-Doppler antenna vector from TLV 320."""
    tid: int
    range_bin: int
    vel_mmps: int       # track |velocity| in mm/s (int16, diagnostic)
    range_m: float      # range_bin * rangeStep, metres
    vec: np.ndarray     # (num_virt_ant,) complex64 zero-Doppler antenna vector


@dataclass
class TrackBinCube:
    """Per-still/fallen-track per-bin zero-Doppler antenna vectors (TLV 320)."""
    num_virt_ant: int
    entries: list["TrackBinEntry"] = field(default_factory=list)

    def by_track(self) -> dict[int, np.ndarray]:
        """{tid: (n_bins, num_virt_ant) complex64}, bins ordered by range_bin.
        The per-track slab is the input to server-side MUSIC (angle -> person vs
        furniture) and slow-time phase (breathing) once accumulated over frames."""
        grouped: dict[int, list[np.ndarray]] = {}
        for e in sorted(self.entries, key=lambda e: (e.tid, e.range_bin)):
            grouped.setdefault(e.tid, []).append(e.vec)
        return {tid: np.stack(v) for tid, v in grouped.items()}


@dataclass
class Pose:
    """One track's fall/pose signals from TLV 321 (both on-chip legs).

    MLP leg (pose/falling_prob) = falling motion + free pose; window leg
    (down/h_s_cm) = sustained down-state, robust to the track freeze that breaks
    the MLP. The server OR-fuses them and cleans with the cube second-check
    (pc/falldet/clean.py)."""
    tid: int
    pose: int              # 0=Stood 1=Sat 2=Lying 3=Falling, 0xFF=unknown
    falling_prob: float    # P(Falling), 0..1 (firmware sends 0..255)
    valid: bool            # MLP leg valid (False until the 8-frame window filled)
    down: bool             # window leg: sustained down-state latched
    h_s_cm: int            # window leg: 2nd-highest point height above floor, cm
    low_run: int           # window leg: consecutive low frames
    win_valid: bool        # window leg had >=2 points this frame

    @property
    def label(self) -> str:
        return POSE_LABELS.get(self.pose, "Unknown")


@dataclass
class Target:
    """One tracked target from TLV 308 (trackerProc_Target, GTRACK_3D)."""
    tid: int
    x: float; y: float; z: float          # position, metres (relative to radar)
    vx: float; vy: float; vz: float        # velocity, m/s

    @property
    def speed(self) -> float:
        return float(np.hypot(np.hypot(self.vx, self.vy), self.vz))


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

    def range_antenna(self) -> RangeAntenna | None:
        """Zero-Doppler range-antenna block (start_bin + (num_bins, 16) complex64).
        None if the TLV is not present."""
        for t in self.tlvs:
            if t.type == TLV_RANGE_ANTENNA:
                return parse_range_antenna(t.payload)
        return None

    def antenna_complex(self) -> np.ndarray | None:
        """Deprecated: the (num_bins, 16) complex64 vectors from the range-antenna
        TLV. These are indexed by RANGE BIN, not by detection. Prefer range_antenna()."""
        ra = self.range_antenna()
        return None if ra is None else ra.data

    def track_bin_cube(self) -> "TrackBinCube | None":
        """Tracker-driven per-bin zero-Doppler cube (TLV 320). None if absent."""
        for t in self.tlvs:
            if t.type == TLV_TRACK_BIN_CUBE:
                return parse_track_bin_cube(t.payload)
        return None

    def targets(self) -> list["Target"]:
        """Tracked targets from TLV 308 (empty list if no tracker output)."""
        for t in self.tlvs:
            if t.type == TLV_TARGET_LIST:
                return parse_target_list(t.payload)
        return []

    def poses(self) -> dict[int, "Pose"]:
        """Per-track pose from TLV 321, keyed by tid (empty if absent)."""
        for t in self.tlvs:
            if t.type == TLV_POSE:
                return parse_pose_list(t.payload)
        return {}

    def point_cloud(self) -> "PointCloud | None":
        """Minor-motion point cloud (TLV 3001), Cartesian + SNR. None if absent."""
        for t in self.tlvs:
            if t.type == TLV_POINT_CLOUD:
                return parse_point_cloud(t.payload)
        return None


def parse_detected_points(payload: bytes) -> np.ndarray:
    n = len(payload) // _POINT.size
    if n == 0:
        return np.empty((0, 4), dtype=np.float32)
    arr = np.frombuffer(payload[: n * _POINT.size], dtype="<f4")
    return arr.reshape(n, 4).copy()


def parse_range_antenna(payload: bytes) -> RangeAntenna:
    """Parse the range-antenna zero-Doppler TLV (type 8).

    Wire format: [start_bin u16][num_bins u16] then num_bins*16 cmplx16ImRe_t
    (int16 imag, int16 real). Returns a RangeAntenna with (num_bins, 16) complex64.
    """
    if len(payload) < _RA_SUBHDR.size:
        return RangeAntenna(0, np.empty((0, _NUM_VIRT_ANT), dtype=np.complex64))
    start_bin, num_bins = _RA_SUBHDR.unpack_from(payload, 0)
    body = payload[_RA_SUBHDR.size:]
    count = num_bins * _NUM_VIRT_ANT
    avail = len(body) // _BYTES_PER_ANT
    count = min(count, avail)
    if count == 0:
        return RangeAntenna(start_bin, np.empty((0, _NUM_VIRT_ANT), dtype=np.complex64))
    raw = np.frombuffer(body[:count * _BYTES_PER_ANT], dtype='<i2').astype(np.float32)
    raw = raw.reshape(-1, _NUM_VIRT_ANT, 2)  # (num_bins, 16, [imag, real])
    data = (raw[..., 1] + 1j * raw[..., 0]).astype(np.complex64)  # real=[..,1], imag=[..,0]
    return RangeAntenna(int(start_bin), data)


# Backward-compat alias (old name); returns just the (num_bins, 16) array.
def parse_antenna_complex(payload: bytes) -> np.ndarray:
    return parse_range_antenna(payload).data


def parse_track_bin_cube(payload: bytes) -> TrackBinCube:
    """Parse the tracker-driven per-bin zero-Doppler cube TLV (type 320).

    Wire: [u16 num_entries][u16 num_virt_ant] then per entry
    [u32 tid][u16 range_bin][i16 vel_mmps][f32 range_m] +
    num_virt_ant * cmplx16ImRe_t (int16 imag, int16 real -- imag FIRST).
    """
    if len(payload) < _TBC_SUBHDR.size:
        return TrackBinCube(0, [])
    n_ent, n_ant = _TBC_SUBHDR.unpack_from(payload, 0)
    off = _TBC_SUBHDR.size
    vec_bytes = n_ant * _BYTES_PER_ANT
    entries: list[TrackBinEntry] = []
    for _ in range(n_ent):
        if off + _TBC_ENTRY.size + vec_bytes > len(payload):
            break
        tid, rbin, vel, rng = _TBC_ENTRY.unpack_from(payload, off)
        off += _TBC_ENTRY.size
        raw = np.frombuffer(payload[off:off + vec_bytes], dtype="<i2").astype(np.float32)
        off += vec_bytes
        raw = raw.reshape(n_ant, 2)                    # (num_virt_ant, [imag, real])
        vec = (raw[:, 1] + 1j * raw[:, 0]).astype(np.complex64)
        entries.append(TrackBinEntry(int(tid), int(rbin), int(vel), float(rng), vec))
    return TrackBinCube(int(n_ant), entries)


@dataclass
class PointCloud:
    """Minor-motion point cloud (TLV 3001), decoded to Cartesian + SNR (radar frame)."""
    xyz: np.ndarray      # (N,3) float32 metres
    snr: np.ndarray      # (N,) float32 linear
    doppler: np.ndarray  # (N,) float32 m/s


def parse_point_cloud(payload: bytes) -> PointCloud:
    """Parse TLV 3001: [5f unit header] then N x (i8 el, i8 az, i16 dop, u16 rng, u16 snr).
    el/az in unit-radians, rng in unit-metres -> Cartesian x,y,z (radar frame)."""
    empty = PointCloud(np.empty((0, 3), np.float32), np.empty(0, np.float32),
                       np.empty(0, np.float32))
    if len(payload) < _PC_UNIT.size:
        return empty
    elU, azU, dopU, rngU, snrU = _PC_UNIT.unpack_from(payload, 0)
    body = payload[_PC_UNIT.size:]
    n = len(body) // _PC_DT.itemsize
    if n == 0:
        return empty
    a = np.frombuffer(body[:n * _PC_DT.itemsize], dtype=_PC_DT)
    el = a["el"].astype(np.float32) * elU
    az = a["az"].astype(np.float32) * azU
    r = a["rng"].astype(np.float32) * rngU
    ce = np.cos(el)
    xyz = np.stack([r * ce * np.sin(az), r * ce * np.cos(az), r * np.sin(el)],
                   axis=1).astype(np.float32)
    return PointCloud(xyz, a["snr"].astype(np.float32) * snrU,
                      a["dop"].astype(np.float32) * dopU)


def parse_target_list(payload: bytes) -> list[Target]:
    """Parse TLV 308: array of trackerProc_Target (GTRACK_3D, 112 B each).

    Only the leading tid + position + velocity are decoded; the error covariance,
    gate gain and confidence tail are skipped by striding _TARGET_SIZE.
    """
    out: list[Target] = []
    n = len(payload) // _TARGET_SIZE
    for i in range(n):
        off = i * _TARGET_SIZE
        tid, px, py, pz, vx, vy, vz, _ax, _ay, _az = _TARGET.unpack_from(payload, off)
        out.append(Target(int(tid), px, py, pz, vx, vy, vz))
    return out


def parse_pose_list(payload: bytes) -> dict[int, Pose]:
    """Parse TLV 321: uint16 numResults, uint16 reserved, then numResults * 8 B."""
    if len(payload) < _POSE_HDR.size:
        return {}
    n, _reserved = _POSE_HDR.unpack_from(payload, 0)
    out: dict[int, Pose] = {}
    off = _POSE_HDR.size
    for _ in range(n):
        if off + _POSE_ENTRY_SIZE > len(payload):
            break
        tid, pose, fp, valid, down, hs, low, wv = _POSE_ENTRY.unpack_from(payload, off)
        out[int(tid)] = Pose(int(tid), int(pose), fp / 255.0, bool(valid),
                             bool(down), int(hs), int(low), bool(wv))
        off += _POSE_ENTRY_SIZE
    return out


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


def _encode_cmplx16(cx: np.ndarray) -> bytes:
    """(..., N) complex64 -> cmplx16ImRe_t bytes (int16 imag first, then real)."""
    pairs = np.stack([cx.imag, cx.real], axis=-1)   # imag first, then real
    return np.rint(pairs).astype("<i2").tobytes()


def build_frame(points: np.ndarray, frame_number: int = 1,
                range_antenna: tuple[int, np.ndarray] | None = None,
                track_bin_cube: "TrackBinCube | None" = None) -> bytes:
    """Encode points into a TI frame (for tests / offline replay).

    If *range_antenna* is provided as (start_bin, data) where data is
    (num_bins, 16) complex64, a TLV_RANGE_ANTENNA (type 8) block is appended,
    encoded as int16 cmplx16ImRe_t (imag first) to match the firmware.
    If *track_bin_cube* is provided, a TLV_TRACK_BIN_CUBE (type 320) block is
    appended with the same cmplx16 encoding.
    """
    pts = np.asarray(points, dtype="<f4").reshape(-1, 4)
    tlv_payload = pts.tobytes()
    tlv = _TLV_HDR.pack(TLV_DETECTED_POINTS, len(tlv_payload)) + tlv_payload
    num_tlvs = 1

    if range_antenna is not None:
        start_bin, data = range_antenna
        cx = np.asarray(data, dtype=np.complex64).reshape(-1, _NUM_VIRT_ANT)
        ra_payload = _RA_SUBHDR.pack(start_bin, cx.shape[0]) + _encode_cmplx16(cx)
        tlv += _TLV_HDR.pack(TLV_RANGE_ANTENNA, len(ra_payload)) + ra_payload
        num_tlvs += 1

    if track_bin_cube is not None:
        tbc = track_bin_cube
        body = _TBC_SUBHDR.pack(len(tbc.entries), tbc.num_virt_ant)
        for e in tbc.entries:
            vec = np.asarray(e.vec, dtype=np.complex64).reshape(tbc.num_virt_ant)
            body += _TBC_ENTRY.pack(e.tid, e.range_bin, e.vel_mmps, e.range_m)
            body += _encode_cmplx16(vec)
        tlv += _TLV_HDR.pack(TLV_TRACK_BIN_CUBE, len(body)) + body
        num_tlvs += 1

    total = HEADER_SIZE + len(tlv)
    # version, totalPacketLen, platform, frameNumber, timeCpuCycles,
    # numDetectedObjMajor, numDetectedObjMinor, numTLVs, subFrameNumber
    header = _HDR.pack(MAGIC, 1, total, 0, frame_number, 0, len(pts), 0, num_tlvs, 0)
    return header + tlv
