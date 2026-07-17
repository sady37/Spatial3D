"""Tests for the TI TLV frame parser (uses synthetic frames, no hardware)."""

import io

import numpy as np

import struct

from spatial3d.tlv import (
    MAGIC,
    TLV_DETECTED_POINTS,
    TLV_POSE,
    TLV_RANGE_ANTENNA,
    TLV_TRACK_BIN_CUBE,
    Frame,
    Tlv,
    TrackBinCube,
    TrackBinEntry,
    build_frame,
    parse_frame,
    parse_pose_list,
    read_frame,
)

POINTS = np.array(
    [[1.0, 2.0, 0.5, 0.1], [-0.5, 3.0, 1.2, -0.3]], dtype=np.float32
)


def test_build_parse_roundtrip():
    frame = parse_frame(build_frame(POINTS, frame_number=42))
    assert frame.header.frame_number == 42
    assert frame.header.num_detected_obj == 2
    assert frame.tlvs[0].type == TLV_DETECTED_POINTS
    np.testing.assert_allclose(frame.detected_points(), POINTS, rtol=1e-6)


def test_read_frame_syncs_past_garbage():
    stream = io.BytesIO(b"\x11\x22garbage" + build_frame(POINTS))
    frame = read_frame(stream)
    np.testing.assert_allclose(frame.detected_points(), POINTS, rtol=1e-6)


def test_read_two_back_to_back_frames():
    stream = io.BytesIO(build_frame(POINTS, 1) + build_frame(POINTS, 2))
    assert read_frame(stream).header.frame_number == 1
    assert read_frame(stream).header.frame_number == 2


def test_magic_word_value():
    assert MAGIC == bytes([2, 1, 4, 3, 6, 5, 8, 7])


def test_range_antenna_roundtrip():
    # 3 range bins x 16 antennas, integer-valued so int16 quantization is exact
    data = (np.arange(3 * 16).reshape(3, 16)
            + 1j * np.arange(3 * 16, 0, -1).reshape(3, 16)).astype(np.complex64)
    frame = parse_frame(build_frame(POINTS, range_antenna=(7, data)))
    assert frame.tlvs[-1].type == TLV_RANGE_ANTENNA
    ra = frame.range_antenna()
    assert ra is not None
    assert ra.start_bin == 7
    assert ra.num_bins == 3
    np.testing.assert_allclose(ra.data, data, rtol=0, atol=0)


def test_track_bin_cube_roundtrip():
    # two tracks: tid 1 has 3 bins (still), tid 2 has 2 bins; 16 virtual antennas.
    def vec(seed):
        return (np.arange(16) + seed + 1j * np.arange(16, 0, -1)).astype(np.complex64)
    entries = [
        TrackBinEntry(tid=1, range_bin=40, vel_mmps=20, range_m=3.40, vec=vec(0)),
        TrackBinEntry(tid=1, range_bin=41, vel_mmps=20, range_m=3.49, vec=vec(100)),
        TrackBinEntry(tid=1, range_bin=42, vel_mmps=20, range_m=3.57, vec=vec(200)),
        TrackBinEntry(tid=2, range_bin=18, vel_mmps=5, range_m=1.53, vec=vec(300)),
        TrackBinEntry(tid=2, range_bin=19, vel_mmps=5, range_m=1.62, vec=vec(400)),
    ]
    tbc_in = TrackBinCube(num_virt_ant=16, entries=entries)
    frame = parse_frame(build_frame(POINTS, track_bin_cube=tbc_in))
    assert frame.tlvs[-1].type == TLV_TRACK_BIN_CUBE
    tbc = frame.track_bin_cube()
    assert tbc is not None
    assert tbc.num_virt_ant == 16
    assert len(tbc.entries) == 5
    e0 = tbc.entries[0]
    assert (e0.tid, e0.range_bin, e0.vel_mmps) == (1, 40, 20)
    np.testing.assert_allclose(e0.range_m, 3.40, rtol=0, atol=1e-4)
    np.testing.assert_allclose(e0.vec, vec(0), rtol=0, atol=0)
    # by_track groups a track's bins into a (n_bins, 16) slab ordered by range_bin
    slabs = tbc.by_track()
    assert set(slabs) == {1, 2}
    assert slabs[1].shape == (3, 16) and slabs[2].shape == (2, 16)
    np.testing.assert_allclose(slabs[1][2], vec(200), rtol=0, atol=0)


def test_pose_tlv_roundtrip():
    # TLV 321: uint16 numResults, uint16 reserved, then 8 B per entry.
    entries = [(0, 2, 128, 1), (5, 3, 255, 1), (9, 0xFF, 0, 0)]
    body = struct.pack("<HH", len(entries), 0)
    for tid, pose, fp, valid in entries:
        body += struct.pack("<IBBBB", tid, pose, fp, valid, 0)

    poses = parse_pose_list(body)
    assert set(poses) == {0, 5, 9}
    assert poses[0].label == "Lying" and poses[0].valid
    np.testing.assert_allclose(poses[0].falling_prob, 128 / 255, atol=1e-6)
    assert poses[5].label == "Falling" and poses[5].falling_prob == 1.0
    assert poses[9].label == "Unknown" and not poses[9].valid

    # via Frame.poses(); absent TLV -> empty dict
    frame = Frame(header=None, tlvs=[Tlv(type=TLV_POSE, payload=body)])
    assert frame.poses()[5].label == "Falling"
    assert Frame(header=None, tlvs=[]).poses() == {}
