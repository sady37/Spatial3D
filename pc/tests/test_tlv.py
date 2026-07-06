"""Tests for the TI TLV frame parser (uses synthetic frames, no hardware)."""

import io

import numpy as np

from spatial3d.tlv import (
    MAGIC,
    TLV_DETECTED_POINTS,
    TLV_RANGE_ANTENNA,
    build_frame,
    parse_frame,
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
