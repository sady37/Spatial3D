"""Tests for the TI TLV frame parser (uses synthetic frames, no hardware)."""

import io

import numpy as np

from spatial3d.tlv import (
    MAGIC,
    TLV_DETECTED_POINTS,
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
