"""Tests for the per-bin MUSIC pipeline (synthetic frames, no hardware)."""

import numpy as np
import pytest

from spatial3d.music import (
    awrl6844_array,
    estimate_covariance,
    music_doa,
    spatial_smoothing_2d,
    subarray_array,
)
from spatial3d.range_music import (
    DR_M,
    LAYERS,
    BinAccumulator,
    bin_range_m,
    covariances_to_points,
    parse_layers_from_cfg,
    parse_music_params_from_cfg,
    room_scan_plan,
    spherical_to_cart,
)
from spatial3d.tlv import RangeAntenna, build_frame, parse_frame


# --- geometry ---------------------------------------------------------------
def test_bin_range_matches_cfg_edges():
    # profile_4T4R_music.cfg: bins 87-330 == slant range 2.03-7.73 m
    assert bin_range_m(87) == pytest.approx(2.039, abs=0.01)
    assert bin_range_m(330) == pytest.approx(7.734, abs=0.01)


def test_spherical_to_cart_conventions():
    # boresight (az=0,el=0) -> +y
    xyz = spherical_to_cart(3.0, 0.0, 0.0)[0]
    np.testing.assert_allclose(xyz, [0, 3, 0], atol=1e-9)
    # +az -> +x ; +el -> +z
    assert spherical_to_cart(2.0, np.deg2rad(30), 0.0)[0][0] > 0
    assert spherical_to_cart(2.0, 0.0, np.deg2rad(30))[0][2] > 0


# --- parametric scan plan ---------------------------------------------------
def test_scan_plan_h2_matches_cfg_start_bin():
    # H=2m, tilt 35, el-down 45 -> near floor 0.35m horiz -> slant 2.03m -> bin 87
    p = room_scan_plan(mount_height_m=2.0, room_far_horiz_m=6.0, fps=10)
    assert p["start_bin"] == 87
    assert p["near_horiz_m"] == pytest.approx(0.35, abs=0.02)
    assert p["r_min_m"] == pytest.approx(2.03, abs=0.02)
    assert p["H_MOUNT"] == 2.0


def test_scan_plan_start_bin_scales_with_height():
    b2 = room_scan_plan(mount_height_m=2.0)["start_bin"]
    b25 = room_scan_plan(mount_height_m=2.5)["start_bin"]
    assert b25 > b2                              # higher mount -> larger near slant
    assert b25 == pytest.approx(108, abs=2)


def test_scan_plan_5fps_fits_one_window_10fps_does_not():
    p10 = room_scan_plan(mount_height_m=2.0, room_far_horiz_m=6.0, fps=10)
    p5 = room_scan_plan(mount_height_m=2.0, room_far_horiz_m=6.0, fps=5)
    assert p10["max_bins_per_frame"] < p10["total_bins"]   # 10fps: needs layers
    assert not p10["fits_one_window"]
    assert p5["max_bins_per_frame"] > p5["total_bins"]     # 5fps: fits
    assert p5["fits_one_window"]
    assert p5["rangeAntennaOutput"].startswith("rangeAntennaOutput 87 ")


# --- accumulator ------------------------------------------------------------
def _ra(start_bin, num_bins, fill):
    data = np.full((num_bins, 16), fill, dtype=np.complex64)
    return RangeAntenna(start_bin, data)


def test_accumulator_caps_at_k_and_counts():
    acc = BinAccumulator(k=3)
    for _ in range(5):
        acc.add(_ra(87, 4, 1 + 1j))
    counts = acc.counts()
    assert set(counts) == {87, 88, 89, 90}
    assert all(c == 3 for c in counts.values())          # capped at k
    assert acc.is_layer_full(87, 4)
    assert not acc.is_layer_full(87, 5)                   # bin 91 unseen


def test_accumulator_from_built_frames():
    acc = BinAccumulator(k=10)
    data = (np.arange(2 * 16).reshape(2, 16)
            + 1j * np.arange(2 * 16)[::-1].reshape(2, 16)).astype(np.complex64)
    frame = parse_frame(build_frame(np.zeros((0, 4), np.float32),
                                    range_antenna=(100, data)))
    for _ in range(4):
        acc.add(frame.range_antenna())
    assert acc.counts() == {100: 4, 101: 4}
    covs = acc.covariances(min_snapshots=4)
    assert covs[100].shape == (16, 16)
    # Hermitian
    np.testing.assert_allclose(covs[100], covs[100].conj().T, atol=1e-4)


def test_layers_are_contiguous_and_cover_room():
    starts = [s for s, _ in LAYERS]
    ends = [s + n for s, n in LAYERS]
    assert starts[0] == 87
    for (s, n), nxt in zip(LAYERS, starts[1:] + [ends[-1]]):
        assert s + n == nxt                              # no gaps/overlap
    assert ends[-1] == 331                               # last bin 330


# --- cfg auto-read ----------------------------------------------------------
_MUSIC_CFG = """\
sensorStop 0
% Layer rolling plan:
%   Layer 2:  rangeAntennaOutput 169 82 1   (3.96-5.88 m)
%   Layer 3:  rangeAntennaOutput 251 80 1   (5.88-7.73 m)
%   Disable:  rangeAntennaOutput 0 0 0
% spatial3d: snapshots=50 min_snapshots=10 rounds=4 timeout=90
rangeAntennaOutput 87 82 1
sensorStart 0 0 0 0
"""


def test_parse_layers_from_cfg(tmp_path):
    cfg = tmp_path / "music.cfg"
    cfg.write_text(_MUSIC_CFG)
    layers = parse_layers_from_cfg(str(cfg))
    # active line + Layer2/3 comments, sorted, disable (0 0 0) dropped
    assert layers == [(87, 82), (169, 82), (251, 80)]


def test_parse_music_params_from_cfg(tmp_path):
    cfg = tmp_path / "music.cfg"
    cfg.write_text(_MUSIC_CFG)
    params = parse_music_params_from_cfg(str(cfg))
    assert params["snapshots"] == 50 and isinstance(params["snapshots"], int)
    assert params["min_snapshots"] == 10
    assert params["rounds"] == 4
    assert params["timeout"] == 90


def test_parse_layers_falls_back_when_absent(tmp_path):
    cfg = tmp_path / "plain.cfg"
    cfg.write_text("sensorStop 0\nsensorStart 0 0 0 0\n")
    assert parse_layers_from_cfg(str(cfg)) == list(LAYERS)
    assert parse_music_params_from_cfg(str(cfg)) == {}


# --- per-bin DOA ------------------------------------------------------------
def _source_covariance(array, az_deg, el_deg, k=50, snr_db=25.0, seed=0):
    """Static-target model: constant response + independent noise per snapshot."""
    rng = np.random.default_rng(seed)
    a = array.steering_vector(np.deg2rad(az_deg), np.deg2rad(el_deg))
    npow = 10 ** (-snr_db / 10)
    noise = np.sqrt(npow / 2) * (rng.standard_normal((k, array.n_antennas))
                                 + 1j * rng.standard_normal((k, array.n_antennas)))
    return estimate_covariance(a[None, :] + noise)


def test_single_source_doa_recovers_angle():
    array = awrl6844_array()
    R = _source_covariance(array, az_deg=20.0, el_deg=-10.0)
    dets = music_doa(R, array, n_signals=1, az_range=(-45, 45),
                     el_range=(-45, 20), resolution_deg=1.0)
    assert dets, "no DOA peak found"
    az, el, _ = dets[0]
    assert abs(az - 20.0) <= 3.0
    assert abs(el - (-10.0)) <= 3.0


def test_covariances_to_points_maps_bins_and_angles():
    array = awrl6844_array()
    covs = {
        100: _source_covariance(array, az_deg=25.0, el_deg=0.0, seed=1),
        200: _source_covariance(array, az_deg=-25.0, el_deg=5.0, seed=2),
    }
    pts = covariances_to_points(covs, array, n_signals=1, resolution_deg=1.0,
                                max_peaks_per_bin=1)
    assert pts.shape[0] == 2
    by_bin = {int(row[4]): row for row in pts}
    # range column matches bin*DR
    assert by_bin[100][5] == pytest.approx(100 * DR_M, abs=1e-3)
    assert by_bin[200][5] == pytest.approx(200 * DR_M, abs=1e-3)
    # +az -> +x, -az -> -x
    assert by_bin[100][0] > 0
    assert by_bin[200][0] < 0


# --- 2D spatial smoothing (coherent sources) --------------------------------
def _coherent_stack(array, sources, k=60, snr_db=30.0, seed=7):
    rng = np.random.default_rng(seed)
    signal = np.zeros(array.n_antennas, dtype=np.complex128)
    for az, el in sources:  # equal, constant amplitude -> fully coherent
        signal += array.steering_vector(np.deg2rad(az), np.deg2rad(el))
    npow = 10 ** (-snr_db / 10)
    noise = np.sqrt(npow / 2) * (rng.standard_normal((k, array.n_antennas))
                                 + 1j * rng.standard_normal((k, array.n_antennas)))
    return signal[None, :] + noise


def test_spatial_smoothing_resolves_coherent_sources():
    full = awrl6844_array()
    sources = [(-30.0, 0.0), (30.0, 0.0)]
    stack = _coherent_stack(full, sources)

    # Plain covariance is ~rank-1 for coherent sources: eigengap says 1 signal.
    R_plain = estimate_covariance(stack)
    ev = np.sort(np.linalg.eigvalsh(R_plain))[::-1]
    assert ev[0] / ev[1] > 50                 # dominant single eigenvalue

    # 2D smoothing restores rank; a 3x3 sub-array resolves both sources.
    R_s = spatial_smoothing_2d(stack, grid=(4, 4), sub=(3, 3))
    sub = subarray_array((3, 3))
    dets = music_doa(R_s, sub, n_signals=2, az_range=(-45, 45),
                     el_range=(-20, 20), resolution_deg=1.0)
    found_az = sorted(d[0] for d in dets[:2])
    assert len(dets) >= 2, f"smoothing failed to resolve: {dets}"
    assert min(found_az) < -15 and max(found_az) > 15


def test_subarray_array_dimension_matches_smoothed_cov():
    stack = _coherent_stack(awrl6844_array(), [(10.0, 0.0)])
    for sub in [(2, 2), (3, 3), (2, 3)]:
        R = spatial_smoothing_2d(stack, grid=(4, 4), sub=sub)
        arr = subarray_array(sub)
        assert R.shape == (sub[0] * sub[1], sub[0] * sub[1])
        assert arr.n_antennas == sub[0] * sub[1]


# --- room transform ---------------------------------------------------------
def test_to_room_lifts_and_carries_extra_columns():
    from spatial3d.music_collect import to_room
    # radar point straight ahead at 3 m, with power/bin/range extra cols
    pts = np.array([[0.0, 3.0, 0.0, 5.0, 128.0, 3.0]], dtype=np.float32)
    room = to_room(pts)
    assert room.shape == (1, 6)               # extra columns carried through
    assert room[0, 2] == pytest.approx(2.0 - 3.0 * np.sin(np.radians(35.0)), abs=1e-3)
    np.testing.assert_allclose(room[0, 3:], [5.0, 128.0, 3.0], atol=1e-4)
