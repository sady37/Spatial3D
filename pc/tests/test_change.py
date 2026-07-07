"""Tests for change.py — energy-density change detection (baseline vs event)."""

import numpy as np

from spatial3d.change import (
    ChangeEvent,
    detect_changes,
    energy_change,
    energy_density,
)


def test_energy_density_normalized_and_power_weighted():
    # two points in the same voxel with total power 5
    cloud = np.array([[0.0, 1.0, 0.5, 2.0], [0.02, 1.0, 0.5, 3.0]])
    g, meta = energy_density(cloud, voxel_size=0.3)
    assert np.isclose(g.sum(), 1.0)          # normalised distribution
    assert np.isclose(g.max(), 1.0)          # all energy in one voxel


def test_energy_density_drops_out_of_range():
    cloud = np.array([[0.0, 3.0, 0.5, 1.0],   # in range
                      [100.0, 3.0, 0.5, 1.0],  # x out of range
                      [0.0, 3.0, -1.0, 1.0]])  # below floor (z_range low)
    g, _ = energy_density(cloud, normalize=False)
    assert np.isclose(g.sum(), 1.0)          # only the first point counted


def test_identical_clouds_no_change():
    rng = np.random.default_rng(1)
    cloud = np.column_stack([rng.uniform(-2, 2, 40), rng.uniform(1, 5, 40),
                             rng.uniform(0, 1.5, 40), rng.uniform(1, 3, 40)])
    diff, _ = energy_change(cloud, cloud, voxel_size=0.3)
    assert np.allclose(diff, 0.0)


def test_detect_person_appeared_and_frame_gone():
    rng = np.random.default_rng(0)
    # identical background in both -> cancels
    bg = np.column_stack([rng.uniform(-2, 2, 60), rng.uniform(1, 5, 60),
                          rng.uniform(0, 1.2, 60), rng.uniform(1, 2, 60)])
    frame = np.column_stack([rng.normal(1.0, 0.05, 25), rng.normal(3.5, 0.05, 25),
                             rng.normal(0.8, 0.05, 25), rng.uniform(3, 4, 25)])
    person = np.column_stack([rng.normal(-0.5, 0.08, 25), rng.normal(3.5, 0.1, 25),
                              rng.normal(0.3, 0.05, 25), rng.uniform(3, 4, 25)])
    baseline = np.vstack([bg, frame])      # has the frame, no person
    event = np.vstack([bg, person])        # has the person, no frame

    events, diff, meta = detect_changes(baseline, event, voxel_size=0.3,
                                        rel_threshold=0.3, min_voxels=1)
    appeared = [e for e in events if e.kind == "appeared"]
    gone = [e for e in events if e.kind == "gone"]
    assert appeared and gone

    ap = max(appeared, key=lambda e: e.magnitude)
    assert abs(ap.center[0] - (-0.5)) < 0.5 and abs(ap.center[1] - 3.5) < 0.5
    assert ap.fall_zone                    # person is low -> fall/lie

    gn = max(gone, key=lambda e: e.magnitude)
    assert abs(gn.center[0] - 1.0) < 0.5 and abs(gn.center[1] - 3.5) < 0.5
    assert not gn.fall_zone                # 'gone' is never a fall


def test_no_events_below_threshold():
    rng = np.random.default_rng(2)
    cloud = np.column_stack([rng.uniform(-2, 2, 40), rng.uniform(1, 5, 40),
                             rng.uniform(0, 1.5, 40), rng.uniform(1, 3, 40)])
    events, _, _ = detect_changes(cloud, cloud, voxel_size=0.3)
    assert events == []                    # identical -> nothing changed
