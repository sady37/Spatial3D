"""Tests for cluster.py — DBSCAN clustering + bounding-box furniture detection."""

import numpy as np

from spatial3d.cluster import (
    ObjectBox,
    classify_box,
    cluster_and_box,
    detect_objects,
    fit_boxes,
    stable_voxel_centers,
)


def test_cluster_two_separated_blobs():
    rng = np.random.default_rng(0)
    a = rng.normal([0.0, 1.0, 0.5], 0.08, (30, 3))
    b = rng.normal([2.5, 3.0, 0.8], 0.08, (30, 3))
    pts = np.vstack([a, b])
    labels, boxes = cluster_and_box(pts, eps=0.3, min_samples=5, min_points=5)
    assert len(boxes) == 2
    centers = sorted(b.center[0] for b in boxes)
    assert abs(centers[0] - 0.0) < 0.2
    assert abs(centers[1] - 2.5) < 0.2


def test_box_bounds_and_footprint():
    pts = np.array([[0, 0, 0], [1, 2, 0.5]], dtype=float)
    boxes = fit_boxes(pts, np.array([0, 0]), min_points=2)
    assert len(boxes) == 1
    b = boxes[0]
    np.testing.assert_allclose(b.center, [0.5, 1.0, 0.25])
    np.testing.assert_allclose(b.size, [1.0, 2.0, 0.5])
    assert b.footprint == 2.0


def test_classify_bands():
    assert classify_box(np.array([0, 0, 0.15]), np.array([1, 1, 0.2])) == "floor"
    assert classify_box(np.array([0, 0, 0.8]), np.array([1.0, 1.0, 0.6])) == "table"
    assert classify_box(np.array([0, 0, 1.7]), np.array([0.3, 0.3, 0.5])) == "wall/person"


def test_noise_label_skipped():
    pts = np.array([[0, 0, 0], [5, 5, 5]], dtype=float)
    boxes = fit_boxes(pts, np.array([-1, -1]))   # all noise
    assert boxes == []


def test_stable_voxel_centers_drops_ghosts():
    # 20 points in one voxel above floor, 5 below-floor ghosts
    good = np.tile([1.05, 2.05, 0.55], (20, 1))
    ghost = np.tile([1.0, 2.0, -1.0], (5, 1))
    centers, dens = stable_voxel_centers(np.vstack([good, ghost]),
                                         voxel_size=0.3, min_density=10, z_min=-0.1)
    assert len(centers) == 1                 # ghosts dropped, one dense voxel
    assert dens[0] == 20


def test_detect_objects_runs_on_synthetic_room():
    rng = np.random.default_rng(1)
    floor = rng.uniform([-2, 0, -0.05], [2, 5, 0.05], (200, 3))
    table = rng.normal([0.5, 2.0, 0.8], 0.06, (60, 3))     # a table cluster
    cloud = np.vstack([floor, table])
    boxes, above = detect_objects(cloud, voxel_size=0.3, min_density=3,
                                  floor_z=0.35, eps=0.5, min_samples=2, min_points=2)
    assert len(boxes) >= 1
    # the table cluster should be recovered near (0.5, 2.0)
    assert any(abs(b.center[0] - 0.5) < 0.4 and abs(b.center[1] - 2.0) < 0.4
               for b in boxes)
