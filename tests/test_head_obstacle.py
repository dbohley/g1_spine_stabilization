"""
Pure geometry tests for kinematics/collision.py's build_head_obstacle.
No G1HandIK needed -- fast.
"""

import numpy as np
import pytest

from g1_spine_stabilization.kinematics.collision import (
    build_head_obstacle,
    head_sphere_center_from_marker,
)


def test_true_shape_matches_radius_exactly():
    head = build_head_obstacle([0.36, 0.0, 0.10], 0.10, margin_m=0.0)
    assert head.true_shape.radius == pytest.approx(0.10)


def test_padded_shape_adds_margin_to_radius():
    head = build_head_obstacle([0.36, 0.0, 0.10], 0.10, margin_m=0.04)
    assert head.padded_shape.radius == pytest.approx(0.14)


def test_padded_equals_true_when_margin_zero():
    head = build_head_obstacle([0.36, 0.0, 0.10], 0.10, margin_m=0.0)
    assert head.padded_shape.radius == pytest.approx(head.true_shape.radius)


def test_placement_translation_matches_center():
    center = np.array([0.42, -0.05, 0.12])
    head = build_head_obstacle(center, 0.10, margin_m=0.0)
    np.testing.assert_allclose(head.placement.translation, center)


def test_placement_rotation_is_identity():
    head = build_head_obstacle([0.36, 0.0, 0.10], 0.10, margin_m=0.04)
    np.testing.assert_allclose(head.placement.rotation, np.eye(3), atol=1e-12)


def test_sphere_center_from_marker_shifts_down_marker_up_axis():
    # Marker with identity rotation -- column 2 (its "out of surface" axis) is world Z.
    marker_pos = np.array([0.4, 0.0, 0.3])
    marker_R = np.eye(3)
    center = head_sphere_center_from_marker(marker_pos, marker_R, radius=0.08)
    np.testing.assert_allclose(center, [0.4, 0.0, 0.22])


def test_sphere_center_from_marker_uses_markers_own_tilted_axis():
    # Marker rotated 90 deg about Y: its local Z (column 2) now points along world +X,
    # so the "shift down the marker's own surface normal" moves along X, not world Z.
    marker_pos = np.array([0.5, 0.0, 0.3])
    marker_R = np.array([
        [0.0, 0.0, 1.0],
        [0.0, 1.0, 0.0],
        [-1.0, 0.0, 0.0],
    ])
    center = head_sphere_center_from_marker(marker_pos, marker_R, radius=0.1)
    np.testing.assert_allclose(center, [0.4, 0.0, 0.3], atol=1e-12)
