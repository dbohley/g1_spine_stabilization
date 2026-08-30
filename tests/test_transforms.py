"""Unit tests for utils/transforms.py."""

import math

import numpy as np
import pytest

from g1_spine_stabilization.utils.transforms import (
    Rx,
    make_se3,
    se3_from_xyz_rpy,
    slerp_rotation,
    smoothstep,
)


def test_rx_zero_is_identity():
    np.testing.assert_allclose(Rx(0.0), np.eye(3), atol=1e-12)


def test_rx_pi_over_two_maps_y_to_z():
    R = Rx(math.pi / 2)
    rotated = R @ np.array([0.0, 1.0, 0.0])
    np.testing.assert_allclose(rotated, np.array([0.0, 0.0, 1.0]), atol=1e-12)


def test_make_se3_translation_and_rotation():
    T = make_se3([1, 2, 3], np.eye(3))
    assert T.shape == (4, 4)
    np.testing.assert_allclose(T[:3, :3], np.eye(3))
    np.testing.assert_allclose(T[:3, 3], np.array([1, 2, 3]))
    np.testing.assert_allclose(T[3, :], np.array([0, 0, 0, 1]))


def test_se3_from_xyz_rpy_zero_rpy_is_identity_rotation():
    T = se3_from_xyz_rpy([0.1, 0.2, 0.3], [0.0, 0.0, 0.0])
    np.testing.assert_allclose(T[:3, :3], np.eye(3), atol=1e-12)
    np.testing.assert_allclose(T[:3, 3], [0.1, 0.2, 0.3])


def test_se3_from_xyz_rpy_matches_urdf_fixed_axis_convention():
    """R = Rz(yaw) @ Ry(pitch) @ Rx(roll) -- same convention independently
    used by perception/frames.py's optical_to_d435_from_config()."""
    roll, pitch, yaw = 0.3, -0.2, 0.5
    T = se3_from_xyz_rpy([0.0, 0.0, 0.0], [roll, pitch, yaw])

    cz, sz = math.cos(yaw), math.sin(yaw)
    cy, sy = math.cos(pitch), math.sin(pitch)
    cx, sx = math.cos(roll), math.sin(roll)
    Rz = np.array([[cz, -sz, 0], [sz, cz, 0], [0, 0, 1]])
    Ry = np.array([[cy, 0, sy], [0, 1, 0], [-sy, 0, cy]])
    Rx_ = np.array([[1, 0, 0], [0, cx, -sx], [0, sx, cx]])
    expected = Rz @ Ry @ Rx_

    np.testing.assert_allclose(T[:3, :3], expected, atol=1e-10)


def test_slerp_rotation_at_half_with_equal_endpoints():
    R = Rx(0.7)  # arbitrary non-identity rotation
    np.testing.assert_allclose(slerp_rotation(R, R, 0.5), R, atol=1e-12)


def test_slerp_rotation_endpoints():
    R1 = Rx(0.3)
    R2 = Rx(1.2)
    np.testing.assert_allclose(slerp_rotation(R1, R2, 0.0), R1, atol=1e-12)
    np.testing.assert_allclose(slerp_rotation(R1, R2, 1.0), R2, atol=1e-12)


def test_smoothstep_endpoints_and_midpoint():
    assert smoothstep(0.0) == pytest.approx(0.0)
    assert smoothstep(1.0) == pytest.approx(1.0)
    assert smoothstep(0.5) == pytest.approx(0.5)


def test_smoothstep_clamps():
    assert smoothstep(-1.0) == pytest.approx(0.0)
    assert smoothstep(2.0) == pytest.approx(1.0)
