"""Tests for compute_press_wrenches — the inward inter-palm press wrenches for
the Jacobian force controller.

Verifies inward direction/sign at theta=0, magnitude scaling with F_cmd, rotation
of the press axis with theta, zero moment, and the equal-and-opposite property.
"""
import numpy as np
import pytest

from g1_spine_stabilization.trajectory.targets import compute_press_wrenches, press_axis
from g1_spine_stabilization.utils.transforms import Rx


def test_zero_theta_inward_signs():
    """Left palm (at +y) presses toward center (-y); right palm (at -y) presses +y."""
    W_L, W_R = compute_press_wrenches(0.0, 5.0)
    np.testing.assert_allclose(W_L[:3], [0.0, -5.0, 0.0], atol=1e-12)
    np.testing.assert_allclose(W_R[:3], [0.0, +5.0, 0.0], atol=1e-12)


def test_zero_moment():
    W_L, W_R = compute_press_wrenches(0.4, 3.0)
    np.testing.assert_allclose(W_L[3:], 0.0, atol=1e-12)
    np.testing.assert_allclose(W_R[3:], 0.0, atol=1e-12)


def test_magnitude_scales_with_F_cmd():
    for F in [0.0, 1.0, 10.0]:
        W_L, W_R = compute_press_wrenches(0.3, F)
        assert np.linalg.norm(W_L[:3]) == pytest.approx(F, abs=1e-9)
        assert np.linalg.norm(W_R[:3]) == pytest.approx(F, abs=1e-9)


def test_axis_rotates_with_theta():
    theta = 0.5
    W_L, W_R = compute_press_wrenches(theta, 1.0)
    u = Rx(theta) @ np.array([0.0, 1.0, 0.0])
    np.testing.assert_allclose(W_L[:3], -u, atol=1e-12)
    np.testing.assert_allclose(W_R[:3], +u, atol=1e-12)


def test_equal_and_opposite():
    W_L, W_R = compute_press_wrenches(0.7, 4.0)
    np.testing.assert_allclose(W_L[:3], -W_R[:3], atol=1e-12)


def test_zero_force_is_zero_wrench():
    W_L, W_R = compute_press_wrenches(1.1, 0.0)
    assert np.allclose(W_L, 0.0) and np.allclose(W_R, 0.0)


# ---------------------------------------------------------------------
# press_axis + compute_press_wrenches(grasp_frame_R=...)
# ---------------------------------------------------------------------

def test_press_axis_default_matches_pelvis_y():
    for theta in (0.0, 0.3, -0.7):
        np.testing.assert_allclose(
            press_axis(theta), Rx(theta) @ np.array([0.0, 1.0, 0.0]), atol=1e-12,
        )


def test_press_axis_identity_matches_none():
    theta = 0.4
    np.testing.assert_allclose(
        press_axis(theta, grasp_frame_R=np.eye(3)), press_axis(theta, grasp_frame_R=None),
        atol=1e-12,
    )


def test_grasp_frame_r_none_matches_original_behavior():
    """grasp_frame_R=None (default) must reproduce today's pelvis-Y wrench
    axis byte-for-byte -- regression guard for every existing caller that
    doesn't pass it."""
    for theta in (0.0, 0.3, -0.7):
        W_L0, W_R0 = compute_press_wrenches(theta, 4.0)
        W_L1, W_R1 = compute_press_wrenches(theta, 4.0, grasp_frame_R=None)
        np.testing.assert_array_equal(W_L0, W_L1)
        np.testing.assert_array_equal(W_R0, W_R1)


def test_grasp_frame_r_rotates_the_press_axis():
    """A non-identity grasp_frame_R must rotate the wrench axis the same way
    it already rotates compute_hand_targets's position arc (same G[:, 1]
    convention) -- this is the fix for the stale-axis gap: previously this
    function ignored grasp_frame_R entirely."""
    theta = 0.0
    F = 3.0
    # 90 degree rotation about pelvis Z: the object's local Y (column 1) now
    # points along world -X instead of world +Y -- same G used in
    # test_grasp_frame_r_rotates_position_and_orientation_together
    # (test_targets_squeeze.py) for the analogous compute_hand_targets check.
    c, s = np.cos(np.pi / 2), np.sin(np.pi / 2)
    G = np.array([
        [c, -s, 0.0],
        [s,  c, 0.0],
        [0.0, 0.0, 1.0],
    ])
    W_L, W_R = compute_press_wrenches(theta, F, grasp_frame_R=G)
    expected_axis = G[:, 1]
    np.testing.assert_allclose(W_L[:3], -F * expected_axis, atol=1e-12)
    np.testing.assert_allclose(W_R[:3], +F * expected_axis, atol=1e-12)
    # and NOT the old hardcoded pelvis-Y axis
    assert not np.allclose(W_L[:3], [0.0, -F, 0.0], atol=1e-6)
