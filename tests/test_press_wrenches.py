"""Tests for compute_press_wrenches — the inward inter-palm press wrenches for
the Jacobian force controller.

Verifies inward direction/sign at theta=0, magnitude scaling with F_cmd, rotation
of the press axis with theta, zero moment, and the equal-and-opposite property.
"""
import numpy as np
import pytest

from g1_spine_stabilization.trajectory.targets import compute_press_wrenches
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
