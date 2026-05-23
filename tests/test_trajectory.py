"""Unit tests for trajectory targets + sources."""

import numpy as np
import pytest

from g1_spine_stabilization.trajectory.targets import (
    BOX_CENTER_DEFAULT,
    BOX_HALF_WIDTH_DEFAULT,
    L_EE_DEFAULT,
    R_EE_DEFAULT,
    R_L_BASE,
    R_R_BASE,
    compute_hand_targets,
)
from g1_spine_stabilization.trajectory.sources import sine_trajectory


def test_compute_hand_targets_at_theta_zero_matches_defaults():
    L_target, R_target = compute_hand_targets(
        BOX_CENTER_DEFAULT, BOX_HALF_WIDTH_DEFAULT, 0.0
    )
    np.testing.assert_allclose(L_target[:3, 3], L_EE_DEFAULT, atol=1e-6)
    np.testing.assert_allclose(R_target[:3, 3], R_EE_DEFAULT, atol=1e-6)


def test_compute_hand_targets_at_theta_zero_matches_base_rotations():
    L_target, R_target = compute_hand_targets(
        BOX_CENTER_DEFAULT, BOX_HALF_WIDTH_DEFAULT, 0.0
    )
    np.testing.assert_allclose(L_target[:3, :3], R_L_BASE, atol=1e-12)
    np.testing.assert_allclose(R_target[:3, :3], R_R_BASE, atol=1e-12)


def test_sine_trajectory_at_zero():
    assert sine_trajectory(0.0, 1.0, 1.0) == pytest.approx(0.0)


def test_sine_trajectory_quarter_period_peak():
    # freq=1Hz, t=0.25s → quarter period → peak amplitude
    assert sine_trajectory(0.25, 1.0, 1.0) == pytest.approx(1.0)
