"""Unit tests for the IK solver.

These tests use only Pinocchio + CasADi (no CoppeliaSim required).
"""

import numpy as np
import pytest

from g1_spine_stabilization.controllers.pose_tracking import ARM_DEFAULT_Q
from g1_spine_stabilization.kinematics.ik_solver import (
    G1HandIK,
    L_SHOULDER_ROLL_MIN,
    R_SHOULDER_ROLL_MAX,
)
from g1_spine_stabilization.trajectory.targets import (
    L_EE_DEFAULT,
    R_EE_DEFAULT,
)


# Building the IK solver is slow (Pinocchio + CasADi setup), so share one
# instance across all tests in this module.
@pytest.fixture(scope="module")
def ik():
    return G1HandIK()


def test_g1handik_instantiates(ik):
    assert ik is not None
    assert ik.reduced.model.nq == 14


def test_fk_at_zero_config_matches_documented_values(ik):
    L_pose, R_pose = ik.fk(np.zeros(14))
    np.testing.assert_allclose(L_pose.translation, L_EE_DEFAULT, atol=1e-3)
    np.testing.assert_allclose(R_pose.translation, R_EE_DEFAULT, atol=1e-3)


def test_fk_at_default_arm_pose_close_to_defaults(ik):
    # ARM_DEFAULT_Q is the natural standing pose (bent elbows, non-zero
    # shoulder rolls). L_EE_DEFAULT/R_EE_DEFAULT are measured at the zero
    # config, so the two configurations differ by ~1-2 cm in Cartesian space.
    # The 2.5 cm tolerance just sanity-checks that the standing pose puts the
    # hands in the same neighborhood.
    L_pose, R_pose = ik.fk(ARM_DEFAULT_Q)
    np.testing.assert_allclose(L_pose.translation, L_EE_DEFAULT, atol=2.5e-2)
    np.testing.assert_allclose(R_pose.translation, R_EE_DEFAULT, atol=2.5e-2)


def test_ik_round_trip_at_default_pose(ik):
    L_pose, R_pose = ik.fk(ARM_DEFAULT_Q)

    L_target = np.eye(4)
    L_target[:3, 3]  = L_pose.translation
    L_target[:3, :3] = L_pose.rotation
    R_target = np.eye(4)
    R_target[:3, 3]  = R_pose.translation
    R_target[:3, :3] = R_pose.rotation

    sol_q, _ = ik.solve_ik(L_target, R_target, current_q=ARM_DEFAULT_Q)
    L_actual, R_actual = ik.fk(sol_q)

    L_pos_err = np.linalg.norm(L_actual.translation - L_target[:3, 3])
    R_pos_err = np.linalg.norm(R_actual.translation - R_target[:3, 3])
    L_rot_err = np.linalg.norm(L_actual.rotation - L_target[:3, :3])
    R_rot_err = np.linalg.norm(R_actual.rotation - R_target[:3, :3])

    # IK regularization (0.02 * ||q||^2) trades off slightly against
    # position accuracy; 5 mm is the practical floor with the locked weights.
    assert L_pos_err < 5e-3, f"L position error {L_pos_err} too large"
    assert R_pos_err < 5e-3, f"R position error {R_pos_err} too large"
    assert L_rot_err < 5e-2, f"L rotation error {L_rot_err} too large"
    assert R_rot_err < 5e-2, f"R rotation error {R_rot_err} too large"


def test_ik_respects_shoulder_roll_bounds(ik):
    L_pose, R_pose = ik.fk(ARM_DEFAULT_Q)

    L_target = np.eye(4)
    L_target[:3, 3]  = L_pose.translation
    L_target[:3, :3] = L_pose.rotation
    R_target = np.eye(4)
    R_target[:3, 3]  = R_pose.translation
    R_target[:3, :3] = R_pose.rotation

    sol_q, _ = ik.solve_ik(L_target, R_target, current_q=ARM_DEFAULT_Q)

    eps = 1e-6
    assert sol_q[1] >= L_SHOULDER_ROLL_MIN - eps, (
        f"L_shoulder_roll {sol_q[1]} violates min {L_SHOULDER_ROLL_MIN}"
    )
    assert sol_q[8] <= R_SHOULDER_ROLL_MAX + eps, (
        f"R_shoulder_roll {sol_q[8]} violates max {R_SHOULDER_ROLL_MAX}"
    )
