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
    compute_hand_targets,
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


def test_solve_ik_refined_converges_closer_than_a_single_solve(ik):
    # Regression test for a confirmed real bug: a single solve_ik() call
    # warm-started very far from the target (e.g. an extreme test-start
    # pose reaching for the pregrasp standoff) can terminate "successfully"
    # (IPOPT reports convergence, no ipopt_exhaustion_count hit) at a
    # solution that is still several cm off in Cartesian space -- a
    # local-minimum/warm-start-quality effect, not a hard solver failure.
    # Reproduced via an isolated repro using this exact far-start scenario
    # (see docs/followups.md and the sdk_pose_tracking.py Phase 1a comment).
    far_q = np.array([
        0.0518, 0.1502, -0.4450, 0.2788, 0.3090, -1.4979, 0.3441,
        0.0230, -0.1507, 0.4142, 0.3409, -0.0744, -1.5369, -0.1171,
    ])
    # Pregrasp-standoff-shaped target (same geometry as the real Phase 1a
    # goal_q call that reproduced this bug) -- an arbitrary/nearby target
    # does not reliably trigger the local-minimum effect (verified: with
    # ARM_DEFAULT_Q as the target instead, a single solve already converges
    # to ~1.6cm from this same far_q, well under the bug's ~9cm).
    box_center = np.array([0.355, 0.0, 0.195])
    L_target, R_target = compute_hand_targets(box_center, box_half_width=0.16,
                                               theta=0.0, squeeze=-0.08)

    single_q, _ = ik.solve_ik(L_target, R_target, current_q=far_q)
    ik.reset_smoothing(far_q)  # isolate solve_ik_refined from single_q's filter state
    refined_q = ik.solve_ik_refined(L_target, R_target, far_q)

    L_single, _ = ik.fk(single_q)
    L_refined, _ = ik.fk(refined_q)
    single_err = np.linalg.norm(L_single.translation - L_target[:3, 3])
    refined_err = np.linalg.norm(L_refined.translation - L_target[:3, 3])

    assert refined_err < single_err, (
        f"Expected solve_ik_refined ({refined_err:.4f}m) to converge closer "
        f"than a single solve_ik() call ({single_err:.4f}m) from a far warm start"
    )
    # Not asserting against the tight steady-state regularization floor
    # (~1.8mm, see test_ik_round_trip_at_default_pose) -- n_refine=3 passes
    # substantially improve on a single solve but aren't guaranteed to fully
    # converge in that few iterations. 2cm is a looser bound confirming real,
    # substantial improvement, not just "any" improvement.
    assert refined_err < 2e-2, f"L position error {refined_err} too large after refinement"


def test_reset_smoothing_seeds_filter_and_init_data(ik):
    q = ARM_DEFAULT_Q.copy() + 0.1  # arbitrary, distinct from zero/default

    ik.reset_smoothing(q)

    np.testing.assert_allclose(ik.smooth_filter.filtered_data, q)
    np.testing.assert_allclose(ik.init_data, q)
    assert len(ik.smooth_filter.queue) == ik.smooth_filter.window_size
    for sample in ik.smooth_filter.queue:
        np.testing.assert_allclose(sample, q)


def test_reset_smoothing_clears_stale_queue_and_prevents_jump(ik):
    # Populate the filter with samples from a pose far from where we're
    # about to "arrive" -- mimics Phase 1a's RRT-planned execute_joint_path
    # moving the arm without ever calling solve_ik(), leaving the filter's
    # queue stale relative to the arm's real position.
    L_far, R_far = ik.fk(np.zeros(14))
    L_far_target = np.eye(4)
    L_far_target[:3, 3], L_far_target[:3, :3] = L_far.translation, L_far.rotation
    R_far_target = np.eye(4)
    R_far_target[:3, 3], R_far_target[:3, :3] = R_far.translation, R_far.rotation
    for _ in range(4):
        ik.solve_ik(L_far_target, R_far_target, current_q=np.zeros(14))

    # Simulate the arm having actually moved (via execute_joint_path, not
    # solve_ik) to ARM_DEFAULT_Q -- a real "gap" the filter never saw.
    q_arrived = ARM_DEFAULT_Q.copy()
    ik.reset_smoothing(q_arrived)

    # First solve_ik() after the reset, targeting the pose consistent with
    # where we just arrived: the filtered output should stay close to
    # q_arrived, not jump toward the stale pre-reset (zero-config) samples.
    L_pose, R_pose = ik.fk(q_arrived)
    L_target = np.eye(4)
    L_target[:3, 3], L_target[:3, :3] = L_pose.translation, L_pose.rotation
    R_target = np.eye(4)
    R_target[:3, 3], R_target[:3, :3] = R_pose.translation, R_pose.rotation

    sol_q, _ = ik.solve_ik(L_target, R_target, current_q=q_arrived)

    jump = np.linalg.norm(sol_q - q_arrived)
    assert jump < 0.1, (
        f"Expected a small post-reset delta, got a {jump:.3f} rad jump -- "
        f"reset_smoothing did not prevent blending against stale history."
    )
