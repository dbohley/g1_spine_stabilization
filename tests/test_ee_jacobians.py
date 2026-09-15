"""Tests for G1HandIK.ee_jacobians — the EE Jacobians used by the Jacobian
force controller (tau = J^T W).

Verifies the Jacobian against a finite-difference of FK (linear + angular), that
each arm's Jacobian is structurally independent of the other arm's joints, and
that a single-arm wrench therefore loads only that arm's joints.
"""
import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from g1_spine_stabilization.kinematics.ik_solver import G1HandIK
from g1_spine_stabilization.trajectory.targets import compute_press_wrenches


@pytest.fixture(scope="module")
def ik():
    """Shared IK solver -- cold start (~10s) amortized across this file."""
    return G1HandIK(gravity_comp_enabled=True)


def _fk(ik, q):
    L, R = ik.fk(q)
    return (np.array(L.translation), np.array(L.rotation),
            np.array(R.translation), np.array(R.rotation))


def _log3(R):
    return Rotation.from_matrix(R).as_rotvec()


def test_jacobian_matches_central_difference(ik):
    """Column-by-column: J[:3,j] ≈ dp/dq_j and J[3:,j] ≈ world-frame drot/dq_j."""
    rng = np.random.default_rng(0)
    q = 0.2 * rng.standard_normal(14)
    J_L, J_R = ik.ee_jacobians(q)
    h = 1e-6
    for j in range(14):
        dq = np.zeros(14)
        dq[j] = h
        Lp_t, Lp_R, Rp_t, Rp_R = _fk(ik, q + dq)
        Lm_t, Lm_R, Rm_t, Rm_R = _fk(ik, q - dq)

        dL_lin = (Lp_t - Lm_t) / (2 * h)
        dR_lin = (Rp_t - Rm_t) / (2 * h)
        dL_ang = _log3(Lp_R @ Lm_R.T) / (2 * h)   # LOCAL_WORLD_ALIGNED -> world-frame omega
        dR_ang = _log3(Rp_R @ Rm_R.T) / (2 * h)

        np.testing.assert_allclose(J_L[:3, j], dL_lin, atol=1e-4)
        np.testing.assert_allclose(J_L[3:, j], dL_ang, atol=1e-4)
        np.testing.assert_allclose(J_R[:3, j], dR_lin, atol=1e-4)
        np.testing.assert_allclose(J_R[3:, j], dR_ang, atol=1e-4)


def test_offarm_columns_are_structurally_zero(ik):
    """Left EE cannot be moved by right-arm joints, and vice versa."""
    q = 0.15 * np.random.default_rng(1).standard_normal(14)
    J_L, J_R = ik.ee_jacobians(q)
    assert np.max(np.abs(J_L[:, 7:14])) < 1e-10   # left J independent of right joints
    assert np.max(np.abs(J_R[:, 0:7])) < 1e-10    # right J independent of left joints


def test_single_arm_wrench_loads_only_its_arm(ik):
    """tau_L = J_L^T W_L has ~0 torque on the right joints (and symmetrically)."""
    q = np.zeros(14)
    J_L, J_R = ik.ee_jacobians(q)
    W_L, W_R = compute_press_wrenches(theta=0.0, F_cmd=5.0)
    tau_L = J_L.T @ W_L
    tau_R = J_R.T @ W_R
    assert np.max(np.abs(tau_L[7:14])) < 1e-9
    assert np.max(np.abs(tau_R[0:7])) < 1e-9
    # and the loaded halves are non-trivial (the wrench actually does something)
    assert np.max(np.abs(tau_L[0:7])) > 1e-3
    assert np.max(np.abs(tau_R[7:14])) > 1e-3


def test_jacobian_shape(ik):
    J_L, J_R = ik.ee_jacobians(np.zeros(14))
    assert J_L.shape == (6, 14)
    assert J_R.shape == (6, 14)
