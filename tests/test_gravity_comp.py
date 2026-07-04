"""
SDK-5.5: Gravity compensation tests (hardware-free).

All tests exercise the Pinocchio model correction path only — no robot
connection, no DDS, no camera.
"""

import numpy as np
import pytest
import pinocchio as pin

from g1_spine_stabilization.kinematics.ik_solver import (
    G1HandIK,
    INSPIRE_HAND_MASS,
    _WRIST_MASS,
    _make_composite_wrist_inertia,
)
from g1_spine_stabilization.trajectory.targets import compute_hand_targets
from g1_spine_stabilization.utils.config import load_config


@pytest.fixture(scope="module")
def ik_comp():
    """G1HandIK with gravity comp enabled (corrected 1.0 kg hand inertia)."""
    return G1HandIK(gravity_comp_enabled=True)


@pytest.fixture(scope="module")
def ik_no_comp():
    """G1HandIK with gravity comp disabled (URDF lumped inertia unchanged)."""
    return G1HandIK(gravity_comp_enabled=False)


# Default arm pose used for torque assertion tests
_ARM_DEFAULT_Q = np.array([
    -0.07641,  0.3355,  -0.38365, -0.04522,  1.18319,  0.3721,  0.10582,
    -0.0737,  -0.3477,   0.37916, -0.03927, -1.17787,  0.37036, -0.0962,
])


def test_composite_wrist_mass(ik_comp):
    """Wrist body mass must equal bare wrist + 1.0 kg hand after correction."""
    expected = _WRIST_MASS + INSPIRE_HAND_MASS  # ~1.0846 kg
    rm = ik_comp.reduced.model
    for jname in ("left_wrist_yaw_joint", "right_wrist_yaw_joint"):
        jid = rm.getJointId(jname)
        actual = rm.inertias[jid].mass
        assert abs(actual - expected) < 1e-4, (
            f"{jname}: expected {expected:.4f} kg, got {actual:.4f} kg"
        )


def test_gravity_vector_matches_potential_energy_gradient(ik_comp):
    """
    computeGeneralizedGravity must equal the numerical gradient of
    computePotentialEnergy at several random configurations.
    A wrong gravity direction or inertia frame convention fails this immediately.
    """
    rng = np.random.default_rng(42)
    rm = ik_comp.reduced.model
    rd = ik_comp.reduced.data
    eps = 1e-6

    for _ in range(5):
        q = pin.randomConfiguration(rm)
        g_analytical = pin.computeGeneralizedGravity(rm, rd, q).copy()

        g_numerical = np.zeros(rm.nv)
        for i in range(rm.nv):
            q_plus  = q.copy(); q_plus[i]  += eps
            q_minus = q.copy(); q_minus[i] -= eps
            E_plus  = pin.computePotentialEnergy(rm, rd, q_plus)
            E_minus = pin.computePotentialEnergy(rm, rd, q_minus)
            g_numerical[i] = (E_plus - E_minus) / (2 * eps)

        np.testing.assert_allclose(
            g_analytical, g_numerical, atol=1e-4,
            err_msg="Gravity vector does not match PE gradient — check inertia frame"
        )


def test_shoulder_pitch_torque_meaningful(ik_comp):
    """
    With 1.0 kg hands at ARM_DEFAULT_Q (arms extended), shoulder_pitch joints
    must carry significant gravity torque (> 2 N·m each).
    Indices 0 = left_shoulder_pitch, 7 = right_shoulder_pitch in 14-DOF order.
    """
    rm = ik_comp.reduced.model
    rd = ik_comp.reduced.data
    q = _ARM_DEFAULT_Q.copy()
    tau = pin.computeGeneralizedGravity(rm, rd, q)
    assert abs(tau[0]) > 2.0, (
        f"L shoulder_pitch gravity torque too small: {tau[0]:.3f} N·m (expected > 2)"
    )
    assert abs(tau[7]) > 2.0, (
        f"R shoulder_pitch gravity torque too small: {tau[7]:.3f} N·m (expected > 2)"
    )


def test_gravity_comp_disabled_leaves_mass_unchanged(ik_no_comp):
    """With comp disabled, wrist body mass stays at the lumped URDF value (~0.276 kg)."""
    rm = ik_no_comp.reduced.model
    for jname in ("left_wrist_yaw_joint", "right_wrist_yaw_joint"):
        jid = rm.getJointId(jname)
        actual = rm.inertias[jid].mass
        # Lumped URDF: wrist (0.085) + hand_base (0.141) + fingers (~0.05) ≈ 0.276 kg
        # Must be clearly less than the corrected value (1.0846 kg)
        assert actual < 0.5, (
            f"{jname}: gravity_comp=False but mass={actual:.4f} kg looks corrected"
        )


def test_solve_ik_returns_nonzero_tauff(ik_comp):
    """solve_ik must return a non-trivial sol_tauff when gravity comp is active."""
    box_center = np.array([0.35, 0.0, 0.0])
    L_target, R_target = compute_hand_targets(box_center, box_half_width=0.13, theta=0.0)
    q0  = _ARM_DEFAULT_Q.copy()
    dq0 = np.zeros(14)
    _, sol_tauff = ik_comp.solve_ik(L_target, R_target, q0, dq0)
    assert sol_tauff is not None
    assert np.linalg.norm(sol_tauff) > 1.0, (
        f"sol_tauff norm {np.linalg.norm(sol_tauff):.3f} is suspiciously small"
    )
