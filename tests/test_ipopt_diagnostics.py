"""Hardware-free tests for G1HandIK's IPOPT max_iter-exhaustion diagnostics.

Pure observability -- solver behavior (the opti.debug.value(...) fallback)
is unchanged; these tests verify the new counter and enriched print only.
"""
import numpy as np
import pytest

from g1_spine_stabilization.kinematics.ik_solver import G1HandIK
from g1_spine_stabilization.trajectory.targets import compute_hand_targets


@pytest.fixture(scope="module")
def ik():
    """Shared IK solver -- cold start (~10s) amortized across this file's tests."""
    return G1HandIK()


_ARM_DEFAULT_Q = np.array([
    -0.07641,  0.3355,  -0.38365, -0.04522,  1.18319,  0.3721,  0.10582,
    -0.0737,  -0.3477,   0.37916, -0.03927, -1.17787,  0.37036, -0.0962,
])


def test_exhaustion_count_starts_at_zero():
    fresh_ik = G1HandIK()
    assert fresh_ik.ipopt_exhaustion_count == 0


def test_normal_solve_does_not_increment_counter(ik):
    before = ik.ipopt_exhaustion_count
    box_center = np.array([0.35, 0.0, 0.0])
    L_target, R_target = compute_hand_targets(box_center, box_half_width=0.13, theta=0.0)
    ik.solve_ik(L_target, R_target, _ARM_DEFAULT_Q.copy(), np.zeros(14))
    assert ik.ipopt_exhaustion_count == before


def _fault_after_real_solve(opti):
    """Runs the real solve (so opti.debug has valid state to fall back to),
    then raises -- simulating a post-solve failure without leaving the Opti
    stack in an unsolved state, which opti.debug.value() requires."""
    original_solve = opti.solve
    def fault_solve():
        original_solve()
        raise RuntimeError("simulated IPOPT failure")
    return fault_solve


def test_solver_exception_increments_counter_and_falls_back(ik, monkeypatch, capsys):
    monkeypatch.setattr(ik.opti, "solve", _fault_after_real_solve(ik.opti))
    before = ik.ipopt_exhaustion_count
    box_center = np.array([0.35, 0.0, 0.0])
    L_target, R_target = compute_hand_targets(box_center, box_half_width=0.13, theta=0.0)
    sol_q, sol_tauff = ik.solve_ik(L_target, R_target, _ARM_DEFAULT_Q.copy(), np.zeros(14))

    assert ik.ipopt_exhaustion_count == before + 1
    assert sol_q.shape == (14,)
    assert sol_tauff.shape == (14,)
    out = capsys.readouterr().out
    assert f"exhaustion #{before + 1}" in out


def test_counter_is_cumulative_across_calls(ik, monkeypatch):
    monkeypatch.setattr(ik.opti, "solve", _fault_after_real_solve(ik.opti))
    before = ik.ipopt_exhaustion_count
    box_center = np.array([0.35, 0.0, 0.0])
    L_target, R_target = compute_hand_targets(box_center, box_half_width=0.13, theta=0.0)
    ik.solve_ik(L_target, R_target, _ARM_DEFAULT_Q.copy(), np.zeros(14))
    ik.solve_ik(L_target, R_target, _ARM_DEFAULT_Q.copy(), np.zeros(14))
    assert ik.ipopt_exhaustion_count == before + 2
