"""Hardware-free tests verifying that a Ctrl-C during opti.solve() propagates
as a real KeyboardInterrupt instead of being swallowed as an IPOPT exhaustion.

CasADi's Ipopt interface checks for a pending SIGINT during long solves and
aborts by raising a C++ KeyboardInterruptException, which crosses into Python
as a plain Exception (not a real KeyboardInterrupt -- that type is a
BaseException, so it was never caught by solve_ik()'s `except Exception`
in the first place) whose message contains the literal solver return_status
'NonIpopt_Exception_Thrown'. Confirmed against a real interrupted live-hardware
run where the control loop repeatedly printed "IK warning (exhaustion #N)"
instead of stopping on Ctrl-C.
"""
import numpy as np
import pytest

from g1_spine_stabilization.kinematics.ik_solver import G1HandIK
from g1_spine_stabilization.trajectory.targets import compute_hand_targets

_ARM_DEFAULT_Q = np.array([
    -0.07641,  0.3355,  -0.38365, -0.04522,  1.18319,  0.3721,  0.10582,
    -0.0737,  -0.3477,   0.37916, -0.03927, -1.17787,  0.37036, -0.0962,
])


@pytest.fixture(scope="module")
def ik():
    """Shared IK solver -- cold start (~10s) amortized across this file's tests."""
    return G1HandIK()


def _fault_solve(opti, message):
    """Runs the real solve (so opti.debug has valid state), then raises with
    the given message -- mirrors test_ipopt_diagnostics.py's
    _fault_after_real_solve helper."""
    original_solve = opti.solve
    def fault_solve():
        original_solve()
        raise RuntimeError(message)
    return fault_solve


def _targets():
    box_center = np.array([0.35, 0.0, 0.0])
    return compute_hand_targets(box_center, box_half_width=0.13, theta=0.0)


def test_interrupted_solve_raises_keyboard_interrupt(ik, monkeypatch):
    monkeypatch.setattr(
        ik.opti, "solve",
        _fault_solve(
            ik.opti,
            "Error in Opti::solve [OptiNode] ... Solver failed. "
            "return_status is 'NonIpopt_Exception_Thrown'",
        ),
    )
    L_target, R_target = _targets()
    with pytest.raises(KeyboardInterrupt):
        ik.solve_ik(L_target, R_target, _ARM_DEFAULT_Q.copy(), np.zeros(14))


def test_interrupted_solve_does_not_increment_exhaustion_count(ik, monkeypatch):
    monkeypatch.setattr(
        ik.opti, "solve",
        _fault_solve(
            ik.opti,
            "Error in Opti::solve [OptiNode] ... Solver failed. "
            "return_status is 'NonIpopt_Exception_Thrown'",
        ),
    )
    before = ik.ipopt_exhaustion_count
    L_target, R_target = _targets()
    with pytest.raises(KeyboardInterrupt):
        ik.solve_ik(L_target, R_target, _ARM_DEFAULT_Q.copy(), np.zeros(14))
    assert ik.ipopt_exhaustion_count == before


def test_genuine_solver_failure_still_increments_exhaustion_count(ik, monkeypatch):
    """Regression guard: only the interrupt signature is special-cased --
    ordinary solver failures must keep falling back to opti.debug.value(),
    unchanged from before this fix."""
    monkeypatch.setattr(
        ik.opti, "solve",
        _fault_solve(ik.opti, "simulated ordinary IPOPT failure"),
    )
    before = ik.ipopt_exhaustion_count
    L_target, R_target = _targets()
    sol_q, sol_tauff = ik.solve_ik(L_target, R_target, _ARM_DEFAULT_Q.copy(), np.zeros(14))
    assert ik.ipopt_exhaustion_count == before + 1
    assert sol_q.shape == (14,)
    assert sol_tauff.shape == (14,)
