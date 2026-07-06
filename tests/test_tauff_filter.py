"""Hardware-free tests for ExponentialMovingAverageFilter and its sol_tauff wiring.

Covers: the isolated EMA filter class, and an integration check that
G1HandIK's tauff smoothing actually reduces the per-call torque step
compared to an unfiltered instance solving the same target sequence.
"""
import numpy as np
import pytest

from g1_spine_stabilization.kinematics.ik_solver import ExponentialMovingAverageFilter, G1HandIK
from g1_spine_stabilization.trajectory.targets import compute_hand_targets


_ARM_DEFAULT_Q = np.array([
    -0.07641,  0.3355,  -0.38365, -0.04522,  1.18319,  0.3721,  0.10582,
    -0.0737,  -0.3477,   0.37916, -0.03927, -1.17787,  0.37036, -0.0962,
])


# ---------------------------------------------------------------------
# Isolated ExponentialMovingAverageFilter tests
# ---------------------------------------------------------------------

def test_first_call_passes_through_unchanged():
    f = ExponentialMovingAverageFilter(alpha=0.3, data_size=1)
    f.add_data([5.0])
    assert f.filtered_data == pytest.approx([5.0])


def test_blend_arithmetic_exact_value():
    f = ExponentialMovingAverageFilter(alpha=0.5, data_size=1)
    f.add_data([0.0])
    f.add_data([10.0])
    # y = 0.5*10 + 0.5*0 = 5.0
    assert f.filtered_data == pytest.approx([5.0])
    f.add_data([10.0])
    # y = 0.5*10 + 0.5*5 = 7.5
    assert f.filtered_data == pytest.approx([7.5])


def test_alpha_one_is_always_passthrough():
    f = ExponentialMovingAverageFilter(alpha=1.0, data_size=1)
    f.add_data([1.0])
    assert f.filtered_data == pytest.approx([1.0])
    f.add_data([99.0])
    assert f.filtered_data == pytest.approx([99.0])


def test_constant_input_converges_to_that_constant():
    f = ExponentialMovingAverageFilter(alpha=0.3, data_size=1)
    for _ in range(50):
        f.add_data([4.0])
    assert f.filtered_data == pytest.approx([4.0], abs=1e-6)


def test_constructor_rejects_out_of_range_alpha():
    with pytest.raises(AssertionError):
        ExponentialMovingAverageFilter(alpha=0.0)
    with pytest.raises(AssertionError):
        ExponentialMovingAverageFilter(alpha=1.5)


# ---------------------------------------------------------------------
# Integration: filtered vs unfiltered G1HandIK tauff step size
# ---------------------------------------------------------------------

@pytest.fixture(scope="module")
def ik_tauff_filtered():
    return G1HandIK(tauff_filter_enabled=True, tauff_filter_alpha=0.3)


@pytest.fixture(scope="module")
def ik_tauff_unfiltered():
    return G1HandIK(tauff_filter_enabled=False)


def _solve_two_step_sequence(ik_instance):
    box_center = np.array([0.35, 0.0, 0.0])
    L_a, R_a = compute_hand_targets(box_center, box_half_width=0.13, theta=0.0)
    L_b, R_b = compute_hand_targets(box_center, box_half_width=0.13, theta=0.3)

    q0 = _ARM_DEFAULT_Q.copy()
    sol_q_a, tauff_a = ik_instance.solve_ik(L_a, R_a, q0, np.zeros(14))
    sol_q_b, tauff_b = ik_instance.solve_ik(L_b, R_b, sol_q_a, np.zeros(14))
    return tauff_a, tauff_b


def test_first_call_identical_between_filtered_and_unfiltered(ik_tauff_filtered, ik_tauff_unfiltered):
    """EMA's first call passes through unchanged, so these module-scoped
    instances' very first solve must match exactly. Must run before any other
    test in this module calls .solve_ik() on either fixture."""
    box_center = np.array([0.35, 0.0, 0.0])
    L_a, R_a = compute_hand_targets(box_center, box_half_width=0.13, theta=0.0)
    _, tauff_filtered_first = ik_tauff_filtered.solve_ik(L_a, R_a, _ARM_DEFAULT_Q.copy(), np.zeros(14))
    _, tauff_unfiltered_first = ik_tauff_unfiltered.solve_ik(L_a, R_a, _ARM_DEFAULT_Q.copy(), np.zeros(14))
    np.testing.assert_allclose(tauff_filtered_first, tauff_unfiltered_first, atol=1e-9)


def test_filtered_tauff_step_is_smaller_than_unfiltered(ik_tauff_filtered, ik_tauff_unfiltered):
    tauff_a_f, tauff_b_f = _solve_two_step_sequence(ik_tauff_filtered)
    tauff_a_u, tauff_b_u = _solve_two_step_sequence(ik_tauff_unfiltered)

    step_filtered = np.linalg.norm(tauff_b_f - tauff_a_f)
    step_unfiltered = np.linalg.norm(tauff_b_u - tauff_a_u)
    assert step_filtered < step_unfiltered
