"""Hardware-free tests for WeightedMovingFilter (regression: reversed recency weighting bug).

The bug: self.queue is FIFO (oldest at index 0), so zip(weights, queue)
paired weights[0]=0.4 (the largest weight) with the OLDEST sample instead of
the newest -- backwards from the conventional "weighted moving average" and
from the vendored SDK's own reference implementation using the identical
weight vector (~/g1_inspire_sdk/g1_inspire_sdk/_common/weighted_moving_filter.py,
which uses np.convolve(..., mode="valid") -- convolution flips the kernel,
giving the newest sample the largest weight). Fixed by zipping against
reversed(queue).
"""
import numpy as np
import pytest

from g1_spine_stabilization.kinematics.ik_solver import WeightedMovingFilter


def test_partial_window_passes_through_last_sample_unchanged():
    """Below window_size, filtered_data is just the most recent sample (unaffected by the fix)."""
    f = WeightedMovingFilter([0.4, 0.3, 0.2, 0.1], data_size=1)
    f.add_data([5.0])
    assert f.filtered_data == pytest.approx([5.0])
    f.add_data([7.0])
    assert f.filtered_data == pytest.approx([7.0])


def test_filled_window_weights_newest_sample_heaviest():
    """The money assertion: buggy code gives 1.0 here, fixed code gives 2.0."""
    f = WeightedMovingFilter([0.4, 0.3, 0.2, 0.1], data_size=1)
    for x in [0.0, 1.0, 2.0, 3.0]:
        f.add_data([x])
    # newest=3 (weight 0.4), then 2 (0.3), 1 (0.2), oldest=0 (0.1)
    expected = 0.4 * 3.0 + 0.3 * 2.0 + 0.2 * 1.0 + 0.1 * 0.0
    assert f.filtered_data == pytest.approx([expected])
    assert expected == pytest.approx(2.0)


def test_slides_forward_recency_weighting_persists():
    f = WeightedMovingFilter([0.4, 0.3, 0.2, 0.1], data_size=1)
    for x in [0.0, 1.0, 2.0, 3.0, 4.0]:
        f.add_data([x])
    # window is now [1,2,3,4], newest=4
    expected = 0.4 * 4.0 + 0.3 * 3.0 + 0.2 * 2.0 + 0.1 * 1.0
    assert f.filtered_data == pytest.approx([expected])
    assert expected == pytest.approx(3.0)


def test_group_delay_is_one_sample_not_two():
    """Steady-state group delay on a ramp must be 1 sample (fixed), not 2 (buggy)."""
    f = WeightedMovingFilter([0.4, 0.3, 0.2, 0.1], data_size=1)
    for n in range(20):
        f.add_data([float(n)])
    n = 19
    assert f.filtered_data == pytest.approx([n - 1.0])


def test_multi_dof_shape_14():
    """Per-column weighting is correct for the real 14-DOF joint-vector shape."""
    f = WeightedMovingFilter([0.4, 0.3, 0.2, 0.1], data_size=14)
    samples = [np.full(14, float(i)) for i in range(4)]
    for s in samples:
        f.add_data(s)
    expected = 0.4 * samples[3] + 0.3 * samples[2] + 0.2 * samples[1] + 0.1 * samples[0]
    np.testing.assert_allclose(f.filtered_data, expected)
