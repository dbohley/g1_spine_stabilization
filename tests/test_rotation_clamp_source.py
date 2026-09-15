"""Hardware-free unit tests for RotationClampSource (safety clamp on the
tracked target rotation). See docs/six_dof_tracking_fix.md.
"""
import numpy as np
import pytest

from g1_spine_stabilization.trajectory.sources import RateLimitedSource, RotationClampSource
from g1_spine_stabilization.utils.transforms import Rx


class _FakeSource:
    """Returns a pre-programmed sequence of (box_center, theta) pairs."""

    def __init__(self, box_centers, thetas):
        self._box_centers = [np.asarray(b, dtype=float) for b in box_centers]
        self._thetas = list(thetas)
        self._i = 0

    def startup(self):
        return self._box_centers[0].copy()

    def step(self):
        bc, th = self._box_centers[self._i], self._thetas[self._i]
        self._i += 1
        return bc.copy(), th


def test_unconfigured_is_identity_passthrough():
    inner = _FakeSource([[0.0, 0.0, 0.0]], [Rx(1.0)])
    wrapped = RotationClampSource(inner)
    wrapped.startup()
    _, R = wrapped.step()
    np.testing.assert_allclose(R, Rx(1.0), atol=1e-12)


def test_scalar_input_is_converted_and_clamped():
    inner = _FakeSource([[0.0, 0.0, 0.0]], [1.0])   # scalar theta in
    wrapped = RotationClampSource(inner, max_angle=0.2)
    wrapped.startup()
    _, R = wrapped.step()
    np.testing.assert_allclose(R, Rx(0.2), atol=1e-9)


def test_angle_clamped_when_over_max():
    inner = _FakeSource([[0.0, 0.0, 0.0]], [Rx(1.0)])
    wrapped = RotationClampSource(inner, max_angle=0.3)
    wrapped.startup()
    _, R = wrapped.step()
    np.testing.assert_allclose(R, Rx(0.3), atol=1e-9)


def test_passes_through_unclamped_when_under_max():
    inner = _FakeSource([[0.0, 0.0, 0.0]], [Rx(0.1)])
    wrapped = RotationClampSource(inner, max_angle=0.3)
    wrapped.startup()
    _, R = wrapped.step()
    np.testing.assert_allclose(R, Rx(0.1), atol=1e-9)


def test_disabled_axis_kills_response_on_that_axis():
    from scipy.spatial.transform import Rotation
    R_yaw = Rotation.from_euler('z', 0.5).as_matrix()
    inner = _FakeSource([[0.0, 0.0, 0.0]], [R_yaw])
    wrapped = RotationClampSource(inner, enabled_axes=[True, False, False])
    wrapped.startup()
    _, R = wrapped.step()
    np.testing.assert_allclose(R, np.eye(3), atol=1e-9)


def test_getattr_proxies_to_inner():
    class _Inner(_FakeSource):
        @property
        def object_lost(self):
            return True

    inner = _Inner([[0.0, 0.0, 0.0]], [Rx(0.1)])
    wrapped = RotationClampSource(inner, max_angle=0.3)
    assert wrapped.object_lost is True


def test_clamp_wins_over_rate_limiters_unclamped_first_step():
    """The safety property this wrapper exists for: RateLimitedSource's
    first step() after startup() is deliberately unclamped (a fresh seed),
    so a huge accumulated rotation would otherwise pass straight through on
    the very first tick. RotationClampSource, wired outermost, must still
    catch it."""
    R_huge = Rx(2.5)  # a large, in-range rotation
    inner = _FakeSource([[0.0, 0.0, 0.0]], [R_huge])
    rate_limited = RateLimitedSource(inner, max_theta_rate=0.01, max_pos_rate=0.01)
    clamped = RotationClampSource(rate_limited, max_angle=0.2)
    clamped.startup()
    _, R = clamped.step()
    assert R is not None
    np.testing.assert_allclose(R, Rx(0.2), atol=1e-9)
