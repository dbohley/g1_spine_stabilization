"""Hardware-free unit tests for RateLimitedSource (target-level slew limiter).

No IK, no SDK, no camera -- a fake injectable-clock + fake inner source
exercise the clamp logic in isolation.
"""
import numpy as np
import pytest

from g1_spine_stabilization.trajectory.sources import RateLimitedSource


class _FakeClock:
    """Returns a pre-programmed sequence of monotonic timestamps."""

    def __init__(self, times):
        self._times = list(times)
        self._i = 0

    def __call__(self):
        t = self._times[self._i]
        self._i += 1
        return t


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


def test_startup_returns_inner_value_unchanged():
    inner = _FakeSource([[0.35, 0.1, 0.2]], [0.0])
    wrapped = RateLimitedSource(inner, max_theta_rate=1.0, max_pos_rate=1.0)
    np.testing.assert_allclose(wrapped.startup(), [0.35, 0.1, 0.2])


def test_first_step_passes_through_unclamped_even_on_huge_jump():
    inner = _FakeSource([[0.35, 0.0, 0.2]], [10.0])  # huge theta on the very first step
    wrapped = RateLimitedSource(inner, max_theta_rate=0.1, max_pos_rate=0.1,
                                clock=_FakeClock([0.0]))
    wrapped.startup()
    _, theta = wrapped.step()
    assert theta == pytest.approx(10.0)


def test_theta_within_limit_passes_through_unchanged():
    inner = _FakeSource(
        [[0.35, 0.0, 0.2], [0.35, 0.0, 0.2]],
        [0.0, 0.005],  # small delta over dt=0.1s with max_rate=1.0 (max_dtheta=0.1)
    )
    wrapped = RateLimitedSource(inner, max_theta_rate=1.0, max_pos_rate=1.0,
                                clock=_FakeClock([0.0, 0.1]))
    wrapped.startup()
    wrapped.step()
    _, theta2 = wrapped.step()
    assert theta2 == pytest.approx(0.005)


def test_theta_clamped_when_jump_exceeds_rate_limit():
    inner = _FakeSource(
        [[0.35, 0.0, 0.2], [0.35, 0.0, 0.2]],
        [0.0, 10.0],  # huge jump on the second step
    )
    wrapped = RateLimitedSource(inner, max_theta_rate=1.0, max_pos_rate=1.0,
                                clock=_FakeClock([0.0, 0.1]))
    wrapped.startup()
    wrapped.step()
    _, theta2 = wrapped.step()
    assert theta2 == pytest.approx(0.1)  # clamped to max_theta_rate * dt


def test_box_center_clamped_preserving_direction():
    a = [0.35, 0.0, 0.2]
    b = [0.35, 0.4, 0.2]  # 0.4m jump in +y
    inner = _FakeSource([a, b], [0.0, 0.0])
    wrapped = RateLimitedSource(inner, max_theta_rate=100.0, max_pos_rate=0.1,
                                clock=_FakeClock([0.0, 0.1]))
    wrapped.startup()
    wrapped.step()
    bc2, _ = wrapped.step()
    np.testing.assert_allclose(bc2, [0.35, 0.01, 0.2], atol=1e-9)  # 0.1 m/s * 0.1s


def test_box_center_within_limit_passes_through_unchanged():
    a = [0.35, 0.0, 0.2]
    b = [0.35, 0.005, 0.2]  # small delta, well under the 0.1 m/s * 0.1s = 0.01m budget
    inner = _FakeSource([a, b], [0.0, 0.0])
    wrapped = RateLimitedSource(inner, max_theta_rate=100.0, max_pos_rate=0.1,
                                clock=_FakeClock([0.0, 0.1]))
    wrapped.startup()
    wrapped.step()
    bc2, _ = wrapped.step()
    np.testing.assert_allclose(bc2, b, atol=1e-9)


def test_uses_measured_dt_not_a_fixed_control_dt():
    """Same theta jump, different measured dt, must clamp to a different amount."""
    def run_with_dt(dt):
        inner = _FakeSource([[0.35, 0.0, 0.2], [0.35, 0.0, 0.2]], [0.0, 10.0])
        wrapped = RateLimitedSource(inner, max_theta_rate=1.0, max_pos_rate=1.0,
                                    clock=_FakeClock([0.0, dt]))
        wrapped.startup()
        wrapped.step()
        _, theta2 = wrapped.step()
        return theta2

    assert run_with_dt(0.02) == pytest.approx(0.02)  # tight clamp at "assumed" 50Hz dt
    assert run_with_dt(1.0) == pytest.approx(1.0)    # looser clamp at measured 1Hz dt


def test_getattr_proxies_unknown_attributes_to_inner():
    class _Inner(_FakeSource):
        @property
        def object_lost(self):
            return True

    inner = _Inner([[0.0, 0.0, 0.0]], [0.0])
    wrapped = RateLimitedSource(inner, max_theta_rate=1.0, max_pos_rate=1.0)
    assert wrapped.object_lost is True


def test_restarting_via_startup_reseeds_unclamped_state():
    inner = _FakeSource(
        [[0.35, 0.0, 0.2], [0.35, 0.0, 0.2]],
        [0.0, 10.0],
    )
    wrapped = RateLimitedSource(inner, max_theta_rate=0.1, max_pos_rate=1.0,
                                clock=_FakeClock([5.0]))
    wrapped.startup()
    _, theta = wrapped.step()  # first call after a fresh startup(): unclamped
    assert theta == pytest.approx(0.0)
