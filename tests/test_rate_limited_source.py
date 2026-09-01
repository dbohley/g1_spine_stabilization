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
    """Same theta jump, different measured dt, must clamp to a different amount
    (both dt values here are under the default max_dt=0.5s ceiling, so the
    gap-capping fix below doesn't interfere with this)."""
    def run_with_dt(dt):
        inner = _FakeSource([[0.35, 0.0, 0.2], [0.35, 0.0, 0.2]], [0.0, 10.0])
        wrapped = RateLimitedSource(inner, max_theta_rate=1.0, max_pos_rate=1.0,
                                    clock=_FakeClock([0.0, dt]))
        wrapped.startup()
        wrapped.step()
        _, theta2 = wrapped.step()
        return theta2

    assert run_with_dt(0.02) == pytest.approx(0.02)  # tight clamp at "assumed" 50Hz dt
    assert run_with_dt(0.3) == pytest.approx(0.3)    # looser clamp at measured ~3Hz dt


def test_gap_longer_than_max_dt_does_not_inflate_the_allowed_delta():
    """Regression test for a confirmed real bug: dt is measured since the
    LAST step() call, with no assumption step() is called on a schedule.
    This codebase's controllers only start calling step() once Phase 2
    begins -- Phase 1a/1b/1.5 (RRT reach, straight-in close, hand-close)
    can combine to several real seconds with ZERO step() calls. Without
    capping dt, that gap would inflate max_theta_rate*dt far past the
    ceiling this limiter exists to enforce -- i.e. the very first step()
    after such a gap would sail through completely unclamped, exactly
    backwards from the limiter's purpose."""
    inner = _FakeSource(
        [[0.35, 0.0, 0.2], [0.35, 0.0, 0.2]],
        [0.0, 10.0],  # huge jump after a long real gap
    )
    GAP_S = 2.0  # e.g. Phase 1.5's hand-close, with no step() calls in between
    MAX_DT = 0.5
    wrapped = RateLimitedSource(inner, max_theta_rate=1.0, max_pos_rate=1.0,
                                clock=_FakeClock([0.0, GAP_S]), max_dt=MAX_DT)
    wrapped.startup()
    wrapped.step()
    _, theta2 = wrapped.step()
    # Without the cap, this would clamp to 1.0*2.0=2.0 (unclamped -- 10.0
    # itself is under that). WITH the cap, dt is capped to MAX_DT=0.5 ->
    # max_dtheta = 1.0*0.5 = 0.5, correctly still clamping the jump.
    assert theta2 == pytest.approx(MAX_DT)


def test_max_dt_default_does_not_affect_normal_fast_polling():
    """A normal, fast-polling loop's real inter-step gaps stay far under the
    default max_dt=0.5s ceiling, so behavior for realistic control rates is
    completely unchanged by this fix."""
    inner = _FakeSource(
        [[0.35, 0.0, 0.2], [0.35, 0.0, 0.2]],
        [0.0, 10.0],
    )
    wrapped = RateLimitedSource(inner, max_theta_rate=1.0, max_pos_rate=1.0,
                                clock=_FakeClock([0.0, 0.1]))  # 0.1s, default max_dt=0.5s
    wrapped.startup()
    wrapped.step()
    _, theta2 = wrapped.step()
    assert theta2 == pytest.approx(0.1)  # unaffected by the (much larger) default ceiling


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
