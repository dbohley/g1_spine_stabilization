"""Hardware-free unit tests for NoiseFilteredSource (real-marker noise filter).

No IK, no SDK, no camera -- a fake inner source exercises the median+EMA
logic in isolation, mirroring tests/test_rate_limited_source.py's pattern.
"""
import numpy as np
import pytest

from g1_spine_stabilization.trajectory.sources import NoiseFilteredSource, RateLimitedSource


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


def test_startup_resets_internal_state():
    inner = _FakeSource([[0.0, 0.0, 0.0]] * 6, [0.0, 0.0, 0.0, 0.0, 0.0, 5.0])
    wrapped = NoiseFilteredSource(inner, median_window=3, ema_alpha=0.5)
    wrapped.startup()
    for _ in range(3):
        wrapped.step()
    wrapped.startup()
    # After a fresh startup(), history/EMA state must not leak -- the next
    # step() should behave as a first-ever call (median passthrough, EMA seed).
    inner2 = _FakeSource([[1.0, 2.0, 3.0]], [7.0])
    wrapped2 = NoiseFilteredSource(inner2, median_window=3, ema_alpha=0.5)
    wrapped2.startup()
    box, theta = wrapped2.step()
    np.testing.assert_allclose(box, [1.0, 2.0, 3.0])
    assert theta == pytest.approx(7.0)


def test_median_rejects_single_frame_outlier():
    # 5 warmup zeros fill the window first, THEN one spike, then more zeros --
    # the spike must land after the window is already full to be testable.
    thetas = [0.0, 0.0, 0.0, 0.0, 0.0, 5.0, 0.0, 0.0, 0.0, 0.0]
    inner = _FakeSource([[0.0, 0.0, 0.0]] * len(thetas), thetas)
    wrapped = NoiseFilteredSource(inner, median_window=5, ema_alpha=1.0)  # ema_alpha=1 isolates median stage
    wrapped.startup()
    outputs = [wrapped.step()[1] for _ in thetas]
    assert max(abs(o) for o in outputs) < 1.0, (
        f"median-of-5 should reject the single 5.0 outlier, got {outputs}"
    )


def test_exact_median_of_three_hand_computed():
    thetas = [1.0, 5.0, 2.0]
    inner = _FakeSource([[0.0, 0.0, 0.0]] * 3, thetas)
    wrapped = NoiseFilteredSource(inner, median_window=3, ema_alpha=1.0)  # ema_alpha=1 isolates median stage
    wrapped.startup()
    # step 1: window=[1.0], below window size -> passthrough
    _, t1 = wrapped.step()
    assert t1 == pytest.approx(1.0)
    # step 2: window=[1.0, 5.0], below window size -> passthrough
    _, t2 = wrapped.step()
    assert t2 == pytest.approx(5.0)
    # step 3: window=[1.0, 5.0, 2.0], full -> median = 2.0
    _, t3 = wrapped.step()
    assert t3 == pytest.approx(2.0)


def test_ema_reduces_variance_of_small_jitter():
    rng = np.random.default_rng(42)
    n = 200
    thetas = [float(rng.normal(0.0, 0.05)) for _ in range(n)]
    inner = _FakeSource([[0.0, 0.0, 0.0]] * n, thetas)
    # median_window=1 isolates the EMA stage
    wrapped = NoiseFilteredSource(inner, median_window=1, ema_alpha=0.1)
    wrapped.startup()
    outputs = [wrapped.step()[1] for _ in thetas]
    assert np.var(outputs) < np.var(thetas)


def test_median_window_one_and_alpha_one_is_identity_passthrough():
    thetas = [1.0, -3.0, 2.5, 0.0]
    box_centers = [[0.1, 0.2, 0.3], [0.4, -0.1, 0.0], [1.0, 1.0, 1.0], [0.0, 0.0, 0.0]]
    inner = _FakeSource(box_centers, thetas)
    wrapped = NoiseFilteredSource(inner, median_window=1, ema_alpha=1.0)
    wrapped.startup()
    for bc, th in zip(box_centers, thetas):
        out_bc, out_th = wrapped.step()
        np.testing.assert_allclose(out_bc, bc)
        assert out_th == pytest.approx(th)


def test_constructor_rejects_even_or_nonpositive_window():
    inner = _FakeSource([[0.0, 0.0, 0.0]], [0.0])
    with pytest.raises(ValueError):
        NoiseFilteredSource(inner, median_window=4)
    with pytest.raises(ValueError):
        NoiseFilteredSource(inner, median_window=0)
    with pytest.raises(ValueError):
        NoiseFilteredSource(inner, median_window=-1)


def test_getattr_proxies_to_inner():
    class _Inner(_FakeSource):
        @property
        def object_lost(self):
            return True

    inner = _Inner([[0.0, 0.0, 0.0]], [0.0])
    wrapped = NoiseFilteredSource(inner, median_window=3, ema_alpha=0.5)
    assert wrapped.object_lost is True


def test_composes_correctly_inside_rate_limited_source():
    class _Inner(_FakeSource):
        @property
        def object_lost(self):
            return True

    inner = _Inner([[0.35, 0.0, 0.2]] * 3, [0.0, 0.01, 0.02])
    noise_filtered = NoiseFilteredSource(inner, median_window=3, ema_alpha=0.5)
    rate_limited = RateLimitedSource(noise_filtered, max_theta_rate=1.0, max_pos_rate=1.0)
    rate_limited.startup()
    for _ in range(3):
        rate_limited.step()
    assert rate_limited.object_lost is True
