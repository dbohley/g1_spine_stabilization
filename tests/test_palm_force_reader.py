"""Unit tests for the hardware-free parts of the palm-force sensing layer.

Covers the `palm_force_newtons` reduction (baseline subtraction, counts->N,
peak, active count) and PalmForceReader's dry-run contract + side-combination
logic. The live DDS-subscribe path is not unit-tested (needs hardware); dry-run
is exercised here, and the combine logic is exercised by injecting matrices
directly into the reader's latest-sample buffer.
"""

import numpy as np
import pytest

from g1_spine_stabilization.controllers.palm_force import (
    PALM_SHAPE,
    PalmForceReader,
    palm_force_newtons,
)


# ---------------------------------------------------------------------------
# palm_force_newtons reduction
# ---------------------------------------------------------------------------

def test_reduction_baseline_subtraction_and_scale():
    palm = np.full(PALM_SHAPE, 100.0)
    baseline = np.full(PALM_SHAPE, 40.0)
    # 112 cells * (100-40)=60 -> 6720 counts; /1000 counts_per_newton -> 6.72 N
    f, peak, active = palm_force_newtons(palm, baseline, counts_per_newton=1000.0, noise_threshold=0.0)
    assert f == pytest.approx(112 * 60 / 1000.0)
    assert peak == pytest.approx(60.0)
    assert active == 112


def test_reduction_clips_below_baseline_to_zero():
    palm = np.full(PALM_SHAPE, 10.0)
    baseline = np.full(PALM_SHAPE, 40.0)  # palm below rest -> clipped to 0
    f, peak, active = palm_force_newtons(palm, baseline, counts_per_newton=1.0, noise_threshold=0.0)
    assert f == 0.0
    assert peak == 0.0
    assert active == 0


def test_reduction_active_threshold():
    palm = np.full(PALM_SHAPE, 40.0)
    palm[0, 0] = 200.0  # one strong cell (160 over baseline)
    baseline = np.full(PALM_SHAPE, 40.0)
    f, peak, active = palm_force_newtons(palm, baseline, counts_per_newton=1.0, noise_threshold=10.0)
    assert peak == pytest.approx(160.0)
    assert active == 1  # only the one cell exceeds the 10-count threshold
    assert f == pytest.approx(160.0)


# ---------------------------------------------------------------------------
# PalmForceReader dry-run contract
# ---------------------------------------------------------------------------

def test_dry_run_returns_zero_and_no_subscribers():
    r = PalmForceReader(["left", "right"], dry_run=True, counts_per_newton=1000.0)
    assert r.force("left") == 0.0
    assert r.force("right") == 0.0
    assert r.peak_counts("left") == 0.0
    assert r.target_force() == 0.0
    assert r.guard_force() == 0.0
    r.capture_baseline()   # no-op, must not raise
    r.close()              # no-op, must not raise


def test_invalid_signal_rejected():
    with pytest.raises(ValueError):
        PalmForceReader(["left"], dry_run=True, counts_per_newton=1.0, signal="max")


# ---------------------------------------------------------------------------
# Side-combination logic (inject matrices directly, bypassing DDS)
# ---------------------------------------------------------------------------

def _reader_with(matrices, signal="min"):
    # Construct in dry-run so no DDS subscribers are built, then flip the flag
    # to exercise the live reduction/combine path against injected matrices.
    r = PalmForceReader(list(matrices), dry_run=True, counts_per_newton=1.0,
                        noise_threshold=0.0, signal=signal)
    r.dry_run = False
    for side, m in matrices.items():
        r._latest[side] = np.asarray(m, dtype=float)
        r._baseline[side] = np.zeros(PALM_SHAPE)
    return r


def test_target_force_min_of_sides():
    left = np.full(PALM_SHAPE, 1.0)   # sum 112
    right = np.full(PALM_SHAPE, 3.0)  # sum 336
    r = _reader_with({"left": left, "right": right}, signal="min")
    assert r.target_force() == pytest.approx(112.0)   # min
    assert r.guard_force() == pytest.approx(336.0)     # max


def test_target_force_mean_of_sides():
    left = np.full(PALM_SHAPE, 1.0)   # 112
    right = np.full(PALM_SHAPE, 3.0)  # 336
    r = _reader_with({"left": left, "right": right}, signal="mean")
    assert r.target_force() == pytest.approx((112.0 + 336.0) / 2)


def test_force_returns_zero_before_any_message():
    # Live reduction path but no DDS message has arrived yet -> latest is None
    # -> 0.0, no crash. Built dry-run so no real subscriber is created.
    r = PalmForceReader(["left"], dry_run=True, counts_per_newton=1.0)
    r.dry_run = False
    assert r._latest["left"] is None
    assert r.force("left") == 0.0
