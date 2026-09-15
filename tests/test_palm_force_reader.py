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
    WristWrenchForceReader,
    palm_force_newtons,
    fit_calibration_series,
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


# ---------------------------------------------------------------------------
# fit_calibration_series -- multi-point calibration regression
# (scripts/read_palm_force.py's --calibrate-series)
# ---------------------------------------------------------------------------

def test_fit_recovers_exact_slope_and_intercept_on_perfect_line():
    forces = [1.0, 2.0, 3.0, 4.0]
    counts = [200.0 * f + 50.0 for f in forces]   # counts = 200*force + 50, exactly
    fit = fit_calibration_series(forces, counts)
    assert fit["slope"] == pytest.approx(200.0)
    assert fit["intercept"] == pytest.approx(50.0)
    assert fit["r2"] == pytest.approx(1.0)
    assert fit["per_point_ratio"] == pytest.approx([(200 * f + 50) / f for f in forces])


def test_fit_r2_drops_below_one_for_a_noisy_nonlinear_series():
    forces = [1.0, 2.0, 3.0, 4.0]
    counts = [180.0, 420.0, 550.0, 1200.0]   # deliberately not a clean line
    fit = fit_calibration_series(forces, counts)
    assert fit["r2"] < 0.98


def test_fit_requires_at_least_two_points():
    with pytest.raises(ValueError):
        fit_calibration_series([1.0], [200.0])


def test_fit_rejects_mismatched_shapes():
    with pytest.raises(ValueError):
        fit_calibration_series([1.0, 2.0], [200.0, 400.0, 600.0])


# ---------------------------------------------------------------------------
# WristWrenchForceReader -- joint-torque-derived grip force (no hardware
# wrist F/T sensor exists on this robot/SDK; see the class docstring).
# ---------------------------------------------------------------------------

class _FakeIKForWrench:
    """Each arm's Jacobian block is [I3, 0] in its own first-3-rows/cols,
    zero elsewhere. Verified numerically (not just derived by hand):
    pinv([[I3,0],[0,0]]) == its own transpose, so pinv(J[:, :7].T) @ tau_ext
    recovers tau_ext's own first 3 components as the raw wrench's force part
    (zero moment) exactly -- no algebra risk on the test side."""

    def __init__(self, gravity=None):
        self._gravity = np.zeros(14) if gravity is None else np.asarray(gravity, dtype=float)

    def gravity_torque(self, q):
        return self._gravity.copy()

    def ee_jacobians(self, q):
        J_L = np.zeros((6, 14))
        J_L[:3, :3] = np.eye(3)
        J_R = np.zeros((6, 14))
        J_R[:3, 7:10] = np.eye(3)
        return J_L, J_R


class _FakeRobotForWrench:
    def __init__(self, tau, q=None):
        self._tau = np.asarray(tau, dtype=float)
        self._q = np.zeros(14) if q is None else np.asarray(q, dtype=float)

    def get_state(self):
        return {"arm": {"q": self._q, "tau": self._tau}}


def test_wrist_wrench_sign_convention_symmetric_press_reads_positive_both_hands():
    """A genuine symmetric inward press produces OPPOSITE raw reaction
    wrenches between the two hands (Newton's third law: compute_press_wrenches
    has the left palm pressing the object with -F*u, the right with +F*u, so
    the object's reaction ON each hand is the negative of that). The reader's
    dot-with-+u (left) / dot-with--u (right) convention must turn those
    opposite raw signs into a MATCHING POSITIVE reading for both hands --
    getting this backwards would make the crush guard silently wrong, not
    just noisy."""
    F = 5.0
    # tau_ext = gravity_torque(q) - tau (see _estimate's live-tested sign
    # correction), and gravity defaults to 0 here, so tau_ext = -tau --
    # negate what's assigned to tau to land on the desired raw wrenches.
    tau = np.zeros(14)
    tau[:7] = [0.0, -F, 0.0, 0.0, 0.0, 0.0, 0.0]    # -> raw W_L[:3] = [0, +F, 0]
    tau[7:] = [0.0, F, 0.0, 0.0, 0.0, 0.0, 0.0]     # -> raw W_R[:3] = [0, -F, 0]
    reader = WristWrenchForceReader(_FakeRobotForWrench(tau), _FakeIKForWrench())
    reader.update(theta=0.0)
    assert reader.force("left") == pytest.approx(F)
    assert reader.force("right") == pytest.approx(F)


def test_wrist_wrench_zero_external_torque_reads_zero():
    """tau_measured == gravity_torque(q) -> tau_ext == 0 -> both hands 0N."""
    gravity = np.array([1.0, 2.0, 3.0, 0.0, 0.0, 0.0, 0.0] * 2)
    ik = _FakeIKForWrench(gravity=gravity)
    reader = WristWrenchForceReader(_FakeRobotForWrench(gravity.copy()), ik)
    reader.update(theta=0.0)
    assert reader.force("left") == pytest.approx(0.0, abs=1e-9)
    assert reader.force("right") == pytest.approx(0.0, abs=1e-9)


def test_wrist_wrench_baseline_capture_zeroes_a_constant_bias():
    F = 3.0
    tau = np.zeros(14)
    tau[:7] = [0.0, F, 0.0, 0.0, 0.0, 0.0, 0.0]
    reader = WristWrenchForceReader(_FakeRobotForWrench(tau), _FakeIKForWrench())
    reader.capture_baseline(frames=3, sleep=0.0)
    reader.update(theta=0.0)
    assert reader.force("left") == pytest.approx(0.0, abs=1e-9)


def test_wrist_wrench_target_and_guard_aggregation():
    # tau_ext = gravity_torque(q) - tau, gravity defaults to 0 -> negate.
    tau = np.zeros(14)
    tau[:7] = [0.0, -2.0, 0.0, 0.0, 0.0, 0.0, 0.0]   # -> force('left')  = +2
    tau[7:] = [0.0, 6.0, 0.0, 0.0, 0.0, 0.0, 0.0]    # -> force('right') = +6
    robot, ik = _FakeRobotForWrench(tau), _FakeIKForWrench()

    reader_mean = WristWrenchForceReader(robot, ik, signal="mean")
    reader_mean.update(theta=0.0)
    assert reader_mean.target_force() == pytest.approx((2.0 + 6.0) / 2)
    assert reader_mean.guard_force() == pytest.approx(6.0)

    reader_min = WristWrenchForceReader(robot, ik, signal="min")
    reader_min.update(theta=0.0)
    assert reader_min.target_force() == pytest.approx(2.0)
    assert reader_min.guard_force() == pytest.approx(6.0)


def test_wrist_wrench_rejects_invalid_signal():
    with pytest.raises(ValueError):
        WristWrenchForceReader(_FakeRobotForWrench(np.zeros(14)), _FakeIKForWrench(), signal="bogus")


def test_wrist_wrench_degrades_to_zero_without_raising_when_state_unavailable():
    class _BrokenRobot:
        def get_state(self):
            raise RuntimeError("no state")

    reader = WristWrenchForceReader(_BrokenRobot(), _FakeIKForWrench())
    reader.update(theta=0.0)   # must not raise
    assert reader.force("left") == 0.0
    assert reader.force("right") == 0.0
