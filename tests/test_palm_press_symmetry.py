"""Regression tests for the live-found asymmetric-press bug.

Live symptom (force_source: wrist_wrench, signal: mean, --target-force 3.0):
the press depth s ran to squeeze_max regardless of the requested target, and
the grasp frame was driven hard toward one side. Three independent mechanisms
are covered here, each fixing one part of that:

1. A NEGATIVE per-hand reading dragging the combined mean below target, so the
   loop read "under target" and kept pressing inward even though the other
   hand was already at or above the setpoint.
2. Seating on the aggregate alone, which cannot distinguish a balanced grip
   from one hand doing all the work.
3. Advancing the press every tick, so each increment was integrated against a
   force reading that had not yet reflected the previous increment.
"""

import numpy as np
import pytest

from g1_spine_stabilization.controllers.palm_force import (
    PalmPressController,
    PressSeatDetector,
    WristWrenchForceReader,
)


# ----------------------------------------------------------------------
# 1. Negative readings must not pull the pair inward
# ----------------------------------------------------------------------

def _reader_reading(left, right, signal="mean"):
    """A WristWrenchForceReader with its per-hand estimate forced directly,
    bypassing the robot/IK entirely -- these tests are about aggregation."""
    r = WristWrenchForceReader.__new__(WristWrenchForceReader)
    r.signal = signal
    r._bias = np.zeros(2)
    r._latest = np.array([float(left), float(right)])
    return r


def test_negative_hand_does_not_drag_the_mean_below_target():
    """THE live bug: left is already ABOVE a 3N target and right reads
    negative. Un-clipped, the mean is 1.5N -- the press loop reads "under
    target" and keeps driving both palms inward. Clipped, the negative hand
    contributes 0 and the pair reads at target."""
    r = _reader_reading(left=5.0, right=-2.0, signal="mean")
    assert r.target_force() == pytest.approx(2.5)   # (5 + 0) / 2, NOT (5 - 2) / 2 = 1.5


def test_negative_hand_clipped_under_min_signal_too():
    r = _reader_reading(left=5.0, right=-2.0, signal="min")
    assert r.target_force() == pytest.approx(0.0)   # min(5, 0), NOT -2.0


def test_a_negative_reading_cannot_make_the_press_advance_further():
    """End-to-end on the press law: the clipped signal must not command a
    larger inward step than an honest zero reading would."""
    c_neg = PalmPressController(kappa=1e-3, s_max=0.05, force_min=0.0,
                                force_target=3.0, force_max=20.0)
    c_zero = PalmPressController(kappa=1e-3, s_max=0.05, force_min=0.0,
                                 force_target=3.0, force_max=20.0)
    c_neg.step(_reader_reading(5.0, -2.0).target_force())
    c_zero.step(_reader_reading(5.0, 0.0).target_force())
    assert c_neg.s == pytest.approx(c_zero.s)


def test_guard_force_is_left_unclipped():
    """The crush guard only fires above force_max, so a negative value is
    already inert there -- leaving it raw keeps diagnostics honest."""
    assert _reader_reading(-4.0, -2.0).guard_force() == pytest.approx(-2.0)


# ----------------------------------------------------------------------
# 2. Seating must require BOTH hands loaded (symmetry)
# ----------------------------------------------------------------------

def test_lopsided_grip_does_not_seat_even_though_the_mean_is_on_target():
    """left +6 / right 0 has exactly the same mean as a balanced +3/+3, so the
    aggregate alone cannot reject it. The per-hand gate must."""
    d = PressSeatDetector(force_target=3.0, tolerance=0.5, frames=3, per_hand_min=1.0)
    for _ in range(20):
        d.update(3.0, per_hand=(6.0, 0.0))
    assert not d.seated


def test_balanced_grip_at_the_same_mean_does_seat():
    d = PressSeatDetector(force_target=3.0, tolerance=0.5, frames=3, per_hand_min=1.0)
    for _ in range(3):
        d.update(3.0, per_hand=(3.0, 3.0))
    assert d.seated


def test_per_hand_gate_is_opt_in():
    """per_hand_min=None keeps the original aggregate-only behavior."""
    d = PressSeatDetector(force_target=3.0, tolerance=0.5, frames=3, per_hand_min=None)
    for _ in range(3):
        d.update(3.0, per_hand=(6.0, 0.0))
    assert d.seated


def test_recovering_symmetry_seats_after_a_fresh_consecutive_run():
    """A lopsided stretch resets the counter; seating still needs `frames`
    consecutive good ticks afterwards."""
    d = PressSeatDetector(force_target=3.0, tolerance=0.5, frames=3, per_hand_min=1.0)
    for _ in range(5):
        d.update(3.0, per_hand=(6.0, 0.0))
    d.update(3.0, per_hand=(3.0, 3.0))
    d.update(3.0, per_hand=(3.0, 3.0))
    assert not d.seated
    d.update(3.0, per_hand=(3.0, 3.0))
    assert d.seated


# ----------------------------------------------------------------------
# 3. Step-and-settle cadence
# ----------------------------------------------------------------------

def _stepper(**kw):
    defaults = dict(kappa=1e-2, s_max=0.05, force_min=0.0, force_target=3.0,
                    force_max=20.0, step_increment=0.002, settle_time=0.4)
    defaults.update(kw)
    return PalmPressController(**defaults)


def test_single_advance_is_bounded_by_step_increment():
    """A large force error must not be spent as one big lunge inward."""
    c = _stepper(kappa=1.0)          # huge gain vs. a 3N error
    c.step(0.0, now=0.0)
    assert c.s == pytest.approx(0.002)


def test_press_holds_between_advances():
    """Ticks inside the settle window must not move s at all -- this is the
    'wait and read' half of move-in / wait / move-in."""
    c = _stepper()
    c.step(0.0, now=0.0)
    s_after_first = c.s
    for t in (0.05, 0.1, 0.2, 0.39):
        c.step(0.0, now=t)
        assert c.s == pytest.approx(s_after_first)


def test_press_advances_again_once_the_settle_time_elapses():
    c = _stepper()
    c.step(0.0, now=0.0)
    c.step(0.0, now=0.5)
    assert c.s == pytest.approx(0.004)


def test_continuous_mode_is_unchanged_when_step_increment_is_none():
    """The original law must be byte-identical when the feature is off: every
    call advances by kappa*error, with no settle hold. kappa is kept small
    enough here that two ticks stay well below s_max, so this measures the
    integral law itself rather than the saturation clamp."""
    c = _stepper(step_increment=None, kappa=1e-3)
    c.step(0.0, now=0.0)
    c.step(0.0, now=0.0)                   # same timestamp: no hold in continuous mode
    assert c.s == pytest.approx(2 * 1e-3 * 3.0)


def test_crush_guard_retreat_ignores_the_settle_hold():
    """Backing off an over-pressed palm must never wait on a timer."""
    c = _stepper()
    c.step(0.0, now=0.0)
    s_before = c.s
    c.step(0.0, F_guard=100.0, now=0.01)   # well inside the settle window
    assert c.s < s_before


def test_reset_clears_the_settle_timer():
    c = _stepper()
    c.step(0.0, now=0.0)
    c.reset()
    assert c.s == 0.0
    c.step(0.0, now=0.01)                  # would be blocked by a stale timer
    assert c.s == pytest.approx(0.002)


def test_rejects_invalid_step_increment_and_settle_time():
    with pytest.raises(ValueError):
        _stepper(step_increment=0.0)
    with pytest.raises(ValueError):
        _stepper(settle_time=-1.0)
