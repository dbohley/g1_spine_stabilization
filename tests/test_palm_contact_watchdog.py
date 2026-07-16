"""Unit tests for PalmContactWatchdog — the head-drop / contact-loss safety guard.

Covers the state machine on the combined (min) palm force: arming on first
contact, the immediate below-floor freeze, the latched trip after loss_frames
consecutive below-floor frames, transient-dip recovery, hysteresis validation,
the never-armed (dry-run) no-op, and reset.
"""

import pytest

from g1_spine_stabilization.controllers.palm_force import PalmContactWatchdog


def _make(**kw):
    defaults = dict(touch_threshold=0.5, loss_floor=0.2, loss_frames=3)
    defaults.update(kw)
    return PalmContactWatchdog(**defaults)


def test_starts_unarmed_and_not_frozen():
    w = _make()
    assert not w.armed
    assert not w.tripped
    assert not w.press_frozen


def test_arms_on_touch():
    w = _make(touch_threshold=0.5)
    w.update(0.1)          # below touch -> stays unarmed
    assert not w.armed
    w.update(0.6)          # crosses touch -> arms
    assert w.armed
    assert not w.tripped


def test_no_freeze_before_arming_even_at_zero_force():
    """Pre-contact the approach must be able to build first touch: never frozen."""
    w = _make()
    for _ in range(100):
        w.update(0.0)      # never reaches touch_threshold
    assert not w.armed
    assert not w.press_frozen
    assert not w.tripped


def test_freezes_immediately_on_below_floor_after_arming():
    """A below-floor reading freezes the press the same tick, before the latch."""
    w = _make(loss_frames=3)
    w.update(5.0)          # arm (well-gripped)
    assert not w.press_frozen
    w.update(0.1)          # first below-floor frame
    assert w.press_frozen  # frozen immediately...
    assert not w.tripped   # ...but not yet latched


def test_trips_after_loss_frames_consecutive_below_floor():
    w = _make(loss_floor=0.2, loss_frames=3)
    w.update(5.0)                      # arm
    w.update(0.1)                      # 1
    w.update(0.1)                      # 2
    assert not w.tripped
    w.update(0.1)                      # 3 -> latch
    assert w.tripped
    assert w.press_frozen


def test_transient_dip_recovers_and_does_not_trip():
    """A single below-floor frame that recovers resets the counter (no false trip)."""
    w = _make(loss_frames=3)
    w.update(5.0)                      # arm
    w.update(0.1)                      # dip (frozen this frame)
    assert w.press_frozen
    w.update(5.0)                      # recover
    assert not w.press_frozen
    assert not w.tripped
    # counter reset: needs a fresh full run of below-floor frames to trip
    w.update(0.1)
    w.update(0.1)
    assert not w.tripped
    w.update(0.1)
    assert w.tripped


def test_trip_is_latched_even_if_force_returns():
    """Once contact is confirmed lost, recovery does NOT un-trip (a dropped head
    that is caught again must still surface to the human)."""
    w = _make(loss_frames=2)
    w.update(5.0)
    w.update(0.0)
    w.update(0.0)                      # trip
    assert w.tripped
    for _ in range(10):
        w.update(5.0)                  # force comes back
    assert w.tripped                   # still latched
    assert w.press_frozen


def test_rejects_loss_floor_above_touch_threshold():
    with pytest.raises(ValueError):
        PalmContactWatchdog(touch_threshold=0.2, loss_floor=0.5, loss_frames=3)


def test_equal_floor_and_threshold_allowed():
    # loss_floor == touch_threshold is the degenerate no-hysteresis case, allowed.
    w = PalmContactWatchdog(touch_threshold=0.5, loss_floor=0.5, loss_frames=1)
    w.update(0.5)          # >= touch -> arms; not < floor (0.5), so not below
    assert w.armed
    assert not w.press_frozen


def test_reset_clears_all_state():
    w = _make(loss_frames=1)
    w.update(5.0)
    w.update(0.0)                      # arm + trip
    assert w.armed and w.tripped
    w.reset()
    assert not w.armed
    assert not w.tripped
    assert not w.press_frozen


def test_never_arms_in_dry_run_zero_force_is_noop():
    """Dry-run: reader returns 0 N every frame -> guard never arms, never freezes,
    so the preview/headless press path is unchanged."""
    w = _make()
    for _ in range(500):
        w.update(0.0)
    assert not w.armed
    assert not w.tripped
    assert not w.press_frozen
