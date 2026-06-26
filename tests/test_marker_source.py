"""Tests for MarkerSource and SineSource trajectory drivers.

Uses MockMarkerTracker — no camera or hardware required.
"""
import math
import time

import numpy as np
import pytest

from g1_spine_stabilization.perception.tracker import MockMarkerTracker
from g1_spine_stabilization.perception.types import MarkerObservation
from g1_spine_stabilization.trajectory.sources import MarkerSource, SineSource


OBJ_ID = 0
TGT_ID = 1
BOX_CENTER = np.array([0.35, 0.0, 0.10])


def _make_obs(marker_id, pos, theta=0.0):
    """Build a MarkerObservation with an Rx(theta) rotation at pos."""
    ct, st = math.cos(theta), math.sin(theta)
    pose = np.eye(4)
    pose[:3, :3] = np.array([[1, 0, 0], [0, ct, -st], [0, st, ct]])
    pose[:3, 3] = pos
    return MarkerObservation(
        marker_id=marker_id,
        pose_in_pelvis=pose,
        timestamp=0.0,
        reprojection_error=0.0,
    )


def _make_tracker(obj_pos, tgt_theta, tgt_pos=None):
    if tgt_pos is None:
        tgt_pos = BOX_CENTER.copy()
    obj_obs = _make_obs(OBJ_ID, obj_pos)
    tgt_obs = _make_obs(TGT_ID, tgt_pos, theta=tgt_theta)
    # Interleave: first step gives obj, second gives tgt (then repeats)
    return MockMarkerTracker([obj_obs, tgt_obs, obj_obs, tgt_obs])


class _RepeatTracker:
    """Always returns both markers with a fixed target theta."""
    def __init__(self, obj_pos, tgt_theta=0.0, tgt_pos=None):
        self._obs = {
            OBJ_ID: _make_obs(OBJ_ID, obj_pos),
            TGT_ID: _make_obs(TGT_ID, tgt_pos if tgt_pos is not None else obj_pos, tgt_theta),
        }

    def step(self):
        return list(self._obs.values())

    def get_latest(self, marker_id):
        return self._obs.get(marker_id)


# ---------------------------------------------------------------------------
# MarkerSource tests
# ---------------------------------------------------------------------------

def test_orient_only_theta_from_target_marker():
    """theta extracted from target marker matches arctan2(R[2,1], R[2,2])."""
    expected_theta = 0.4
    tracker = _RepeatTracker(obj_pos=BOX_CENTER, tgt_theta=expected_theta)
    source = MarkerSource(tracker, OBJ_ID, TGT_ID, mode="orient-only")
    source.startup()
    _, theta = source.step()
    assert abs(theta - expected_theta) < 1e-6, (
        f"Expected theta {expected_theta}, got {theta}"
    )


def test_orient_only_box_center_fixed_after_startup():
    """In orient-only mode, box_center stays fixed even when target marker moves."""
    fixed_obj_pos = BOX_CENTER.copy()
    moving_tgt_pos = np.array([0.60, 0.1, 0.5])

    class _MovingTargetTracker:
        def __init__(self):
            self._latest = {}
        def step(self):
            obj_obs = _make_obs(OBJ_ID, fixed_obj_pos)
            tgt_obs = _make_obs(TGT_ID, moving_tgt_pos)
            self._latest[OBJ_ID] = obj_obs
            self._latest[TGT_ID] = tgt_obs
            return [obj_obs, tgt_obs]
        def get_latest(self, mid):
            return self._latest.get(mid)

    source = MarkerSource(_MovingTargetTracker(), OBJ_ID, TGT_ID, mode="orient-only")
    box_center = source.startup()
    np.testing.assert_allclose(box_center, fixed_obj_pos, atol=1e-6)

    for _ in range(3):
        bc, _ = source.step()
        np.testing.assert_allclose(bc, fixed_obj_pos, atol=1e-6,
                                   err_msg="box_center should remain fixed in orient-only mode")


def test_6dof_box_center_tracks_target_marker():
    """In 6dof mode, box_center matches the target marker's position each step."""
    obj_pos = BOX_CENTER.copy()
    tgt_pos = np.array([0.40, 0.05, 0.15])
    tracker = _RepeatTracker(obj_pos=obj_pos, tgt_pos=tgt_pos)
    source = MarkerSource(tracker, OBJ_ID, TGT_ID, mode="6dof")
    source.startup()
    bc, _ = source.step()
    np.testing.assert_allclose(bc, tgt_pos, atol=1e-6,
                                err_msg="In 6dof mode, box_center should match target marker position")


def test_step_returns_last_known_on_empty_frame():
    """When no new detection, step() returns the previously seen values."""
    class _TwoShotThenEmpty:
        """Returns both markers for the first two steps (startup phases), then nothing."""
        def __init__(self):
            self._steps = 0
            self._latest = {}
        def step(self):
            self._steps += 1
            if self._steps <= 2:
                obj = _make_obs(OBJ_ID, BOX_CENTER)
                tgt = _make_obs(TGT_ID, BOX_CENTER, theta=0.3)
                self._latest[OBJ_ID] = obj
                self._latest[TGT_ID] = tgt
                return [obj, tgt]
            return []   # no detection from step 3 onward
        def get_latest(self, mid):
            return self._latest.get(mid)

    source = MarkerSource(_TwoShotThenEmpty(), OBJ_ID, TGT_ID, mode="orient-only", countdown=0.0)
    source.startup()
    # Both steps with no detection return the pre-seeded theta from startup
    _, first_theta = source.step()
    _, second_theta = source.step()
    assert abs(second_theta - first_theta) < 1e-6, (
        "step() should return last known theta when no new detection"
    )


class _StartupOnlyTracker:
    """Returns obj+tgt for the first two steps (startup phases A and B), then nothing."""
    def __init__(self, obj_pos, tgt_theta):
        self._steps = 0
        self._latest = {}
        self._obj_obs = _make_obs(OBJ_ID, obj_pos)
        self._tgt_obs = _make_obs(TGT_ID, obj_pos, theta=tgt_theta)

    def step(self):
        self._steps += 1
        if self._steps <= 2:
            self._latest[OBJ_ID] = self._obj_obs
            self._latest[TGT_ID] = self._tgt_obs
            return [self._obj_obs, self._tgt_obs]
        return []

    def get_latest(self, mid):
        return self._latest.get(mid)


# ---------------------------------------------------------------------------
# SineSource tests
# ---------------------------------------------------------------------------

def test_sine_source_startup_returns_box_center():
    source = SineSource(BOX_CENTER, amplitude=0.5, freq=0.3)
    bc = source.startup()
    np.testing.assert_allclose(bc, BOX_CENTER, atol=1e-9)


def test_sine_source_step_returns_float_theta():
    source = SineSource(BOX_CENTER, amplitude=0.5, freq=0.3)
    source.startup()
    bc, theta = source.step()
    assert isinstance(theta, float)
    assert bc.shape == (3,)


def test_marker_source_invalid_mode():
    tracker = _RepeatTracker(obj_pos=BOX_CENTER)
    with pytest.raises(ValueError, match="mode must be"):
        MarkerSource(tracker, OBJ_ID, TGT_ID, mode="unknown")


# ---------------------------------------------------------------------------
# Startup pre-seeding, stale detection, object monitoring
# ---------------------------------------------------------------------------

def test_startup_preseeds_theta_from_target_marker():
    """startup() pre-seeds _last_theta from the target marker; step() with no
    detection returns that value (not 0.0)."""
    expected_theta = 0.7
    tracker = _StartupOnlyTracker(BOX_CENTER, tgt_theta=expected_theta)
    source = MarkerSource(tracker, OBJ_ID, TGT_ID, mode="orient-only", countdown=0.0)
    source.startup()
    # step 3 onward: no detection → should return pre-seeded theta
    _, theta = source.step()
    assert abs(theta - expected_theta) < 1e-6, (
        f"Expected pre-seeded theta {expected_theta}, got {theta}"
    )


def test_target_stale_seconds_increments():
    """target_stale_seconds increments by control_dt each step where target is absent."""
    dt = 0.02
    tracker = _StartupOnlyTracker(BOX_CENTER, tgt_theta=0.0)
    source = MarkerSource(tracker, OBJ_ID, TGT_ID, mode="orient-only",
                          countdown=0.0, control_dt=dt)
    source.startup()
    assert source.target_stale_seconds == 0.0
    source.step()
    assert abs(source.target_stale_seconds - dt) < 1e-9
    source.step()
    assert abs(source.target_stale_seconds - 2 * dt) < 1e-9


def test_object_lost_after_threshold():
    """object_lost becomes True after object marker is absent for more than 2s."""
    dt = 0.02
    tracker = _StartupOnlyTracker(BOX_CENTER, tgt_theta=0.0)
    source = MarkerSource(tracker, OBJ_ID, TGT_ID, mode="orient-only",
                          countdown=0.0, control_dt=dt)
    source.startup()
    assert not source.object_lost
    # 2s / 0.02s = 100 steps; need > 100 to cross the threshold
    for _ in range(101):
        source.step()
    assert source.object_lost, "object_lost should be True after 101 steps with no object marker"
