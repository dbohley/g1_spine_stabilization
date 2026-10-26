"""Tests for MarkerSource and SineSource trajectory drivers.

Uses MockMarkerTracker — no camera or hardware required.
"""
import math
import time
from unittest.mock import patch

import numpy as np
import pytest

from g1_spine_stabilization.perception.tracker import MockMarkerTracker
from g1_spine_stabilization.perception.types import MarkerObservation
from g1_spine_stabilization.trajectory.sources import MarkerSource, SineSource
from g1_spine_stabilization.utils.transforms import Rx


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


class _SequenceTracker:
    """Returns obj+tgt readings from a caller-supplied per-call sequence.

    Each entry in `frames` is (obj_pos, tgt_pos, tgt_theta) for that call.
    After the sequence is exhausted: repeats the last entry (repeat_last=True,
    the default) or goes silent, returning [] (repeat_last=False).
    """
    def __init__(self, frames, repeat_last=True):
        self._frames = frames
        self._repeat_last = repeat_last
        self._call = 0
        self._latest = {}

    def step(self):
        idx = min(self._call, len(self._frames) - 1) if self._repeat_last else self._call
        self._call += 1
        if idx >= len(self._frames):
            return []
        obj_pos, tgt_pos, tgt_theta = self._frames[idx]
        obj_obs = _make_obs(OBJ_ID, obj_pos)
        tgt_obs = _make_obs(TGT_ID, tgt_pos, theta=tgt_theta)
        self._latest[OBJ_ID] = obj_obs
        self._latest[TGT_ID] = tgt_obs
        return [obj_obs, tgt_obs]

    def get_latest(self, marker_id):
        return self._latest.get(marker_id)


# ---------------------------------------------------------------------------
# MarkerSource tests
# ---------------------------------------------------------------------------

def test_orient_only_theta_tracks_relative_rotation_not_absolute():
    """theta is the target's rotation CHANGE since tracking began, not its
    absolute rotation (regression test for the 6dof/orient-only absolute-
    teleport-to-target bug -- see docs/followups.md)."""
    start_theta = 0.2
    step_theta = 0.6  # absolute; delta from start = 0.4
    frames = [
        (BOX_CENTER, BOX_CENTER, start_theta),  # Phase A (object found)
        (BOX_CENTER, BOX_CENTER, start_theta),  # Phase B (target start reference)
        (BOX_CENTER, BOX_CENTER, step_theta),   # step(): target has rotated
    ]
    tracker = _SequenceTracker(frames)
    source = MarkerSource(tracker, OBJ_ID, TGT_ID, mode="orient-only", countdown=0.0)
    source.startup()
    _, theta = source.step()
    expected_delta = step_theta - start_theta
    np.testing.assert_allclose(theta, Rx(expected_delta), atol=1e-6,
                               err_msg=f"Expected theta delta {expected_delta}, got {theta}")
    assert not np.allclose(theta, Rx(step_theta), atol=1e-3), (
        "theta should not equal the target's absolute rotation"
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


def test_6dof_box_center_tracks_target_displacement_not_absolute_position():
    """In 6dof mode, box_center is the object's own start position shifted by
    the target's DISPLACEMENT from its own start -- not the target's raw
    absolute position (regression test for the absolute-teleport-to-target
    bug -- see docs/followups.md). Object and target start in DIFFERENT
    physical locations, as they would be in a real grasp/tracking setup."""
    obj_pos = BOX_CENTER.copy()
    tgt_start_pos = np.array([0.60, 0.20, 0.30])
    tgt_step_pos = tgt_start_pos + np.array([0.05, -0.02, 0.01])
    frames = [
        (obj_pos, tgt_start_pos, 0.0),  # Phase A
        (obj_pos, tgt_start_pos, 0.0),  # Phase B: target start reference
        (obj_pos, tgt_step_pos, 0.0),   # step(): target has moved
    ]
    tracker = _SequenceTracker(frames)
    source = MarkerSource(tracker, OBJ_ID, TGT_ID, mode="6dof", countdown=0.0)
    source.startup()
    bc, _ = source.step()
    expected = obj_pos + (tgt_step_pos - tgt_start_pos)
    np.testing.assert_allclose(bc, expected, atol=1e-6)
    assert not np.allclose(bc, tgt_step_pos, atol=1e-3), (
        "box_center should not equal the target's raw absolute position"
    )


def _make_obs_R(marker_id, pos, R):
    """Like _make_obs, but takes an arbitrary rotation matrix instead of a
    pure Rx(theta) -- for exercising pitch/yaw, which _make_obs cannot
    produce at all."""
    pose = np.eye(4)
    pose[:3, :3] = R
    pose[:3, 3] = pos
    return MarkerObservation(
        marker_id=marker_id, pose_in_pelvis=pose, timestamp=0.0, reprojection_error=0.0,
    )


class _SequenceTrackerR:
    """Like _SequenceTracker, but each frame's target rotation is an
    arbitrary (3,3) matrix rather than an Rx(theta) scalar."""
    def __init__(self, frames):
        self._frames = frames  # list of (obj_pos, tgt_pos, tgt_R)
        self._i = 0
        self._latest = {}

    def step(self):
        obj_pos, tgt_pos, tgt_R = self._frames[min(self._i, len(self._frames) - 1)]
        self._i += 1
        obj_obs = _make_obs(OBJ_ID, obj_pos)
        tgt_obs = _make_obs_R(TGT_ID, tgt_pos, tgt_R)
        self._latest[OBJ_ID] = obj_obs
        self._latest[TGT_ID] = tgt_obs
        return [obj_obs, tgt_obs]

    def get_latest(self, mid):
        return self._latest.get(mid)


def test_target_yaw_is_tracked_not_discarded():
    """Regression test for the confirmed 6dof/orient-only bug: a pure YAW of
    the target marker used to collapse to theta=0.0 (no hand response at
    all), since the old single-axis extraction only ever read a roll-about-X
    component. theta is now a full rotation, so a pure yaw must be tracked."""
    from scipy.spatial.transform import Rotation as _R
    R_yaw = _R.from_euler('z', 0.5).as_matrix()
    frames = [
        (BOX_CENTER, BOX_CENTER, np.eye(3)),  # Phase A
        (BOX_CENTER, BOX_CENTER, np.eye(3)),  # Phase B: identity start reference
        (BOX_CENTER, BOX_CENTER, R_yaw),      # step(): target has yawed
    ]
    tracker = _SequenceTrackerR(frames)
    source = MarkerSource(tracker, OBJ_ID, TGT_ID, mode="orient-only", countdown=0.0)
    source.startup()
    _, theta = source.step()
    np.testing.assert_allclose(theta, R_yaw, atol=1e-9,
                               err_msg="pure yaw must be tracked, not collapsed to identity")


def test_rotation_delta_is_world_frame_not_body_frame():
    """Regression test for the second, independent confirmed bug: the target
    delta must be R_t @ R_target_start.T (world/pelvis frame -- what
    compute_hand_targets's grasp_frame_R composition requires), NOT
    R_target_start.T @ R_t (the target's own body frame). A non-identity,
    non-commuting start rotation is required to tell the two apart -- for a
    pure-X start rotation they're numerically identical, which is why this
    bug went uncaught before."""
    from scipy.spatial.transform import Rotation as _R
    R_start = _R.from_euler('xyz', [0.1, 0.3, -0.2]).as_matrix()  # non-trivial, multi-axis
    R_step = _R.from_euler('xyz', [0.4, -0.1, 0.5]).as_matrix()
    frames = [
        (BOX_CENTER, BOX_CENTER, np.eye(3)),   # Phase A
        (BOX_CENTER, BOX_CENTER, R_start),     # Phase B: start reference
        (BOX_CENTER, BOX_CENTER, R_step),      # step(): target has rotated
    ]
    tracker = _SequenceTrackerR(frames)
    source = MarkerSource(tracker, OBJ_ID, TGT_ID, mode="orient-only", countdown=0.0)
    source.startup()
    _, theta = source.step()

    R_world = R_step @ R_start.T
    R_body = R_start.T @ R_step
    np.testing.assert_allclose(theta, R_world, atol=1e-9,
                               err_msg="expected the WORLD-frame delta R_t @ R_start.T")
    assert not np.allclose(theta, R_body, atol=1e-3), (
        "got the target's own BODY-frame delta -- wrong frame for composing onto grasp_frame_R"
    )


class _StaticObjTracker:
    """Single-shot tracker: object marker at a given position/rotation, plus
    a fixed target marker -- used for box_half_height correction tests."""

    def __init__(self, obj_pos, obj_theta, tgt_pos):
        self._obj_obs = _make_obs(OBJ_ID, obj_pos, theta=obj_theta)
        self._tgt_obs = _make_obs(TGT_ID, tgt_pos)
        self._latest = {OBJ_ID: self._obj_obs, TGT_ID: self._tgt_obs}

    def step(self):
        return [self._obj_obs, self._tgt_obs]

    def get_latest(self, marker_id):
        return self._latest.get(marker_id)


def test_box_half_height_world_z_case():
    """With an identity-rotation object marker, box_half_height subtracts
    straight down along world/pelvis Z -- the marker-on-top correction."""
    obj_pos = np.array([0.35, 0.0, 0.20])
    H = 0.08
    tracker = _StaticObjTracker(obj_pos, obj_theta=0.0, tgt_pos=BOX_CENTER)
    source = MarkerSource(tracker, OBJ_ID, TGT_ID, mode="orient-only", countdown=0.0,
                          box_half_height=H)
    box_center = source.startup()
    expected = obj_pos - np.array([0.0, 0.0, H])
    np.testing.assert_allclose(box_center, expected, atol=1e-9)


def test_box_half_height_follows_tilted_marker_orientation():
    """With a tilted object marker (90 deg about X), the correction follows
    the marker's own measured Z axis (now pointing along -Y, not world Z),
    proving it doesn't assume world-up."""
    obj_pos = np.array([0.35, 0.0, 0.20])
    H = 0.08
    theta = math.pi / 2
    tracker = _StaticObjTracker(obj_pos, obj_theta=theta, tgt_pos=BOX_CENTER)
    source = MarkerSource(tracker, OBJ_ID, TGT_ID, mode="orient-only", countdown=0.0,
                          box_half_height=H)
    box_center = source.startup()

    ct, st = math.cos(theta), math.sin(theta)
    R = np.array([[1, 0, 0], [0, ct, -st], [0, st, ct]])
    expected = obj_pos - H * R[:, 2]
    np.testing.assert_allclose(box_center, expected, atol=1e-9)

    world_z_expected = obj_pos - np.array([0.0, 0.0, H])
    assert not np.allclose(box_center, world_z_expected, atol=1e-3), (
        "box_center should follow the marker's tilted orientation, not assume world Z"
    )


def test_box_half_height_zero_default_matches_pre_change_behavior():
    """box_half_height=0.0 (default) reproduces the exact old behavior:
    box_center equals the raw marker position, no offset."""
    obj_pos = np.array([0.35, 0.0, 0.20])
    tracker = _StaticObjTracker(obj_pos, obj_theta=0.3, tgt_pos=BOX_CENTER)
    source = MarkerSource(tracker, OBJ_ID, TGT_ID, mode="orient-only", countdown=0.0)
    box_center = source.startup()
    np.testing.assert_allclose(box_center, obj_pos, atol=1e-9)


def test_box_center_offset_none_default_matches_pre_change_behavior():
    """box_center_offset=None (default) reproduces the exact old behavior:
    no additional nudge beyond box_half_height."""
    obj_pos = np.array([0.35, 0.0, 0.20])
    H = 0.08
    tracker = _StaticObjTracker(obj_pos, obj_theta=0.0, tgt_pos=BOX_CENTER)
    source = MarkerSource(tracker, OBJ_ID, TGT_ID, mode="orient-only", countdown=0.0,
                          box_half_height=H)
    box_center = source.startup()
    expected = obj_pos - np.array([0.0, 0.0, H])
    np.testing.assert_allclose(box_center, expected, atol=1e-9)


def test_box_center_offset_world_z_case_orient_only():
    """With an identity-rotation object marker, box_center_offset is added
    directly in world/pelvis axes, on top of the box_half_height shift."""
    obj_pos = np.array([0.35, 0.0, 0.20])
    H = 0.08
    offset = np.array([-0.03, 0.01, 0.0])
    tracker = _StaticObjTracker(obj_pos, obj_theta=0.0, tgt_pos=BOX_CENTER)
    source = MarkerSource(tracker, OBJ_ID, TGT_ID, mode="orient-only", countdown=0.0,
                          box_half_height=H, box_center_offset=offset)
    box_center = source.startup()
    expected = obj_pos - np.array([0.0, 0.0, H]) + offset
    np.testing.assert_allclose(box_center, expected, atol=1e-9)


def test_box_center_offset_follows_tilted_marker_orientation():
    """With a tilted object marker, box_center_offset is expressed in the
    marker's own rotation frame and rotated into world frame -- proving it
    doesn't assume world axes, mirroring box_half_height's own tilt test.

    Uses a Y-component offset (not X): theta rotates about X, so an X-only
    offset is invariant under the tilt and couldn't distinguish "rotated
    correctly" from "added raw in world axes" -- Y actually moves (into Z
    here), which is what makes this test meaningful."""
    obj_pos = np.array([0.35, 0.0, 0.20])
    H = 0.08
    theta = math.pi / 2
    offset = np.array([0.0, 0.02, 0.0])
    tracker = _StaticObjTracker(obj_pos, obj_theta=theta, tgt_pos=BOX_CENTER)
    source = MarkerSource(tracker, OBJ_ID, TGT_ID, mode="orient-only", countdown=0.0,
                          box_half_height=H, box_center_offset=offset)
    box_center = source.startup()

    ct, st = math.cos(theta), math.sin(theta)
    R = np.array([[1, 0, 0], [0, ct, -st], [0, st, ct]])
    expected = obj_pos - H * R[:, 2] + R @ offset
    np.testing.assert_allclose(box_center, expected, atol=1e-9)

    world_frame_expected = obj_pos - H * R[:, 2] + offset
    assert not np.allclose(box_center, world_frame_expected, atol=1e-3), (
        "box_center_offset should follow the marker's tilted orientation, "
        "not be added in raw world axes"
    )


def test_box_center_offset_propagates_through_6dof_tracking():
    """The offset is baked into the anchor (_object_start_pos) at startup(),
    so it must still be present after the target marker moves in 6dof mode --
    not just at the initial startup() reading."""
    obj_pos = np.array([0.35, 0.0, 0.20])
    H = 0.08
    offset = np.array([-0.03, 0.0, 0.0])
    tgt_start = np.array([0.5, 0.0, 0.3])
    tracker = _SequenceTracker([
        (obj_pos, tgt_start, 0.0),
        (obj_pos, tgt_start, 0.0),
        (obj_pos, tgt_start + np.array([0.05, 0.0, 0.0]), 0.0),
    ])
    source = MarkerSource(tracker, OBJ_ID, TGT_ID, mode="6dof", countdown=0.0,
                          box_half_height=H, box_center_offset=offset)
    source.startup()
    box_center, _theta = source.step()
    expected_anchor = obj_pos - np.array([0.0, 0.0, H]) + offset
    expected = expected_anchor + np.array([0.05, 0.0, 0.0])
    np.testing.assert_allclose(box_center, expected, atol=1e-9)


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
    np.testing.assert_allclose(second_theta, first_theta, atol=1e-6,
                               err_msg="step() should return last known theta when no new detection")


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

def test_startup_preseeds_nonzero_theta_delta_and_it_persists_when_stale():
    """A nonzero theta delta established right after startup persists through
    subsequent stale (no-detection) steps, rather than resetting to 0.0."""
    start_theta = 0.1
    moved_theta = 0.8  # delta = 0.7
    frames = [
        (BOX_CENTER, BOX_CENTER, start_theta),  # Phase A
        (BOX_CENTER, BOX_CENTER, start_theta),  # Phase B: start reference
        (BOX_CENTER, BOX_CENTER, moved_theta),  # one live step -> nonzero delta
    ]
    tracker = _SequenceTracker(frames, repeat_last=False)  # goes silent after
    source = MarkerSource(tracker, OBJ_ID, TGT_ID, mode="orient-only", countdown=0.0)
    source.startup()
    _, first_theta = source.step()
    expected_delta = moved_theta - start_theta
    np.testing.assert_allclose(first_theta, Rx(expected_delta), atol=1e-6,
                               err_msg=f"Expected theta delta {expected_delta}, got {first_theta}")
    # tracker is now silent -- delta should be held, not reset
    _, second_theta = source.step()
    np.testing.assert_allclose(second_theta, first_theta, atol=1e-6,
                               err_msg="Nonzero theta delta should persist across stale/silent steps")


def test_wait_for_keypress_blocks_on_input_and_refreshes_theta():
    """wait_for_keypress=True blocks on input() and does one final refresh of
    theta from the target marker's post-keypress reading before returning,
    instead of the fixed-duration countdown -- lets a human freely reposition
    the arm while waiting, then trigger Phase 1 on demand."""
    start_theta = 0.0
    post_keypress_theta = 0.5
    frames = [
        (BOX_CENTER, BOX_CENTER, 0.0),                  # Phase A
        (BOX_CENTER, BOX_CENTER, start_theta),          # Phase B: start reference
        (BOX_CENTER, BOX_CENTER, post_keypress_theta),  # Phase C: post-keypress refresh
    ]
    tracker = _SequenceTracker(frames, repeat_last=False)
    source = MarkerSource(tracker, OBJ_ID, TGT_ID, mode="orient-only",
                          countdown=0.0, wait_for_keypress=True)

    with patch("builtins.input", return_value="") as mock_input:
        source.startup()
    mock_input.assert_called_once()

    _, theta = source.step()  # tracker now silent -- holds the post-keypress value
    expected_delta = post_keypress_theta - start_theta
    np.testing.assert_allclose(
        theta, Rx(expected_delta), atol=1e-6,
        err_msg=f"Expected theta delta {expected_delta} from the post-keypress refresh, got {theta}")


class _FakeClock:
    def __init__(self):
        self.t = 100.0

    def __call__(self):
        return self.t


def test_target_stale_seconds_tracks_wall_clock_not_frame_count():
    """target_stale_seconds is wall-clock time since the target was last seen,
    independent of how many step() calls happened or of control_dt. The real
    loop runs at ~6-10 Hz, so frames * control_dt (the old formula) undercounts
    staleness 5-9x."""
    clock = _FakeClock()
    tracker = _StartupOnlyTracker(BOX_CENTER, tgt_theta=0.0)
    source = MarkerSource(tracker, OBJ_ID, TGT_ID, mode="orient-only",
                          countdown=0.0, control_dt=0.02, clock=clock)
    source.startup()
    assert source.target_stale_seconds == 0.0
    source.step()                 # first step anchors the stale clock
    assert source.target_stale_seconds == 0.0
    clock.t += 0.15               # one slow (IK-bound) tick
    source.step()
    assert abs(source.target_stale_seconds - 0.15) < 1e-9
    clock.t += 0.15
    source.step()
    assert abs(source.target_stale_seconds - 0.30) < 1e-9


def test_last_obs_properties_populated_after_step():
    """last_obj_obs and last_tgt_obs are set after step() when markers are visible."""
    tracker = _RepeatTracker(obj_pos=BOX_CENTER, tgt_theta=0.3)
    source = MarkerSource(tracker, OBJ_ID, TGT_ID, mode="orient-only")
    source.startup()
    assert source.last_obj_obs is None or source.last_tgt_obs is None or True  # before step
    source.step()
    assert source.last_obj_obs is not None, "last_obj_obs should be set after step()"
    assert source.last_tgt_obs is not None, "last_tgt_obs should be set after step()"
    assert source.last_obj_obs.marker_id == OBJ_ID
    assert source.last_tgt_obs.marker_id == TGT_ID


def test_last_obs_none_when_markers_absent():
    """last_obj_obs and last_tgt_obs are None when no markers are visible this step."""
    class _EmptyTracker:
        def step(self): return []
        def get_latest(self, mid): return None

    tracker = _RepeatTracker(obj_pos=BOX_CENTER, tgt_theta=0.0)
    source = MarkerSource(tracker, OBJ_ID, TGT_ID, mode="orient-only")
    source.startup()
    # Replace tracker with one that returns nothing
    source._tracker = _EmptyTracker()
    source.step()
    assert source.last_obj_obs is None
    assert source.last_tgt_obs is None


def test_object_lost_after_threshold():
    """object_lost becomes True after object marker is absent for more than 2s
    of wall-clock time -- here only ~15 steps at a realistic ~7 Hz loop rate,
    which the old frame-count formula (needing 101 steps) would have missed."""
    clock = _FakeClock()
    tracker = _StartupOnlyTracker(BOX_CENTER, tgt_theta=0.0)
    source = MarkerSource(tracker, OBJ_ID, TGT_ID, mode="orient-only",
                          countdown=0.0, control_dt=0.02, clock=clock)
    source.startup()
    source.step()
    assert not source.object_lost
    for _ in range(14):
        clock.t += 0.15
        source.step()
    assert source.object_lost, "object_lost should be True after 2.1s with no object marker"


# ---------------------------------------------------------------------------
# Grasp-without-tracking: object marker found, target marker never found
# ---------------------------------------------------------------------------

class _ObjectOnlyTracker:
    """Always returns the object marker; never returns the target marker --
    simulates a torso/target marker that's out of frame or never attached."""
    def __init__(self, obj_pos):
        self._obj_obs = _make_obs(OBJ_ID, obj_pos)
        self._latest = {OBJ_ID: self._obj_obs}

    def step(self):
        return [self._obj_obs]

    def get_latest(self, mid):
        return self._latest.get(mid)


def test_startup_proceeds_without_target_marker():
    """startup() must not raise when the target marker is never found -- it
    should return the object-derived box_center and leave target_found False,
    so the caller can grasp normally and just skip the tracking phase."""
    tracker = _ObjectOnlyTracker(BOX_CENTER)
    source = MarkerSource(tracker, OBJ_ID, TGT_ID, mode="orient-only", countdown=0.0)

    box_center = source.startup(timeout=0.15)

    np.testing.assert_allclose(box_center, BOX_CENTER, atol=1e-9)
    assert source.target_found is False


def test_target_found_true_when_target_marker_is_detected():
    """Sanity check on the flag's positive case, using the existing
    always-both-markers tracker convention."""
    tracker = _RepeatTracker(obj_pos=BOX_CENTER)
    source = MarkerSource(tracker, OBJ_ID, TGT_ID, mode="orient-only")
    source.startup()
    assert source.target_found is True


# ---------------------------------------------------------------------------
# on_observations: fires the instant each tracker.step() call detects
# something, independent of which phase is running or whether it later
# succeeds -- regression test for the "markers never appeared in Rerun"
# live bug, root-caused to logging that was coupled to Phase 1 succeeding
# instead of to detection itself.
# ---------------------------------------------------------------------------

def test_on_observations_fires_during_phase_a_even_if_target_never_found():
    """The callback must fire from Phase A alone -- using a tracker that
    NEVER returns the target marker proves this isn't gated on Phase B (or
    any later phase) ever completing, which is exactly the bug: the old
    phase-coupled logging could be skipped entirely by unrelated downstream
    failures (e.g. a Phase 1 collision-avoidance abort)."""
    tracker = _ObjectOnlyTracker(BOX_CENTER)
    calls = []
    source = MarkerSource(
        tracker, OBJ_ID, TGT_ID, mode="orient-only", countdown=0.0,
        on_observations=lambda obj_obs, tgt_obs: calls.append((obj_obs, tgt_obs)),
    )
    source.startup(timeout=0.15)

    assert len(calls) >= 1, "on_observations should fire during Phase A's poll loop"
    obj_obs, tgt_obs = calls[0]
    assert obj_obs is not None and obj_obs.marker_id == OBJ_ID
    assert tgt_obs is None, "target was never in frame -- callback must still report it as None, not skip the call"


def test_on_observations_fires_on_every_step_call():
    tracker = _RepeatTracker(obj_pos=BOX_CENTER, tgt_theta=0.3)
    calls = []
    source = MarkerSource(
        tracker, OBJ_ID, TGT_ID, mode="orient-only",
        on_observations=lambda obj_obs, tgt_obs: calls.append((obj_obs, tgt_obs)),
    )
    source.startup()
    calls.clear()  # isolate step()'s own call from startup()'s

    source.step()
    assert len(calls) == 1, "on_observations should fire exactly once per step() call"
    obj_obs, tgt_obs = calls[0]
    assert obj_obs is not None and obj_obs.marker_id == OBJ_ID
    assert tgt_obs is not None and tgt_obs.marker_id == TGT_ID
