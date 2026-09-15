"""
Trajectory drivers — produce the time-varying signal that the controller tracks.

sine_trajectory is the original placeholder. MarkerSource replaces it for live
AprilTag-driven control, exposing the same (box_center, theta) interface so the
controller doesn't change.
"""

import math
import time

import numpy as np
from scipy.spatial.transform import Rotation

from g1_spine_stabilization.utils.transforms import (
    is_scalar_theta, as_rotation, clamp_rotation, rotation_angle,
)


def sine_trajectory(t, amplitude, freq):
    """Sine wave rotation trajectory.

    Args:
        t:         time in seconds
        amplitude: peak amplitude in radians
        freq:      frequency in Hz

    Returns:
        theta: rotation angle in radians at time t
    """
    return amplitude * math.sin(2 * math.pi * freq * t)


class SineSource:
    """Wraps sine_trajectory with the same .step() interface as MarkerSource.

    Useful for the SDK preview script when no camera is available.
    """

    def __init__(self, box_center, amplitude, freq):
        self._box_center = np.asarray(box_center, dtype=float)
        self._amplitude = amplitude
        self._freq = freq
        self._start = None

    def startup(self) -> np.ndarray:
        self._start = time.monotonic()
        return self._box_center.copy()

    def step(self) -> tuple:
        t = time.monotonic() - self._start
        theta = sine_trajectory(t, self._amplitude, self._freq)
        return self._box_center.copy(), theta


class MarkerSource:
    """Drives (box_center, theta) from live AprilTag marker observations.

    theta is now a FULL (3,3) pelvis-frame rotation matrix (previously a
    single roll-about-X scalar -- a confirmed bug that silently discarded
    the target marker's pitch and yaw; see docs/six_dof_tracking_fix.md).
    It is the target's rotation CHANGE since tracking began, expressed in
    the PELVIS/world frame (not the target marker's own body frame -- see
    _rotation_from_target's docstring for why that distinction matters).

    Two tracking modes:
      "orient-only" — box_center is captured from the object marker once at
          startup and held fixed; theta tracks the target marker's full
          rotation RELATIVE TO the target's own starting orientation (not
          the target's absolute orientation -- see below). Genuinely 3-DoF
          orientation tracking.
      "6dof"        — box_center tracks the object's own starting position,
          offset by however far the target marker has moved from ITS OWN
          starting position (a relative/delta displacement, not the target's
          absolute position); theta is the same relative-rotation tracking
          as orient-only. Genuinely 6-DoF (3 translation + 3 rotation).

    Both modes track the target's DISPLACEMENT from where it was when
    tracking began, not its absolute pose. This matters because the object
    (held in the robot's hands) and the target (e.g. held by a person) are
    generally in different physical locations -- if box_center were simply
    set to the target's raw absolute position, the hands would be commanded
    to wherever the target physically is in space, not to the object's own
    position shifted by how far the target has moved. (This was a real bug,
    found via a live-hardware test with a real object and target in
    different locations -- see docs/followups.md.)

    Args:
        tracker:           a MarkerTracker or duck-typed equivalent whose step()
                           returns list[obs] for the current frame and
                           get_latest(id) returns the last known obs.
        object_marker_id:  AprilTag ID of the object marker (pose at grasp time).
        target_marker_id:  AprilTag ID of the target marker (live tracking).
        mode:              "orient-only" or "6dof".
        countdown:         seconds to count down after both markers are detected
                           in startup() before returning. 0.0 skips the countdown
                           (use for mock trackers and unit tests). Ignored when
                           wait_for_keypress=True.
        wait_for_keypress: if True, Phase C blocks on Enter instead of counting
                           down a fixed duration -- lets a human freely
                           reposition the arm (e.g. into an unusual test
                           configuration) before triggering Phase 1's approach,
                           since the caller reads the arm's live joint state
                           immediately after startup() returns. Default False
                           preserves the original timed-countdown behavior.
        control_dt:        control loop period in seconds; used for stale-time
                           calculations (does not introduce delays in step()).
        box_half_height:   the object marker is mounted on TOP of the held
                           object, not at its center -- half the box's height
                           (meters), subtracted along the object marker's own
                           measured "out of the surface" axis (column 2 of its
                           rotation matrix, per detector.py's solvePnP object
                           points convention) at startup() Phase A, to recover
                           the box's true geometric center. Uses the marker's
                           actual measured orientation rather than assuming
                           world-up, so it's still correct if the box isn't
                           placed perfectly level. Default 0.0 preserves the
                           old behavior (marker position taken as box_center
                           directly).
        box_center_offset: additional [x, y, z] nudge to box_center, expressed
                           in the object marker's own rotation frame (same
                           axis convention as box_half_height's shift above --
                           X/Y are the marker's in-plane axes, Z is its "out
                           of surface" axis), rotated into world/pelvis frame
                           and added on top of the box_half_height shift at
                           startup() Phase A. Mirrors kinematics/collision.py's
                           head_sphere_center_from_marker `offset` param
                           exactly, but for the actual grasp target rather
                           than the (separate, unrelated) RRT collision
                           sphere. Lets the real grasp point be fit to a
                           marker that isn't perfectly centered on the held
                           object, without touching box_half_height itself.
                           None (default) reproduces the old behavior exactly
                           (no additional nudge).
        on_observations:   optional callback(obj_obs, tgt_obs) invoked on
                           EVERY self._tracker.step() call this class makes
                           (Phase A/B/C's wait/countdown loops AND step()),
                           immediately when each poll returns -- independent
                           of whatever the caller does with box_center/theta
                           afterward. Intended for a live-visualization hook
                           (e.g. logging marker poses to Rerun the instant
                           they're detected) that must not depend on later
                           control-flow (a Phase 1 collision-avoidance abort,
                           for example, must not prevent markers detected
                           during Phase A/B/C from ever having been shown).
                           Either argument may be None if that marker wasn't
                           seen this poll. Default None disables the hook.
    """

    def __init__(self, tracker, object_marker_id, target_marker_id,
                 mode="orient-only", countdown=0.0, control_dt=0.02,
                 box_half_height=0.0, box_center_offset=None,
                 wait_for_keypress=False, on_observations=None):
        if mode not in ("orient-only", "6dof"):
            raise ValueError(f"mode must be 'orient-only' or '6dof', got {mode!r}")
        self._tracker = tracker
        self._obj_id = object_marker_id
        self._tgt_id = target_marker_id
        self._mode = mode
        self._countdown = countdown
        self._wait_for_keypress = wait_for_keypress
        self._control_dt = control_dt
        self._box_half_height = float(box_half_height)
        self._box_center_offset = (None if box_center_offset is None
                                   else np.asarray(box_center_offset, dtype=float))
        self._on_observations = on_observations  # see _step_tracker()
        self._object_start_pos = None  # object's own position, set once by startup(), never changes
        self._object_marker_pos = None  # RAW object marker pose (pre-box_half_height-offset),
        self._object_marker_R = None    # set once by startup() Phase A -- see object_marker_pose
        self._target_start_pos = None  # target's position when tracking began (Phase B)
        self._target_start_rot = None  # target's rotation when tracking began (Phase B)
        self._box_center = None        # live returned value; == object_start_pos in orient-only mode
        self._target_found = False     # set True by startup() Phase B; see target_found property
        self._last_rotation = np.eye(3)
        self._last_obj_obs = None
        self._last_tgt_obs = None
        # Stale-target tracking
        self._target_stale_frames = 0
        self._stale_warned_05 = False
        self._stale_warned_3 = False
        # Object-lost monitoring
        self._object_stale_frames = 0
        self._object_warned_2s = False
        self._object_warned_5s = False

    def _rotation_from_target(self, R_current: np.ndarray) -> np.ndarray:
        """Target's FULL rotation relative to its own starting orientation
        (not absolute), in the PELVIS/world frame.

        R_current = ΔR_world @ R_target_start, so ΔR_world = R_current @
        R_target_start.T -- the WORLD-frame delta, NOT the body-frame delta
        (R_target_start.T @ R_current, expressed in the target's own
        starting axes). This matters because compute_hand_targets composes
        the returned rotation as `R @ grasp_frame_R @ R_BASE`, left-
        multiplying it onto grasp_frame_R -- the object marker's own
        ABSOLUTE pelvis-frame orientation. Left-multiplying an absolute
        pelvis-frame orientation only type-checks if this is itself a
        pelvis-frame delta. (The two forms coincide only when the rotation
        is a pure single-axis rotation, since single-axis rotations commute
        with their own transpose -- which is why a previous single-axis-only
        version of this method could use the wrong (body-frame) form without
        it ever being caught by a test. See docs/six_dof_tracking_fix.md.)
        """
        return R_current @ self._target_start_rot.T

    def _box_center_from_target_position(self, pos_current: np.ndarray) -> np.ndarray:
        """Object's own start position, shifted by the target's displacement from its own start."""
        return self._object_start_pos + (pos_current - self._target_start_pos)

    def _step_tracker(self):
        """The ONE place self._tracker.step() is called from this class.
        Fires self._on_observations(obj_obs, tgt_obs) immediately, before
        returning -- every phase (A/B/C's wait/countdown loops and step())
        routes through here, so the callback is guaranteed to see every
        detection the instant it happens, regardless of what any later phase
        does with it (see on_observations in the class docstring)."""
        current = self._tracker.step()
        obj_obs = next((o for o in current if o.marker_id == self._obj_id), None)
        tgt_obs = next((o for o in current if o.marker_id == self._tgt_id), None)
        if self._on_observations is not None:
            self._on_observations(obj_obs, tgt_obs)
        return obj_obs, tgt_obs, current

    def startup(self, timeout=10.0) -> np.ndarray:
        """Block until both markers are detected, then optionally count down.

        Phase A: waits for the object marker → records the object's own start position.
        Phase B: waits for the target marker → records its start pose (position + rotation)
                 as the reference for relative tracking; _last_rotation starts at identity.
        Phase C: counts down self._countdown seconds while continuously
                 refreshing readings; skipped when countdown=0.0.

        Returns the box_center (pelvis frame) as soon as the object marker
        is found -- Phase 0/1/1.5 (the grasp) only ever need box_center, not
        theta, so they can proceed even if the target marker is never found.
        Raises RuntimeError only if the OBJECT marker times out (there's
        nothing to grasp without it). If the TARGET marker times out, this
        prints a warning, leaves target_found False, and still returns
        box_center normally -- callers check target_found to decide whether
        to run the tracking phase at all.
        """
        # Phase A: object marker
        print(f"[MarkerSource] Waiting for object marker (id={self._obj_id})…")
        deadline = time.monotonic() + timeout
        found_obj = False
        while time.monotonic() < deadline:
            obj_obs, _tgt_obs, _current = self._step_tracker()
            if obj_obs is not None:
                marker_pos = obj_obs.pose_in_pelvis[:3, 3]
                marker_R   = obj_obs.pose_in_pelvis[:3, :3]
                self._object_marker_pos = marker_pos.copy()
                self._object_marker_R = marker_R.copy()
                # Marker sits on top of the object -- shift down along its own
                # measured "out of the surface" axis to recover the true center.
                self._object_start_pos = marker_pos - self._box_half_height * marker_R[:, 2]
                if self._box_center_offset is not None:
                    self._object_start_pos = (self._object_start_pos
                                              + marker_R @ self._box_center_offset)
                self._box_center = self._object_start_pos.copy()
                marker_rpy_deg = np.degrees(Rotation.from_matrix(marker_R).as_euler('xyz'))
                print(f"[MarkerSource] Object marker detected: box_center={np.round(self._box_center, 4)}")
                print(f"[MarkerSource] Object marker orientation (rpy, deg): "
                      f"[{marker_rpy_deg[0]:+.1f}, {marker_rpy_deg[1]:+.1f}, {marker_rpy_deg[2]:+.1f}]")
                found_obj = True
                break
            time.sleep(0.05)
        if not found_obj:
            raise RuntimeError(
                f"[MarkerSource] Object marker (id={self._obj_id}) not detected within {timeout}s"
            )

        # Phase B: target marker
        print(f"[MarkerSource] Waiting for target marker (id={self._tgt_id})…")
        deadline = time.monotonic() + timeout
        found_tgt = False
        while time.monotonic() < deadline:
            _obj_obs, tgt_obs, _current = self._step_tracker()
            if tgt_obs is not None:
                self._target_start_pos = tgt_obs.pose_in_pelvis[:3, 3].copy()
                self._target_start_rot = tgt_obs.pose_in_pelvis[:3, :3].copy()
                self._last_rotation = np.eye(3)  # no displacement yet -- this IS the target's start
                self._target_found = True
                print(f"[MarkerSource] Target marker detected (start reference recorded)")
                found_tgt = True
                break
            time.sleep(0.05)
        if not found_tgt:
            print(f"[MarkerSource] WARNING: target marker (id={self._tgt_id}) not detected "
                  f"within {timeout}s -- proceeding to grasp without tracking "
                  f"(target_found=False; caller should skip the tracking phase).")

        # Phase C: countdown, keypress gate, or skip (countdown=0.0 and no keypress wait)
        if self._wait_for_keypress:
            print("[MarkerSource] Both markers visible. Press Enter when ready "
                  "to begin tracking (you can reposition the arm now)…")
            input()
            # One final refresh so theta/box_center reflect the target's
            # current reading at the moment of the keypress, not a stale one
            # from whenever the second marker was first detected.
            _obj_obs, tgt_obs, _current = self._step_tracker()
            if tgt_obs is not None:
                self._last_rotation = self._rotation_from_target(tgt_obs.pose_in_pelvis[:3, :3])
                if self._mode == "6dof":
                    self._box_center = self._box_center_from_target_position(tgt_obs.pose_in_pelvis[:3, 3])
            print("[MarkerSource] Go!")
        elif self._countdown > 0.0:
            print(f"[MarkerSource] Both markers visible. Tracking starts in {self._countdown:.0f}s…")
            t_end = time.monotonic() + self._countdown
            prev_tick = int(self._countdown) + 1
            while time.monotonic() < t_end:
                remaining = t_end - time.monotonic()
                tick = int(remaining) + 1
                if tick < prev_tick:
                    prev_tick = tick
                    print(f"  {tick}…")
                # Refresh target readings during countdown
                _obj_obs, tgt_obs, _current = self._step_tracker()
                if tgt_obs is not None:
                    self._last_rotation = self._rotation_from_target(tgt_obs.pose_in_pelvis[:3, :3])
                    if self._mode == "6dof":
                        self._box_center = self._box_center_from_target_position(tgt_obs.pose_in_pelvis[:3, 3])
                time.sleep(0.05)
            print("[MarkerSource] Go!")

        return self._box_center.copy()

    def step(self) -> tuple:
        """Return (box_center, theta) for the current frame.

        Uses the current frame's detections from tracker.step(). If the target
        marker is absent this frame, returns the last known values and warns
        after 0.5s and 3.0s of continuous absence. Also monitors the object
        marker and sets object_lost after 2s of continuous absence.

        Returns:
            box_center: (3,) ndarray in pelvis frame
            theta:      (3,3) ndarray, target's FULL rotation relative to its
                        own start, in the pelvis frame (see
                        _rotation_from_target's docstring)
        """
        if self._box_center is None:
            raise RuntimeError("Call startup() before step()")

        obj_obs, tgt_obs, _current = self._step_tracker()

        # --- Target marker ---
        self._last_tgt_obs = tgt_obs
        if tgt_obs is not None:
            self._last_rotation = self._rotation_from_target(tgt_obs.pose_in_pelvis[:3, :3])
            if self._mode == "6dof":
                self._box_center = self._box_center_from_target_position(tgt_obs.pose_in_pelvis[:3, 3])
            self._target_stale_frames = 0
            self._stale_warned_05 = False
            self._stale_warned_3 = False
        else:
            self._target_stale_frames += 1
            stale_s = self._target_stale_frames * self._control_dt
            if stale_s >= 0.5 and not self._stale_warned_05:
                print(f"[MarkerSource] WARNING: target marker not seen for {stale_s:.1f}s — holding last known pose")
                self._stale_warned_05 = True
            if stale_s >= 3.0 and not self._stale_warned_3:
                print(f"[MarkerSource] WARNING: target marker not seen for {stale_s:.1f}s — holding last known pose")
                self._stale_warned_3 = True

        # --- Object marker monitoring ---
        self._last_obj_obs = obj_obs
        if obj_obs is not None:
            self._object_stale_frames = 0
            self._object_warned_2s = False
            self._object_warned_5s = False
        else:
            self._object_stale_frames += 1
            lost_s = self._object_stale_frames * self._control_dt
            if lost_s > 2.0 and not self._object_warned_2s:
                print(f"[MarkerSource] WARNING: object marker not seen for {lost_s:.1f}s — object may have moved!")
                self._object_warned_2s = True
            if lost_s > 5.0 and not self._object_warned_5s:
                print(f"[MarkerSource] WARNING: object marker not seen for {lost_s:.1f}s — holding position")
                self._object_warned_5s = True

        return self._box_center.copy(), self._last_rotation.copy()

    @property
    def box_center(self):
        """Current tracked object-center pose (pelvis frame), or None before
        startup(). This is the corrected/shifted point compute_hand_targets
        actually builds the L/R hand targets from -- distinct from the raw
        object marker's own pose (see box_half_height in the class docstring)."""
        return None if self._box_center is None else self._box_center.copy()

    @property
    def object_marker_pose(self):
        """(marker_pos, marker_R) -- the RAW object marker pose as detected
        in startup()'s Phase A, in the pelvis frame, BEFORE the
        box_half_height offset (unlike box_center, which IS offset -- see its
        docstring). None until Phase A completes. Used to place obstacles
        (e.g. the RRT* head sphere) whose own anchor offset differs from
        box_half_height's -- see kinematics/collision.py's
        head_sphere_center_from_marker."""
        if self._object_marker_pos is None:
            return None
        return self._object_marker_pos.copy(), self._object_marker_R.copy()

    @property
    def last_obj_obs(self):
        """Most recent object marker observation from the last step(), or None."""
        return self._last_obj_obs

    @property
    def last_tgt_obs(self):
        """Most recent target marker observation from the last step(), or None."""
        return self._last_tgt_obs

    @property
    def target_found(self) -> bool:
        """True if the target marker was detected during startup()'s Phase B.
        False means startup() timed out waiting for it and proceeded with
        the grasp anyway (box_center is still valid, from the object
        marker alone) -- callers should skip the tracking phase (Phase 2)
        when this is False, since there's no target motion to track."""
        return self._target_found

    @property
    def target_stale_seconds(self) -> float:
        """Seconds since target marker was last seen this frame (0.0 if seen)."""
        return self._target_stale_frames * self._control_dt

    @property
    def object_lost(self) -> bool:
        """True if object marker has not been seen for more than 2s during tracking."""
        return self._object_stale_frames * self._control_dt > 2.0

    @property
    def tracker(self):
        """The wrapped tracker (e.g. MarkerTracker), for diagnostics such as
        get_diagnostic_pose(). Not available on SineSource."""
        return self._tracker

    @property
    def obj_id(self):
        """AprilTag ID of the object marker."""
        return self._obj_id

    @property
    def tgt_id(self):
        """AprilTag ID of the target marker."""
        return self._tgt_id


class RateLimitedSource:
    """Slew-rate-limits (box_center, theta) from any wrapped trajectory source.

    Clamped against the ACTUAL measured wall-clock dt between step() calls
    (not the assumed control_dt). The control loop's true period is set by
    IK solve cost (~90-180ms measured), not control_dt (0.02s/50Hz), so a
    source sampling a real-time signal (SineSource's sine, or a live/mock
    marker's rotation) produces per-step deltas much larger than a properly
    -paced loop would, which IK's warm start then chases abruptly. This
    decouples "how fast the commanded target is allowed to move" from "how
    slowly the control loop happens to actually run" -- a slew-rate-limiter
    on the INPUT target, architecturally distinct from G1HandIK's
    WeightedMovingFilter (a moving average on the IK's OUTPUT), so the two
    do not double-filter.

    `max_dt` caps the dt used in the rate-limit math itself (default 0.5s).
    Root cause this fixes: dt is measured since the LAST step() call, with
    no assumption step() is called on any particular schedule -- callers in
    this codebase only start calling step() once Phase 2 (tracking) begins,
    so a long real gap (Phase 1a's RRT reach + Phase 1b's straight-in close
    + Phase 1.5's hand-close, easily several real seconds combined) can
    elapse between startup() and the first step() call, or between the last
    Phase 2 iteration before an interruption and a subsequent resumption.
    Without this cap, that inflated dt scales max_theta_rate*dt /
    max_pos_rate*dt proportionally, so a multi-second gap can let the
    limiter permit a swing far larger than the sine/marker signal's own
    amplitude in one single step -- i.e. the rate limiter provides
    essentially NO protection on exactly the tick right after such a gap,
    the opposite of its purpose. Capping dt at a small ceiling means a long
    real gap is treated as if only max_dt of time passed, preserving the
    limiter's real per-tick constraint regardless of how long the caller
    went without calling step(). Does not affect normal operation: an
    active Phase 2 loop's real inter-step gaps (even with the documented
    ~90-180ms IK solve cost) stay well under the default 0.5s ceiling.

    Duck-types the same .startup()/.step() contract as SineSource/MarkerSource,
    so it wraps either uniformly. Unknown attributes (e.g. MarkerSource's
    last_obj_obs/object_lost/target_stale_seconds) are proxied to the
    wrapped source via __getattr__.

    theta MAY be either a scalar (a roll-about-X float, the original
    representation) or a (3,3) rotation matrix (MarkerSource's current, full
    3-DoF representation -- see docs/six_dof_tracking_fix.md). The scalar
    path below is UNCHANGED from before -- deliberately a separate code path
    rather than routed through the matrix math, since the matrix rotvec
    round-trip only reduces to the exact same arithmetic for |theta| <= pi,
    and existing tests deliberately exercise larger values. For a (3,3)
    input, the same clamp is applied geodesically: the rotation from the
    last output to this step's target is decomposed into an axis-angle
    (rotvec), the angle is capped at max_theta_rate*dt (direction
    preserved), and the result is re-composed. max_theta_rate then bounds
    the TOTAL angular speed (rotvec norm) rather than only the X component --
    the same number, generalized, strictly tighter per-axis than before.
    A source that returns one type on one call and the other type on a
    later call (without an intervening startup()) raises TypeError.
    """

    def __init__(self, inner, max_theta_rate, max_pos_rate, clock=time.monotonic,
                 max_dt=0.5):
        self._inner = inner
        self._max_theta_rate = float(max_theta_rate)  # rad/s
        self._max_pos_rate = float(max_pos_rate)       # m/s
        self._clock = clock
        self._max_dt = float(max_dt)                   # s
        self._last_t = None
        self._last_box_center = None
        self._last_theta = None       # scalar branch state
        self._last_rotation = None    # matrix branch state
        self._is_scalar = None        # which branch is active (None = unseeded)

    def startup(self, *args, **kwargs):
        box_center = self._inner.startup(*args, **kwargs)
        self._last_t = None  # next step() is always unclamped (fresh seed)
        self._is_scalar = None
        return box_center

    def step(self) -> tuple:
        box_center, theta = self._inner.step()
        box_center = np.asarray(box_center, dtype=float)
        scalar = is_scalar_theta(theta)
        now = self._clock()

        if self._last_t is None:
            self._is_scalar = scalar
            if scalar:
                theta = float(theta)
                self._last_theta = theta
            else:
                theta = np.asarray(theta, dtype=float).copy()
                self._last_rotation = theta.copy()
            self._last_box_center = box_center.copy()
            self._last_t = now
            return box_center.copy(), theta

        if scalar != self._is_scalar:
            # Re-seed rather than raise: this runs inside the Phase 2 hot loop,
            # which only catches KeyboardInterrupt, so raising here would kill
            # a live tracking run outright. A type flip is a programming error,
            # not a safety condition -- warn loudly and carry on from the new
            # type (this tick is unclamped, exactly like a fresh startup seed).
            print(f"[RateLimitedSource] WARNING: inner source's theta type changed "
                  f"mid-run (was {'scalar' if self._is_scalar else 'a (3,3) rotation'}, "
                  f"now {'scalar' if scalar else 'a (3,3) rotation'}) -- re-seeding "
                  f"and continuing unclamped for this step.")
            self._is_scalar = scalar
            if scalar:
                theta = float(theta)
                self._last_theta = theta
            else:
                theta = np.asarray(theta, dtype=float).copy()
                self._last_rotation = theta.copy()
            self._last_box_center = box_center.copy()
            self._last_t = now
            return box_center.copy(), theta

        dt = max(now - self._last_t, 1e-6)
        dt = min(dt, self._max_dt)
        self._last_t = now

        if scalar:
            theta = float(theta)
            dtheta = theta - self._last_theta
            max_dtheta = self._max_theta_rate * dt
            if abs(dtheta) > max_dtheta:
                dtheta = math.copysign(max_dtheta, dtheta)
            self._last_theta = self._last_theta + dtheta
            out_theta = self._last_theta
        else:
            R_target = np.asarray(theta, dtype=float)
            dR = self._last_rotation.T @ R_target
            v = Rotation.from_matrix(dR).as_rotvec()
            angle = np.linalg.norm(v)
            max_angle = self._max_theta_rate * dt
            if angle > max_angle and angle > 0.0:
                v = v * (max_angle / angle)
            self._last_rotation = self._last_rotation @ Rotation.from_rotvec(v).as_matrix()
            out_theta = self._last_rotation.copy()

        delta = box_center - self._last_box_center
        dist = np.linalg.norm(delta)
        max_dist = self._max_pos_rate * dt
        if dist > max_dist and dist > 0.0:
            delta = delta * (max_dist / dist)
        self._last_box_center = self._last_box_center + delta

        return self._last_box_center.copy(), out_theta

    def __getattr__(self, name):
        return getattr(self._inner, name)


def _anchored_median_rotation(hist):
    """Elementwise median of a short list of (3,3) rotations, computed
    SAFELY near a +/-pi wrap by anchoring on the history's first rotation
    and taking rotvecs of small deltas RELATIVE to that anchor (rather than
    taking an elementwise median of the rotations' own raw, possibly-large
    rotvecs, which is wrap-unsafe). Exact for the in-range pure-X case:
    median(theta_i - theta_0) + theta_0 == median(theta_i)."""
    R_anchor = hist[0]
    deltas = np.stack(
        [Rotation.from_matrix(R_anchor.T @ R).as_rotvec() for R in hist], axis=0
    )
    med_delta = np.median(deltas, axis=0)
    return R_anchor @ Rotation.from_rotvec(med_delta).as_matrix()


class NoiseFilteredSource:
    """Smooths real-sensor (box_center, theta) noise from a wrapped source.

    Two stages, deliberately in this order:
      1. Median-of-N (per-DOF): rejects a single-frame outlier (e.g. a
         partially-occluded/motion-blurred AprilTag misdecode for exactly one
         frame) that an EMA alone would partially incorporate.
      2. Single-pole EMA: smooths the remaining small, continuous jitter that
         solvePnP produces even for a stationary marker -- noise well BELOW
         RateLimitedSource's clamp threshold, which only engages on deltas
         LARGER than its configured rate.

    Architecturally distinct from RateLimitedSource (cleans the raw signal
    vs. bounds the rate of the already-clean signal) and from G1HandIK's
    WeightedMovingFilter (smooths solve_ik's OUTPUT sol_q; this smooths the
    trajectory's INPUT before compute_hand_targets/solve_ik ever see it).
    Intended composition: source -> NoiseFilteredSource -> RateLimitedSource
    -> controller (noise-filtering innermost since it cleans the raw signal;
    slew-limiting outermost since it bounds the rate of that cleaned signal,
    including any real fast motion that follows).

    Duck-types .startup()/.step(); proxies unknown attributes to the wrapped
    source via __getattr__, matching RateLimitedSource's pattern.

    theta MAY be either a scalar or a (3,3) rotation matrix (see
    RateLimitedSource's docstring and docs/six_dof_tracking_fix.md); the
    scalar path is UNCHANGED from before. For a (3,3) input, both stages are
    done geodesically rather than elementwise on raw rotvecs, to avoid a
    wrap-around hazard near +/-pi that elementwise rotvec math doesn't
    handle correctly: the median anchors the history's first rotation and
    takes an elementwise median of small SMALL rotvecs relative to that
    anchor (exact for the in-range pure-X case); the EMA blends geodesically
    (R_ema <- R_ema @ Exp(alpha * Log(R_ema.T @ R_med))), which also reduces
    to the exact scalar EMA formula for pure-X input. A source that returns
    one type on one call and the other on a later call raises TypeError.
    """

    def __init__(self, inner, median_window=5, ema_alpha=0.3):
        if median_window < 1 or median_window % 2 == 0:
            raise ValueError(
                f"median_window must be a positive odd integer, got {median_window}"
            )
        self._inner = inner
        self._median_window = int(median_window)
        self._ema_alpha = float(ema_alpha)
        self._box_hist = []
        self._theta_hist = []
        self._ema_box = None
        self._ema_theta = None
        self._is_scalar = None  # which branch is active (None = unseeded)

    def startup(self, *args, **kwargs):
        box_center = self._inner.startup(*args, **kwargs)
        self._box_hist, self._theta_hist = [], []
        self._ema_box, self._ema_theta = None, None
        self._is_scalar = None
        return box_center

    def step(self) -> tuple:
        box_center, theta = self._inner.step()
        box_center = np.asarray(box_center, dtype=float)
        scalar = is_scalar_theta(theta)
        if self._is_scalar is None:
            self._is_scalar = scalar
        elif scalar != self._is_scalar:
            # Reset history and carry on rather than raising -- see the matching
            # comment in RateLimitedSource.step() for why an exception here
            # would kill a live Phase 2 run.
            print(f"[NoiseFilteredSource] WARNING: inner source's theta type changed "
                  f"mid-run (was {'scalar' if self._is_scalar else 'a (3,3) rotation'}, "
                  f"now {'scalar' if scalar else 'a (3,3) rotation'}) -- clearing the "
                  f"filter history and continuing.")
            self._is_scalar = scalar
            self._box_hist, self._theta_hist = [], []
            self._ema_box, self._ema_theta = None, None
        theta = float(theta) if scalar else np.asarray(theta, dtype=float).copy()

        self._box_hist.append(box_center.copy())
        self._theta_hist.append(theta)
        if len(self._box_hist) > self._median_window:
            self._box_hist.pop(0)
            self._theta_hist.pop(0)

        if len(self._box_hist) < self._median_window:
            med_box, med_theta = box_center, theta
        else:
            med_box = np.median(np.stack(self._box_hist, axis=0), axis=0)
            if scalar:
                med_theta = float(np.median(self._theta_hist))
            else:
                med_theta = _anchored_median_rotation(self._theta_hist)

        if self._ema_box is None:
            self._ema_box = med_box.copy()
            self._ema_theta = med_theta if scalar else med_theta.copy()
        else:
            self._ema_box = self._ema_alpha * med_box + (1.0 - self._ema_alpha) * self._ema_box
            if scalar:
                self._ema_theta = self._ema_alpha * med_theta + (1.0 - self._ema_alpha) * self._ema_theta
            else:
                v = Rotation.from_matrix(self._ema_theta.T @ med_theta).as_rotvec()
                self._ema_theta = self._ema_theta @ Rotation.from_rotvec(self._ema_alpha * v).as_matrix()

        return self._ema_box.copy(), (self._ema_theta if scalar else self._ema_theta.copy())

    def __getattr__(self, name):
        return getattr(self._inner, name)


class RotationClampSource:
    """Safety clamp on the tracked rotation from a wrapped source -- caps its
    TOTAL angle and/or restricts it to a subset of axes. See
    utils.transforms.clamp_rotation (this class is a thin .step() wrapper
    around it) and docs/six_dof_tracking_fix.md.

    Unlike RateLimitedSource (a RATE bound -- how fast the commanded value
    may change per tick), this is a POSITION bound -- how far the commanded
    value may ever be from identity, regardless of how it got there. Both
    matter together: RateLimitedSource's first step after startup() is
    deliberately UNCLAMPED (a fresh seed), so a large real target rotation
    accumulated during Phase 1/1.5 (several real seconds with no step()
    calls) would otherwise pass straight through on the very first Phase-2
    tick. Wire this OUTERMOST (after RateLimitedSource) so it has the last
    word on every tick, including that first one.

    Always converts its input to a (3,3) matrix via as_rotation and returns
    a (3,3) matrix (never a bare scalar) -- compute_hand_targets/press_axis
    accept either, so there's no reason for this wrapper (which sits right
    at the safety boundary) to re-derive a lossy scalar.

    max_angle=None and enabled_axes=None (both defaults) is an exact
    identity passthrough -- disabled by construction, matching this
    codebase's "new wrapper reproduces old behavior when unconfigured"
    convention.

    Duck-types .startup()/.step(); proxies unknown attributes to the wrapped
    source via __getattr__, matching RateLimitedSource/NoiseFilteredSource.
    """

    def __init__(self, inner, max_angle=None, enabled_axes=None,
                 warn_interval_s=2.0, clock=time.monotonic):
        self._inner = inner
        self._max_angle = None if max_angle is None else float(max_angle)
        self._enabled_axes = enabled_axes
        self._warn_interval_s = float(warn_interval_s)
        self._clock = clock
        self._last_warn_t = None

    def startup(self, *args, **kwargs):
        return self._inner.startup(*args, **kwargs)

    def step(self) -> tuple:
        box_center, theta = self._inner.step()
        R_in = as_rotation(theta)
        R_out = clamp_rotation(R_in, max_angle=self._max_angle, enabled_axes=self._enabled_axes)
        if not np.allclose(R_in, R_out, atol=1e-9):
            # Rate-limited rather than once-per-run: a single early message
            # leaves no way to tell the clamp is STILL engaging later in the
            # run, which makes a clamped arm look like a frozen one.
            now = self._clock()
            if self._last_warn_t is None or (now - self._last_warn_t) >= self._warn_interval_s:
                self._last_warn_t = now
                print(f"[RotationClampSource] Tracked rotation clamped: "
                      f"requested {math.degrees(rotation_angle(R_in)):.1f}° -> "
                      f"{math.degrees(rotation_angle(R_out)):.1f}° "
                      f"(trajectory.tracked_rotation limits). Tracking continues.")
        return box_center, R_out

    def __getattr__(self, name):
        return getattr(self._inner, name)


class PredictiveSource:
    """Adds the torso GP's predicted lead to a wrapped marker source.

    Cancels tracking LAG, not static frame error. The controller's Phase-2
    loop runs at ~6-10Hz and each command lands one tick late against a
    moving target; the GP (gp_predict/export) predicts where the target
    marker is heading, so the commanded pose leads the measured one by
    roughly that tick.

    This is CONTROLLER_MATH.md's SS1-4 "feedback + GP feedforward" form:
    the wrapped source keeps producing the measured (box_center, theta) and
    this class adds `twist * dt` on top. Nothing about the servo changes.

    Frames -- the trap worth reading twice. TorsoGP.twist() returns
    [v, omega] in the target marker's OWN BODY frame at the current step,
    not the pelvis frame:
      * linear  is rotated into pelvis   (R_meas @ v_body)
      * angular is right-multiplied      (R @ Exp(w_body * dt))
    Left-multiplying the angular part would apply a body-frame rotation as
    though it were a pelvis-frame one -- the exact bug class that made
    tracking silently single-axis before docs/six_dof_tracking_fix.md.

    The GP refreshes at its own rate (step_dt ~228ms, ~4.4Hz) while this
    steps at the controller's rate; twist() is a VELOCITY, so the held
    value is simply re-integrated over each tick's measured dt. No
    interpolation between predictions is needed (or wanted -- bracketing
    two predictions would reintroduce a tick of lag).

    Deliberately wired INSIDE RateLimitedSource/RotationClampSource so a
    prediction can never escape the existing safety envelope.

    Falls back to a zero lead (i.e. exactly the wrapped source's own output)
    whenever the GP has no prediction yet -- the export's rule #4: freeze,
    never constant-twist extrapolate.
    """

    def __init__(self, inner, gp, lead_scale=1.0, max_lead_dt=0.5,
                 require_arm=True):
        self._inner = inner
        self._gp = gp
        self._lead_scale = float(lead_scale)
        self._max_lead_dt = float(max_lead_dt)
        # require_arm=True (default): the lead stays off until arm() is
        # called, so the controller controls WHEN the GP anchors. False
        # anchors at startup() -- only sensible for a source with no
        # approach/grasp phase in front of it (e.g. a bare replay harness).
        self._require_arm = bool(require_arm)
        self._armed = False
        self._twist = None
        self._last_t = None
        self.n_accepted = 0      # GP refreshes (diagnostics)
        self.n_steps = 0         # ticks seen

    def startup(self, *args, **kwargs):
        box_center = self._inner.startup(*args, **kwargs)
        self._twist = None
        self._last_t = None
        self.n_accepted = 0
        self.n_steps = 0
        # NOT anchored here. startup() runs before Phase 1a -- long before the
        # hands have even reached the head -- so anchoring now would violate
        # the export's rule #1 (anchor at the START of a motion segment) by
        # several seconds of approach, grasp and press. The controller calls
        # arm() at the right instant instead; until then step() passes the
        # wrapped source through untouched.
        self._armed = not self._require_arm
        if self._armed:
            self._gp.reset()
        return box_center

    def arm(self):
        """Anchor the GP now and start applying the lead.

        Called by the controller once the grip is established (after the palm
        press seats), which is when the human actually begins moving the
        torso. Idempotent -- a second call is ignored, so it is safe to call
        from inside a per-tick loop.
        """
        if self._armed:
            return False
        self._armed = True
        self._gp.reset()
        self._twist = None
        self._last_t = None
        return True

    @property
    def armed(self) -> bool:
        return self._armed

    def step(self) -> tuple:
        box_center, theta = self._inner.step()
        box_center = np.asarray(box_center, dtype=float)
        self.n_steps += 1

        if not self._armed:
            # Pass-through until the controller arms us. Deliberately does NOT
            # feed gp.update() either: samples taken during the approach would
            # become the anchor's velocity history, which is exactly the stale
            # pre-motion state rule #1 says to avoid.
            return box_center, theta

        obs = getattr(self._inner, "last_tgt_obs", None)
        if obs is None:
            return box_center, theta

        now = time.monotonic()
        p = obs.pose_in_pelvis[:3, 3]
        R_meas = obs.pose_in_pelvis[:3, :3]
        if self._gp.update(now, p, R_meas):
            tw = self._gp.twist()
            if tw is not None:
                self._twist = tw[0]
                self.n_accepted += 1

        if self._twist is None:            # no prediction yet -> freeze
            self._last_t = now
            return box_center, theta

        dt = 0.0 if self._last_t is None else (now - self._last_t)
        self._last_t = now
        # Cap dt for the same reason RateLimitedSource caps its own: a long
        # gap with no step() calls (Phase 1a/1b/1.5) would otherwise
        # integrate the held twist into an enormous one-tick lead.
        dt = min(max(dt, 0.0), self._max_lead_dt) * self._lead_scale
        if dt <= 0.0:
            return box_center, theta

        v_body, w_body = self._twist[:3], self._twist[3:]

        # Position lead only makes sense when box_center actually tracks the
        # target's displacement (6dof). orient-only holds it fixed by
        # design, so leading it there would add translation the mode
        # deliberately excludes.
        if getattr(self._inner, "_mode", None) == "6dof":
            box_center = box_center + (R_meas @ v_body) * dt

        if is_scalar_theta(theta):
            # Scalar theta is a roll about pelvis X; take the matching
            # component of the predicted angular velocity in the pelvis frame.
            theta = float(theta) + float((R_meas @ w_body)[0]) * dt
        else:
            R_theta = as_rotation(theta)
            lead = Rotation.from_rotvec(w_body * dt).as_matrix()
            # theta is a PELVIS-frame delta while w_body is body-frame, so
            # the lead is conjugated into the pelvis frame before composing.
            theta = R_theta @ (R_meas @ lead @ R_meas.T)

        return box_center, theta

    def __getattr__(self, name):
        return getattr(self._inner, name)
