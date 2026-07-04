"""
Trajectory drivers — produce the time-varying signal that the controller tracks.

sine_trajectory is the original placeholder. MarkerSource replaces it for live
AprilTag-driven control, exposing the same (box_center, theta) interface so the
controller doesn't change.
"""

import math
import time

import numpy as np


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

    Two tracking modes:
      "orient-only" — box_center is captured from the object marker once at
          startup and held fixed; theta follows the target marker's X-rotation.
      "6dof"        — box_center tracks the target marker's position each step;
          theta still follows the target marker's X-rotation.

    Args:
        tracker:           a MarkerTracker or duck-typed equivalent whose step()
                           returns list[obs] for the current frame and
                           get_latest(id) returns the last known obs.
        object_marker_id:  AprilTag ID of the object marker (pose at grasp time).
        target_marker_id:  AprilTag ID of the target marker (live tracking).
        mode:              "orient-only" or "6dof".
        countdown:         seconds to count down after both markers are detected
                           in startup() before returning. 0.0 skips the countdown
                           (use for mock trackers and unit tests).
        control_dt:        control loop period in seconds; used for stale-time
                           calculations (does not introduce delays in step()).
    """

    def __init__(self, tracker, object_marker_id, target_marker_id,
                 mode="orient-only", countdown=0.0, control_dt=0.02):
        if mode not in ("orient-only", "6dof"):
            raise ValueError(f"mode must be 'orient-only' or '6dof', got {mode!r}")
        self._tracker = tracker
        self._obj_id = object_marker_id
        self._tgt_id = target_marker_id
        self._mode = mode
        self._countdown = countdown
        self._control_dt = control_dt
        self._box_center = None      # set by startup()
        self._last_theta = 0.0
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

    def startup(self, timeout=10.0) -> np.ndarray:
        """Block until both markers are detected, then optionally count down.

        Phase A: waits for the object marker → records box_center.
        Phase B: waits for the target marker → pre-seeds _last_theta.
        Phase C: counts down self._countdown seconds while continuously
                 refreshing readings; skipped when countdown=0.0.

        Returns the box_center (pelvis frame). Raises RuntimeError on timeout.
        """
        # Phase A: object marker
        print(f"[MarkerSource] Waiting for object marker (id={self._obj_id})…")
        deadline = time.monotonic() + timeout
        found_obj = False
        while time.monotonic() < deadline:
            current = self._tracker.step()
            obj_obs = next((o for o in current if o.marker_id == self._obj_id), None)
            if obj_obs is not None:
                self._box_center = obj_obs.pose_in_pelvis[:3, 3].copy()
                print(f"[MarkerSource] Object marker detected: box_center={np.round(self._box_center, 4)}")
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
            current = self._tracker.step()
            tgt_obs = next((o for o in current if o.marker_id == self._tgt_id), None)
            if tgt_obs is not None:
                R = tgt_obs.pose_in_pelvis[:3, :3]
                self._last_theta = float(np.arctan2(R[2, 1], R[2, 2]))
                if self._mode == "6dof":
                    self._box_center = tgt_obs.pose_in_pelvis[:3, 3].copy()
                print(f"[MarkerSource] Target marker detected: theta={math.degrees(self._last_theta):.1f}°")
                found_tgt = True
                break
            time.sleep(0.05)
        if not found_tgt:
            raise RuntimeError(
                f"[MarkerSource] Target marker (id={self._tgt_id}) not detected within {timeout}s"
            )

        # Phase C: countdown (skip when countdown=0.0)
        if self._countdown > 0.0:
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
                current = self._tracker.step()
                tgt_obs = next((o for o in current if o.marker_id == self._tgt_id), None)
                if tgt_obs is not None:
                    R = tgt_obs.pose_in_pelvis[:3, :3]
                    self._last_theta = float(np.arctan2(R[2, 1], R[2, 2]))
                    if self._mode == "6dof":
                        self._box_center = tgt_obs.pose_in_pelvis[:3, 3].copy()
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
            theta:      float, Rx rotation angle in radians
        """
        if self._box_center is None:
            raise RuntimeError("Call startup() before step()")

        current = self._tracker.step()

        # --- Target marker ---
        tgt_obs = next((o for o in current if o.marker_id == self._tgt_id), None)
        self._last_tgt_obs = tgt_obs
        if tgt_obs is not None:
            R = tgt_obs.pose_in_pelvis[:3, :3]
            self._last_theta = float(np.arctan2(R[2, 1], R[2, 2]))
            if self._mode == "6dof":
                self._box_center = tgt_obs.pose_in_pelvis[:3, 3].copy()
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
        obj_obs = next((o for o in current if o.marker_id == self._obj_id), None)
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

        return self._box_center.copy(), self._last_theta

    @property
    def last_obj_obs(self):
        """Most recent object marker observation from the last step(), or None."""
        return self._last_obj_obs

    @property
    def last_tgt_obs(self):
        """Most recent target marker observation from the last step(), or None."""
        return self._last_tgt_obs

    @property
    def target_stale_seconds(self) -> float:
        """Seconds since target marker was last seen this frame (0.0 if seen)."""
        return self._target_stale_frames * self._control_dt

    @property
    def object_lost(self) -> bool:
        """True if object marker has not been seen for more than 2s during tracking."""
        return self._object_stale_frames * self._control_dt > 2.0


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

    Duck-types the same .startup()/.step() contract as SineSource/MarkerSource,
    so it wraps either uniformly. Unknown attributes (e.g. MarkerSource's
    last_obj_obs/object_lost/target_stale_seconds) are proxied to the
    wrapped source via __getattr__.
    """

    def __init__(self, inner, max_theta_rate, max_pos_rate, clock=time.monotonic):
        self._inner = inner
        self._max_theta_rate = float(max_theta_rate)  # rad/s
        self._max_pos_rate = float(max_pos_rate)       # m/s
        self._clock = clock
        self._last_t = None
        self._last_box_center = None
        self._last_theta = None

    def startup(self, *args, **kwargs):
        box_center = self._inner.startup(*args, **kwargs)
        self._last_t = None  # next step() is always unclamped (fresh seed)
        return box_center

    def step(self) -> tuple:
        box_center, theta = self._inner.step()
        box_center = np.asarray(box_center, dtype=float)
        theta = float(theta)
        now = self._clock()

        if self._last_t is None:
            self._last_box_center = box_center.copy()
            self._last_theta = theta
            self._last_t = now
            return box_center.copy(), theta

        dt = max(now - self._last_t, 1e-6)
        self._last_t = now

        dtheta = theta - self._last_theta
        max_dtheta = self._max_theta_rate * dt
        if abs(dtheta) > max_dtheta:
            dtheta = math.copysign(max_dtheta, dtheta)
        self._last_theta = self._last_theta + dtheta

        delta = box_center - self._last_box_center
        dist = np.linalg.norm(delta)
        max_dist = self._max_pos_rate * dt
        if dist > max_dist and dist > 0.0:
            delta = delta * (max_dist / dist)
        self._last_box_center = self._last_box_center + delta

        return self._last_box_center.copy(), self._last_theta

    def __getattr__(self, name):
        return getattr(self._inner, name)
