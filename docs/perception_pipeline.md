# Perception Pipeline: AprilTag Pose Tracking for a Humanoid Robot

A Unitree G1 humanoid holds an object in both hands and has to match the
object's pose to a moving reference, such as a person's torso. A RealSense
D435 on the robot's head tracks two AprilTags:

- an **object marker** on the held object, read once at grasp time;
- a **target marker** on the reference, read every control tick.

This document covers the perception stack that turns camera frames into a
pose command for the arm controller. Most of it is about the work a lab demo
can skip: dropped frames, occlusion, noise, slow and irregular loop timing,
calibration error, and a camera mounted on a body that moves.

All perception code is mine. It lives in
[`g1_spine_stabilization/perception/`](../g1_spine_stabilization/perception/)
and the marker sources in
[`trajectory/sources.py`](../g1_spine_stabilization/trajectory/sources.py).

---

## 1. Architecture

```
 Frame source            Detector                  Tracker                  Pose source stack (filters run inner → outer)
 ───────────────         ─────────────────         ──────────────────       ────────────────────────────────────────────────────────
 RealSenseSource   ──►   AprilTagDetector    ──►   MarkerTracker      ──►   MarkerSource → NoiseFilteredSource → PredictiveSource
  (color | IR)            cv2.aruco + PnP           optical → mount           → RateLimitedSource → RotationClampSource → IK / arm
 VideoFileSource          (stateless)               → pelvis transform
 ImageFileSource                                    dropped-frame handling
 (dry-run synthetic)                                reprojection gate
```

Each layer has a small interface and can be replaced on its own:

| Layer | Contract | Swappable for |
|---|---|---|
| Frame source | `read() → (frame, timestamp) \| None` | live D435 color, D435 IR, a video file, a still image, synthetic dry-run frames |
| Detector | `detect(frame, intrinsics) → [RawDetection]`, pure function | another fiducial or a learned detector |
| Tracker | `step() → [MarkerObservation]` in the robot's pelvis frame | `MockMarkerTracker` (scripted poses, no camera) |
| Pose source | `startup() → box_center`, `step() → (box_center, R)` | `SineSource` / animated mock for testing without a camera |

Because of this split, almost everything above the camera driver is tested
without hardware. The suite has stub frame sources, stub detectors, mock
trackers, and injectable clocks. Every live-robot run first goes through a
dry-run preview (`sine → mock → marker`) that uses the same code path.

## 2. Pose estimation

**Detection and PnP**
([`detector.py`](../g1_spine_stabilization/perception/detector.py)). The
detector uses `cv2.aruco.ArucoDetector` with `DICT_APRILTAG_36h11`, then
`cv2.solvePnP` with `SOLVEPNP_IPPE_SQUARE`, a solver built specifically for
square planar targets. Object points are written out explicitly in the
detector's corner order (TL, TR, BR, BL); the deprecated
`estimatePoseSingleMarkers` isn't used. Each detection carries its mean
**reprojection error**, which is used for quality gating (section 3).

**Frame chain**
([`frames.py`](../g1_spine_stabilization/perception/frames.py)):

```
pose_in_pelvis = T_d435_to_pelvis · T_optical_to_d435 · pose_in_optical
```

- `T_optical_to_d435` converts the camera optical frame (OpenCV: X right,
  Y down, Z forward) to the ROS camera-mount frame (X forward, Y left, Z up),
  using the standard REP-103/105 rotation.
- `T_d435_to_pelvis` comes from **forward kinematics on the robot's URDF**
  (Pinocchio), because the camera is on the head, not on a fixed base.

**Live waist correction.** The first version of `T_d435_to_pelvis` assumed
the waist joints were at zero. On the real robot, the balance controller
tilts the waist to counter a loaded, outstretched arm. FK shows that a 5°
waist pitch shifts the camera mount by **37 mm**, and the error grows further
out at the marker. The fix reads the live waist encoders after the robot
connects, recomputes FK
([`waist_q_full`](../g1_spine_stabilization/perception/frames.py#L51)), and
updates the tracker in place with
[`set_T_d435_to_pelvis`](../g1_spine_stabilization/perception/tracker.py#L143).

**Infrared stream for a wider field of view.** Both markers often didn't fit
in the color camera's view. The D435's IR stereo module has a wider field of
view (87×58° against 69×42° for the color sensor), but the vendored SDK only
exposes color. I added an IR mode that talks to `pyrealsense2` directly
([`_connect_infrared`](../g1_spine_stabilization/perception/frame_sources.py#L108))
and maps IR detections into the color frame using the camera's **factory
IR→color extrinsic, read live**, so no second manual calibration is needed.

One pitfall was caught before it shipped: `rs.extrinsics.rotation` is stored
**column-major**. Reading it row-major silently gives the transpose, which is
a valid rotation matrix but the wrong one. The fix is an explicit `order="F"`
reshape
([`_extrinsics_to_se3`](../g1_spine_stabilization/perception/frame_sources.py#L49)),
with a test that uses a deliberately asymmetric matrix so a transpose can't
pass.

Validation on the real camera: the same marker seen through the color path
and the IR path agreed to **3.0 mm / 1.5°** in the pelvis frame. In the same
scan, IR saw both markers while color saw only one.

## 3. Handling real-world failures

The guiding rule is **warn, don't halt.** The perception layer runs inside a
control loop while the robot is holding an object. Any exception that escapes
it could interrupt a grasp, so each failure mode degrades to a safe, defined
behavior and logs it.

### 3.1 Dropped frames and camera faults

[`MarkerTracker.step()`](../g1_spine_stabilization/perception/tracker.py#L75)
never raises because of the camera. A read that throws (a librealsense
`wait_for_frames` timeout, a USB hiccup) or returns an empty frame counts as
**dropped**, and `step()` returns `[]`, the same "nothing seen this tick"
result as a frame with no markers. The layer above already handles that case
by holding the last known pose (3.2), so a camera outage needs no special
handling downstream.

Logging is edge-triggered: one line on the first drop, a reminder every 30
consecutive drops (about 1 s), and a line on recovery that gives the outage
length. A sustained outage is loud without flooding the console. A `stats`
property exposes `frames_ok`, `frames_dropped`, `consecutive_dropped`, and
`detections_rejected` for post-run review. `KeyboardInterrupt` still
propagates, so the operator can always stop the run.

### 3.2 Occlusion and markers leaving the frame

When the target marker isn't seen,
[`MarkerSource.step()`](../g1_spine_stabilization/trajectory/sources.py#L339)
**holds the last known pose** and tracks how long the marker has been
missing, with warnings at 0.5 s and 3 s. If the object marker is missing for
more than 2 s, `object_lost` is set, which the controller uses as an
advisory.

Startup is split into phases. If the target marker never appears, the robot
still grasps using the object marker alone, then holds until the operator
releases it. It doesn't abort.

**A timing bug fixed during this cleanup.** Staleness used to be computed as
`frames_missed × control_dt`, with `control_dt` = 20 ms (the nominal 50 Hz).
The real loop runs at 6–10 Hz because each IK solve takes about 100 ms, so
every staleness threshold fired **5–9× late**: the "2 s" object-lost flag
actually tripped after 10–17 s. Staleness is now measured with a monotonic
clock that tests can replace. The regression test shows `object_lost`
tripping after about 15 realistic ticks; the old formula needed 101.

### 3.3 Pose noise and outliers

Even on a static tag, PnP poses jitter from frame to frame.
[`NoiseFilteredSource`](../g1_spine_stabilization/trajectory/sources.py#L633)
runs two filters in series:

1. **Median over the last N frames.** This rejects single-frame outliers
   such as a corner mis-detection or a momentary PnP flip, which a plain
   moving average would smear into the output instead of removing.
2. **Exponential moving average (EMA)** for smoothness.

Rotations are filtered **on SO(3)**, not on raw rotation-vector components,
because component-wise averaging breaks near the ±π wrap. The median is
taken in the tangent space around an anchor rotation; the EMA blends
geodesically: `R ← R · Exp(α · Log(Rᵀ R_med))`.

### 3.4 Bad pose fits

Motion blur, glare, or a partly covered tag can produce a pose that looks
confident but is wrong. The tracker's opt-in `max_reprojection_error` gate
drops detections whose PnP reprojection error is above a pixel threshold
(config: `perception.max_reprojection_error_px`, off by default until a
threshold is measured on real data).

### 3.5 Irregular loop timing and jumps after gaps

[`RateLimitedSource`](../g1_spine_stabilization/trajectory/sources.py#L466)
limits how fast the target position and rotation can change, using the
**measured** time since the last tick, not the assumed one. A second fix
handles long gaps: the elapsed time used in the limit is capped
(`max_dt`, 0.5 s). Without the cap, a few seconds with no ticks during the
grasp phases would have allowed a 120° step on the first tracking tick. With
it, a long pause can't be used as permission for a large jump.

### 3.6 Latency compensation

Each arm command lands about one control tick after the measurement it came
from. [`PredictiveSource`](../g1_spine_stabilization/trajectory/sources.py#L806)
adds a feedforward lead from a Gaussian-process model of torso motion, which
predicts the marker's twist. Two details matter here:

- The prediction is in the marker's **body frame**: velocity is rotated into
  the pelvis frame, and angular velocity is right-multiplied.
- The predictor sits **inside** the rate limiter and the rotation clamp, so
  a bad prediction can't escape the safety envelope. When no prediction is
  available, the lead is zero; it is never extrapolated at a constant rate.

### 3.7 Safety envelope

[`RotationClampSource`](../g1_spine_stabilization/trajectory/sources.py#L743)
is the outermost layer. It bounds the total tracked rotation and can turn
off individual axes, so no perception error can command a pose beyond a
configured angle.

## 4. Two design decisions worth calling out

**Track the target's displacement, not its absolute pose.** The object (in
the robot's hands) and the target (held by a person) are in different
places. An early version set the grasp center to the target's absolute
position, which sent the hands to wherever the target was. Both tracking
modes now apply the target's displacement from its own starting pose to the
object's starting pose.

**Rotation deltas must be in the world frame.** A later investigation found
that "6-DoF" tracking was really 3+1 DoF: the target's rotation was reduced
to a single roll angle, and the delta was taken in the marker's body frame
when it needed to be in the pelvis frame. Pure single-axis rotations commute,
which is why tests never caught it. The fix computes `ΔR = R_now · R_startᵀ`
(world frame) and carries full rotation matrices through every filter. It
added regression tests that fail against the old code, including one where
pure yaw produced zero response. Full write-up:
[`six_dof_tracking_fix.md`](six_dof_tracking_fix.md).

## 5. Debugging story: when "calibration error" wasn't calibration

Live markers showed up about **10 cm / 9°** off from their measured positions.
I built a closed-form extrinsic solver
([`calibration.py`](../g1_spine_stabilization/perception/calibration.py))
and a calibration script with a held-out verification mode, and fitted
corrections. A fit that matched within 6 mm at one point then diverged by
**6.8 cm** at a second, non-collinear point. That is the signature of a fit
absorbing some other error rather than measuring the mount.

The cause was the configured **marker size**. A wrong tag edge length scales
every PnP translation linearly. Once the size was measured correctly, the
recomputed extrinsic landed close to the standard default. Because no fit had
been checked against a held-out point, I reverted to the standard default
rather than keep an unverified correction. I documented a multi-point
procedure (3–4 non-collinear points, at least one held out) as the bar for
replacing it.

The intermediate frames (optical, mount, pelvis) are kept per marker
([`get_diagnostic_pose`](../g1_spine_stabilization/perception/tracker.py)) so
a physical measurement can be compared stage by stage. That is how this kind
of error gets isolated to one transform.

## 6. Current status and limitations

- **Extrinsic calibration** uses the standard default and hasn't been verified
  at multiple points. The expected end-to-end error budget is about
  5–15 mm at the hand.
- **Timestamps:** staleness is measured on the control loop's clock, not the
  frame capture time. Color frames use the SDK's timestamp and IR frames are
  stamped when read. Feeding hardware capture timestamps through would give
  exact age-of-measurement for latency compensation.
- **Planar PnP ambiguity:** IPPE returns the better of two candidate poses,
  and the median filter catches isolated flips. A tag seen nearly face-on at
  a distance can still flip for several frames in a row. A temporal
  consistency check against the previous pose would harden this.
- The dropped-frame handling and reprojection gate added in this cleanup are
  covered by hardware-free tests. They haven't yet been exercised against a
  real camera fault.
