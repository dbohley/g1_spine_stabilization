"""Shared trial-data recorder for robot and human pose-tracking runs.

Depends only on numpy/scipy/pickle/stdlib -- no robot, no IK, no
g1_inspire_sdk import -- so it can be imported unmodified by both
controllers/sdk_pose_tracking.py (robot Phase 2) and
scripts/record_human_trial.py (perception-only, no robot). Importing the
SAME module from both call sites is what guarantees the two produce
identical pickle schemas, rather than relying on two independently
maintained implementations staying in sync.

Full reference (schema, filename convention, camera/calibration invariance
properties, marker axis convention): docs/data_recording_conventions.md.

=== Reference-frame convention for angle_diff_rpy (read before using) ===

angle_diff_rpy decomposes the SINGLE relative rotation R_reference.T @
R_moving into per-axis (roll, pitch, yaw) angles -- NOT independent
per-marker Euler angles subtracted component-wise. Decomposition happens
in R_reference's OWN local axes (roll about reference's local X, pitch
about the once-rotated Y, yaw about the twice-rotated Z) -- this is the
standard biomechanics convention of describing a distal/moving segment's
orientation relative to a proximal/stable reference segment (e.g. head
relative to torso), NOT a symmetric "difference" between the two markers.

RunRecorder.record() always calls this with `target` as the reference and
`object` as the moving segment: angle_diff_rpy(R_target, R_object). This
is correct for BOTH conventions this module serves, because `target`
already plays the "stable reference" role by definition in each:
  - Robot (box-tracking): target = the external pose the held object must
    track (the tracking goal / reference signal).
  - Human neck trial: target_marker_id = torso (the reference), object_
    marker_id = head (the thing being actively moved) -- so recorded
    angle_diff_rpy values describe the head's rotation relative to the
    torso's own axes, matching standard neck-kinematics reporting.

This is a DIFFERENT, INDEPENDENT computation from trajectory/sources.py's
MarkerSource._theta_from_target_rotation -- that function computes a
single-axis rotation of the target relative to ITS OWN STARTING POSE
(used only to drive compute_hand_targets/the IK during live tracking), not
a multi-axis instantaneous object-vs-target comparison. Don't conflate the
two just because both involve "target" and "rotation."

object_angular_velocity/target_angular_velocity (below) are DELIBERATELY
in a DIFFERENT frame from angle_diff_rpy: they're computed directly from
each marker's own consecutive CAMERA OPTICAL-FRAME readings (OpenCV/
REP-103: X right, Y down, Z forward) -- no T_optical_to_d435/
T_d435_to_pelvis composition at all, not even the pelvis frame object_pose/
target_pose are expressed in. This is intentional, not an oversight: their
purpose is cross-trial NORMALIZATION (how fast was the torso/head actually
moving in this trial, to compare against a different trial where it moved
more or less) -- a MAGNITUDE-only use. Magnitude is exactly invariant to
any fixed rotation composition (rotations preserve vector norm), so
carrying the pelvis-frame composition through added a dependency on a
transform that's partly fictional for a human trial (T_d435_to_pelvis is
FK for a G1 that isn't there) for zero benefit. Do NOT read these fields'
x/y/z components as anatomically meaningful, and do NOT assume they share
axes with angle_diff_rpy -- they don't; use the magnitude
(np.linalg.norm(...)) for normalization, and see
docs/data_recording_conventions.md for the full reasoning.
"""
import os
import pickle
import re
import time
from dataclasses import dataclass
from datetime import datetime
from typing import Optional

import numpy as np
from scipy.spatial.transform import Rotation


_PROJECT_ROOT = os.path.abspath(
    os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..')
)
DEFAULT_RUNS_DIR = os.path.join(_PROJECT_ROOT, 'data', 'runs')
DEFAULT_FILENAME_TEMPLATE = "{origin}_run_{timestamp}_{source}_{tracking_mode}"


def angle_diff_rpy(R_reference: np.ndarray, R_moving: np.ndarray) -> np.ndarray:
    """Per-axis [roll, pitch, yaw] (radians) of R_moving relative to R_reference,
    decomposed IN R_REFERENCE'S OWN LOCAL AXES -- not a symmetric "difference."

    Computed by decomposing the single relative rotation
    R_rel = R_reference.T @ R_moving via scipy's intrinsic-xyz Euler
    decomposition (roll about reference's local X, pitch about the
    once-rotated Y, yaw about the twice-rotated Z) -- not by independently
    decomposing each marker's rotation and subtracting the results, which
    only approximates the true relative rotation for small angles.

    Swapping the two arguments is NOT just a sign flip for non-trivial
    rotations (R_rel's inverse has a different Euler decomposition than
    -R_rel elementwise, except to first order for small angles) -- which
    argument is "reference" is a real, consequential choice. Parameters are
    named reference/moving rather than object/target deliberately: this
    function is generic (used for both the robot's box-tracking case and
    the human neck-trial case), and "object"/"target" role names don't by
    themselves indicate which one anchors the decomposition. See this
    module's docstring and docs/data_recording_conventions.md for the
    biomechanics rationale and the object/target -> reference/moving
    mapping RunRecorder.record() uses.
    """
    R_rel = R_reference.T @ R_moving
    return Rotation.from_matrix(R_rel).as_euler('xyz')


def angular_velocity_rotvec(
    R_prev: np.ndarray, t_prev: float, R_cur: np.ndarray, t_cur: float
) -> Optional[np.ndarray]:
    """Finite-difference angular velocity (rad/s) between two orientations.

    Returns rotvec(R_cur @ R_prev.T) / dt, in whatever frame R_prev/R_cur
    are expressed in (this function is frame-agnostic; RunRecorder.record()
    is what decides which frame to pass in -- currently each marker's own
    camera optical-frame readings, see this module's docstring). Returns
    None (not NaN, not a divide-by-zero) when dt <= 0 -- covers both the
    first-sample case at the call site and a duplicate/held marker
    timestamp: the control loop calling this (Phase 2 in
    sdk_pose_tracking.py) runs at ~6-10Hz per the documented IK-solve
    timing floor, which can exceed the camera's own frame rate, so the
    "same" marker observation can be seen across more than one recorded
    sample.
    """
    dt = t_cur - t_prev
    if dt <= 0:
        return None
    R_delta = R_cur @ R_prev.T
    return Rotation.from_matrix(R_delta).as_rotvec() / dt


def rotation_magnitude(R_reference: np.ndarray, R_moving: np.ndarray) -> float:
    """Total rotation angle (radians, >= 0) between two orientations:
    Rotation.from_matrix(R_reference.T @ R_moving).magnitude().

    Unlike angle_diff_rpy's per-axis Euler decomposition -- which is
    expressed in R_reference's own local axes, an arbitrary direction fixed
    by however that specific marker happens to be physically mounted -- this
    is a single axis-agnostic number: how far the object has rotated overall,
    correct regardless of which real-world axis the physical rotation
    happens to align with. Used by scripts/gravity_comp_debug.py's
    axis="magnitude" option, which exists precisely because a guessed Euler
    axis (e.g. "pitch") can silently measure ~0 rotation even while the
    tracked marker is visibly being rotated a lot, if the physical rotation
    axis doesn't line up with that guess.
    """
    return float(Rotation.from_matrix(R_reference.T @ R_moving).magnitude())


@dataclass
class MarkerSample:
    """One recorded instant. Marker-derived fields are None when that
    marker wasn't observed this sample (lost/stale marker, or a
    non-marker-backed source like SineSource)."""
    t: float                                        # elapsed seconds since RunRecorder.start()
    wall_time: float                                 # time.time() epoch seconds, for cross-run correlation
    object_pose: Optional[np.ndarray] = None         # 4x4 SE3, pelvis frame
    target_pose: Optional[np.ndarray] = None         # 4x4 SE3, pelvis frame
    object_obs_timestamp: Optional[float] = None      # MarkerObservation.timestamp (camera capture instant)
    target_obs_timestamp: Optional[float] = None
    object_reprojection_error: Optional[float] = None  # pixels, from solvePnP
    target_reprojection_error: Optional[float] = None
    # (3,) rad [roll,pitch,yaw]; None unless both markers seen. Decomposed in
    # the TARGET marker's own local frame (biomechanics-style reference-
    # segment convention: target = torso in a neck trial, the robot's
    # tracking reference otherwise), NOT the object marker's frame -- see
    # angle_diff_rpy()'s docstring and docs/data_recording_conventions.md.
    angle_diff_rpy: Optional[np.ndarray] = None
    # (3,) rad/s; None on first sample, dt<=0, or when pose_in_optical isn't
    # available. Computed from consecutive CAMERA OPTICAL-FRAME readings
    # (not pelvis frame, and not target-anchored like angle_diff_rpy) -- no
    # T_optical_to_d435/T_d435_to_pelvis composition at all. Intended for
    # magnitude-only cross-trial normalization (np.linalg.norm(...)); the
    # x/y/z components are in the camera's own OpenCV convention, not
    # anatomical, and don't share axes with angle_diff_rpy. See this
    # module's docstring and docs/data_recording_conventions.md.
    object_angular_velocity: Optional[np.ndarray] = None
    target_angular_velocity: Optional[np.ndarray] = None
    theta: Optional[float] = None                      # robot-side only: theta fed to compute_hand_targets this step
    # ATI Axia80 F/T sensor wrench, (6,) [Fx,Fy,Fz,Mx,My,Mz] in N / N*m,
    # expressed in the SENSOR frame. Both None when the sensor is disabled or
    # unreachable (recording degrades gracefully, never crashes).
    #   _raw       -- scaled counts straight from the sensor.
    #   _grav_comp -- gravity-cancelled: FORCE channels orientation-aware
    #                 (startup gravity reading rotated by the target/torso
    #                 marker), TORQUE channels tared. Additionally None until
    #                 the startup gravity reference is captured. See
    #                 g1_spine_stabilization/sensors/force_torque.py and
    #                 docs/data_recording_conventions.md.
    load_cell_wrench_raw: Optional[np.ndarray] = None
    load_cell_wrench_grav_comp: Optional[np.ndarray] = None


def default_run_path(
    source_label: str,
    tracking_mode: Optional[str] = None,
    origin: Optional[str] = None,
    output_dir: Optional[str] = None,
    filename_template: Optional[str] = None,
) -> str:
    """Build 'data/runs/<origin>_run_<timestamp>_<source_label>[_<tracking_mode>].pkl'.

    Pure string-building -- does not create directories (RunRecorder.save()
    does that). Cleanly omits the tracking_mode segment (no literal "_None",
    no doubled/trailing underscore) when tracking_mode is None, e.g.
    --source sine. Same for origin: when None, the leading "<origin>_run_"
    segment collapses to a plain "run_..." prefix, so callers that don't
    care about origin (e.g. RunRecorder always passes it, but a direct
    caller need not) get the same filenames as before this prefix existed.
    """
    template = filename_template or DEFAULT_FILENAME_TEMPLATE
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    directory = output_dir or DEFAULT_RUNS_DIR
    if not os.path.isabs(directory):
        directory = os.path.join(_PROJECT_ROOT, directory)
    name = template.format(
        timestamp=timestamp, source=source_label,
        tracking_mode=tracking_mode if tracking_mode is not None else "",
        origin=origin if origin is not None else "",
    )
    name = re.sub(r"_+", "_", name).strip("_")
    return os.path.join(directory, f"{name}.pkl")


class RunRecorder:
    """Accumulates MarkerSample records during a tracking run and pickles
    them exactly once, in a schema shared by both the robot's Phase-2
    recording (controllers/sdk_pose_tracking.py) and the standalone
    human-trial script (scripts/record_human_trial.py) -- so a downstream
    comparison analysis can load either file interchangeably.
    """
    SCHEMA_VERSION = 2   # v2: load_cell_wrench split into _raw + _grav_comp (F/T sensor wired)

    def __init__(
        self,
        origin: str,
        source_label: str,
        tracking_mode: Optional[str] = None,
        output_path: Optional[str] = None,
        output_dir: Optional[str] = None,
        filename_template: Optional[str] = None,
        object_marker_id: Optional[int] = None,
        target_marker_id: Optional[int] = None,
        metadata: Optional[dict] = None,
    ):
        if origin not in ("robot", "human"):
            raise ValueError(f"origin must be 'robot' or 'human', got {origin!r}")
        self.origin = origin
        self.source_label = source_label
        self.tracking_mode = tracking_mode
        self.object_marker_id = object_marker_id
        self.target_marker_id = target_marker_id
        self.metadata = metadata or {}
        self.output_path = output_path or default_run_path(
            source_label, tracking_mode, origin=self.origin,
            output_dir=output_dir, filename_template=filename_template,
        )
        self._samples: list = []
        self._prev_obj = None   # (R_optical_3x3, obs_timestamp) of the last object obs with pose_in_optical
        self._prev_tgt = None   # (R_optical_3x3, obs_timestamp) of the last target obs with pose_in_optical
        self._t0 = None
        self._saved = False

    def start(self) -> None:
        """Call once at the start of the tracking phase. Resets the
        internal clock used for auto-computed `t` values."""
        self._t0 = time.monotonic()

    def record(
        self,
        *,
        object_obs=None,
        target_obs=None,
        theta: Optional[float] = None,
        t: Optional[float] = None,
        wall_time: Optional[float] = None,
        load_cell_wrench_raw: Optional[np.ndarray] = None,
        load_cell_wrench_grav_comp: Optional[np.ndarray] = None,
    ) -> MarkerSample:
        """Compute and append one MarkerSample. Never raises, including
        when object_obs/target_obs is None (stale/lost marker, or a
        non-marker-backed source like SineSource) -- the corresponding
        pose/angle-diff/velocity fields are simply None for that sample.

        load_cell_wrench_raw/_grav_comp: optional (6,) F/T sensor wrenches
        (see sensors/force_torque.py). Passed through verbatim; None when no
        sensor is attached.
        """
        if t is None:
            if self._t0 is None:
                raise RuntimeError("RunRecorder.start() must be called before record() when t is not provided")
            t = time.monotonic() - self._t0
        if wall_time is None:
            wall_time = time.time()

        object_pose = object_obs.pose_in_pelvis.copy() if object_obs is not None else None
        target_pose = target_obs.pose_in_pelvis.copy() if target_obs is not None else None

        angle_diff = None
        if object_pose is not None and target_pose is not None:
            # target = reference (biomechanics-style: torso in a neck trial,
            # the tracking reference for the robot); object = moving. See
            # this module's docstring for why target always anchors the
            # decomposition regardless of which use case is calling.
            angle_diff = angle_diff_rpy(target_pose[:3, :3], object_pose[:3, :3])

        # Angular velocities are computed from each marker's own camera
        # OPTICAL-frame readings, not pelvis frame -- no T_optical_to_d435/
        # T_d435_to_pelvis composition. If pose_in_optical isn't available
        # (e.g. a synthetic observation that didn't provide it), the field
        # is None for that sample -- NOT a silent fallback to pelvis-derived
        # rotation, which would reintroduce exactly the composition being
        # avoided. See this module's docstring for why.
        object_angular_velocity = None
        if object_obs is not None and object_obs.pose_in_optical is not None:
            R_obj_opt = object_obs.pose_in_optical[:3, :3]
            if self._prev_obj is not None:
                R_prev, t_prev = self._prev_obj
                object_angular_velocity = angular_velocity_rotvec(
                    R_prev, t_prev, R_obj_opt, object_obs.timestamp
                )
            self._prev_obj = (R_obj_opt.copy(), object_obs.timestamp)

        target_angular_velocity = None
        if target_obs is not None and target_obs.pose_in_optical is not None:
            R_tgt_opt = target_obs.pose_in_optical[:3, :3]
            if self._prev_tgt is not None:
                R_prev, t_prev = self._prev_tgt
                target_angular_velocity = angular_velocity_rotvec(
                    R_prev, t_prev, R_tgt_opt, target_obs.timestamp
                )
            self._prev_tgt = (R_tgt_opt.copy(), target_obs.timestamp)

        sample = MarkerSample(
            t=t,
            wall_time=wall_time,
            object_pose=object_pose,
            target_pose=target_pose,
            object_obs_timestamp=object_obs.timestamp if object_obs is not None else None,
            target_obs_timestamp=target_obs.timestamp if target_obs is not None else None,
            object_reprojection_error=object_obs.reprojection_error if object_obs is not None else None,
            target_reprojection_error=target_obs.reprojection_error if target_obs is not None else None,
            angle_diff_rpy=angle_diff,
            object_angular_velocity=object_angular_velocity,
            target_angular_velocity=target_angular_velocity,
            theta=theta,
            load_cell_wrench_raw=(
                None if load_cell_wrench_raw is None else np.asarray(load_cell_wrench_raw, float).copy()
            ),
            load_cell_wrench_grav_comp=(
                None if load_cell_wrench_grav_comp is None else np.asarray(load_cell_wrench_grav_comp, float).copy()
            ),
        )
        self._samples.append(sample)
        return sample

    def save(self) -> str:
        """Pickle the accumulated run to self.output_path, creating parent
        directories as needed. IDEMPOTENT: a second call is a no-op
        returning the same path without touching the file again -- this
        matters because save() is meant to be invoked unconditionally from
        a `finally` block regardless of how the run ended, without the
        caller needing to track whether it already ran.
        """
        if self._saved:
            return self.output_path
        os.makedirs(os.path.dirname(self.output_path), exist_ok=True)
        payload = {
            "schema_version": self.SCHEMA_VERSION,
            "origin": self.origin,
            "source_label": self.source_label,
            "tracking_mode": self.tracking_mode,
            "object_marker_id": self.object_marker_id,
            "target_marker_id": self.target_marker_id,
            "metadata": self.metadata,
            "samples": self._samples,
        }
        with open(self.output_path, "wb") as f:
            pickle.dump(payload, f)
        self._saved = True
        print(f"[RunRecorder] Saved {len(self._samples)} samples -> {self.output_path}")
        return self.output_path

    def __len__(self) -> int:
        return len(self._samples)
