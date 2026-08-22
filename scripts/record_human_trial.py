#!/usr/bin/env python3
"""Record a human performing the same bimanual pose-tracking task the robot
does: hold an object marker (steady or moving), track a moving target marker
with it. NO robot, NO IK -- perception-only.

Reuses the exact same camera/marker stack and config as
run_pose_tracking_preview.py (RealSenseSource, AprilTagDetector,
configs/default.yaml's perception: section) and the SAME RunRecorder/
MarkerSample schema (g1_spine_stabilization.data_recording.recorder) as the
robot's Phase-2 recording, so pickle files from either script are
interchangeable inputs to a later comparison analysis.

The camera may need repositioning (no robot arm/hand to avoid), but marker
dictionary/size/IDs and the AprilTag detection stack are unchanged from the
robot pipeline -- this is required for the two datasets to be comparable.

When --record is passed, a cv2 video window (mirroring gravity_comp_debug.py)
shows both markers live and gates the run in two stages before any sample is
recorded: (1) wait for 'r' to be pressed, (2) require a few consecutive
fresh TARGET detections ("reference settle", configs/default.yaml's
data_recording.reference_settle_frames) before locking the pose/gravity
reference used for both the F/T sensor's startup tare and the experimental
sin-model theta below -- a single-frame capture previously produced a
spurious reading here, addressed by this settle per project direction (see
docs/gravity_comp_recording_plan.md). 'q' aborts (before recording starts)
or stops early (once recording, saving what was captured).

Marker tracking for --source realsense reads AprilTag detections directly
per frame (not via MarkerTracker, whose _latest cache never clears -- a
marker is never reported as lost again once first seen). This also
surfaces each marker's live FRESH/STALE status in the video window.

Alongside the raw and plain-tared F/T wrench, this script records
load_cell_wrench_grav_comp using the sin-model force correction (see
sensors/force_torque.py's sin_model_gravity_residual and
configs/default.yaml's force_sensor.debug_sin_model) rather than
GravityCompensator.compensate()'s rotation-matrix reprojection -- promoted
to the primary method per project decision after the rotation-matrix
method measured worse than no correction at all on real trial data; see
docs/gravity_comp_recording_plan.md.

This is camera-only code (no actuation), so unlike the robot scripts it is
safe to run live (not just --dry-run) in a Claude Code session.
"""
import argparse
import math
import os
import sys
import time

_THIS_DIR = os.path.dirname(os.path.abspath(__file__))
_PROJECT_ROOT = os.path.abspath(os.path.join(_THIS_DIR, ".."))
if _PROJECT_ROOT not in sys.path:
    sys.path.insert(0, _PROJECT_ROOT)

import cv2
import numpy as np

from g1_spine_stabilization.perception.detector import AprilTagDetector, CameraIntrinsics
from g1_spine_stabilization.perception.frame_sources import RealSenseSource
from g1_spine_stabilization.perception.frames import (
    optical_to_d435_from_config, d435_to_pelvis_from_pinocchio, compose_to_pelvis,
)
from g1_spine_stabilization.perception.pose_filter import PoseNoiseFilter
from g1_spine_stabilization.perception.types import MarkerObservation
from g1_spine_stabilization.data_recording.recorder import (
    RunRecorder, sin_model_theta_rad, sin_model_wrench, VALID_SIN_MODEL_AXES,
)
from g1_spine_stabilization.sensors.force_torque import ForceTorqueSource, sin_model_gravity_residual
from g1_spine_stabilization.utils.config import load_config

_WINDOW_NAME = "record_human_trial"


def _obs_rotation(obs):
    """3x3 rotation from a MarkerObservation (optical frame preferred, pelvis
    fallback), or None -- the torso/target orientation for F/T gravity comp."""
    if obs is None:
        return None
    pose_opt = getattr(obs, "pose_in_optical", None)
    if pose_opt is not None:
        return pose_opt[:3, :3]
    return obs.pose_in_pelvis[:3, :3]


class _DirectMultiMarkerTracker:
    """Per-frame AprilTag read for --source realsense -- replaces MarkerTracker
    (whose _latest cache never clears, silently freezing a stale pose forever
    once a marker first appears, with no way to tell). Tracks exactly the two
    marker ids in `labels`, drawing axes + a label for EVERY detection (labeled
    or not) into self.last_frame each step() so a live cv2 window can show the
    whole scene. Mirrors gravity_comp_debug.py's single-marker direct-read
    pattern, generalized to two markers with explicit per-marker freshness
    (is_fresh/seconds_since_fresh), which MarkerTracker never exposed.
    """

    def __init__(self, source, detector, intrinsics, T_opt_to_d435, T_d435_to_pelvis,
                 labels, marker_size, pose_filter_cfg=None):
        self._source = source
        self._detector = detector
        self._intrinsics = intrinsics
        self._T_opt_to_d435 = T_opt_to_d435
        self._T_d435_to_pelvis = T_d435_to_pelvis
        self._labels = labels
        self._marker_size = marker_size
        self._latest: dict = {}
        self._last_fresh_elapsed: dict = {}
        self._fresh_this_step: set = set()
        self.last_frame = None
        self._t_start = time.monotonic()

        pose_filter_cfg = pose_filter_cfg or {}
        if pose_filter_cfg.get("enabled", True):
            self._filters = {
                marker_id: PoseNoiseFilter(
                    median_window=pose_filter_cfg.get("median_window", 5),
                    ema_alpha=pose_filter_cfg.get("ema_alpha", 0.3),
                )
                for marker_id in labels
            }
        else:
            self._filters = None

    def step(self):
        read = self._source.read()
        self._fresh_this_step = set()
        if read is None:
            return []
        frame, ts = read
        raw = self._detector.detect(frame, self._intrinsics)
        elapsed = time.monotonic() - self._t_start

        fresh_obs = []
        for det in raw:
            R = det.pose_in_optical[:3, :3]
            t_vec = det.pose_in_optical[:3, 3]
            rvec, _ = cv2.Rodrigues(R)
            cv2.drawFrameAxes(
                frame, self._intrinsics.camera_matrix,
                np.asarray(self._intrinsics.distortion), rvec, t_vec,
                self._marker_size * 0.5,
            )
            label = self._labels.get(det.marker_id, f"id={det.marker_id} (unlabeled)")
            cv2.putText(
                frame, f"{label} err={det.reprojection_error:.2f}px",
                tuple(det.corners[0].astype(int)),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2,
            )
            if det.marker_id in self._labels:
                filtered_optical = (
                    self._filters[det.marker_id].filter(det.pose_in_optical)
                    if self._filters is not None else det.pose_in_optical
                )
                pose_pelvis = compose_to_pelvis(filtered_optical, self._T_opt_to_d435, self._T_d435_to_pelvis)
                obs = MarkerObservation(
                    marker_id=det.marker_id, pose_in_pelvis=pose_pelvis,
                    # reprojection_error/timestamp stay tied to the RAW per-frame
                    # detection (a detection-quality/freshness signal) even though
                    # pose_in_optical/pose_in_pelvis are now filtered -- see
                    # perception/pose_filter.py.
                    timestamp=ts, reprojection_error=det.reprojection_error,
                    pose_in_optical=filtered_optical.copy(),
                )
                self._latest[det.marker_id] = obs
                self._last_fresh_elapsed[det.marker_id] = elapsed
                self._fresh_this_step.add(det.marker_id)
                fresh_obs.append(obs)

        self.last_frame = frame
        return fresh_obs

    def get_latest(self, marker_id):
        return self._latest.get(marker_id)

    def is_fresh(self, marker_id) -> bool:
        return marker_id in self._fresh_this_step

    def seconds_since_fresh(self, marker_id):
        last = self._last_fresh_elapsed.get(marker_id)
        if last is None:
            return None
        return (time.monotonic() - self._t_start) - last


def _build_mock_display_frame(t, obs_obj, obs_tgt):
    """Blank canvas + status text -- lets --source mock drive the SAME
    wait-for-'r'/settle/recording video-window flow as --source realsense,
    with no real camera involved."""
    frame = np.zeros((480, 640, 3), dtype=np.uint8)
    cv2.putText(frame, "MOCK SOURCE -- no real camera", (20, 40),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
    cv2.putText(frame, f"t={t:.1f}s", (20, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
    obj_pos = obs_obj.pose_in_pelvis[:3, 3]
    tgt_pos = obs_tgt.pose_in_pelvis[:3, 3]
    cv2.putText(frame, f"OBJECT pos={np.round(obj_pos, 3).tolist()}", (20, 120),
                cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 0), 1)
    cv2.putText(frame, f"TARGET pos={np.round(tgt_pos, 3).tolist()}", (20, 150),
                cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 0), 1)
    return frame


class _AnimatedHumanMockTracker:
    """Hardware-free smoke-test tracker: object fixed, target oscillates in
    rotation over wall-clock time. Script-local (mirrors
    run_pose_tracking_preview.py's _AnimatedMockTracker) since scripts/ is
    not part of the installed package."""

    def __init__(self, object_marker_id, target_marker_id,
                 box_center=(0.35, 0.0, 0.1), amplitude=0.4, freq=0.2):
        self._obj_id = object_marker_id
        self._tgt_id = target_marker_id
        self._box_center = np.asarray(box_center, dtype=float)
        self._amplitude = amplitude
        self._freq = freq
        self._latest: dict = {}
        self._t_start = time.monotonic()
        self.last_frame = None

    def step(self):
        t = time.monotonic() - self._t_start

        obj_pose = np.eye(4)
        obj_pose[:3, 3] = self._box_center
        obs_obj = MarkerObservation(
            marker_id=self._obj_id, pose_in_pelvis=obj_pose,
            timestamp=t, reprojection_error=0.0,
            # No real camera -- reuse the synthetic pose as a stand-in
            # "optical frame" reading so --record exercises the recorder's
            # camera-frame angular-velocity path in this hardware-free mode.
            pose_in_optical=obj_pose,
        )

        theta = self._amplitude * math.sin(2 * math.pi * self._freq * t)
        ct, st = math.cos(theta), math.sin(theta)
        tgt_pose = np.eye(4)
        tgt_pose[:3, :3] = np.array([[1, 0, 0], [0, ct, -st], [0, st, ct]])
        tgt_pose[:3, 3] = self._box_center
        obs_tgt = MarkerObservation(
            marker_id=self._tgt_id, pose_in_pelvis=tgt_pose,
            timestamp=t, reprojection_error=0.0,
            pose_in_optical=tgt_pose,
        )

        self._latest[self._obj_id] = obs_obj
        self._latest[self._tgt_id] = obs_tgt
        self.last_frame = _build_mock_display_frame(t, obs_obj, obs_tgt)
        return [obs_obj, obs_tgt]

    def get_latest(self, marker_id):
        return self._latest.get(marker_id)

    def is_fresh(self, marker_id) -> bool:
        return marker_id in self._latest   # synthetic source, never stale once seen

    def seconds_since_fresh(self, marker_id):
        return 0.0 if marker_id in self._latest else None


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", default=None, help="Path to YAML config")
    parser.add_argument(
        "--source", choices=["realsense", "mock"], default="realsense",
        help="Camera source (default: realsense)",
    )
    parser.add_argument(
        "--dry-run", action="store_true",
        help="Synthetic camera frames, no real camera required (--source realsense only)",
    )
    parser.add_argument(
        "--tracking", choices=["orient-only", "6dof"], default="orient-only",
        help="Label only (no compute_hand_targets math here) -- records which "
             "physical setup this trial used, for filename/metadata parity with "
             "the robot script",
    )
    parser.add_argument(
        "--record", nargs="?", const="__AUTO__", default=None, metavar="PATH",
        help="Save this trial to a pickle file (same schema as the robot recorder). "
             "Omit PATH to auto-generate under data/runs/. If --record is omitted "
             "entirely, the script still runs (e.g. for a camera-positioning "
             "rehearsal) but nothing is saved.",
    )
    parser.add_argument(
        "--duration", type=float, default=None,
        help="Seconds to record. Default: run until Ctrl-C.",
    )
    parser.add_argument(
        "--force-sensor", dest="force_sensor", action="store_true", default=None,
        help="Force-enable the ATI F/T sensor (overrides force_sensor.enabled in config).",
    )
    parser.add_argument(
        "--no-force-sensor", dest="force_sensor", action="store_false",
        help="Disable the F/T sensor for this run (e.g. sensor unplugged).",
    )
    args = parser.parse_args()

    cfg = load_config(args.config)
    perception = cfg["perception"]
    obj_id = perception["object_marker_id"]
    tgt_id = perception["target_marker_id"]

    print("[record_human_trial] Marker roles (configs/default.yaml):")
    print(f"  Object marker  ID={obj_id}  -> attach to the held object")
    print(f"  Target marker  ID={tgt_id}  -> the reference to track")

    frame_source = None
    if args.source == "realsense":
        frame_source = RealSenseSource(dry_run=args.dry_run)
        native = frame_source.intrinsics_native
        intrinsics = CameraIntrinsics(
            fx=native["fx"], fy=native["fy"], cx=native["cx"], cy=native["cy"],
            distortion=np.asarray(native["distortion"]),
        )
        detector = AprilTagDetector(perception["marker_dict"], perception["marker_size"])
        T_opt_to_d435 = optical_to_d435_from_config(perception["d435_link_to_optical"])
        # No robot/IK object in scope -- pure Pinocchio FK on the URDF for a
        # camera-mount-frame constant, same pattern as visualize_markers.py's
        # run_realsense(). No actuation; this is the SAME reference frame the
        # robot pipeline uses, which is what makes the two datasets comparable.
        T_d435_to_pelvis = d435_to_pelvis_from_pinocchio()
        tracker = _DirectMultiMarkerTracker(
            frame_source, detector, intrinsics, T_opt_to_d435, T_d435_to_pelvis,
            labels={obj_id: "OBJECT", tgt_id: "TARGET"},
            marker_size=perception["marker_size"],
            pose_filter_cfg=perception.get("pose_filter", {}),
        )
    else:
        tracker = _AnimatedHumanMockTracker(obj_id, tgt_id)

    recorder = None
    sin_model_ref_force_n = None
    sin_model_axis = None
    if args.record is not None:
        data_recording_cfg = cfg.get("data_recording", {})
        if not data_recording_cfg.get("enabled", True):
            print("[record_human_trial] --record passed but data_recording.enabled=false; ignoring.")
        else:
            sin_model_cfg = cfg.get("force_sensor", {}).get("debug_sin_model", {})
            sin_model_ref_force_n = sin_model_cfg.get("reference_force_n", 7.5)
            sin_model_axis = sin_model_cfg.get("axis", "magnitude")
            if sin_model_axis not in VALID_SIN_MODEL_AXES:
                raise ValueError(
                    f"force_sensor.debug_sin_model.axis={sin_model_axis!r} must be one "
                    f"of {sorted(VALID_SIN_MODEL_AXES)}"
                )
            output_path = None if args.record == "__AUTO__" else args.record
            recorder = RunRecorder(
                origin="human", source_label=args.source, tracking_mode=args.tracking,
                output_path=output_path,
                output_dir=data_recording_cfg.get("output_dir"),
                filename_template=data_recording_cfg.get("filename_template"),
                object_marker_id=obj_id, target_marker_id=tgt_id,
                metadata={
                    "perception": dict(perception),
                    # Promoted to the primary load_cell_wrench_grav_comp method
                    # per project decision -- see docs/gravity_comp_recording_plan.md.
                    "sin_model_config": {
                        "reference_force_n": sin_model_ref_force_n,
                        "axis": sin_model_axis,
                        "status": "primary_grav_comp_method",
                    },
                },
            )
            print(f"[record_human_trial] Recording enabled -> {recorder.output_path}")
    else:
        print("[record_human_trial] --record not passed -- running WITHOUT saving "
              "(use for camera-positioning rehearsal only).")

    # ATI F/T sensor (only meaningful when recording). Degrades to None
    # wrenches if unreachable, so this script always runs. Same camera/marker/
    # sensor stack as the robot path -- see docs/force_sensor_setup.md.
    wrench_source = None
    if recorder is not None:
        wrench_source = ForceTorqueSource.from_config(cfg, enabled_override=args.force_sensor)
        wrench_source.start()

    R_ref_pelvis = None
    if recorder is not None:
        reference_settle_frames = int(
            cfg.get("data_recording", {}).get("reference_settle_frames", 5)
        )

        print("[record_human_trial] Press 'r' in the video window to start recording, "
              "'q' to abort.")
        while True:
            tracker.step()
            frame = tracker.last_frame.copy()
            cv2.putText(frame, "Press 'r' to start recording, 'q' to quit", (10, 25),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
            cv2.imshow(_WINDOW_NAME, frame)
            key = cv2.waitKey(1) & 0xFF
            if key == ord("q"):
                print("[record_human_trial] Aborted before recording started -- nothing saved.")
                cv2.destroyAllWindows()
                wrench_source.close()
                if frame_source is not None:
                    frame_source.close()
                return
            if key == ord("r"):
                break

        print(f"[record_human_trial] Locking reference: need {reference_settle_frames} "
              f"consecutive fresh TARGET detections...")
        settle_count = 0
        while settle_count < reference_settle_frames:
            tracker.step()
            frame = tracker.last_frame.copy()
            cv2.putText(frame, f"Locking reference... {settle_count}/{reference_settle_frames}",
                        (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 200, 255), 2)
            cv2.imshow(_WINDOW_NAME, frame)
            settle_count = settle_count + 1 if tracker.is_fresh(tgt_id) else 0
            key = cv2.waitKey(1) & 0xFF
            if key == ord("q"):
                print("[record_human_trial] Aborted during reference lock -- nothing saved.")
                cv2.destroyAllWindows()
                wrench_source.close()
                if frame_source is not None:
                    frame_source.close()
                return

        tgt_ref_obs = tracker.get_latest(tgt_id)
        R_ref_pelvis = tgt_ref_obs.pose_in_pelvis[:3, :3].copy()
        wrench_source.capture_reference(_obs_rotation(tgt_ref_obs))
        print("[record_human_trial] Reference locked. Recording starts now.")
        recorder.start()
    else:
        print("[record_human_trial] Ready. Hold the object marker steady (or move it), "
              "move the target marker to perform the tracking task. Ctrl-C to stop.")

    t_start = time.monotonic()
    _last_print_t = t_start - 1.0
    try:
        while True:
            elapsed = time.monotonic() - t_start
            if args.duration is not None and elapsed >= args.duration:
                break

            tracker.step()
            obj_obs = tracker.get_latest(obj_id)
            tgt_obs = tracker.get_latest(tgt_id)
            R_torso = _obs_rotation(tgt_obs)

            wrench_raw = wrench_tared = None
            sin_model_theta = None
            sin_model_force = None
            wrench_grav_comp = None
            if wrench_source is not None:
                # 3rd return (rotation-matrix GravityCompensator.compensate()) is
                # intentionally discarded here -- promoted out of the recorded
                # schema in favor of sin_model_wrench() below, per project
                # decision (see docs/gravity_comp_recording_plan.md). Still
                # computed/available via gravity_comp_debug.py for comparison.
                wrench_raw, wrench_tared, _ = wrench_source.read_all(R_torso)
                if wrench_tared is not None and R_ref_pelvis is not None and tgt_obs is not None:
                    sin_model_theta = sin_model_theta_rad(
                        sin_model_axis, R_ref_pelvis, tgt_obs.pose_in_pelvis[:3, :3]
                    )
                    residual = sin_model_gravity_residual(sin_model_theta, sin_model_ref_force_n)
                    sin_model_force = float(np.linalg.norm(wrench_tared[:3])) - residual
                    wrench_grav_comp = sin_model_wrench(wrench_tared, sin_model_force)

            if recorder is not None:
                recorder.record(
                    t=elapsed, object_obs=obj_obs, target_obs=tgt_obs,
                    load_cell_wrench_raw=wrench_raw,
                    load_cell_wrench_grav_comp=wrench_grav_comp,
                    load_cell_wrench_tared=wrench_tared,
                    sin_model_theta_rad=sin_model_theta,
                    sin_model_force_corrected=sin_model_force,
                )

                frame = tracker.last_frame.copy()
                tgt_status = "FRESH" if tracker.is_fresh(tgt_id) else "STALE"
                cv2.putText(frame, f"RECORDING samples={len(recorder)} target={tgt_status}",
                            (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
                cv2.imshow(_WINDOW_NAME, frame)
                if cv2.waitKey(1) & 0xFF == ord("q"):
                    print("[record_human_trial] 'q' pressed -- stopping early (saving what we have).")
                    break

            if time.monotonic() - _last_print_t >= 1.0:
                _last_print_t = time.monotonic()
                print(f"[record_human_trial] t={elapsed:.1f}s  "
                      f"object={'seen' if obj_obs is not None else 'NOT SEEN'}  "
                      f"target={'seen' if tgt_obs is not None else 'NOT SEEN'}  "
                      f"samples={len(recorder) if recorder is not None else 0}")

            if args.source == "mock":
                time.sleep(0.02)   # avoid pegging a CPU core; no actuation, not timing-critical
    except KeyboardInterrupt:
        print("\n[record_human_trial] KeyboardInterrupt -- stopping.")
    finally:
        if recorder is not None:
            recorder.save()
        if wrench_source is not None:
            wrench_source.close()
        if frame_source is not None:
            frame_source.close()
        cv2.destroyAllWindows()
    print("[record_human_trial] Done.")


if __name__ == "__main__":
    main()
