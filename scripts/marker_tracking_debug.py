#!/usr/bin/env python3
"""Live marker-tracking debug tool: visual + terminal, no pickle written.

Combines visualize_markers.py's annotated cv2 window with the derived data
record_human_trial.py's RunRecorder computes (angle_diff_rpy, per-marker
angular velocity) -- but role-labeled OBJECT/TARGET per configs/default.yaml
and printed live instead of pickled. Meant as a sanity check of the
recording pipeline's math against what you can see on screen, before
trusting a --record run's output.

Not a data-collection entry point: no --record/--duration/force-sensor
flags. For that, use scripts/record_human_trial.py.
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
from g1_spine_stabilization.perception.types import MarkerObservation
from g1_spine_stabilization.data_recording.recorder import angle_diff_rpy, angular_velocity_rotvec
from g1_spine_stabilization.utils.config import load_config


class _AngularVelocityTracker:
    """Per-marker-id finite-difference angular velocity from consecutive
    camera-optical-frame readings, mirroring RunRecorder's _prev_obj/
    _prev_tgt bookkeeping (data_recording/recorder.py)."""

    def __init__(self):
        self._prev = {}

    def update(self, marker_id, R_optical, timestamp):
        vel = None
        if marker_id in self._prev:
            R_prev, t_prev = self._prev[marker_id]
            vel = angular_velocity_rotvec(R_prev, t_prev, R_optical, timestamp)
        self._prev[marker_id] = (R_optical.copy(), timestamp)
        return vel


class _AnimatedMockTracker:
    """Hardware-free smoke-test source: object fixed, target oscillates in
    rotation over wall-clock time. Script-local (mirrors record_human_trial.
    py's _AnimatedHumanMockTracker) since scripts/ is not part of the
    installed package."""

    def __init__(self, object_marker_id, target_marker_id,
                 box_center=(0.35, 0.0, 0.1), amplitude=0.4, freq=0.2):
        self._obj_id = object_marker_id
        self._tgt_id = target_marker_id
        self._box_center = np.asarray(box_center, dtype=float)
        self._amplitude = amplitude
        self._freq = freq
        self._t_start = time.monotonic()

    def step(self):
        t = time.monotonic() - self._t_start

        obj_pose = np.eye(4)
        obj_pose[:3, 3] = self._box_center
        obs_obj = MarkerObservation(
            marker_id=self._obj_id, pose_in_pelvis=obj_pose,
            timestamp=t, reprojection_error=0.0, pose_in_optical=obj_pose,
        )

        theta = self._amplitude * math.sin(2 * math.pi * self._freq * t)
        ct, st = math.cos(theta), math.sin(theta)
        tgt_pose = np.eye(4)
        tgt_pose[:3, :3] = np.array([[1, 0, 0], [0, ct, -st], [0, st, ct]])
        tgt_pose[:3, 3] = self._box_center
        obs_tgt = MarkerObservation(
            marker_id=self._tgt_id, pose_in_pelvis=tgt_pose,
            timestamp=t, reprojection_error=0.0, pose_in_optical=tgt_pose,
        )
        return obs_obj, obs_tgt


def _print_status(elapsed, obj_pose, tgt_pose, obj_err, tgt_err, obj_vel, tgt_vel):
    if obj_pose is None or tgt_pose is None:
        print(f"[marker_tracking_debug] t={elapsed:.1f}s  "
              f"object={'seen' if obj_pose is not None else 'NOT SEEN'}  "
              f"target={'seen' if tgt_pose is not None else 'NOT SEEN'}")
        return

    obj_pos = obj_pose[:3, 3]
    tgt_pos = tgt_pose[:3, 3]
    print(f"[marker_tracking_debug] t={elapsed:.1f}s  "
          f"object pos=[{obj_pos[0]:.3f}, {obj_pos[1]:.3f}, {obj_pos[2]:.3f}]  "
          f"target pos=[{tgt_pos[0]:.3f}, {tgt_pos[1]:.3f}, {tgt_pos[2]:.3f}]")

    diff_deg = np.degrees(angle_diff_rpy(tgt_pose[:3, :3], obj_pose[:3, :3]))
    print(f"  angle_diff_rpy (target frame, deg) = "
          f"[{diff_deg[0]:+.1f}, {diff_deg[1]:+.1f}, {diff_deg[2]:+.1f}]")

    for label, vel in (("object", obj_vel), ("target", tgt_vel)):
        if vel is None:
            print(f"  {label}_angular_velocity = n/a (needs 2+ samples)")
        else:
            print(f"  {label}_angular_velocity = "
                  f"[{vel[0]:+.2f}, {vel[1]:+.2f}, {vel[2]:+.2f}] rad/s  "
                  f"(|.|={np.linalg.norm(vel):.2f})")

    print(f"  reproj_err: object={obj_err:.2f}px target={tgt_err:.2f}px")


def run_realsense(config, dry_run=False):
    perception = config["perception"]
    obj_id = perception["object_marker_id"]
    tgt_id = perception["target_marker_id"]
    print(f"[marker_tracking_debug] Marker roles: OBJECT id={obj_id}  TARGET id={tgt_id}")

    source = RealSenseSource(dry_run=dry_run)
    native = source.intrinsics_native
    intrinsics = CameraIntrinsics(
        fx=native["fx"], fy=native["fy"], cx=native["cx"], cy=native["cy"],
        distortion=np.asarray(native["distortion"]),
    )
    detector = AprilTagDetector(perception["marker_dict"], perception["marker_size"])
    T_opt_to_d435 = optical_to_d435_from_config(perception["d435_link_to_optical"])
    T_d435_to_pelvis = d435_to_pelvis_from_pinocchio()
    ang_vel_tracker = _AngularVelocityTracker()

    print("[marker_tracking_debug] Streaming. Press 'q' to quit.")
    t_start = time.monotonic()
    try:
        while True:
            read = source.read()
            if read is None:
                break
            frame, ts = read
            raw = detector.detect(frame, intrinsics)

            by_id = {}
            for det in raw:
                R = det.pose_in_optical[:3, :3]
                t_vec = det.pose_in_optical[:3, 3]
                rvec, _ = cv2.Rodrigues(R)
                cv2.drawFrameAxes(
                    frame, intrinsics.camera_matrix,
                    np.asarray(intrinsics.distortion), rvec, t_vec,
                    perception["marker_size"] * 0.5,
                )
                if det.marker_id == obj_id:
                    label = "OBJECT"
                elif det.marker_id == tgt_id:
                    label = "TARGET"
                else:
                    label = f"id={det.marker_id} (unlabeled)"
                cv2.putText(
                    frame, f"{label} err={det.reprojection_error:.2f}px",
                    tuple(det.corners[0].astype(int)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2,
                )
                pose_pelvis = compose_to_pelvis(det.pose_in_optical, T_opt_to_d435, T_d435_to_pelvis)
                by_id[det.marker_id] = (pose_pelvis, det.reprojection_error, R)

            cv2.imshow("Marker tracking debug", frame)

            obj_hit = by_id.get(obj_id)
            tgt_hit = by_id.get(tgt_id)
            obj_vel = ang_vel_tracker.update(obj_id, obj_hit[2], ts) if obj_hit else None
            tgt_vel = ang_vel_tracker.update(tgt_id, tgt_hit[2], ts) if tgt_hit else None

            elapsed = time.monotonic() - t_start
            _print_status(
                elapsed,
                obj_hit[0] if obj_hit else None, tgt_hit[0] if tgt_hit else None,
                obj_hit[1] if obj_hit else None, tgt_hit[1] if tgt_hit else None,
                obj_vel, tgt_vel,
            )

            if cv2.waitKey(1) & 0xFF == ord("q"):
                break
    finally:
        source.close()
        cv2.destroyAllWindows()


def run_mock(config):
    perception = config["perception"]
    obj_id = perception["object_marker_id"]
    tgt_id = perception["target_marker_id"]
    print(f"[marker_tracking_debug] Marker roles: OBJECT id={obj_id}  TARGET id={tgt_id}")
    print("[marker_tracking_debug] Mock mode: no camera window. Ctrl-C to stop.")

    tracker = _AnimatedMockTracker(obj_id, tgt_id)
    ang_vel_tracker = _AngularVelocityTracker()
    t_start = time.monotonic()
    try:
        while True:
            obs_obj, obs_tgt = tracker.step()
            elapsed = time.monotonic() - t_start
            obj_vel = ang_vel_tracker.update(obj_id, obs_obj.pose_in_optical[:3, :3], obs_obj.timestamp)
            tgt_vel = ang_vel_tracker.update(tgt_id, obs_tgt.pose_in_optical[:3, :3], obs_tgt.timestamp)
            _print_status(
                elapsed, obs_obj.pose_in_pelvis, obs_tgt.pose_in_pelvis,
                obs_obj.reprojection_error, obs_tgt.reprojection_error,
                obj_vel, tgt_vel,
            )
            time.sleep(0.15)
    except KeyboardInterrupt:
        print("\n[marker_tracking_debug] KeyboardInterrupt -- stopping.")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", default=None, help="Path to YAML config")
    parser.add_argument("--source", choices=["realsense", "mock"], default="realsense")
    parser.add_argument(
        "--dry-run", action="store_true",
        help="Synthetic camera frames, no real camera required (--source realsense only)",
    )
    args = parser.parse_args()
    config = load_config(args.config)
    if args.source == "realsense":
        run_realsense(config, dry_run=args.dry_run)
    else:
        run_mock(config)


if __name__ == "__main__":
    main()
