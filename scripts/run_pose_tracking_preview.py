#!/usr/bin/env python3
"""Multi-mode pose-tracking preview via the SDK Rerun visualizer.

Runs the same three-phase bimanual controller as run_pose_tracking_sim.py but
drives G1InspireRobot(dry_run=True) instead of CoppeliaSim, so the motion
appears in the Rerun viewer without actuating the real robot.

Modes (--source):
  sine    (default) — sine-wave trajectory, no camera needed.
  marker  — live RealSense + AprilTag tracking; move the target marker in front
             of the camera and watch the arms respond in Rerun.
  mock    — animated MockMarkerTracker; no camera, validates the full
             marker→IK→SDK path headlessly.

Tracking (--tracking, for marker/mock only):
  orient-only  (default) — box center fixed from object marker at startup;
                           theta tracks target marker X-rotation.
  6dof         — box center also tracks target marker translation.

SAFETY NOTE: This script runs with dry_run=True. Live robot: set dry_run=False
— that is SDK-6 and must be run by a human with the robot mounted and E-stop
in reach.
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

import numpy as np

from g1_spine_stabilization.kinematics.ik_solver import G1HandIK
from g1_spine_stabilization.perception.detector import AprilTagDetector, CameraIntrinsics
from g1_spine_stabilization.perception.frame_sources import RealSenseSource
from g1_spine_stabilization.perception.frames import (
    optical_to_d435_from_config, d435_to_pelvis_from_pinocchio,
)
from g1_spine_stabilization.perception.tracker import MarkerTracker
from g1_spine_stabilization.perception.types import MarkerObservation
from g1_spine_stabilization.trajectory.targets import L_EE_DEFAULT
from g1_spine_stabilization.trajectory.sources import SineSource, MarkerSource
from g1_spine_stabilization.controllers.sdk_pose_tracking import run_pose_tracking_sdk
from g1_spine_stabilization.utils.config import load_config


class _AnimatedMockTracker:
    """Time-varying mock tracker for headless preview testing.

    Returns both markers on every step:
    - Object marker (id from config): fixed at the natural box-center position.
    - Target marker (id from config): rotates about X with a sine wave.
    """

    def __init__(self, object_marker_id, target_marker_id, box_center,
                 amplitude=0.4, freq=0.2):
        self._obj_id = object_marker_id
        self._tgt_id = target_marker_id
        self._box_center = np.asarray(box_center, dtype=float)
        self._amplitude = amplitude
        self._freq = freq
        self._latest: dict = {}
        self._t_start = time.monotonic()

    def step(self):
        t = time.monotonic() - self._t_start

        obj_pose = np.eye(4)
        obj_pose[:3, 3] = self._box_center
        obs_obj = MarkerObservation(
            marker_id=self._obj_id,
            pose_in_pelvis=obj_pose,
            timestamp=t,
            reprojection_error=0.0,
        )

        theta = self._amplitude * math.sin(2 * math.pi * self._freq * t)
        ct, st = math.cos(theta), math.sin(theta)
        tgt_pose = np.eye(4)
        tgt_pose[:3, :3] = np.array([[1, 0, 0], [0, ct, -st], [0, st, ct]])
        tgt_pose[:3, 3] = self._box_center
        obs_tgt = MarkerObservation(
            marker_id=self._tgt_id,
            pose_in_pelvis=tgt_pose,
            timestamp=t,
            reprojection_error=0.0,
        )

        self._latest[self._obj_id] = obs_obj
        self._latest[self._tgt_id] = obs_tgt
        return [obs_obj, obs_tgt]

    def get_latest(self, marker_id):
        return self._latest.get(marker_id)


def _run_detect(perception, obj_id, tgt_id):
    """Open the camera, scan for ~5s, print detected AprilTag IDs, then return."""
    print("[detect] Opening RealSense camera…")
    frame_source = RealSenseSource()
    native = frame_source.intrinsics_native
    intrinsics = CameraIntrinsics(
        fx=native["fx"], fy=native["fy"],
        cx=native["cx"], cy=native["cy"],
        distortion=np.asarray(native["distortion"]),
    )
    detector = AprilTagDetector(perception["marker_dict"], perception["marker_size"])

    counts: dict = {}
    total_frames = 0
    t_end = time.monotonic() + 5.0
    print("[detect] Scanning for markers (5s)…")
    while time.monotonic() < t_end:
        result = frame_source.read()
        if result is None:
            continue
        frame, _ = result
        detections = detector.detect(frame, intrinsics)
        total_frames += 1
        for det in detections:
            counts[det.marker_id] = counts.get(det.marker_id, 0) + 1

    print(f"[detect] Scanned {total_frames} frames:")
    if not counts:
        print("  (no markers detected)")
    else:
        for mid, cnt in sorted(counts.items()):
            role = ""
            if mid == obj_id:
                role = "  ← object_marker_id in config"
            elif mid == tgt_id:
                role = "  ← target_marker_id in config"
            print(f"  ID={mid}  seen in {cnt}/{total_frames} frames{role}")
    print("[detect] Configure IDs in configs/default.yaml → perception.object_marker_id / target_marker_id")


def main():
    parser = argparse.ArgumentParser(
        description="Pose-tracking preview in Rerun (dry_run=True, no real robot)"
    )
    parser.add_argument(
        "--source", choices=["sine", "marker", "mock"], default="sine",
        help="Trajectory source (default: sine)",
    )
    parser.add_argument(
        "--tracking", choices=["orient-only", "6dof"], default="orient-only",
        help="Tracking mode for marker/mock sources (default: orient-only)",
    )
    parser.add_argument("--config", default=None, help="Path to YAML config")
    parser.add_argument(
        "--detect", action="store_true",
        help="Scan for AprilTag IDs for ~5s and exit (requires --source marker)",
    )
    args = parser.parse_args()

    cfg = load_config(args.config)
    perception = cfg["perception"]
    obj_id = perception["object_marker_id"]
    tgt_id = perception["target_marker_id"]

    print("[preview] Marker roles (configurable in configs/default.yaml):")
    print(f"  Object marker  ID={obj_id}  → attach to the held object/box  (captures starting pose)")
    print(f"  Target marker  ID={tgt_id}  → the reference you move to command the arms")

    # -------------------------------------------------------------------
    # --detect mode: scan for marker IDs and exit (no robot, no IK)
    # -------------------------------------------------------------------
    if args.detect:
        if args.source != "marker":
            parser.error("--detect requires --source marker")
        _run_detect(perception, obj_id, tgt_id)
        return

    # -------------------------------------------------------------------
    # Load IK (reused for the controller AND d435_to_pelvis computation)
    # -------------------------------------------------------------------
    print("[preview] Loading IK solver…")
    ik = G1HandIK()

    # Default box center in pelvis frame (same as run_pose_tracking_sim)
    box_size = list(cfg["box"]["size"])
    box_center_pelvis = np.array([
        L_EE_DEFAULT[0], 0.0, L_EE_DEFAULT[2],
    ])

    # -------------------------------------------------------------------
    # SDK: Preview + Robot  (created first so preview is available to sources)
    # -------------------------------------------------------------------
    from g1_inspire_sdk.preview import RerunPreview
    from g1_inspire_sdk import G1InspireRobot
    from g1_inspire_sdk.camera import RealSenseCamera

    print("[preview] Opening Rerun viewer…")
    preview = RerunPreview(spawn=True)

    robot = G1InspireRobot(
        dry_run=True,
        arm_side="both",
        hand_side="both",
        preview=preview,
    )

    # -------------------------------------------------------------------
    # Build the trajectory source
    # -------------------------------------------------------------------
    cam_for_preview = None   # set for sine/mock; marker mode drives it via tracker

    if args.source == "sine":
        source = SineSource(
            box_center=box_center_pelvis,
            amplitude=cfg["trajectory"]["sine"]["amplitude"],
            freq=cfg["trajectory"]["sine"]["frequency"],
        )
        duration = None   # use config default (rotation_duration)
        cam_for_preview = RealSenseCamera(dry_run=True)
        cam_for_preview.connect()
        cam_for_preview.attach_preview(preview)

    elif args.source == "mock":
        mock_tracker = _AnimatedMockTracker(
            object_marker_id=obj_id,
            target_marker_id=tgt_id,
            box_center=box_center_pelvis,
            amplitude=cfg["trajectory"]["sine"]["amplitude"],
            freq=cfg["trajectory"]["sine"]["frequency"],
        )
        source = MarkerSource(mock_tracker, obj_id, tgt_id, mode=args.tracking, countdown=0.0)
        duration = None   # use config default
        cam_for_preview = RealSenseCamera(dry_run=True)
        cam_for_preview.connect()
        cam_for_preview.attach_preview(preview)

    elif args.source == "marker":
        print("[preview] Setting up live RealSense + AprilTag tracker…")
        frame_source = RealSenseSource(preview=preview)
        native = frame_source.intrinsics_native
        intrinsics = CameraIntrinsics(
            fx=native["fx"], fy=native["fy"],
            cx=native["cx"], cy=native["cy"],
            distortion=np.asarray(native["distortion"]),
        )
        detector = AprilTagDetector(perception["marker_dict"], perception["marker_size"])
        T_opt_to_d435  = optical_to_d435_from_config(perception["d435_link_to_optical"])
        T_d435_to_pelvis = d435_to_pelvis_from_pinocchio(ik=ik)
        tracker = MarkerTracker(
            frame_source, detector, intrinsics, T_opt_to_d435, T_d435_to_pelvis
        )
        source = MarkerSource(tracker, obj_id, tgt_id, mode=args.tracking, countdown=3.0)
        duration = float("inf")   # run until Ctrl+C
        # cam_for_preview stays None: RealSenseSource.read() auto-pushes frames via the SDK

    print("[preview] Connecting robot (dry_run=True)…")
    robot.connect()
    robot.initialize()
    robot.start_streaming()
    print("[preview] Robot ready. Starting controller…")

    try:
        run_pose_tracking_sdk(robot, ik, cfg, source, duration=duration,
                              preview=preview, cam_for_preview=cam_for_preview)
    finally:
        robot.shutdown()
        if cam_for_preview is not None:
            cam_for_preview.shutdown()
        print("[preview] Shutdown complete.")


if __name__ == "__main__":
    main()
