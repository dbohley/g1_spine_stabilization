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

SAFETY NOTE: Defaults to dry_run=True (safe, no hardware). Pass --no-dry-run
for live-robot mode — that is SDK-6 and must be run by a human with the robot
mounted and E-stop in reach. Live mode prints a confirmation gate before
connecting; Ctrl-C at the prompt aborts safely.
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
from g1_spine_stabilization.trajectory.sources import (
    SineSource, MarkerSource, RateLimitedSource, NoiseFilteredSource,
)
from g1_spine_stabilization.controllers.sdk_pose_tracking import run_pose_tracking_sdk
from g1_spine_stabilization.data_recording.recorder import RunRecorder
from g1_spine_stabilization.utils.config import load_config


class _AnimatedMockTracker:
    """Time-varying mock tracker for headless preview testing.

    Returns both markers on every step:
    - Object marker (id from config): fixed, simulating a marker mounted on
      TOP of the box (box_center + box_half_height, identity rotation) --
      mirrors the real physical setup so this mock exercises the same
      marker-on-top correction MarkerSource applies to a real reading.
    - Target marker (id from config): rotates about X with a sine wave.
    """

    def __init__(self, object_marker_id, target_marker_id, box_center,
                 amplitude=0.4, freq=0.2, box_half_height=0.0):
        self._obj_id = object_marker_id
        self._tgt_id = target_marker_id
        self._box_center = np.asarray(box_center, dtype=float)
        self._box_half_height = box_half_height
        self._amplitude = amplitude
        self._freq = freq
        self._latest: dict = {}
        self._t_start = time.monotonic()

    def step(self):
        t = time.monotonic() - self._t_start

        obj_pose = np.eye(4)
        obj_pose[:3, 3] = self._box_center + np.array([0.0, 0.0, self._box_half_height])
        obs_obj = MarkerObservation(
            marker_id=self._obj_id,
            pose_in_pelvis=obj_pose,
            timestamp=t,
            reprojection_error=0.0,
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
            marker_id=self._tgt_id,
            pose_in_pelvis=tgt_pose,
            timestamp=t,
            reprojection_error=0.0,
            pose_in_optical=tgt_pose,
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
    parser.add_argument(
        "--net", default="eth0",
        help="DDS network interface (e.g. eth0). Ignored in dry-run.",
    )
    parser.add_argument(
        "--dry-run", dest="dry_run", action="store_true", default=True,
        help="Simulate commands without connecting to hardware (default: True)",
    )
    parser.add_argument(
        "--no-dry-run", dest="dry_run", action="store_false",
        help="Connect to live hardware (HUMAN-RUN ONLY; E-stop required)",
    )
    parser.add_argument(
        "--record", nargs="?", const="__AUTO__", default=None, metavar="PATH",
        help="Record Phase-2 marker/pose data (object+target pose, per-axis angle "
             "diff, angular velocities, time) to a pickle file for later comparison "
             "against human-trial recordings (scripts/record_human_trial.py). Pass "
             "with no PATH to auto-generate a timestamped filename under data/runs/. "
             "Works with any --source; --source sine has no real AprilTag markers, "
             "so only time/theta are recorded (pose/angle-diff/velocity fields are "
             "None) -- still useful for exercising the recording pipeline without "
             "hardware. Disabled by default.",
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

    if not args.dry_run:
        print("[preview] WARNING: live mode selected.")
        print("[preview]   - Robot must be mounted and powered")
        print("[preview]   - E-stop must be in reach")
        print("[preview]   - initialize() will move arms to default pose")
        if args.source == "marker":
            print("[preview]   - Live camera + AprilTag tracking will drive the arms in real time")
        input("[preview] Press Enter to continue (Ctrl-C to abort)...")

    # -------------------------------------------------------------------
    # Load IK (reused for the controller AND d435_to_pelvis computation)
    # -------------------------------------------------------------------
    print("[preview] Loading IK solver…")
    gravity_comp_enabled = cfg.get("gravity_comp", {}).get("enabled", True)
    tauff_filter_cfg = cfg.get("gravity_comp", {}).get("tauff_filter", {})
    ik = G1HandIK(
        gravity_comp_enabled=gravity_comp_enabled,
        tauff_filter_enabled=tauff_filter_cfg.get("enabled", True),
        tauff_filter_alpha=tauff_filter_cfg.get("alpha", 0.3),
    )

    # Default box center in pelvis frame (same as run_pose_tracking_sim)
    box_size = list(cfg["box"]["size"])
    box_half_height = box_size[2] / 2   # object marker sits on TOP of the box, not its center
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
        network_interface=args.net,
        dry_run=args.dry_run,
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
            box_half_height=box_half_height,
        )
        source = MarkerSource(mock_tracker, obj_id, tgt_id, mode=args.tracking, countdown=0.0,
                              box_half_height=box_half_height)
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
        print(f"[preview] Configured marker_size={perception['marker_size']:.4f}m — "
              f"verify this matches your real printed marker's physical size; "
              f"a mismatch here scales solvePnP's recovered translation and can look "
              f"like a calibration error.")
        T_opt_to_d435  = optical_to_d435_from_config(perception["d435_link_to_optical"])
        T_d435_to_pelvis = d435_to_pelvis_from_pinocchio(ik=ik)
        tracker = MarkerTracker(
            frame_source, detector, intrinsics, T_opt_to_d435, T_d435_to_pelvis
        )
        source = MarkerSource(tracker, obj_id, tgt_id, mode=args.tracking, countdown=3.0,
                              box_half_height=box_half_height)
        duration = float("inf")   # run until Ctrl+C
        # cam_for_preview stays None: RealSenseSource.read() auto-pushes frames via the SDK

        # ---------------------------------------------------------------
        # Noise-filter the real marker signal (marker mode only -- sine/mock
        # are noiseless by construction, so a median+EMA filter would only
        # add avoidable lag/distortion there). See configs/default.yaml:
        # trajectory.noise_filter.
        # ---------------------------------------------------------------
        noise_filter_cfg = cfg.get("trajectory", {}).get("noise_filter", {})
        if noise_filter_cfg.get("enabled", True):
            median_window = noise_filter_cfg.get("median_window", 5)
            ema_alpha = noise_filter_cfg.get("ema_alpha", 0.3)
            source = NoiseFilteredSource(
                source, median_window=median_window, ema_alpha=ema_alpha,
            )
            print(f"[preview] Noise filter enabled: median_window={median_window}, "
                  f"ema_alpha={ema_alpha:.2f}")

    # -------------------------------------------------------------------
    # Rate-limit the trajectory source (uniform across sine/mock/marker).
    # See configs/default.yaml: trajectory.rate_limit.
    # -------------------------------------------------------------------
    rate_limit_cfg = cfg.get("trajectory", {}).get("rate_limit", {})
    if rate_limit_cfg.get("enabled", True):
        max_theta_rate_deg_s = rate_limit_cfg.get("max_theta_rate_deg_s", 60.0)
        max_pos_rate_m_s = rate_limit_cfg.get("max_pos_rate_m_s", 0.3)
        source = RateLimitedSource(
            source,
            max_theta_rate=math.radians(max_theta_rate_deg_s),
            max_pos_rate=max_pos_rate_m_s,
        )
        print(f"[preview] Rate limiter enabled: {max_theta_rate_deg_s:.1f} deg/s, "
              f"{max_pos_rate_m_s:.2f} m/s")

    # -------------------------------------------------------------------
    # Optional data recording (--record). See g1_spine_stabilization/
    # data_recording/recorder.py and scripts/record_human_trial.py -- both
    # import RunRecorder from the same module, so the resulting pickle
    # schemas are identical and directly comparable.
    # -------------------------------------------------------------------
    recorder = None
    if args.record is not None:
        data_recording_cfg = cfg.get("data_recording", {})
        if not data_recording_cfg.get("enabled", True):
            print("[preview] --record was passed but data_recording.enabled=false in "
                  "config -- recording is disabled globally; ignoring --record.")
        else:
            output_path = None if args.record == "__AUTO__" else args.record
            recorder = RunRecorder(
                origin="robot",
                source_label=args.source,
                tracking_mode=args.tracking if args.source in ("mock", "marker") else None,
                output_path=output_path,
                output_dir=data_recording_cfg.get("output_dir"),
                filename_template=data_recording_cfg.get("filename_template"),
                object_marker_id=obj_id,
                target_marker_id=tgt_id,
                metadata={"perception": dict(perception)},
            )
            if args.source == "sine":
                print("[preview] WARNING: --record with --source sine has no real "
                      "AprilTag markers -- only time/theta will be recorded; pose/"
                      "angle-diff/angular-velocity fields will be None.")
            print(f"[preview] Recording enabled -> {recorder.output_path}")

    print(f"[preview] Connecting robot (dry_run={args.dry_run})…")
    robot.connect()
    robot.initialize()
    robot.start_streaming()
    print("[preview] Robot ready. Starting controller…")

    try:
        run_pose_tracking_sdk(robot, ik, cfg, source, duration=duration,
                              preview=preview, cam_for_preview=cam_for_preview,
                              recorder=recorder)
    finally:
        robot.shutdown()
        if cam_for_preview is not None:
            cam_for_preview.shutdown()
        if args.source == "marker":
            frame_source.close()
        print("[preview] Shutdown complete.")


if __name__ == "__main__":
    main()
