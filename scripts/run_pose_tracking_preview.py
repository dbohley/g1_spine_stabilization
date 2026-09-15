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
                           theta tracks the target marker's full 3-DoF
                           rotation (genuine 3-DoF orientation tracking).
  6dof         — box center also tracks target marker translation, so this
                           is genuine 6-DoF (3 translation + 3 rotation).
                           See docs/six_dof_tracking_fix.md.

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
from scipy.spatial.transform import Rotation

from g1_spine_stabilization.kinematics.ik_solver import G1HandIK
from g1_spine_stabilization.perception.detector import AprilTagDetector, CameraIntrinsics
from g1_spine_stabilization.perception.frame_sources import RealSenseSource
from g1_spine_stabilization.perception.frames import (
    optical_to_d435_from_config, d435_to_pelvis_from_pinocchio, waist_q_full,
)
from g1_spine_stabilization.perception.tracker import MarkerTracker
from g1_spine_stabilization.perception.types import MarkerObservation
from g1_spine_stabilization.trajectory.targets import L_EE_DEFAULT
from g1_spine_stabilization.trajectory.sources import (
    SineSource, MarkerSource, RateLimitedSource, NoiseFilteredSource, RotationClampSource,
    PredictiveSource,
)
from g1_spine_stabilization.controllers.sdk_pose_tracking import run_pose_tracking_sdk
from g1_spine_stabilization.controllers.palm_force import clamp_target_force
from g1_spine_stabilization.controllers.joint_path_execution import log_marker_observations
from g1_spine_stabilization.data_recording.recorder import RunRecorder
from g1_spine_stabilization.sensors.force_torque import ForceTorqueSource
from g1_spine_stabilization.utils.config import load_config
from g1_spine_stabilization.utils.transforms import se3_from_xyz_rpy
from g1_spine_stabilization.diagnostics.run_log import RUN_LOG
from g1_spine_stabilization.diagnostics.sdk_state_tap import start_sdk_state_tap


class _AnimatedMockTracker:
    """Time-varying mock tracker for headless preview testing.

    Returns both markers on every step:
    - Object marker (id from config): fixed, simulating a marker mounted on
      TOP of the box (box_center + box_half_height, identity rotation) --
      mirrors the real physical setup so this mock exercises the same
      marker-on-top correction MarkerSource applies to a real reading.
    - Target marker (id from config): rotates with a per-axis-weighted sine
      wave (see `axes`).

    `axes` (default (1.0, 0.0, 0.0)) is a per-axis [X, Y, Z] amplitude
    weight for the target's synthetic rotation, in the pelvis frame. The
    default reproduces the ORIGINAL pure roll-about-X motion byte-for-byte
    (Rx(amplitude*sin(w)) exactly) -- so every pre-existing mock run is
    unchanged. Passing e.g. (1.0, 1.0, 1.0) exercises pitch and yaw with a
    120-degree phase offset between axes (so the rotation axis genuinely
    varies over time, rather than staying fixed and just scaled) -- this is
    the only camera-free way to verify MarkerSource's full 3-DoF rotation
    tracking (see docs/six_dof_tracking_fix.md); a co-phased scaled rotvec
    would still only ever exercise a single, fixed rotation axis.
    """

    def __init__(self, object_marker_id, target_marker_id, box_center,
                 amplitude=0.4, freq=0.2, box_half_height=0.0, axes=(1.0, 0.0, 0.0)):
        self._obj_id = object_marker_id
        self._tgt_id = target_marker_id
        self._box_center = np.asarray(box_center, dtype=float)
        self._box_half_height = box_half_height
        self._amplitude = amplitude
        self._freq = freq
        self._axes = np.asarray(axes, dtype=float)
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

        w = 2 * math.pi * self._freq * t
        # 120-degree phase offset per axis so the rotation axis itself
        # varies over time when more than one axes[] weight is nonzero --
        # with the default axes=(1,0,0) this reduces to exactly
        # Rx(amplitude*sin(w)), byte-identical to the original motion.
        rotvec = self._amplitude * self._axes * np.array([
            math.sin(w),
            math.sin(w + 2 * math.pi / 3),
            math.sin(w + 4 * math.pi / 3),
        ])
        tgt_pose = np.eye(4)
        tgt_pose[:3, :3] = Rotation.from_rotvec(rotvec).as_matrix()
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
    cam_cfg = perception.get("camera", {})
    frame_source = RealSenseSource(
        width=cam_cfg.get("width", 640), height=cam_cfg.get("height", 480),
        fps=cam_cfg.get("fps", 30), stream=cam_cfg.get("stream", "color"),
        emitter_enabled=cam_cfg.get("emitter_enabled", False),
    )
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
    parser.add_argument(
        "--mock-rotation-axes", type=float, nargs=3, default=[1.0, 0.0, 0.0],
        metavar=("X", "Y", "Z"),
        help="--source mock only: per-axis [X, Y, Z] amplitude weight for the "
             "synthetic target rotation (pelvis frame). Default '1 0 0' is the "
             "original pure roll-about-X motion. '1 1 1' exercises pitch and "
             "yaw with a time-varying rotation axis -- the only camera-free "
             "way to verify the full 3-DoF rotation tracking fix (see "
             "docs/six_dof_tracking_fix.md).",
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
    parser.add_argument(
        "--force-sensor", dest="force_sensor", action="store_true", default=None,
        help="Force-enable the ATI F/T sensor during --record (overrides "
             "force_sensor.enabled in config).",
    )
    parser.add_argument(
        "--no-force-sensor", dest="force_sensor", action="store_false",
        help="Disable the F/T sensor for this run (e.g. sensor unplugged).",
    )
    parser.add_argument(
        "--target-force", type=float, default=None, metavar="N",
        help="Override palm_press.force_target (Newtons) for this run -- the "
             "palm-press loop will drive the hands together until this force "
             "is read on the palms, then hold it. Clamped to the configured "
             "[force_min, force_max] safety bounds (force_max is the hard "
             "crush-guard ceiling -- see docs/palm_force_control.md for how "
             "to pick a reasonable value via a pull-test). No effect if "
             "palm_press.enabled is false in config. Default: use the "
             "configured palm_press.force_target unchanged.",
    )
    parser.add_argument(
        "-v", "--verbose", action="store_true",
        help="Print the per-marker transform-chain diagnostic (optical -> "
             "d435 mount -> pelvis positions) in Phase 2's ~1Hz status block. "
             "A calibration-debugging aid -- off by default, since it adds "
             "six lines per second to the run output.",
    )
    parser.add_argument(
        "--gp-predict", dest="gp_predict", action="store_true", default=None,
        help="Enable the torso-GP feedforward for this run (overrides "
             "trajectory.gp_predict.enabled). Adds the GP's predicted lead of "
             "the TARGET marker to the tracked pose, cancelling the ~1-tick "
             "lag of the ~6-10Hz loop. --source marker only; wrapped INSIDE "
             "the rate limiter and rotation clamp so a prediction can never "
             "bypass those bounds. The GP anchors after the palm press seats "
             "(trajectory.gp_predict.arm_after_seat).",
    )
    parser.add_argument(
        "--no-gp-predict", dest="gp_predict", action="store_false",
        help="Disable the torso-GP feedforward for this run.",
    )
    parser.add_argument(
        "--gp-lead-scale", type=float, default=None, metavar="S",
        help="Override trajectory.gp_predict.lead_scale for this run. 1.0 = "
             "the GP's own predicted velocity integrated over the measured "
             "tick dt; <1 is a conservative first live test. Values >1 are "
             "not recommended (aiming past the effective latency is worse "
             "than no prediction at all).",
    )
    parser.add_argument(
        "--wait-for-keypress", action="store_true",
        help="--source marker only: after both markers are detected, wait for "
             "Enter instead of the fixed 3s countdown before Phase 1 begins. "
             "Useful on its own (time to double check the Rerun view before "
             "committing to Phase 1) and combines with --test-start-left/"
             "--test-start-right below. Default: off (fixed countdown).",
    )
    parser.add_argument(
        "--test-start-left", type=float, nargs=6, default=None,
        metavar=("X", "Y", "Z", "ROLL", "PITCH", "YAW"),
        help="Actively drive the LEFT hand to this pose (meters, radians, "
             "pelvis frame, fixed-axis rpy) before tracking begins, to test "
             "whether Phase 1 correctly recovers from an arbitrary/weird "
             "starting configuration. The arm is always actively commanded "
             "there (never physically pushed/backdriven). May be given alone "
             "(right hand holds its current pose) or together with "
             "--test-start-right. Default: none (skip).",
    )
    parser.add_argument(
        "--pause-after-test-pose", action="store_true", default=False,
        help="After driving to --test-start-left/--test-start-right, hold that "
             "pose and wait for Enter before starting any tracking. Lets you "
             "inspect/adjust the arm at the typed pose. Default: off.",
    )
    parser.add_argument(
        "--test-start-right", type=float, nargs=6, default=None,
        metavar=("X", "Y", "Z", "ROLL", "PITCH", "YAW"),
        help="Actively drive the RIGHT hand to this pose before tracking "
             "begins. See --test-start-left.",
    )
    parser.add_argument(
        "--log-joint-path", default=None, metavar="PATH",
        help="Write a per-tick CSV log of every joint command sent during "
             "Phase 1a's RRT*-planned approach (controllers/"
             "joint_path_execution.py's execute_joint_path) to this path -- "
             "timestamp, segment index, step index, segment length/time/"
             "n_steps, per-tick delta norm, and the full q_cmd vector. "
             "Diagnostic tool for tracing a sudden-motion incident back to "
             "the exact waypoint/tick that produced it; independent of "
             "Rerun. No-op if collision_avoidance.enabled is false (Phase 1a "
             "doesn't call execute_joint_path in that case). Default: off.",
    )
    parser.add_argument(
        "--diagnostic-log", default=None, metavar="PATH",
        help="Write ONE global, wall-clock-anchored JSONL log spanning the "
             "whole run to this path -- every command this codebase sends "
             "(Phase 1a per-tick, Phase 1b per-tick, Phase 3 per-tick, the "
             "Phase1a->1b transition handoff-vs-fresh-state gap, the RRT "
             "waypoint plan) PLUS a read-only poll of the vendored SDK's "
             "own internal dry-run arm state (G1Arm._q_target/_cmd_q_state/"
             "_sim_q/_arm_velocity_limit -- see diagnostics/sdk_state_tap.py) "
             "at ~4ms resolution. Built to root-cause a smooth-then-violent "
             "motion bug by watching real data instead of guessing at a "
             "mechanism -- see diagnostics/run_log.py. Independent of "
             "--log-joint-path (which stays Phase-1a-only, CSV). "
             "Default: off (zero added overhead).",
    )
    args = parser.parse_args()

    if args.diagnostic_log is not None:
        RUN_LOG.start(args.diagnostic_log)
        print(f"[preview] Diagnostic log: {args.diagnostic_log}")

    cfg = load_config(args.config)
    perception = cfg["perception"]
    obj_id = perception["object_marker_id"]
    tgt_id = perception["target_marker_id"]

    gp_cfg_override = cfg.setdefault("trajectory", {}).setdefault("gp_predict", {})
    if args.gp_predict is not None:
        gp_cfg_override["enabled"] = args.gp_predict
    if args.gp_lead_scale is not None:
        gp_cfg_override["lead_scale"] = args.gp_lead_scale
    if gp_cfg_override.get("enabled", False) and args.source != "marker":
        print(f"[preview] NOTE: GP feedforward requested but --source is "
              f"'{args.source}'; it applies to --source marker only and will "
              f"be skipped this run.")

    if args.target_force is not None:
        palm_press_cfg = cfg.setdefault("palm_press", {})
        if not palm_press_cfg.get("enabled", False):
            print(f"[preview] WARNING: --target-force {args.target_force:.2f}N was "
                  f"given but palm_press.enabled is false in config -- palm-press "
                  f"grip is off, so this has NO effect. (Finger-force mode's "
                  f"equivalent, grip_force.tau_baseline, is config-only.)")
        else:
            force_min = palm_press_cfg.get("force_min", 0.0)
            force_max = palm_press_cfg.get("force_max", 0.0)
            effective, warning = clamp_target_force(args.target_force, force_min, force_max)
            if warning is not None:
                print(f"[preview] WARNING: {warning}")
            print(f"[preview] Palm-press target force: requested={args.target_force:.2f}N, "
                  f"effective={effective:.2f}N  "
                  f"(configured bounds: force_min={force_min:.2f}N, force_max={force_max:.2f}N)")
            palm_press_cfg["force_target"] = effective

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
    box_center_offset = cfg["box"].get("center_offset_m")
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
            axes=args.mock_rotation_axes,
        )
        source = MarkerSource(mock_tracker, obj_id, tgt_id, mode=args.tracking, countdown=0.0,
                              box_half_height=box_half_height,
                              box_center_offset=box_center_offset,
                              on_observations=log_marker_observations)
        duration = None   # use config default
        cam_for_preview = RealSenseCamera(dry_run=True)
        cam_for_preview.connect()
        cam_for_preview.attach_preview(preview)

    elif args.source == "marker":
        print("[preview] Setting up live RealSense + AprilTag tracker…")
        cam_cfg = perception.get("camera", {})
        frame_source = RealSenseSource(
            width=cam_cfg.get("width", 640), height=cam_cfg.get("height", 480),
            fps=cam_cfg.get("fps", 30), stream=cam_cfg.get("stream", "color"),
            emitter_enabled=cam_cfg.get("emitter_enabled", False),
            preview=preview,
            pointcloud=perception.get("pointcloud_enabled", True),
        )
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
        if frame_source.ir_to_color_extrinsic is not None:
            # Infrared mode: perception.d435_link_to_optical is calibrated for
            # the color optical frame, so bridge the gap with the camera's own
            # live-queried factory extrinsic (IR-left -> color) rather than
            # requiring a second manual calibration.
            T_opt_to_d435 = T_opt_to_d435 @ frame_source.ir_to_color_extrinsic
            print(f"[preview] stream=infrared: composed d435_link_to_optical with the "
                  f"camera's live IR-left->color extrinsic.")
        # Zero-waist assumption -- corrected below with the robot's actual
        # live waist reading, once it's connected (low_state isn't populated
        # yet at this point in the script). Used as-is for sine/mock, and as
        # the initial value here until the correction below runs.
        T_d435_to_pelvis = d435_to_pelvis_from_pinocchio(ik=ik)
        # `native` is in OUR convention (fx/fy/cx/cy/resolution, for
        # CameraIntrinsics above) -- set_camera_intrinsics() expects the SDK's
        # own shape (width/height/ppx/ppy), so translate rather than pass
        # native through directly.
        sdk_intrinsics = {
            "width": native["resolution"][0],
            "height": native["resolution"][1],
            "fx": native["fx"],
            "fy": native["fy"],
            "ppx": native["cx"],
            "ppy": native["cy"],
        }
        # The SDK's own RealSenseCamera->preview wiring never passes
        # T_cam_in_world, so the camera/pointcloud/image entities in Rerun
        # render at identity instead of the real D435 mount pose. Re-log the
        # intrinsics ourselves with the pose included so the point cloud lines
        # up with the robot mesh and marker arrows in the viewer.
        preview.set_camera_intrinsics(
            sdk_intrinsics, T_cam_in_world=T_d435_to_pelvis @ T_opt_to_d435,
        )
        pelvis_offset_hotfix = perception.get("pelvis_offset_hotfix")
        if pelvis_offset_hotfix and any(pelvis_offset_hotfix):
            print(f"[preview] HOTFIX: applying pelvis_offset_hotfix={pelvis_offset_hotfix} "
                  f"to every marker reading -- experimental, not part of the real "
                  f"calibration. See configs/default.yaml to revert.")
        tracker = MarkerTracker(
            frame_source, detector, intrinsics, T_opt_to_d435, T_d435_to_pelvis,
            pelvis_offset=pelvis_offset_hotfix,
        )
        source = MarkerSource(tracker, obj_id, tgt_id, mode=args.tracking, countdown=3.0,
                              box_half_height=box_half_height,
                              box_center_offset=box_center_offset,
                              wait_for_keypress=args.wait_for_keypress,
                              on_observations=log_marker_observations)
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

        # ---------------------------------------------------------------
        # Torso-GP feedforward (marker mode only). Wrapped INSIDE the rate
        # limiter and rotation clamp below, so a prediction can never escape
        # those safety bounds. See configs/default.yaml:
        # trajectory.gp_predict and gp_predict/export/CONTROLLER_MATH.md.
        # ---------------------------------------------------------------
        gp_cfg = cfg.get("trajectory", {}).get("gp_predict", {})
        if gp_cfg.get("enabled", False):
            gp_export = os.path.join(_PROJECT_ROOT, "gp_predict", "export")
            if gp_export not in sys.path:
                sys.path.insert(0, gp_export)
            from torso_gp_predictor import TorsoGP
            model_dir = gp_cfg.get("model_dir", "gp_predict/export/models_robot")
            if not os.path.isabs(model_dir):
                model_dir = os.path.join(_PROJECT_ROOT, model_dir)
            gp = TorsoGP(model_dir)
            if tgt_id != 2:
                print(f"[preview] WARNING: trajectory.gp_predict is ON but the target "
                      f"marker is id={tgt_id}; the GP was trained on marker id=2 "
                      f"motion. Predictions on a different marker are unvalidated.")
            source = PredictiveSource(
                source, gp,
                lead_scale=gp_cfg.get("lead_scale", 1.0),
                max_lead_dt=gp_cfg.get("max_lead_dt_s", 0.5),
                # The controller arms this after the palm press seats -- see
                # sdk_pose_tracking.py Phase 2. Until then it is a pass-through.
                require_arm=gp_cfg.get("arm_after_seat", True),
            )
            print(f"[preview] GP feedforward ENABLED: {os.path.basename(model_dir)}, "
                  f"horizons={[round(h * gp.step_dt, 2) for h in gp.hset]}s, "
                  f"lead_scale={gp_cfg.get('lead_scale', 1.0)}")

    # -------------------------------------------------------------------
    # Rate-limit the trajectory source (uniform across sine/mock/marker).
    # See configs/default.yaml: trajectory.rate_limit.
    # -------------------------------------------------------------------
    rate_limit_cfg = cfg.get("trajectory", {}).get("rate_limit", {})
    if rate_limit_cfg.get("enabled", True):
        max_theta_rate_deg_s = rate_limit_cfg.get("max_theta_rate_deg_s", 60.0)
        max_pos_rate_m_s = rate_limit_cfg.get("max_pos_rate_m_s", 0.3)
        max_dt_s = rate_limit_cfg.get("max_dt_s", 0.5)
        source = RateLimitedSource(
            source,
            max_theta_rate=math.radians(max_theta_rate_deg_s),
            max_pos_rate=max_pos_rate_m_s,
            max_dt=max_dt_s,
        )
        print(f"[preview] Rate limiter enabled: {max_theta_rate_deg_s:.1f} deg/s, "
              f"{max_pos_rate_m_s:.2f} m/s (max_dt={max_dt_s:.2f}s)")

    # -------------------------------------------------------------------
    # Safety clamp on the tracked rotation (outermost -- see
    # RotationClampSource's docstring for why it must be last). See
    # configs/default.yaml: trajectory.tracked_rotation,
    # docs/six_dof_tracking_fix.md.
    # -------------------------------------------------------------------
    tracked_rotation_cfg = cfg.get("trajectory", {}).get("tracked_rotation", {})
    if tracked_rotation_cfg.get("enabled", True):
        max_angle_deg = tracked_rotation_cfg.get("max_angle_deg", None)
        enabled_axes = tracked_rotation_cfg.get("enabled_axes", None)
        source = RotationClampSource(
            source,
            max_angle=(None if max_angle_deg is None else math.radians(max_angle_deg)),
            enabled_axes=enabled_axes,
        )
        print(f"[preview] Rotation clamp enabled: max_angle={max_angle_deg}deg, "
              f"enabled_axes={enabled_axes}")

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

    # -------------------------------------------------------------------
    # Optional ATI F/T sensor (only meaningful when recording). Degrades
    # gracefully to None wrenches if the sensor is unreachable. See
    # g1_spine_stabilization/sensors/force_torque.py, docs/force_sensor_setup.md.
    # -------------------------------------------------------------------
    wrench_source = None
    if recorder is not None:
        wrench_source = ForceTorqueSource.from_config(cfg, enabled_override=args.force_sensor)
        wrench_source.start()

    print(f"[preview] Connecting robot (dry_run={args.dry_run})…")
    robot.connect()
    robot.initialize()
    robot.start_streaming()
    print("[preview] Robot ready. Starting controller…")

    if args.diagnostic_log is not None:
        start_sdk_state_tap(robot)

    if args.source == "marker" and robot.arm.low_state is not None:
        # T_d435_to_pelvis above assumed the waist joints sit at exactly
        # zero -- that's only true if the robot's own balance control never
        # nudges them (e.g. to counterbalance a loaded, outstretched arm).
        # low_state is only populated once connected LIVE (dry_run leaves it
        # None -- there's no real robot to read a waist angle from, so the
        # zero assumption is what dry-run is supposed to simulate anyway).
        # Motor indices 12/13/14 = WaistYaw/WaistRoll/WaistPitch (SDK convention).
        model, _ = ik.get_full_model_and_data()
        waist_yaw   = robot.arm.low_state.motor_state[12].q
        waist_roll  = robot.arm.low_state.motor_state[13].q
        waist_pitch = robot.arm.low_state.motor_state[14].q
        q_full = waist_q_full(model, waist_yaw, waist_roll, waist_pitch)
        T_d435_to_pelvis_live = d435_to_pelvis_from_pinocchio(
            locked_joint_config=q_full, ik=ik,
        )
        pos_delta_mm = 1000 * np.linalg.norm(
            T_d435_to_pelvis_live[:3, 3] - T_d435_to_pelvis[:3, 3]
        )
        R_delta = T_d435_to_pelvis[:3, :3].T @ T_d435_to_pelvis_live[:3, :3]
        angle_delta_deg = np.degrees(np.linalg.norm(Rotation.from_matrix(R_delta).as_rotvec()))
        print(f"[preview] Live waist angles (deg): "
              f"yaw={math.degrees(waist_yaw):+.2f} "
              f"roll={math.degrees(waist_roll):+.2f} "
              f"pitch={math.degrees(waist_pitch):+.2f}")
        print(f"[preview] T_d435_to_pelvis shift from the zero-waist assumption: "
              f"{pos_delta_mm:.1f}mm, {angle_delta_deg:.2f}deg")
        tracker.set_T_d435_to_pelvis(T_d435_to_pelvis_live)
        preview.set_camera_intrinsics(
            sdk_intrinsics, T_cam_in_world=T_d435_to_pelvis_live @ T_opt_to_d435,
        )
    elif args.source == "marker":
        print("[preview] --dry-run: no live robot to read a waist angle from -- "
              "keeping the zero-waist assumption for T_d435_to_pelvis.")

    test_start_pose = None
    if args.test_start_left is not None or args.test_start_right is not None:
        L_test = (se3_from_xyz_rpy(args.test_start_left[:3], args.test_start_left[3:])
                  if args.test_start_left is not None else None)
        R_test = (se3_from_xyz_rpy(args.test_start_right[:3], args.test_start_right[3:])
                  if args.test_start_right is not None else None)
        test_start_pose = (L_test, R_test)

    try:
        run_pose_tracking_sdk(robot, ik, cfg, source, duration=duration,
                              preview=preview, cam_for_preview=cam_for_preview,
                              recorder=recorder, wrench_source=wrench_source,
                              test_start_pose=test_start_pose,
                              pause_after_test_pose=args.pause_after_test_pose,
                              log_joint_path=args.log_joint_path,
                              verbose=args.verbose)
    finally:
        robot.shutdown()
        if wrench_source is not None:
            wrench_source.close()
        if cam_for_preview is not None:
            cam_for_preview.shutdown()
        if args.source == "marker":
            frame_source.close()
        if args.diagnostic_log is not None:
            RUN_LOG.stop()
        print("[preview] Shutdown complete.")


if __name__ == "__main__":
    main()
