#!/usr/bin/env python3
"""Calibrate perception.d435_link_to_optical from a known marker position.

Camera-only -- never touches robot.arm/robot.hand, so this is Claude-runnable
per CLAUDE.md's Hard Rules (same category as visualize_markers.py and
run_pose_tracking_preview.py's --detect mode).

Workflow:
  1. Physically place an AprilTag marker at a measured, known position in the
     pelvis frame (x, y, z in meters). Pass --marker-rpy if you measured its
     orientation (e.g. against a squared fixture or a level/protractor); it
     defaults to identity, which is only correct if the marker is truly
     axis-aligned with the pelvis frame -- an eyeballed placement is a
     leading source of residual rotation error in the resulting calibration.
  2. Run this script pointed at that marker:
       python scripts/calibrate_camera_extrinsics.py \\
           --marker-id 0 --marker-pos 0.40 0.0 0.0 --marker-rpy 0.0 0.0 0.0
     It scans for a few seconds, averages the raw camera-frame reading, and
     prints the T_optical_to_d435 correction (xyz + rpy) that would make the
     perception pipeline agree with your measurement. This does NOT edit
     configs/default.yaml -- copy the printed values in yourself.
  3. A single point can't distinguish a true mounting-offset error from a
     perception.marker_size scale error (both look identical at one distance,
     diverge at another). If you have a second marker at a different known
     distance, verify against it in --verify mode (checks the CURRENT config,
     no math):
       python scripts/calibrate_camera_extrinsics.py --verify \\
           --marker-id 2 --marker-pos 0.50 0.0 0.0
     Small residual error there confirms the fix generalizes; a large one
     that grows with distance points at marker_size instead.
"""
import argparse
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
from g1_spine_stabilization.perception.calibration import (
    pose_error, se3_to_xyz_rpy, solve_optical_to_d435,
)
from g1_spine_stabilization.perception.detector import AprilTagDetector, CameraIntrinsics
from g1_spine_stabilization.perception.frame_sources import RealSenseSource
from g1_spine_stabilization.perception.frames import (
    d435_to_pelvis_from_pinocchio, optical_to_d435_from_config,
)
from g1_spine_stabilization.perception.tracker import MarkerTracker
from g1_spine_stabilization.utils.config import load_config
from g1_spine_stabilization.utils.transforms import make_se3


def _average_optical_poses(tracker, marker_id, duration):
    """Scan for `duration` seconds, return the mean SE3 pose_in_optical
    reading for marker_id (translation mean + Rotation.mean()), or None if
    it was never seen."""
    positions = []
    rotations = []
    t_end = time.monotonic() + duration
    n_frames = 0
    while time.monotonic() < t_end:
        tracker.step()
        n_frames += 1
        diag = tracker.get_diagnostic_pose(marker_id)
        if diag is not None:
            T_opt = diag["optical"]
            positions.append(T_opt[:3, 3])
            rotations.append(T_opt[:3, :3])
    print(f"[calibrate] Scanned {n_frames} frames, "
          f"{len(positions)} with marker ID={marker_id} detected.")
    if not positions:
        return None
    xyz = np.mean(np.stack(positions), axis=0)
    R = Rotation.from_matrix(np.stack(rotations)).mean().as_matrix()
    return make_se3(xyz, R)


def _print_xyz_rpy(label, xyz, rpy):
    print(f"[calibrate] {label}:")
    print(f"  xyz: [{xyz[0]:.4f}, {xyz[1]:.4f}, {xyz[2]:.4f}]")
    print(f"  rpy: [{rpy[0]:.4f}, {rpy[1]:.4f}, {rpy[2]:.4f}]")


def main():
    parser = argparse.ArgumentParser(
        description="Calibrate perception.d435_link_to_optical from a known marker position."
    )
    parser.add_argument("--marker-id", type=int, required=True,
                         help="AprilTag ID of the marker to read.")
    parser.add_argument("--marker-pos", type=float, nargs=3, required=True,
                         metavar=("X", "Y", "Z"),
                         help="Known ground-truth marker position in the pelvis frame (m).")
    parser.add_argument("--marker-rpy", type=float, nargs=3, default=[0.0, 0.0, 0.0],
                         metavar=("R", "P", "Y"),
                         help="Known ground-truth marker orientation in the pelvis frame "
                              "(radians, fixed-axis rpy). Defaults to identity, which only "
                              "gives a correct calibration if the physical marker is truly "
                              "axis-aligned with the pelvis frame -- an eyeballed placement "
                              "is a leading source of residual rotation error. Measure this "
                              "(e.g. with a squared fixture or a level/protractor) rather "
                              "than assuming identity if the resulting calibration looks off.")
    parser.add_argument("--duration", type=float, default=5.0,
                         help="Seconds to scan and average over (default: 5.0).")
    parser.add_argument("--verify", action="store_true",
                         help="Check the CURRENT config's residual error against "
                              "--marker-id/--marker-pos; no correction is computed.")
    parser.add_argument("--config", default=None, help="Path to YAML config")
    args = parser.parse_args()

    cfg = load_config(args.config)
    perception = cfg["perception"]

    rpy = np.array(args.marker_rpy, dtype=float)
    R_marker_true = Rotation.from_euler("ZYX", [rpy[2], rpy[1], rpy[0]]).as_matrix()
    T_marker_true_pelvis = make_se3(np.array(args.marker_pos, dtype=float), R_marker_true)

    print("[calibrate] Loading IK solver (for T_d435_to_pelvis via FK)...")
    ik = G1HandIK()
    T_d435_to_pelvis = d435_to_pelvis_from_pinocchio(ik=ik)
    T_optical_to_d435 = optical_to_d435_from_config(perception["d435_link_to_optical"])

    print("[calibrate] Opening RealSense camera...")
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
    if frame_source.ir_to_color_extrinsic is not None:
        T_optical_to_d435 = T_optical_to_d435 @ frame_source.ir_to_color_extrinsic
        print("[calibrate] stream=infrared: composed d435_link_to_optical with the "
              "camera's live IR-left->color extrinsic.")
    detector = AprilTagDetector(perception["marker_dict"], perception["marker_size"])
    tracker = MarkerTracker(
        frame_source, detector, intrinsics, T_optical_to_d435, T_d435_to_pelvis
    )

    print(f"[calibrate] Scanning for marker ID={args.marker_id} "
          f"({args.duration:.1f}s)...")
    pose_in_optical_avg = _average_optical_poses(tracker, args.marker_id, args.duration)
    frame_source.close()

    if pose_in_optical_avg is None:
        print(f"[calibrate] ERROR: marker ID={args.marker_id} was never detected. "
              f"Check it's in view and the ID matches configs/default.yaml.")
        sys.exit(1)

    if args.verify:
        # Compose the pelvis pose using whatever T_optical_to_d435 is
        # CURRENTLY in the config -- no correction math, just a residual check.
        current_pelvis_pose = T_d435_to_pelvis @ T_optical_to_d435 @ pose_in_optical_avg
        pos_err, rot_err = pose_error(current_pelvis_pose, T_marker_true_pelvis)
        print(f"[calibrate] Verifying CURRENT config against marker ID={args.marker_id} "
              f"at known position {args.marker_pos}...")
        print(f"  position error: {pos_err:.4f} m")
        print(f"  rotation error: {rot_err:.2f} deg")
        if pos_err < 0.01 and rot_err < 1.0:
            print("[calibrate] Looks good -- residual error is small.")
        else:
            print("[calibrate] Residual error is still large. If this grows with "
                  "marker distance, perception.marker_size is the more likely "
                  "remaining culprit rather than d435_link_to_optical.")
        return

    # -------------------------------------------------------------------
    # Default mode: compute and print the correction.
    # -------------------------------------------------------------------
    current_pelvis_pose = T_d435_to_pelvis @ T_optical_to_d435 @ pose_in_optical_avg
    pos_err_before, rot_err_before = pose_error(current_pelvis_pose, T_marker_true_pelvis)
    print(f"[calibrate] Current d435_link_to_optical:")
    cur_xyz, cur_rpy = se3_to_xyz_rpy(T_optical_to_d435)
    _print_xyz_rpy("(current)", cur_xyz, cur_rpy)
    print(f"[calibrate] Measured error with current calibration:")
    print(f"  position error: {pos_err_before:.4f} m")
    print(f"  rotation error: {rot_err_before:.2f} deg")

    T_corrected = solve_optical_to_d435(
        T_d435_to_pelvis, T_marker_true_pelvis, pose_in_optical_avg
    )
    new_xyz, new_rpy = se3_to_xyz_rpy(T_corrected)
    _print_xyz_rpy("Computed correction", new_xyz, new_rpy)

    predicted_pelvis_pose = T_d435_to_pelvis @ T_corrected @ pose_in_optical_avg
    pos_err_after, rot_err_after = pose_error(predicted_pelvis_pose, T_marker_true_pelvis)
    print(f"[calibrate] Predicted error after applying correction (sanity check, "
          f"should be ~0):")
    print(f"  position error: {pos_err_after:.6f} m")
    print(f"  rotation error: {rot_err_after:.4f} deg")

    print("\n[calibrate] Paste the values above into configs/default.yaml's "
          "perception.d435_link_to_optical block (NOT auto-applied).")
    print("[calibrate] If you have a second marker at a different known distance, "
          "verify the fix generalizes with:")
    print("  python scripts/calibrate_camera_extrinsics.py --verify "
          "--marker-id <id> --marker-pos <x> <y> <z>")


if __name__ == "__main__":
    main()
