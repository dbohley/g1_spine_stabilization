#!/usr/bin/env python3
"""Move the arms to specified hand poses and hold until Ctrl-C.

No camera, no markers, no trajectory source, no grasp, no tracking. Connects,
interpolates to the given --pose-left/--pose-right, holds there, then runs the
normal safe shutdown.

Holding needs no control loop -- the SDK's 250 Hz publisher thread keeps
re-publishing the last set_joint_targets() call, so the hold is a plain sleep
(the same mechanism shutdown.settle_time already relies on).

Usage:
    # Dry-run (safe, no robot required):
    python scripts/hold_hand_pose.py \
        --pose-right 0.3086 -0.0958 0.1888 -2.3598 1.0183 -0.8298

    # Live hardware -- HUMAN-RUN ONLY, robot mounted and E-stop in reach:
    python scripts/hold_hand_pose.py --no-dry-run --net enx70886b87c0b8 \
        --pose-right 0.3086 -0.0958 0.1888 -2.3598 1.0183 -0.8298

Poses are "x y z roll pitch yaw" in the pelvis frame (metres, radians) -- the
same 6-number format scripts/read_hand_pose.py prints, so its output pastes
straight in. Omit a hand to leave it wherever it currently is.
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent.parent))

from g1_inspire_sdk import G1InspireRobot, RerunPreview
from g1_spine_stabilization.controllers.sdk_pose_tracking import _drive_to_test_pose
from g1_spine_stabilization.kinematics.ik_solver import G1HandIK
from g1_spine_stabilization.utils.config import load_config
from g1_spine_stabilization.utils.transforms import se3_from_xyz_rpy


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Move to specified hand poses and hold until Ctrl-C",
    )
    parser.add_argument(
        "--net", default="enx70886b87c0b8",
        help="DDS network interface. Ignored in dry-run.",
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
        "--pose-left", type=float, nargs=6, default=None,
        metavar=("X", "Y", "Z", "ROLL", "PITCH", "YAW"),
        help="Left hand target, pelvis frame. Omit to hold the left hand "
             "wherever it currently is.",
    )
    parser.add_argument(
        "--pose-right", type=float, nargs=6, default=None,
        metavar=("X", "Y", "Z", "ROLL", "PITCH", "YAW"),
        help="Right hand target, pelvis frame. Omit to hold the right hand "
             "wherever it currently is.",
    )
    parser.add_argument(
        "--preview", action="store_true", default=False,
        help="Open the Rerun viewer",
    )
    args = parser.parse_args()

    if args.pose_left is None and args.pose_right is None:
        parser.error("give at least one of --pose-left / --pose-right")

    cfg = load_config()
    control_dt = cfg["control"]["dt"]
    interpolation_time = cfg["control"]["interpolation_time"]
    settle_time = cfg.get("shutdown", {}).get("settle_time", 0.4)
    gravity_comp_cfg = cfg.get("gravity_comp", {})
    gravity_comp_enabled = gravity_comp_cfg.get("enabled", True)

    L_target = se3_from_xyz_rpy(args.pose_left[:3], args.pose_left[3:]) \
        if args.pose_left is not None else None
    R_target = se3_from_xyz_rpy(args.pose_right[:3], args.pose_right[3:]) \
        if args.pose_right is not None else None

    if not args.dry_run:
        print("[hold_pose] WARNING: live mode selected.")
        print("[hold_pose]   - Robot must be mounted and powered")
        print("[hold_pose]   - E-stop must be in reach")
        print("[hold_pose]   - initialize() will move arms to default pose")
        input("[hold_pose] Press Enter to continue (Ctrl-C to abort)...")

    # Load IK before connect/initialize so the ~10s cold start doesn't land in
    # the middle of a live connection.
    print("[hold_pose] Loading IK solver (may take ~10s on cold start)...")
    tauff_filter_cfg = gravity_comp_cfg.get("tauff_filter", {})
    ik = G1HandIK(
        gravity_comp_enabled=gravity_comp_enabled,
        tauff_filter_enabled=tauff_filter_cfg.get("enabled", True),
        tauff_filter_alpha=tauff_filter_cfg.get("alpha", 0.3),
    )

    preview = RerunPreview(spawn=True) if args.preview else None
    robot = G1InspireRobot(
        network_interface=args.net,
        arm_side="both",
        dry_run=args.dry_run,
        preview=preview,
    )

    # Arm-only lifecycle. G1InspireRobot's own connect()/initialize()/shutdown()
    # unconditionally drive self.hand too, and the Inspire hands' initialize()
    # hard-fails (Modbus timeout -> RuntimeError) when they aren't on the
    # network. This script never commands a hand, so go straight to robot.arm
    # and leave the hand untouched.
    print(f"[hold_pose] Connecting arm (dry_run={args.dry_run})…")
    robot.arm.connect()
    print("[hold_pose] Initializing arm (zero torque -> default pose)…")
    robot.arm.initialize()
    print("[hold_pose] Starting 250 Hz publisher.")
    robot.arm.start_streaming()

    try:
        _drive_to_test_pose(robot, ik, control_dt, interpolation_time,
                            gravity_comp_enabled, L_target, R_target)
        print("[hold_pose] Holding pose. Ctrl-C to release and shut down.")
        while True:
            time.sleep(0.5)
    except KeyboardInterrupt:
        print("\n[hold_pose] Ctrl-C -- releasing.")
    finally:
        # The publisher thread is still re-sending the last commanded pose, so
        # this settle just holds it briefly before shutdown()'s damping command.
        time.sleep(settle_time)
        robot.arm.shutdown()
        print("[hold_pose] Shutdown complete.")


if __name__ == "__main__":
    main()
