#!/usr/bin/env python3
"""Read the robot's ACTUAL current hand poses, for picking --test-start-* values.

Connects to the robot but calls ONLY connect() -- never initialize() or
start_streaming() -- so no motor command is ever sent. Safe to run while the
arm is powered on and physically compliant: pose each arm by hand, run this
script, and copy the printed --test-start-left/--test-start-right line
straight into run_pose_tracking_preview.py instead of guessing numbers.

Usage:
    # Dry-run (safe, no robot required -- prints FK of the fallback default
    # pose, since low_state stays None with no live robot to read):
    python scripts/read_hand_pose.py

    # Live hardware -- HUMAN-RUN ONLY. Robot powered on, compliant, arms
    # posed by hand:
    python scripts/read_hand_pose.py --no-dry-run --net eth0
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent.parent))

import numpy as np
from scipy.spatial.transform import Rotation

from g1_inspire_sdk import G1InspireRobot
from g1_spine_stabilization.kinematics.ik_solver import G1HandIK

# Default q used when the robot state isn't available (e.g. dry_run) --
# same fallback constant as controllers/sdk_pose_tracking.py's _get_q.
ARM_DEFAULT_Q = np.array([
    -0.07641,  0.3355,  -0.38365, -0.04522,  1.18319,  0.3721,  0.10582,
    -0.0737,  -0.3477,   0.37916, -0.03927, -1.17787,  0.37036, -0.0962,
])


def _get_q(robot) -> np.ndarray:
    """Read current arm joint config from the robot (14,). Falls back to ARM_DEFAULT_Q."""
    try:
        state = robot.get_state()
        q = np.asarray(state["arm"]["q"], dtype=float)
        if q.shape == (14,):
            return q
    except Exception:
        pass
    return ARM_DEFAULT_Q.copy()


def rotation_to_rpy(R: np.ndarray) -> tuple:
    """Fixed-axis roll/pitch/yaw from a rotation matrix -- the inverse of
    utils/transforms.py's se3_from_xyz_rpy, which builds
    R = Rz(yaw) @ Ry(pitch) @ Rx(roll) via
    Rotation.from_euler("ZYX", [yaw, pitch, roll]). Uses the matching
    uppercase "ZYX" convention (not trajectory/sources.py's lowercase 'xyz',
    which is a different, non-invertible-with-this-pair convention used only
    for a human-readable marker-orientation print elsewhere)."""
    yaw, pitch, roll = Rotation.from_matrix(R).as_euler("ZYX")
    return roll, pitch, yaw


def main() -> None:
    parser = argparse.ArgumentParser(description="Read current hand poses for --test-start-*")
    parser.add_argument(
        "--net", default="eth0",
        help="DDS network interface (e.g. eth0). Ignored in dry-run.",
    )
    parser.add_argument(
        "--dry-run", dest="dry_run", action="store_true", default=True,
        help="Read the fallback default pose without connecting to hardware (default: True)",
    )
    parser.add_argument(
        "--no-dry-run", dest="dry_run", action="store_false",
        help="Connect to live hardware (HUMAN-RUN ONLY). Only calls connect() -- "
             "no initialize()/start_streaming() -- so no motor command is ever sent.",
    )
    args = parser.parse_args()

    # Only .fk() is needed here, so no config/gravity-comp/tauff-filter
    # settings are relevant -- construct with defaults, matching
    # verify_ik.py's and calibrate_camera_extrinsics.py's pattern.
    print("[read_hand_pose] Loading IK solver (may take ~10s on cold start)...")
    ik = G1HandIK()

    if not args.dry_run:
        print("[read_hand_pose] WARNING: live mode selected.")
        print("[read_hand_pose]   - Robot must be powered on and reachable on the network")
        print("[read_hand_pose]   - Only connect() is called -- no motor command is ever sent")
        input("[read_hand_pose] Press Enter to continue (Ctrl-C to abort)...")

    robot = G1InspireRobot(network_interface=args.net, dry_run=args.dry_run, arm_side="both")
    print(f"[read_hand_pose] Connecting (dry_run={args.dry_run})…")
    robot.connect()
    print("[read_hand_pose] Connected. Reading current joint state (no motor commands sent).")

    current_q = _get_q(robot)
    L_pose, R_pose = ik.fk(current_q)

    for label, pose, flag in (("LEFT", L_pose, "--test-start-left"),
                               ("RIGHT", R_pose, "--test-start-right")):
        x, y, z = pose.translation
        roll, pitch, yaw = rotation_to_rpy(pose.rotation)
        print(f"\n{label} hand:")
        print(f"  position (m):      x={x:.4f} y={y:.4f} z={z:.4f}")
        print(f"  orientation (rad): roll={roll:.4f} pitch={pitch:.4f} yaw={yaw:.4f}")
        print(f"  orientation (deg): roll={np.degrees(roll):.2f} pitch={np.degrees(pitch):.2f} yaw={np.degrees(yaw):.2f}")
        print(f"  CLI flag:          {flag} {x:.4f} {y:.4f} {z:.4f} {roll:.4f} {pitch:.4f} {yaw:.4f}")

    print("\n[read_hand_pose] Done. No motor command was ever sent (connect() only).")


if __name__ == "__main__":
    main()
