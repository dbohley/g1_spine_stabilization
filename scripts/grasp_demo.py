#!/usr/bin/env python3
"""Bimanual grasp/ungrasp demo for Inspire RH56 hands.

Tests that both hands open and close correctly via GraspController.
Intended as a pre-flight check before running the full pose-tracking
controller with grip force.

Phase sequence (see g1_spine_stabilization.controllers.grasp_sequence for
the implementation and the safe-shutdown guarantee -- hand-release and
arm-return-to-default always run before robot.shutdown(), even on
Ctrl-C/exceptions during the grasp/hold phases):
  0. IK arm to grasp pose (palms facing each other, from SDK default zeros)
  A. Hold at open (1 s)
  B. Lerp hand to close (1.5 s)
  C. Hold with force-feedback control (--hold-time, default 3 s)
  D. Lerp hand to open / release (1 s)
  R. IK arm return to standing default

Usage:
    # Dry-run (safe, no robot required):
    python scripts/grasp_demo.py

    # With Rerun visualization (opens viewer):
    python scripts/grasp_demo.py --preview

    # Live hardware — HUMAN-RUN ONLY with robot mounted and E-stop in reach:
    python scripts/grasp_demo.py --no-dry-run --net eth0 --preview

    # One hand only:
    python scripts/grasp_demo.py --side left
    python scripts/grasp_demo.py --side right
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent.parent))

from g1_inspire_sdk import G1InspireRobot, RerunPreview
from g1_spine_stabilization.controllers.grasp_sequence import run_grasp_phases
from g1_spine_stabilization.controllers.grip_force import GraspController
from g1_spine_stabilization.kinematics.ik_solver import G1HandIK
from g1_spine_stabilization.utils.config import load_config


def main() -> None:
    parser = argparse.ArgumentParser(description="Bimanual grasp/ungrasp demo")
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
        "--hold-time", type=float, default=3.0,
        help="Seconds to hold the grasp (default: 3.0)",
    )
    parser.add_argument(
        "--side", choices=["both", "left", "right"], default="both",
        help="Which hand(s) to actuate (default: both)",
    )
    parser.add_argument(
        "--preview", action="store_true", default=False,
        help="Open Rerun viewer and log per-finger force time-series",
    )
    parser.add_argument(
        "--rate", type=float, default=50.0,
        help="Control rate Hz for hand interpolation phases (default: 50.0)",
    )
    args = parser.parse_args()

    cfg = load_config()
    gravity_comp_enabled = cfg.get("gravity_comp", {}).get("enabled", True)

    do_left  = args.side in ("both", "left")
    do_right = args.side in ("both", "right")

    ctrl_l = GraspController(cfg["grip_force"]) if do_left  else None
    ctrl_r = GraspController(cfg["grip_force"]) if do_right else None

    preview = RerunPreview(spawn=True) if args.preview else None

    if not args.dry_run:
        print("[grasp_demo] WARNING: live mode selected.")
        print("[grasp_demo]   - Robot must be mounted and powered")
        print("[grasp_demo]   - E-stop must be in reach")
        print("[grasp_demo]   - initialize() will move arms to default pose")
        input("[grasp_demo] Press Enter to continue (Ctrl-C to abort)...")

    print(f"[grasp_demo] Starting (dry_run={args.dry_run}, side={args.side}, preview={args.preview})")

    # Load IK before connect/initialize so the cold-start delay (~10s) doesn't
    # create a gap inside the live control loop.
    print("[grasp_demo] Loading IK solver (may take ~10s on cold start)...")
    ik = G1HandIK(gravity_comp_enabled=gravity_comp_enabled)

    robot = G1InspireRobot(
        network_interface=args.net,
        arm_side="both",
        hand_side=args.side,
        dry_run=args.dry_run,
        preview=preview,
    )

    print("Connecting..."); robot.connect()
    print("Initializing (zero torque -> default pose)..."); robot.initialize()
    print("Starting 250 Hz publisher."); robot.start_streaming()

    try:
        run_grasp_phases(
            robot, ik, cfg,
            ctrl_l=ctrl_l, ctrl_r=ctrl_r,
            do_left=do_left, do_right=do_right,
            hold_time=args.hold_time, rate=args.rate,
            preview_enabled=args.preview, dry_run=args.dry_run,
        )
    finally:
        robot.shutdown()

    print("[grasp_demo] Done.")


if __name__ == "__main__":
    main()
