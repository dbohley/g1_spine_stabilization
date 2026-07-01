#!/usr/bin/env python3
"""Bimanual grasp/ungrasp demo for Inspire RH56 hands.

Tests that both hands open and close correctly via GraspController.
Intended as a pre-flight check before running the full pose-tracking
controller with grip force.

Phases:
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
import time
from pathlib import Path

import numpy as np
import rerun as rr

sys.path.insert(0, str(Path(__file__).parent.parent))

from g1_inspire_sdk import G1InspireRobot, HAND_SOFT_LOWER_LIMITS, RerunPreview
from g1_spine_stabilization.controllers.grip_force import GraspController
from g1_spine_stabilization.kinematics.ik_solver import G1HandIK
from g1_spine_stabilization.trajectory.targets import (
    compute_hand_targets,
    L_EE_DEFAULT,
    R_EE_DEFAULT,
    R_L_BASE,
    R_R_BASE,
)
from g1_spine_stabilization.utils.config import load_config
from g1_spine_stabilization.utils.transforms import make_se3, slerp_rotation

# Inspire RH56 finger order (matches hand.get_state return order)
_FINGER_NAMES = [
    "index_mcp", "middle_mcp", "ring_mcp",
    "little_mcp", "thumb_rotate", "thumb_flex",
]


def lerp_hand(q_a: np.ndarray, q_b: np.ndarray, t: float) -> np.ndarray:
    return (1.0 - t) * q_a + t * q_b


def smoothstep(alpha: float) -> float:
    return alpha * alpha * (3.0 - 2.0 * alpha)


def _get_q(robot) -> np.ndarray:
    """Read current 14-DOF arm config. Falls back to zeros on error."""
    try:
        state = robot.get_state()
        q = np.asarray(state["arm"]["q"], dtype=float)
        if q.shape == (14,):
            return q
    except Exception:
        pass
    return np.zeros(14)


def _get_hand_state(robot, side: str) -> dict:
    """Return {'q': ..., 'force': ...}. Falls back to zeros on any error."""
    try:
        return robot.hand.get_state(side)
    except Exception:
        return {"q": np.zeros(6), "force": np.zeros(6)}


def _fmt_force(f: np.ndarray) -> str:
    return "[" + " ".join(f"{v:+.2f}" for v in f) + "]"


def _log_hands(
    t: float,
    phase: str,
    do_left: bool,
    do_right: bool,
    state_l: "dict | None",
    state_r: "dict | None",
) -> None:
    """Log phase label and per-finger force to Rerun.

    Commanded q is logged automatically by RerunPreview via
    inspire_hand._mirror_to_preview() on every robot.step() call.
    """
    rr.set_time("time", duration=t)
    rr.log("demo/phase", rr.TextLog(phase))
    for side, state, active in (
        ("left",  state_l, do_left),
        ("right", state_r, do_right),
    ):
        if not active or state is None:
            continue
        force = np.asarray(state["force"], dtype=float)
        for i, name in enumerate(_FINGER_NAMES):
            rr.log(f"hands/{side}/force/{name}", rr.Scalars(float(force[i])))


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
    dt = cfg["control"]["dt"]
    interp_dt = cfg["control"]["dt"]          # 0.02 s — same as pose-tracking
    interp_time = cfg["control"]["interpolation_time"]  # 3.0 s

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
    ik = G1HandIK()

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

    _t0 = time.monotonic()
    h_open  = HAND_SOFT_LOWER_LIMITS.copy()
    h_grasp = (ctrl_l or ctrl_r).close_pose

    # (label, h_start, h_end, duration_s)
    phases = [
        ("open",  h_open, h_open,  1.0),   # Phase A: hold at open after arm reaches grasp
        ("close", h_open, h_grasp, 1.5),   # Phase B: lerp to grasp pose
    ]
    period = 1.0 / max(args.rate, 1.0)

    try:
        # ── Phase 0: IK arm to grasp pose ─────────────────────────────────
        # Start from where initialize() left the arm (SDK zeros) and smoothly
        # interpolate to the bimanual grasp pose. Avoids the jerk that would
        # occur if the first step commanded a completely different joint config.
        print("[grasp_demo] Phase: arm to grasp pose...")
        current_q = _get_q(robot)
        L_cur, R_cur = ik.fk(current_q)
        L_start_pos = L_cur.translation.copy()
        R_start_pos = R_cur.translation.copy()
        L_start_rot = L_cur.rotation.copy()
        R_start_rot = R_cur.rotation.copy()

        box_center     = np.array([L_EE_DEFAULT[0], 0.0, L_EE_DEFAULT[2]])
        box_half_width = cfg["box"]["size"][1] / 2.0
        L_grasp, R_grasp = compute_hand_targets(box_center, box_half_width, 0.0)

        n_steps = int(interp_time / interp_dt)  # 150 at dt=0.02, time=3.0
        for i in range(n_steps):
            alpha = smoothstep(i / n_steps)
            L_pos = (1 - alpha) * L_start_pos + alpha * L_grasp[:3, 3]
            R_pos = (1 - alpha) * R_start_pos + alpha * R_grasp[:3, 3]
            L_R   = slerp_rotation(L_start_rot, L_grasp[:3, :3], alpha)
            R_R   = slerp_rotation(R_start_rot, R_grasp[:3, :3], alpha)
            sol_q, _ = ik.solve_ik(
                make_se3(L_pos, L_R), make_se3(R_pos, R_R),
                current_q, np.zeros(14),
            )
            robot.step(q_arm=sol_q)
            current_q = sol_q
            time.sleep(interp_dt)

        grasp_arm_q = current_q.copy()
        print("[grasp_demo] Arm at grasp pose.")

        # ── Phases A + B: open and close with smoothstep interpolation ────
        for label, ha, hb, duration in phases:
            n_steps = max(1, int(duration * args.rate))
            print(f"[grasp_demo] Phase: {label} ({duration:.1f}s)")
            for s in range(n_steps + 1):
                t = smoothstep(s / n_steps)
                q_hand = lerp_hand(ha, hb, t)
                robot.step(
                    q_arm=grasp_arm_q,
                    q_hand_left=q_hand  if do_left  else None,
                    q_hand_right=q_hand if do_right else None,
                )
                if args.preview:
                    _log_hands(
                        time.monotonic() - _t0, label, do_left, do_right,
                        _get_hand_state(robot, "left")  if do_left  else None,
                        _get_hand_state(robot, "right") if do_right else None,
                    )
                time.sleep(period)

        # Settle and snapshot force-control baseline
        time.sleep(0.5)
        if ctrl_l:
            ctrl_l.arm(_get_hand_state(robot, "left")["q"])
        if ctrl_r:
            ctrl_r.arm(_get_hand_state(robot, "right")["q"])

        # ── Phase C: hold with force-feedback control ─────────────────────
        print(f"[grasp_demo] Phase: hold ({args.hold_time:.1f}s)")
        t_start = time.time()
        while time.time() - t_start < args.hold_time:
            state_l = _get_hand_state(robot, "left")  if ctrl_l else None
            state_r = _get_hand_state(robot, "right") if ctrl_r else None
            q_cmd_l = ctrl_l.step(state_l["force"], state_l["q"], dt) if ctrl_l else None
            q_cmd_r = ctrl_r.step(state_r["force"], state_r["q"], dt) if ctrl_r else None
            robot.step(
                q_arm=grasp_arm_q,
                q_hand_left=q_cmd_l,
                q_hand_right=q_cmd_r,
            )
            if args.preview:
                _log_hands(
                    time.monotonic() - _t0, "hold", do_left, do_right,
                    state_l, state_r,
                )
            if not args.dry_run:
                fl = _fmt_force(state_l["force"]) if state_l else "-"
                fr = _fmt_force(state_r["force"]) if state_r else "-"
                print(f"  force  L:{fl}  R:{fr}")
            time.sleep(dt)

        # ── Phase D: release (lerp back to open) ──────────────────────────
        if ctrl_l:
            ctrl_l.reset()
        if ctrl_r:
            ctrl_r.reset()

        n_steps = max(1, int(1.0 * args.rate))
        print("[grasp_demo] Phase: release (1.0s)")
        for s in range(n_steps + 1):
            t = smoothstep(s / n_steps)
            q_hand = lerp_hand(h_grasp, h_open, t)
            robot.step(
                q_arm=grasp_arm_q,
                q_hand_left=q_hand  if do_left  else None,
                q_hand_right=q_hand if do_right else None,
            )
            if args.preview:
                _log_hands(
                    time.monotonic() - _t0, "release", do_left, do_right,
                    _get_hand_state(robot, "left")  if do_left  else None,
                    _get_hand_state(robot, "right") if do_right else None,
                )
            time.sleep(period)

        # ── Phase R: IK arm return to standing default ────────────────────
        print("[grasp_demo] Phase: arm return to default...")
        L_cur, R_cur = ik.fk(grasp_arm_q)
        L_start_pos = L_cur.translation.copy()
        R_start_pos = R_cur.translation.copy()
        L_start_rot = L_cur.rotation.copy()
        R_start_rot = R_cur.rotation.copy()

        n_steps = int(interp_time / interp_dt)
        for i in range(n_steps):
            alpha = smoothstep(i / n_steps)
            L_pos = (1 - alpha) * L_start_pos + alpha * L_EE_DEFAULT
            R_pos = (1 - alpha) * R_start_pos + alpha * R_EE_DEFAULT
            L_R   = slerp_rotation(L_start_rot, R_L_BASE, alpha)
            R_R   = slerp_rotation(R_start_rot, R_R_BASE, alpha)
            sol_q, _ = ik.solve_ik(
                make_se3(L_pos, L_R), make_se3(R_pos, R_R),
                current_q, np.zeros(14),
            )
            robot.step(q_arm=sol_q)
            current_q = sol_q
            time.sleep(interp_dt)

        print("[grasp_demo] Trajectory done.")
    finally:
        robot.shutdown()

    print("[grasp_demo] Done.")


if __name__ == "__main__":
    main()
