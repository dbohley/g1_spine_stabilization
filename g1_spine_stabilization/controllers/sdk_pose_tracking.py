"""
Bimanual pose-tracking controller (SDK / Rerun preview path).

Mirrors run_pose_tracking_sim but drives G1InspireRobot instead of CoppeliaSim.
CoppeliaSim path (pose_tracking.py) is unchanged.

Three-phase routine:
  1. Interpolate EE targets from default standing pose to grasp pose.
  2. Drive both hands via the trajectory source until duration expires or
     KeyboardInterrupt (use None duration for marker-driven interactive mode).
  3. Interpolate EE targets back to default standing pose.

The caller owns the robot lifecycle: call connect/initialize/start_streaming
before this function, and shutdown in a finally block after.
"""

import math
import time

import numpy as np

from g1_spine_stabilization.utils.transforms import make_se3, slerp_rotation, smoothstep
from g1_spine_stabilization.trajectory.targets import (
    compute_hand_targets,
    R_L_BASE,
    R_R_BASE,
    L_EE_DEFAULT,
    R_EE_DEFAULT,
)

# Default q used when the robot state isn't available yet (e.g. dry_run startup)
ARM_DEFAULT_Q = np.array([
    -0.07641,  0.3355,  -0.38365, -0.04522,  1.18319,  0.3721,  0.10582,
    -0.0737,  -0.3477,   0.37916, -0.03927, -1.17787,  0.37036, -0.0962,
])


def _get_q(robot):
    """Read current arm joint config from the robot (14,). Falls back to ARM_DEFAULT_Q."""
    try:
        state = robot.get_state()
        q = np.asarray(state["arm"]["q"], dtype=float)
        if q.shape == (14,):
            return q
    except Exception:
        pass
    return ARM_DEFAULT_Q.copy()


def run_pose_tracking_sdk(robot, ik, cfg, source, duration=None):
    """Run the three-phase pose-tracking controller via the SDK transport.

    Args:
        robot:    G1InspireRobot instance (connect/initialize/start_streaming
                  already called by the caller).
        ik:       G1HandIK instance (already loaded).
        cfg:      config dict loaded from default.yaml.
        source:   trajectory source with .startup() and .step() -> (box_center, theta).
                  Use SineSource for sine-wave mode, MarkerSource for live AprilTag mode.
        duration: Phase 2 duration in seconds. None = run until KeyboardInterrupt
                  (marker-driven interactive mode).
    """
    control_dt         = cfg["control"]["dt"]
    interpolation_time = cfg["control"]["interpolation_time"]
    if duration is None:
        duration = cfg["control"]["rotation_duration"]

    # ---------------------------------------------------------------
    # Source startup — detect object marker or initialise sine timer
    # ---------------------------------------------------------------
    print("[SDKController] Starting source…")
    box_center = source.startup()
    box_size   = list(cfg["box"]["size"])
    box_half_width = box_size[1] / 2
    print(f"[SDKController] box_center={np.round(box_center, 4)}, half_width={box_half_width:.4f}")

    # ---------------------------------------------------------------
    # Phase 1: Interpolate to grasp pose
    # ---------------------------------------------------------------
    print(f"[SDKController] Phase 1: interpolating to grasp pose over {interpolation_time}s…")
    L_grasp, R_grasp = compute_hand_targets(box_center, box_half_width, 0.0)
    steps_interp = int(interpolation_time / control_dt)

    current_q = _get_q(robot)
    for i in range(steps_interp):
        alpha = smoothstep(i / steps_interp)

        L_pos = (1 - alpha) * L_EE_DEFAULT + alpha * L_grasp[:3, 3]
        R_pos = (1 - alpha) * R_EE_DEFAULT + alpha * R_grasp[:3, 3]
        L_R   = slerp_rotation(R_L_BASE, L_grasp[:3, :3], alpha)
        R_R   = slerp_rotation(R_R_BASE, R_grasp[:3, :3], alpha)

        current_dq = np.zeros(14)
        sol_q, _ = ik.solve_ik(
            make_se3(L_pos, L_R),
            make_se3(R_pos, R_R),
            current_q, current_dq,
        )
        robot.step(q_arm=sol_q)
        current_q = sol_q
        time.sleep(control_dt)

        if i % 50 == 0:
            print(f"  {alpha * 100:.0f}%")

    print("[SDKController] At grasp pose.")

    # ---------------------------------------------------------------
    # Phase 2: Track target
    # ---------------------------------------------------------------
    print(f"[SDKController] Phase 2: tracking target (duration={duration}s)…")
    t_start = time.monotonic()
    step_idx = 0
    _obj_lost_warned = False
    try:
        while True:
            elapsed = time.monotonic() - t_start
            if elapsed >= duration:
                break

            box_center, theta = source.step()
            if getattr(source, 'object_lost', False) and not _obj_lost_warned:
                print("[SDKController] WARNING: object marker lost — box may have fallen! (SDK-6: stop here)")
                _obj_lost_warned = True
            L_target, R_target = compute_hand_targets(box_center, box_half_width, theta)

            current_dq = np.zeros(14)
            sol_q, _ = ik.solve_ik(L_target, R_target, current_q, current_dq)
            robot.step(q_arm=sol_q)
            current_q = sol_q
            time.sleep(control_dt)

            if step_idx % 50 == 0:
                print(f"  t={elapsed:.1f}s  theta={math.degrees(theta):.1f}°")
            step_idx += 1

    except KeyboardInterrupt:
        print("\n[SDKController] KeyboardInterrupt — exiting Phase 2.")

    # ---------------------------------------------------------------
    # Phase 3: Return to default pose
    # ---------------------------------------------------------------
    print("[SDKController] Phase 3: returning to default pose.")

    L_current_pose, R_current_pose = ik.fk(current_q)
    L_start_pos = L_current_pose.translation.copy()
    R_start_pos = R_current_pose.translation.copy()
    L_start_rot = L_current_pose.rotation.copy()
    R_start_rot = R_current_pose.rotation.copy()

    steps_return = int(interpolation_time / control_dt)
    for i in range(steps_return):
        alpha = smoothstep(i / steps_return)

        L_pos = (1 - alpha) * L_start_pos + alpha * L_EE_DEFAULT
        R_pos = (1 - alpha) * R_start_pos + alpha * R_EE_DEFAULT
        L_R   = slerp_rotation(L_start_rot, R_L_BASE, alpha)
        R_R   = slerp_rotation(R_start_rot, R_R_BASE, alpha)

        current_dq = np.zeros(14)
        sol_q, _ = ik.solve_ik(
            make_se3(L_pos, L_R),
            make_se3(R_pos, R_R),
            current_q, current_dq,
        )
        robot.step(q_arm=sol_q)
        current_q = sol_q
        time.sleep(control_dt)

    print("[SDKController] Done.")
