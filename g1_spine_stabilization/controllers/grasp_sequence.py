"""Bimanual grasp/ungrasp phase sequence (SDK transport).

Extracted from scripts/grasp_demo.py so the safe-shutdown behavior below is
unit-testable (scripts/ is excluded from packaging, so nothing under it can
be imported by pytest).

Phases:
  0. IK arm to grasp pose (palms facing each other, from SDK default zeros)
  A. Hold at open (1 s)
  B. Lerp hand to close (1.5 s)
  C. Hold with force-feedback control (hold_time, default 3 s)
  D. Lerp hand to open / release (1 s)
  R. IK arm return to standing default

Phases D and R always run, even if 0/A/B/C raise (including KeyboardInterrupt),
so the arm and hand never get commanded to the SDK's fixed shutdown-damping
cutoff (kp=0, kd=8, tau=0 -- see G1Arm.shutdown()) from an actively-loaded
grasp pose. A brief settle pause (configs/default.yaml: shutdown.settle_time)
holds the final default pose immediately before the caller invokes
robot.shutdown() -- the SDK's 250 Hz publisher thread keeps re-sending the
last set_joint_targets() call for free, so this is a plain time.sleep().

Does NOT call robot.shutdown() -- caller's responsibility, mirroring
sdk_pose_tracking.run_pose_tracking_sdk's existing lifecycle contract.
"""

from __future__ import annotations

import time

import numpy as np
import rerun as rr

from g1_inspire_sdk import HAND_SOFT_LOWER_LIMITS
from g1_spine_stabilization.trajectory.targets import (
    compute_hand_targets,
    apply_palm_tilt,
    L_EE_DEFAULT,
    R_EE_DEFAULT,
    R_L_BASE,
    R_R_BASE,
)
from g1_spine_stabilization.utils.transforms import make_se3, slerp_rotation, smoothstep
from g1_spine_stabilization.controllers.joint_path_execution import (
    execute_joint_path,
    close_phase_steps,
    log_head_sphere,
    log_floor_plane,
    log_hand_collision_boxes,
    log_hand_goal_targets,
)
from g1_spine_stabilization.kinematics.collision import build_head_obstacle_stack
from g1_spine_stabilization.kinematics.rrt_planner import (
    plan_collision_free_approach,
    RRTPlanningError,
)

# Inspire RH56 finger order (matches hand.get_state return order)
_FINGER_NAMES = [
    "index_mcp", "middle_mcp", "ring_mcp",
    "little_mcp", "thumb_rotate", "thumb_flex",
]


def lerp_hand(q_a: np.ndarray, q_b: np.ndarray, t: float) -> np.ndarray:
    return (1.0 - t) * q_a + t * q_b


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


def run_grasp_phases(
    robot,
    ik,
    cfg,
    *,
    ctrl_l,
    ctrl_r,
    do_left: bool,
    do_right: bool,
    hold_time: float,
    rate: float,
    preview_enabled: bool = False,
    dry_run: bool = True,
    log_joint_path: str = None,
) -> None:
    """Run Phase 0 -> A -> B -> C -> D -> R.

    Guarantees hand-release (D) and arm-return-to-default (R), plus a brief
    settle pause, run even if Phase 0/A/B/C raises (including
    KeyboardInterrupt). Re-raises the first exception encountered, from
    whichever phase, after cleanup completes -- so the exception is never
    silently lost, but the safe return always happens first.
    """
    gravity_comp_enabled = cfg.get("gravity_comp", {}).get("enabled", True)
    dt = cfg["control"]["dt"]
    interp_dt = cfg["control"]["dt"]
    interp_time = cfg["control"]["interpolation_time"]

    _t0 = time.monotonic()
    h_open = HAND_SOFT_LOWER_LIMITS.copy()
    h_grasp = (ctrl_l or ctrl_r).close_pose
    phases = [
        ("open",  h_open, h_open,  1.0),   # Phase A: hold at open after arm reaches grasp
        ("close", h_open, h_grasp, 1.5),   # Phase B: lerp to grasp pose
    ]
    period = 1.0 / max(rate, 1.0)

    current_q = np.zeros(14)
    grasp_arm_q = None
    grasp_tauff = None
    pending_exc = None

    try:
        # ── Phase 0: IK arm to grasp pose ─────────────────────────────────
        print("[grasp_demo] Phase: arm to grasp pose...")
        current_q = _get_q(robot)
        L_cur, R_cur = ik.fk(current_q)
        L_start_pos = L_cur.translation.copy()
        R_start_pos = R_cur.translation.copy()
        L_start_rot = L_cur.rotation.copy()
        R_start_rot = R_cur.rotation.copy()

        box_center     = np.array([L_EE_DEFAULT[0], 0.0, L_EE_DEFAULT[2]])
        box_size       = list(cfg["box"]["size"])
        box_half_width = box_size[1] / 2.0
        approach_cfg    = cfg.get("grasp_approach", {})
        pregrasp_offset = approach_cfg.get("pregrasp_offset", 0.04)
        close_time      = approach_cfg.get("close_time", 1.5)
        tilt_L_deg      = approach_cfg.get("palm_tilt_deg_left", [0.0, 0.0, 0.0])
        tilt_R_deg      = approach_cfg.get("palm_tilt_deg_right", [0.0, 0.0, 0.0])
        collision_avoidance_enabled = cfg.get("collision_avoidance", {}).get("enabled", True)

        # No MarkerSource/marker orientation here (grasp_demo has no camera) --
        # sphere_center is always just box_center, unlike sdk_pose_tracking.py's
        # marker-top-anchored placement.
        head_radius = cfg.get("collision_avoidance", {}).get("head_radius_m", 0.10)
        sphere_center = box_center
        # No marker orientation available here at all -- fall back to world
        # -Z (gravity-down) for any configured head_stack_count spheres, a
        # physically reasonable default (object/head sits on a surface below
        # it) when there's no measured "into the object" axis to use instead.
        sphere_stack_axis = np.array([0.0, 0.0, -1.0])
        ca_cfg = cfg.get("collision_avoidance", {})
        if preview_enabled:
            stack_obstacles = build_head_obstacle_stack(
                sphere_center, head_radius, ca_cfg.get("head_margin_m", 0.0),
                stack_count=ca_cfg.get("head_stack_count", 0),
                stack_spacing_m=ca_cfg.get("head_stack_spacing_m", 0.0),
                axis=sphere_stack_axis,
            )
            stack_centers = [obs.placement.translation for obs in stack_obstacles]
            log_head_sphere(stack_centers, head_radius, ca_cfg.get("head_margin_m", 0.0))
            floor_z = ca_cfg.get("floor_z_m", None)
            if floor_z is not None:
                log_floor_plane(floor_z, ca_cfg.get("floor_margin_m", 0.0),
                                 center_xy=sphere_center[:2])

        # Two-stage approach mirroring sdk_pose_tracking.py's Phase 1a/1b:
        # gross reach to a pre-grasp standoff (RRT*-covered when
        # collision_avoidance is enabled), then a short straight-in close
        # onto the object -- unchanged either way, since the final contact
        # approach is a different, already-bounded problem (see
        # kinematics/rrt_planner.py module docstring).
        L_pre, R_pre = compute_hand_targets(box_center, box_half_width, 0.0, squeeze=-pregrasp_offset)
        L_pre, R_pre = apply_palm_tilt(L_pre, R_pre, tilt_L_deg, tilt_R_deg)
        if preview_enabled:
            # Lets a human directly compare "where is Phase 0a/0b's IK/RRT
            # goal" against the head obstacle sphere (log_head_sphere,
            # logged above) in Rerun.
            log_hand_goal_targets(L_pre, R_pre)

        print(f"[grasp_demo] Phase 0a: approaching pre-grasp standoff "
              f"(+{pregrasp_offset * 100:.1f}cm wider) over {interp_time}s…")
        if collision_avoidance_enabled:
            # solve_ik_refined, not a single solve_ik() call -- see the
            # matching comment in sdk_pose_tracking.py's Phase 1a: a
            # one-shot solve from a warm start far from the target can
            # converge to a locally-optimal solution several cm off, which
            # the RRT planner then faithfully reaches as if it were correct.
            goal_q = ik.solve_ik_refined(L_pre, R_pre, current_q)
            waypoints = plan_collision_free_approach(
                ik, current_q, goal_q, sphere_center, head_radius, cfg,
                axis=sphere_stack_axis,
            )
            print(f"[grasp_demo] RRT* path to standoff: {len(waypoints)} waypoints.")
            current_q = execute_joint_path(
                robot, ik, interp_dt, waypoints, interp_time,
                gravity_comp_enabled, None,
                log_collision_geometry=preview_enabled,
                max_speed_rad_s=cfg.get("collision_avoidance", {}).get("max_path_speed_rad_s", None),
                max_cartesian_speed_m_s=cfg.get("collision_avoidance", {}).get("max_hand_speed_m_s", None),
                arrival_tolerance_rad=cfg.get("collision_avoidance", {}).get("arrival_tolerance_rad", None),
                arrival_timeout_s=cfg.get("collision_avoidance", {}).get("arrival_timeout_s", 2.0),
                arrival_poll_interval_s=cfg.get("collision_avoidance", {}).get("arrival_poll_interval_s", 0.02),
                max_step_rad=cfg.get("collision_avoidance", {}).get("max_step_rad", None),
                log_path=log_joint_path,
                cartesian_arrival_tolerance_m=cfg.get("collision_avoidance", {}).get("cartesian_arrival_tolerance_m", None),
                cartesian_arrival_tolerance_rad=cfg.get("collision_avoidance", {}).get("cartesian_arrival_tolerance_rad", None),
            )
            # execute_joint_path drives the arm via set_joint_targets()
            # directly, never calling solve_ik() -- ik.smooth_filter's queue
            # is therefore frozen at whatever it held before Phase 0a
            # started. Without this reset, Phase 0b's first solve_ik() call
            # blends a correct fresh solution against 3 stale, pre-Phase-0a
            # samples, producing a large spurious corrective jump. See
            # G1HandIK.reset_smoothing's docstring.
            ik.reset_smoothing(current_q)
        else:
            n_steps = int(interp_time / interp_dt)
            for i in range(n_steps):
                alpha = smoothstep(i / n_steps)
                L_pos = (1 - alpha) * L_start_pos + alpha * L_pre[:3, 3]
                R_pos = (1 - alpha) * R_start_pos + alpha * R_pre[:3, 3]
                L_R   = slerp_rotation(L_start_rot, L_pre[:3, :3], alpha)
                R_R   = slerp_rotation(R_start_rot, R_pre[:3, :3], alpha)
                sol_q, _ = ik.solve_ik(
                    make_se3(L_pos, L_R), make_se3(R_pos, R_R),
                    current_q, np.zeros(14),
                )
                robot.arm.set_joint_targets(sol_q, None)
                current_q = sol_q
                time.sleep(interp_dt)

        # --- Phase 0b: straight-in lateral close (squeeze -offset -> 0) ---
        L_cur, R_cur = ik.fk(current_q)
        L_start_pos = L_cur.translation.copy()
        R_start_pos = R_cur.translation.copy()
        L_start_rot = L_cur.rotation.copy()
        R_start_rot = R_cur.rotation.copy()

        print(f"[grasp_demo] Phase 0b: closing straight in along the inter-palm "
              f"axis over {close_time}s…")
        steps_close = close_phase_steps(
            close_time, pregrasp_offset, interp_dt,
            cfg.get("collision_avoidance", {}).get("max_hand_speed_m_s", None),
        )
        sol_tauff = np.zeros(14)
        _last_log_t = time.monotonic() - 1.0  # fire on first step
        for i in range(steps_close):
            alpha = smoothstep(i / steps_close)
            squeeze = (1 - alpha) * (-pregrasp_offset)   # -offset -> 0
            L_target, R_target = compute_hand_targets(
                box_center, box_half_width, 0.0, squeeze=squeeze,
            )
            L_target, R_target = apply_palm_tilt(L_target, R_target, tilt_L_deg, tilt_R_deg)
            sol_q, sol_tauff = ik.solve_ik(L_target, R_target, current_q, np.zeros(14))
            robot.arm.set_joint_targets(sol_q, sol_tauff if gravity_comp_enabled else None)
            current_q = sol_q
            if preview_enabled and time.monotonic() - _last_log_t >= 1.0:
                _last_log_t = time.monotonic()
                log_hand_collision_boxes(ik, current_q)
            time.sleep(interp_dt)

        grasp_arm_q = current_q.copy()
        grasp_tauff = sol_tauff.copy()  # cache gravity comp for hold phases
        print("[grasp_demo] Arm at grasp pose.")

        # ── Phases A + B: open and close with smoothstep interpolation ────
        for label, ha, hb, duration in phases:
            n_steps = max(1, int(duration * rate))
            print(f"[grasp_demo] Phase: {label} ({duration:.1f}s)")
            for s in range(n_steps + 1):
                t = smoothstep(s / n_steps)
                q_hand = lerp_hand(ha, hb, t)
                robot.arm.set_joint_targets(grasp_arm_q, grasp_tauff if gravity_comp_enabled else None)
                robot.hand.set_joint_targets(
                    q_left=q_hand  if do_left  else None,
                    q_right=q_hand if do_right else None,
                )
                if preview_enabled:
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
        print(f"[grasp_demo] Phase: hold ({hold_time:.1f}s)")
        t_start = time.time()
        while time.time() - t_start < hold_time:
            state_l = _get_hand_state(robot, "left")  if ctrl_l else None
            state_r = _get_hand_state(robot, "right") if ctrl_r else None
            q_cmd_l = ctrl_l.step(state_l["force"], state_l["q"], dt) if ctrl_l else None
            q_cmd_r = ctrl_r.step(state_r["force"], state_r["q"], dt) if ctrl_r else None
            robot.arm.set_joint_targets(grasp_arm_q, grasp_tauff if gravity_comp_enabled else None)
            robot.hand.set_joint_targets(q_left=q_cmd_l, q_right=q_cmd_r)
            if preview_enabled:
                _log_hands(
                    time.monotonic() - _t0, "hold", do_left, do_right,
                    state_l, state_r,
                )
            if not dry_run:
                fl = _fmt_force(state_l["force"]) if state_l else "-"
                fr = _fmt_force(state_r["force"]) if state_r else "-"
                print(f"  force  L:{fl}  R:{fr}")
            time.sleep(dt)

        print("[grasp_demo] Grasp/hold sequence done.")

    except (KeyboardInterrupt, Exception) as exc:
        pending_exc = exc
        print(f"[grasp_demo] WARNING: {exc!r} during grasp/hold phases -- "
              f"proceeding to safe hand-release + arm-return before shutdown.")
        if isinstance(exc, RRTPlanningError) and preview_enabled:
            # Same gap as sdk_pose_tracking.py: this logging normally lives
            # inside execute_joint_path, which a StartInCollisionError/
            # GoalInCollisionError means never ran -- show the actual hand
            # boxes at the states involved instead of leaving this a
            # text-only abort.
            log_hand_collision_boxes(ik, current_q)
            if "goal_q" in locals():
                log_hand_collision_boxes(ik, goal_q)

    # ── Phase D: release (lerp back to open) -- ALWAYS attempted ──────────
    # Use last-known-good state, not a fresh hardware read: the fault that
    # got us here may itself be inside a state-read (e.g. GraspController's
    # force feedback).
    hold_arm_q    = grasp_arm_q    if grasp_arm_q    is not None else current_q
    hold_tauff    = grasp_tauff    if grasp_tauff    is not None else np.zeros(14)

    try:
        if ctrl_l:
            ctrl_l.reset()
        if ctrl_r:
            ctrl_r.reset()

        n_steps = max(1, int(1.0 * rate))
        print("[grasp_demo] Phase: release (1.0s)")
        for s in range(n_steps + 1):
            t = smoothstep(s / n_steps)
            q_hand = lerp_hand(h_grasp, h_open, t)
            robot.arm.set_joint_targets(hold_arm_q, hold_tauff if gravity_comp_enabled else None)
            robot.hand.set_joint_targets(
                q_left=q_hand  if do_left  else None,
                q_right=q_hand if do_right else None,
            )
            if preview_enabled:
                _log_hands(
                    time.monotonic() - _t0, "release", do_left, do_right,
                    _get_hand_state(robot, "left")  if do_left  else None,
                    _get_hand_state(robot, "right") if do_right else None,
                )
            time.sleep(period)
    except (KeyboardInterrupt, Exception) as exc:
        print(f"[grasp_demo] WARNING: hand release failed ({exc!r}); hand may not be fully open.")
        if pending_exc is None:
            pending_exc = exc

    # ── Phase R: IK arm return to standing default -- ALWAYS attempted ────
    try:
        print("[grasp_demo] Phase: arm return to default...")
        L_cur, R_cur = ik.fk(hold_arm_q)
        L_start_pos = L_cur.translation.copy()
        R_start_pos = R_cur.translation.copy()
        L_start_rot = L_cur.rotation.copy()
        R_start_rot = R_cur.rotation.copy()

        n_steps = int(interp_time / interp_dt)
        return_q = hold_arm_q
        try:
            for i in range(n_steps):
                alpha = smoothstep(i / n_steps)
                L_pos = (1 - alpha) * L_start_pos + alpha * L_EE_DEFAULT
                R_pos = (1 - alpha) * R_start_pos + alpha * R_EE_DEFAULT
                L_R   = slerp_rotation(L_start_rot, R_L_BASE, alpha)
                R_R   = slerp_rotation(R_start_rot, R_R_BASE, alpha)
                sol_q, sol_tauff = ik.solve_ik(
                    make_se3(L_pos, L_R), make_se3(R_pos, R_R),
                    return_q, np.zeros(14),
                )
                robot.arm.set_joint_targets(sol_q, sol_tauff if gravity_comp_enabled else None)
                return_q = sol_q
                time.sleep(interp_dt)
        except (KeyboardInterrupt, Exception) as exc:
            print(f"[grasp_demo] WARNING: {exc!r} during arm-return motion -- "
                  f"proceeding directly to settle.")
            if pending_exc is None:
                pending_exc = exc

        # Always settle, even if the return motion above was interrupted --
        # otherwise a Ctrl-C mid-return skips the settle pause that's supposed
        # to hold the final pose before the SDK's fixed shutdown-damping kicks in.
        print("[grasp_demo] Trajectory done.")
        settle_time = cfg.get("shutdown", {}).get("settle_time", 0.4)
        print(f"[grasp_demo] Settling for {settle_time:.2f}s before shutdown...")
        time.sleep(settle_time)
    except (KeyboardInterrupt, Exception) as exc:
        print(f"[grasp_demo] WARNING: arm return-to-default failed ({exc!r}); shutting down from current state.")
        if pending_exc is None:
            pending_exc = exc

    if pending_exc is not None:
        raise pending_exc
