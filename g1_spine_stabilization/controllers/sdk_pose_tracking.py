"""
Bimanual pose-tracking controller (SDK / Rerun preview path).

Mirrors run_pose_tracking_sim but drives G1InspireRobot instead of CoppeliaSim.
CoppeliaSim path (pose_tracking.py) is unchanged.

Three-phase routine (arms), with a config-driven grip wrapped around Phase 2
(hands, gated by grip_force.enabled -- default True):
  1. Interpolate EE targets from default standing pose to grasp pose.
  1.5. Close both hands to a light grip (grip_force.tau_baseline) and arm
       the force-feedback controller. Skipped if Phase 1 was interrupted.
  2. Drive both hands via the trajectory source until duration expires or
     KeyboardInterrupt (use None duration for marker-driven interactive mode).
     The grip force-feedback loop runs alongside the arm every step.
  Release. Guaranteed hand-release back to open, run whenever grip was
       engaged, regardless of how Phase 1.5/2 ended (normally or via
       KeyboardInterrupt) -- mirrors grasp_sequence.py's guaranteed
       release-before-return discipline (SDK-6-safety), applied to the
       fingers here instead of the arm.
  3. Interpolate EE targets back to default standing pose (hands already
     open by this point).

The caller owns the robot lifecycle: call connect/initialize/start_streaming
before this function, and shutdown in a finally block after.
"""

import math
import time

import numpy as np
from scipy.spatial.transform import Rotation as _Rot

try:
    import rerun as rr
    _HAS_RERUN = True
except ImportError:
    _HAS_RERUN = False

from g1_spine_stabilization.controllers.grip_force import GraspController
from g1_spine_stabilization.utils.transforms import make_se3, slerp_rotation, smoothstep
from g1_spine_stabilization.trajectory.targets import (
    compute_hand_targets,
    R_L_BASE,
    R_R_BASE,
    L_EE_DEFAULT,
    R_EE_DEFAULT,
)

# Hand lerp durations (s) -- match grasp_sequence.py's Phase B/settle/D values
# exactly, for consistency between the two hand-grip call sites.
_HAND_CLOSE_TIME   = 1.5
_HAND_SETTLE_TIME  = 0.5
_HAND_RELEASE_TIME = 1.0


def _lerp_hand_pose(robot, control_dt, q_from, q_to, duration):
    """Smoothstep both hands together from q_from to q_to over duration seconds."""
    n_steps = max(1, int(duration / control_dt))
    for s in range(n_steps + 1):
        t = smoothstep(s / n_steps)
        q_hand = (1 - t) * q_from + t * q_to
        robot.hand.set_joint_targets(q_left=q_hand, q_right=q_hand)
        time.sleep(control_dt)

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


def _rpy_deg(R):
    """Return [roll, pitch, yaw] in degrees from a 3x3 rotation matrix (intrinsic XYZ)."""
    return np.degrees(_Rot.from_matrix(R).as_euler('xyz'))


def _log_marker_info(source, theta, elapsed, preview, L_target=None, R_target=None):
    """Print augmented per-step marker diagnostics and optionally push to Rerun.

    L_target/R_target (optional): the actual computed compute_hand_targets()
    output for this step, logged as a distinctly-colored Rerun entity
    (targets/L_hand, targets/R_hand) separate from the raw marker triads
    (markers/object, markers/target) and the robot's own rendered mesh --
    lets a human directly compare "where is the IK target" vs. "where is the
    marker" vs. "where does the arm actually end up" in the live Rerun view.
    """
    print(f"[SDKController] t={elapsed:.1f}s  "
          f"theta_Rx (target marker rot about robot X axis) = {math.degrees(theta):+.1f}°")

    obj_obs = getattr(source, 'last_obj_obs', None)
    tgt_obs = getattr(source, 'last_tgt_obs', None)

    if obj_obs is not None:
        p = obj_obs.pose_in_pelvis[:3, 3]
        rpy = _rpy_deg(obj_obs.pose_in_pelvis[:3, :3])
        print(f"  Object marker:  pos=[{p[0]:+.3f}, {p[1]:+.3f}, {p[2]:+.3f}]  "
              f"rpy=[{rpy[0]:+.1f}°, {rpy[1]:+.1f}°, {rpy[2]:+.1f}°]")
    else:
        print("  Object marker:  (no observation this step)")

    if tgt_obs is not None:
        p = tgt_obs.pose_in_pelvis[:3, 3]
        rpy = _rpy_deg(tgt_obs.pose_in_pelvis[:3, :3])
        print(f"  Target marker:  pos=[{p[0]:+.3f}, {p[1]:+.3f}, {p[2]:+.3f}]  "
              f"rpy=[{rpy[0]:+.1f}°, {rpy[1]:+.1f}°, {rpy[2]:+.1f}°]")
    else:
        print("  Target marker:  (no observation this step)")

    if obj_obs is not None and tgt_obs is not None:
        R_obj = obj_obs.pose_in_pelvis[:3, :3]
        R_tgt = tgt_obs.pose_in_pelvis[:3, :3]
        R_diff = R_obj.T @ R_tgt   # identity = perfect tracking
        r_diff = _Rot.from_matrix(R_diff)
        q = r_diff.as_quat()       # [x, y, z, w]
        angle_deg = np.degrees(np.linalg.norm(r_diff.as_rotvec()))
        print(f"  Tracking delta: q=[x={q[0]:+.3f}, y={q[1]:+.3f}, "
              f"z={q[2]:+.3f}, w={q[3]:+.3f}]  angle={angle_deg:.1f}°")

    # Transform-chain diagnostic -- helps isolate WHICH stage (raw solvePnP
    # optical-frame reading, d435-mount frame, or final pelvis frame) is
    # responsible for a real-world calibration discrepancy (e.g. a marker
    # appearing at a different height in Rerun than its physical position).
    # Only available when source wraps a real MarkerTracker with
    # get_diagnostic_pose (not SineSource, and not MockMarkerTracker, which
    # has no transform chain to report -- --source mock/sine simply won't
    # print this section).
    tracker = getattr(source, 'tracker', None)
    get_diagnostic_pose = getattr(tracker, 'get_diagnostic_pose', None)
    if get_diagnostic_pose is not None:
        obj_id = getattr(source, 'obj_id', None)
        tgt_id = getattr(source, 'tgt_id', None)
        for _label, _mid in [("Object", obj_id), ("Target", tgt_id)]:
            diag = get_diagnostic_pose(_mid) if _mid is not None else None
            if diag is not None:
                p_opt = diag["optical"][:3, 3]
                p_mnt = diag["d435_mount"][:3, 3]
                p_pel = diag["pelvis"][:3, 3]
                print(f"  {_label} marker transform chain:")
                print(f"    optical:    pos=[{p_opt[0]:+.3f}, {p_opt[1]:+.3f}, {p_opt[2]:+.3f}]")
                print(f"    d435 mount: pos=[{p_mnt[0]:+.3f}, {p_mnt[1]:+.3f}, {p_mnt[2]:+.3f}]")
                print(f"    pelvis:     pos=[{p_pel[0]:+.3f}, {p_pel[1]:+.3f}, {p_pel[2]:+.3f}]")

    _AXIS_LEN = 0.05
    if _HAS_RERUN and preview is not None:
        for _name, _obs in [("markers/object", obj_obs), ("markers/target", tgt_obs)]:
            if _obs is not None:
                _t = _obs.pose_in_pelvis[:3, 3]
                _R = _obs.pose_in_pelvis[:3, :3]
                rr.log(_name, rr.Arrows3D(
                    origins=[_t, _t, _t],
                    vectors=[_R[:, 0] * _AXIS_LEN,
                             _R[:, 1] * _AXIS_LEN,
                             _R[:, 2] * _AXIS_LEN],
                    colors=[[255, 0, 0, 255],
                            [0, 255, 0, 255],
                            [0, 0, 255, 255]],
                ))

        for _name, _T, _color in [
            ("targets/L_hand", L_target, [0, 255, 255, 255]),   # cyan
            ("targets/R_hand", R_target, [255, 0, 255, 255]),   # magenta
        ]:
            if _T is not None:
                _t = _T[:3, 3]
                _R = _T[:3, :3]
                rr.log(_name, rr.Arrows3D(
                    origins=[_t, _t, _t],
                    vectors=[_R[:, 0] * _AXIS_LEN,
                             _R[:, 1] * _AXIS_LEN,
                             _R[:, 2] * _AXIS_LEN],
                    colors=[_color, _color, _color],
                ))


def run_pose_tracking_sdk(robot, ik, cfg, source, duration=None,
                          preview=None, cam_for_preview=None):
    """Run the three-phase pose-tracking controller via the SDK transport.

    Args:
        robot:           G1InspireRobot instance (connect/initialize/start_streaming
                         already called by the caller).
        ik:              G1HandIK instance (already loaded).
        cfg:             config dict loaded from default.yaml.
        source:          trajectory source with .startup() and .step() -> (box_center, theta).
                         Use SineSource for sine-wave mode, MarkerSource for live AprilTag mode.
        duration:        Phase 2 duration in seconds. None = run until KeyboardInterrupt
                         (marker-driven interactive mode).
        preview:         RerunPreview instance; enables Rerun marker-frame logging.
        cam_for_preview: RealSenseCamera(dry_run=True) pre-attached to preview; capture()
                         is called every step to push synthetic frames to the Rerun camera pane.
                         Pass None for marker mode (frames arrive via the tracker).
    """
    control_dt         = cfg["control"]["dt"]
    interpolation_time = cfg["control"]["interpolation_time"]
    if duration is None:
        duration = cfg["control"]["rotation_duration"]
    gravity_comp_enabled = cfg.get("gravity_comp", {}).get("enabled", True)

    grip_cfg     = cfg.get("grip_force", {})
    grip_enabled = grip_cfg.get("enabled", True)
    ctrl_l = GraspController(grip_cfg) if grip_enabled else None
    ctrl_r = GraspController(grip_cfg) if grip_enabled else None

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
    L_current_pose, R_current_pose = ik.fk(current_q)
    L_start_pos = L_current_pose.translation.copy()
    R_start_pos = R_current_pose.translation.copy()
    L_start_rot = L_current_pose.rotation.copy()
    R_start_rot = R_current_pose.rotation.copy()

    _last_log_t = time.monotonic() - 1.0  # fire on first step
    _phase1_interrupted = False
    try:
        for i in range(steps_interp):
            alpha = smoothstep(i / steps_interp)

            L_pos = (1 - alpha) * L_start_pos + alpha * L_grasp[:3, 3]
            R_pos = (1 - alpha) * R_start_pos + alpha * R_grasp[:3, 3]
            L_R   = slerp_rotation(L_start_rot, L_grasp[:3, :3], alpha)
            R_R   = slerp_rotation(R_start_rot, R_grasp[:3, :3], alpha)

            current_dq = np.zeros(14)
            sol_q, sol_tauff = ik.solve_ik(
                make_se3(L_pos, L_R),
                make_se3(R_pos, R_R),
                current_q, current_dq,
            )
            robot.arm.set_joint_targets(sol_q, sol_tauff if gravity_comp_enabled else None)
            current_q = sol_q
            if cam_for_preview is not None:
                cam_for_preview.capture()
            time.sleep(control_dt)

            if time.monotonic() - _last_log_t >= 1.0:
                _last_log_t = time.monotonic()
                print(f"  {alpha * 100:.0f}%")
    except KeyboardInterrupt:
        _phase1_interrupted = True
        print("\n[SDKController] KeyboardInterrupt — exiting Phase 1 "
              "(skipping Phase 2, returning to default).")

    if not _phase1_interrupted:
        print("[SDKController] At grasp pose.")

        # ---------------------------------------------------------------
        # Phase 1.5: Close hands to a light grip
        # ---------------------------------------------------------------
        if grip_enabled:
            print("[SDKController] Phase 1.5: closing hands to grip pose...")
            try:
                _lerp_hand_pose(robot, control_dt, ctrl_l.open_pose, ctrl_l.close_pose, _HAND_CLOSE_TIME)
                time.sleep(_HAND_SETTLE_TIME)
                state_l = robot.hand.get_state("left")
                state_r = robot.hand.get_state("right")
                ctrl_l.arm(np.asarray(state_l["q"], dtype=float))
                ctrl_r.arm(np.asarray(state_r["q"], dtype=float))
                print("[SDKController] Hands gripped.")
            except KeyboardInterrupt:
                print("\n[SDKController] KeyboardInterrupt — exiting hand-close early.")

        # ---------------------------------------------------------------
        # Phase 2: Track target
        # ---------------------------------------------------------------
        print(f"[SDKController] Phase 2: tracking target (duration={duration}s)…")
        t_start = time.monotonic()
        _last_log_t = t_start - 1.0   # fire on first step
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
                sol_q, sol_tauff = ik.solve_ik(L_target, R_target, current_q, current_dq)
                robot.arm.set_joint_targets(sol_q, sol_tauff if gravity_comp_enabled else None)
                current_q = sol_q

                if grip_enabled:
                    state_l = robot.hand.get_state("left")
                    state_r = robot.hand.get_state("right")
                    q_cmd_l = ctrl_l.step(np.asarray(state_l["force"], dtype=float),
                                          np.asarray(state_l["q"], dtype=float), control_dt)
                    q_cmd_r = ctrl_r.step(np.asarray(state_r["force"], dtype=float),
                                          np.asarray(state_r["q"], dtype=float), control_dt)
                    robot.hand.set_joint_targets(q_left=q_cmd_l, q_right=q_cmd_r)

                if cam_for_preview is not None:
                    cam_for_preview.capture()
                time.sleep(control_dt)

                if time.monotonic() - _last_log_t >= 1.0:
                    _last_log_t = time.monotonic()
                    _log_marker_info(source, theta, elapsed, preview,
                                      L_target=L_target, R_target=R_target)
                    if _HAS_RERUN and gravity_comp_enabled:
                        _TAUFF_JOINT_NAMES = [
                            "L_shoulder_pitch", "L_shoulder_roll", "L_shoulder_yaw", "L_elbow",
                            "L_wrist_roll", "L_wrist_pitch", "L_wrist_yaw",
                            "R_shoulder_pitch", "R_shoulder_roll", "R_shoulder_yaw", "R_elbow",
                            "R_wrist_roll", "R_wrist_pitch", "R_wrist_yaw",
                        ]
                        for _i, _n in enumerate(_TAUFF_JOINT_NAMES):
                            rr.log(f"tauff/{_n}", rr.Scalars(float(sol_tauff[_i])))

        except KeyboardInterrupt:
            print("\n[SDKController] KeyboardInterrupt — exiting Phase 2.")

        # ---------------------------------------------------------------
        # Guaranteed hand release -- runs whenever grip was engaged,
        # regardless of how Phase 1.5/2 ended, before Phase 3 begins.
        # ---------------------------------------------------------------
        if grip_enabled:
            print("[SDKController] Releasing hands...")
            try:
                ctrl_l.reset()
                ctrl_r.reset()
                _lerp_hand_pose(robot, control_dt, ctrl_l.close_pose, ctrl_l.open_pose, _HAND_RELEASE_TIME)
                print("[SDKController] Hands released.")
            except KeyboardInterrupt:
                print("\n[SDKController] KeyboardInterrupt during hand release — proceeding to Phase 3.")

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
    _last_log_t = time.monotonic() - 1.0   # fire on first step
    try:
        for i in range(steps_return):
            alpha = smoothstep(i / steps_return)

            L_pos = (1 - alpha) * L_start_pos + alpha * L_EE_DEFAULT
            R_pos = (1 - alpha) * R_start_pos + alpha * R_EE_DEFAULT
            L_R   = slerp_rotation(L_start_rot, R_L_BASE, alpha)
            R_R   = slerp_rotation(R_start_rot, R_R_BASE, alpha)

            current_dq = np.zeros(14)
            sol_q, sol_tauff = ik.solve_ik(
                make_se3(L_pos, L_R),
                make_se3(R_pos, R_R),
                current_q, current_dq,
            )
            robot.arm.set_joint_targets(sol_q, sol_tauff if gravity_comp_enabled else None)
            current_q = sol_q
            if cam_for_preview is not None:
                cam_for_preview.capture()
            time.sleep(control_dt)

            if time.monotonic() - _last_log_t >= 1.0:
                _last_log_t = time.monotonic()
                print(f"  {alpha * 100:.0f}%")
    except KeyboardInterrupt:
        print("\n[SDKController] KeyboardInterrupt — exiting Phase 3 early, proceeding to settle/shutdown.")

    settle_time = cfg.get("shutdown", {}).get("settle_time", 0.4)
    print(f"[SDKController] Settling for {settle_time:.2f}s before shutdown...")
    time.sleep(settle_time)
    print("[SDKController] Done.")
