"""
Bimanual pose-tracking controller (SDK / Rerun preview path).

Mirrors run_pose_tracking_sim but drives G1InspireRobot instead of CoppeliaSim.
CoppeliaSim path (pose_tracking.py) is unchanged.

Three-phase routine (arms), with a config-driven grip wrapped around Phase 2
(hands, gated by grip_force.enabled / palm_press.enabled):
  1. Interpolate EE targets from default standing pose to grasp pose.
  1.5. Close both hands to the grasp pose. If palm_press.enabled, capture the
       no-contact palm baseline; else (finger-force mode) arm the finger
       force-feedback controllers. Skipped if Phase 1 was interrupted.
  2. Drive both hands via the trajectory source until duration expires or
     KeyboardInterrupt (use None duration for marker-driven interactive mode).
     Grip runs alongside the arm every step, in one of two modes:
       - palm_press.enabled: an outer Cartesian press loop reads palm force and
         drives a press depth `s` that shrinks the hand-target separation (the
         IK moves the wrist/elbow to press the palms together); fingers stay at
         the grasp pose. See controllers/palm_force.py, docs/palm_force_control.md.
       - else (finger-force mode): the per-finger TransportGraspController loop
         nudges the fingers toward a target force.
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
from g1_spine_stabilization.controllers.palm_force import (
    PalmForceReader,
    PalmPressController,
    PalmContactWatchdog,
    PalmForceController,
)
from g1_spine_stabilization.controllers.joint_path_execution import (
    execute_joint_path,
    log_head_sphere,
    log_hand_collision_boxes,
)
from g1_spine_stabilization.kinematics.collision import head_sphere_center_from_marker
from g1_spine_stabilization.kinematics.rrt_planner import (
    plan_collision_free_approach,
    RRTPlanningError,
)
from g1_spine_stabilization.utils.transforms import make_se3, slerp_rotation, smoothstep, Rx
from g1_spine_stabilization.trajectory.targets import (
    compute_hand_targets,
    compute_press_wrenches,
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


def _drive_to_test_pose(robot, ik, control_dt, interpolation_time,
                        gravity_comp_enabled, L_target=None, R_target=None):
    """Actively interpolate the arm from wherever it currently is to the
    given L/R SE3 targets, using the same smoothstep+slerp approach Phase 1a
    uses -- lets a human specify an arbitrary/weird test starting
    configuration (typed, not physically imposed) to verify Phase 1 recovers
    correctly from it, with zero backdriving: the arm is always actively
    driven by its own motors, never pushed against them.

    Either L_target or R_target may be None, meaning "hold that hand at its
    current pose" -- only the other hand moves."""
    if L_target is None and R_target is None:
        return

    current_q = _get_q(robot)
    L_current_pose, R_current_pose = ik.fk(current_q)
    L_start_pos = L_current_pose.translation.copy()
    R_start_pos = R_current_pose.translation.copy()
    L_start_rot = L_current_pose.rotation.copy()
    R_start_rot = R_current_pose.rotation.copy()

    if L_target is None:
        L_target = make_se3(L_start_pos, L_start_rot)
    if R_target is None:
        R_target = make_se3(R_start_pos, R_start_rot)

    steps = max(1, int(interpolation_time / control_dt))
    print(f"[SDKController] Driving to specified test start pose over {interpolation_time}s…")
    for i in range(steps):
        alpha = smoothstep(i / steps)
        L_pos = (1 - alpha) * L_start_pos + alpha * L_target[:3, 3]
        R_pos = (1 - alpha) * R_start_pos + alpha * R_target[:3, 3]
        L_R   = slerp_rotation(L_start_rot, L_target[:3, :3], alpha)
        R_R   = slerp_rotation(R_start_rot, R_target[:3, :3], alpha)

        current_dq = np.zeros(14)
        sol_q, sol_tauff = ik.solve_ik(
            make_se3(L_pos, L_R), make_se3(R_pos, R_R), current_q, current_dq,
        )
        robot.arm.set_joint_targets(sol_q, sol_tauff if gravity_comp_enabled else None)
        current_q = sol_q
        time.sleep(control_dt)
    print("[SDKController] Test start pose reached.")


def _rpy_deg(R):
    """Return [roll, pitch, yaw] in degrees from a 3x3 rotation matrix (intrinsic XYZ)."""
    return np.degrees(_Rot.from_matrix(R).as_euler('xyz'))


def _obs_rotation(obs):
    """3x3 rotation from a MarkerObservation (optical frame preferred, pelvis
    fallback), or None. Used as the torso/target orientation for F/T gravity
    compensation -- the relative rotation the compensator uses is frame-
    invariant, so which of the two frames doesn't matter."""
    if obs is None:
        return None
    pose_opt = getattr(obs, 'pose_in_optical', None)
    if pose_opt is not None:
        return pose_opt[:3, :3]
    return obs.pose_in_pelvis[:3, :3]


def _log_marker_info(source, theta, elapsed, preview, L_target=None, R_target=None):
    """Print augmented per-step marker diagnostics and optionally push to Rerun.

    L_target/R_target (optional): the actual computed compute_hand_targets()
    output for this step, logged as a distinctly-colored Rerun entity
    (targets/L_hand, targets/R_hand) separate from the raw marker triads
    (markers/object, markers/target) and the robot's own rendered mesh --
    lets a human directly compare "where is the IK target" vs. "where is the
    marker" vs. "where does the arm actually end up" in the live Rerun view.

    Also logs markers/applied_shift: source.box_center (+ the tracked theta
    rotation), the corrected/tracked object-center pose compute_hand_targets
    is actually built from -- distinct from the raw object marker's own pose.
    With box_half_height > 0 this should sit visibly below markers/object,
    confirming the marker-sits-on-top correction is applied as intended.
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

    # markers/object and markers/target are logged by MarkerSource itself
    # now (trajectory/sources.py's on_observations hook, fired the instant
    # each tracker.step() call detects them) -- not duplicated here.
    _AXIS_LEN = 0.05
    if _HAS_RERUN and preview is not None:
        box_center = getattr(source, 'box_center', None)
        if box_center is not None:
            _R = Rx(theta)
            rr.log("markers/applied_shift", rr.Arrows3D(
                origins=[box_center, box_center, box_center],
                vectors=[_R[:, 0] * _AXIS_LEN,
                         _R[:, 1] * _AXIS_LEN,
                         _R[:, 2] * _AXIS_LEN],
                colors=[[255, 255, 0, 255],
                        [255, 255, 0, 255],
                        [255, 255, 0, 255]],
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
                          preview=None, cam_for_preview=None, recorder=None,
                          wrench_source=None, test_start_pose=None):
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
        recorder:        optional RunRecorder (data_recording.recorder). If provided,
                         recorder.start() runs at the top of Phase 2, recorder.record(...)
                         runs once per Phase-2 iteration, and recorder.save() is
                         guaranteed to run when Phase 2 ends -- normal duration timeout,
                         KeyboardInterrupt, or any other exception -- before falling
                         through to hand-release/Phase 3. Pass None (default) to disable
                         recording entirely.
        wrench_source:   optional ForceTorqueSource (sensors.force_torque). If provided
                         (only meaningful alongside recorder), capture_reference() runs
                         once at the top of Phase 2 and read() runs each iteration to
                         supply the raw + gravity-cancelled F/T wrench to recorder.record().
                         The caller owns its lifecycle (close() in a finally). None = no
                         force data recorded.
        test_start_pose: optional (L_target, R_target) pair of 4x4 SE3 targets
                         (pelvis frame); either element may itself be None,
                         meaning "hold that hand at its current pose" (only
                         the other one moves). If given, the arm is actively
                         driven there (same smoothstep+slerp approach as
                         Phase 1a) before source.startup() runs -- lets a
                         human specify an arbitrary/weird test starting
                         configuration to verify Phase 1 recovers correctly
                         from it, with no backdriving (the arm is always
                         actively commanded, never manually pushed). Pass
                         None (default) to skip entirely.
    """
    control_dt         = cfg["control"]["dt"]
    interpolation_time = cfg["control"]["interpolation_time"]
    if duration is None:
        duration = cfg["control"]["rotation_duration"]
    gravity_comp_enabled = cfg.get("gravity_comp", {}).get("enabled", True)

    grip_cfg     = cfg.get("grip_force", {})
    grip_enabled = grip_cfg.get("enabled", True)

    palm_press_cfg      = cfg.get("palm_press", {})
    palm_press_enabled  = palm_press_cfg.get("enabled", False)
    palm_press_mode     = palm_press_cfg.get("mode", "position")
    palm_press_jacobian = palm_press_enabled and palm_press_mode == "jacobian"
    seat_squeeze        = float(palm_press_cfg.get("seat_squeeze", 0.0))
    tau_press_max       = float(palm_press_cfg.get("tau_press_max", 8.0))

    # Hands are engaged (fingers close to grasp pose) if EITHER the finger-force
    # grip or the palm-press loop is on. The finger force-feedback loop runs only
    # when grip is on AND palm-press is off: palm-press holds the fingers static
    # and regulates grip force via the wrist/arm press instead (see
    # controllers/palm_force.py, docs/palm_force_control.md).
    use_hands    = grip_enabled or palm_press_enabled
    finger_force = grip_enabled and not palm_press_enabled
    ctrl_l = GraspController(grip_cfg) if use_hands else None
    ctrl_r = GraspController(grip_cfg) if use_hands else None

    # Palm-force press loop: outer Cartesian loop driving a scalar press depth s
    # that shrinks the hand-target separation in compute_hand_targets. Reader is
    # a pure DDS consumer (dry_run -> 0 N, so this whole path no-ops headlessly).
    reader = None
    press_ctrl = None
    watchdog = None
    force_ctrl = None
    if palm_press_enabled:
        reader = PalmForceReader(
            ["left", "right"],
            dry_run=bool(getattr(robot, "dry_run", False)),
            counts_per_newton=palm_press_cfg.get("counts_per_newton", 1.0),
            baseline_frames=palm_press_cfg.get("baseline_frames", 20),
            noise_threshold=palm_press_cfg.get("noise_threshold", 0.0),
            signal=palm_press_cfg.get("signal", "min"),
        )
        # Position-mode press loop (the default / fallback). Always constructed so
        # the position path works and its force_target is available for logging.
        press_ctrl = PalmPressController(
            kappa=palm_press_cfg.get("kappa", 0.0),
            s_max=palm_press_cfg.get("squeeze_max", 0.0),
            force_min=palm_press_cfg.get("force_min", 0.0),
            force_target=palm_press_cfg.get("force_target", 0.0),
            force_max=palm_press_cfg.get("force_max", 0.0),
        )
        # Jacobian-mode force controller (Road 1): output is a press-force
        # magnitude F_cmd turned into torque feedforward tau = J^T W. Only built
        # in jacobian mode; kp is untouched. See docs/palm_force_jacobian.md.
        if palm_press_jacobian:
            force_ctrl = PalmForceController(
                kappa_f=palm_press_cfg.get("kappa_f", 0.0),
                force_target=palm_press_cfg.get("force_target", 0.0),
                force_max=palm_press_cfg.get("force_max", 0.0),
                f_des_ramp_time=palm_press_cfg.get("f_des_ramp_time", 0.0),
                feedforward=palm_press_cfg.get("force_feedforward", True),
            )
        # Contact-loss safety guard: freeze the press (never keep squeezing a
        # dropped/slipped head) once the grip is real and force then falls away.
        # No-op in dry-run (reader = 0 N never arms it). Shared by both modes.
        # See palm_force.py.
        watchdog = PalmContactWatchdog(
            touch_threshold=palm_press_cfg.get("touch_threshold", 0.5),
            loss_floor=palm_press_cfg.get("loss_floor", 0.2),
            loss_frames=palm_press_cfg.get("loss_frames", 3),
        )
        print(f"[SDKController] Palm-press grip: mode={palm_press_mode}"
              + (f", seat={seat_squeeze * 1000:.1f}mm, F_des={press_ctrl.force_target:.2f}N, "
                 f"tau_press_max={tau_press_max:.1f}Nm" if palm_press_jacobian else ""))

    # ---------------------------------------------------------------
    # Optional test start pose — drive the arm to a specified arbitrary
    # configuration BEFORE tracking begins, so Phase 1's recovery from it
    # can be verified. See test_start_pose in the docstring above.
    # ---------------------------------------------------------------
    if test_start_pose is not None:
        L_test_target, R_test_target = test_start_pose
        _drive_to_test_pose(robot, ik, control_dt, interpolation_time,
                            gravity_comp_enabled, L_test_target, R_test_target)

    # ---------------------------------------------------------------
    # Source startup — detect object marker or initialise sine timer
    # ---------------------------------------------------------------
    print("[SDKController] Starting source…")
    box_center = source.startup()
    box_size   = list(cfg["box"]["size"])
    box_half_width = box_size[1] / 2
    print(f"[SDKController] box_center={np.round(box_center, 4)}, half_width={box_half_width:.4f}")

    # target_found is only meaningful for MarkerSource -- SineSource etc. have
    # no such concept and always "track", so default True keeps them unaffected.
    tracking_available = getattr(source, "target_found", True)
    if not tracking_available:
        print("[SDKController] No target marker detected at startup -- will grasp "
              "normally and skip the tracking phase (Phase 2).")

    # ---------------------------------------------------------------
    # RRT* head-obstacle sphere center: top of the sphere sits at the object
    # marker's own raw position (source.object_marker_pose), NOT box_center
    # (which is already box_half_height-shifted for GRASP geometry -- a
    # different, unrelated offset). Falls back to box_center for sources with
    # no marker concept (SineSource, or MockMarkerTracker scenarios with no
    # real marker orientation), which reduces to the old plain-center
    # placement. Logged to Rerun immediately (not gated on
    # collision_avoidance_enabled) so the sphere is visible for comparison
    # even when avoidance itself is off.
    # ---------------------------------------------------------------
    head_radius = cfg.get("collision_avoidance", {}).get("head_radius_m", 0.10)
    marker_pose = getattr(source, "object_marker_pose", None)
    if marker_pose is not None:
        sphere_center = head_sphere_center_from_marker(*marker_pose, head_radius)
    else:
        sphere_center = box_center
    if _HAS_RERUN and preview is not None:
        log_head_sphere(sphere_center, head_radius,
                         cfg.get("collision_avoidance", {}).get("head_margin_m", 0.0))

    # ---------------------------------------------------------------
    # Phase 1: two-stage approach.
    #   1a. Approach to a PRE-GRASP STANDOFF -- the grasp pose but with the palms
    #       held `pregrasp_offset` wider than the object (a negative squeeze).
    #       This absorbs the gross/diagonal gross motion and ends with the hands
    #       correctly positioned and oriented AROUND the object, standing clear.
    #   1b. Close STRAIGHT IN along the inter-palm axis by ramping the squeeze
    #       from -pregrasp_offset to 0. compute_hand_targets translates both palms
    #       purely along their facing normals, so the object seats flat between
    #       them instead of the palms coming in diagonally and snagging.
    # Continuum on one axis: standoff (-offset) -> grasp (0) -> press (+s, Phase 2).
    # ---------------------------------------------------------------
    approach_cfg    = cfg.get("grasp_approach", {})
    pregrasp_offset = approach_cfg.get("pregrasp_offset", 0.04)
    close_time      = approach_cfg.get("close_time", 1.5)

    steps_interp = int(interpolation_time / control_dt)
    steps_close  = max(1, int(close_time / control_dt))

    current_q = _get_q(robot)
    L_current_pose, R_current_pose = ik.fk(current_q)
    L_start_pos = L_current_pose.translation.copy()
    R_start_pos = R_current_pose.translation.copy()
    L_start_rot = L_current_pose.rotation.copy()
    R_start_rot = R_current_pose.rotation.copy()

    L_pre, R_pre = compute_hand_targets(box_center, box_half_width, 0.0, squeeze=-pregrasp_offset)

    _last_log_t = time.monotonic() - 1.0  # fire on first step
    _phase1_interrupted = False
    collision_avoidance_enabled = cfg.get("collision_avoidance", {}).get("enabled", True)
    try:
        # --- Phase 1a: approach to the wide pre-grasp standoff ---
        print(f"[SDKController] Phase 1a: approaching pre-grasp standoff "
              f"(+{pregrasp_offset * 100:.1f}cm wider) over {interpolation_time}s…")

        if collision_avoidance_enabled:
            # RRT* plans ONLY this gross reach, in joint space, treating the
            # head as a static obstacle -- solve_ik() is called once (for the
            # standoff goal), not per sample. See kinematics/rrt_planner.py.
            goal_q, _ = ik.solve_ik(L_pre, R_pre, current_q, np.zeros(14))
            waypoints = plan_collision_free_approach(
                ik, current_q, goal_q, sphere_center, head_radius, cfg,
            )
            print(f"[SDKController] RRT* path: {len(waypoints)} waypoints.")
            current_q = execute_joint_path(
                robot, ik, control_dt, waypoints, interpolation_time,
                gravity_comp_enabled, cam_for_preview,
                log_collision_geometry=(_HAS_RERUN and preview is not None),
            )
            print("  100%")
        else:
            for i in range(steps_interp):
                alpha = smoothstep(i / steps_interp)

                L_pos = (1 - alpha) * L_start_pos + alpha * L_pre[:3, 3]
                R_pos = (1 - alpha) * R_start_pos + alpha * R_pre[:3, 3]
                L_R   = slerp_rotation(L_start_rot, L_pre[:3, :3], alpha)
                R_R   = slerp_rotation(R_start_rot, R_pre[:3, :3], alpha)

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
                    if _HAS_RERUN and preview is not None:
                        log_hand_collision_boxes(ik, current_q)

        # --- Phase 1b: straight-in lateral close (squeeze -offset -> 0) ---
        print(f"[SDKController] Phase 1b: closing straight in along the inter-palm "
              f"axis over {close_time}s…")
        for i in range(steps_close):
            alpha = smoothstep(i / steps_close)
            squeeze = (1 - alpha) * (-pregrasp_offset)   # -offset -> 0

            L_target, R_target = compute_hand_targets(
                box_center, box_half_width, 0.0, squeeze=squeeze,
            )
            current_dq = np.zeros(14)
            sol_q, sol_tauff = ik.solve_ik(L_target, R_target, current_q, current_dq)
            robot.arm.set_joint_targets(sol_q, sol_tauff if gravity_comp_enabled else None)
            current_q = sol_q
            if cam_for_preview is not None:
                cam_for_preview.capture()
            time.sleep(control_dt)

            if time.monotonic() - _last_log_t >= 1.0:
                _last_log_t = time.monotonic()
                print(f"  closing {alpha * 100:.0f}%")
                if _HAS_RERUN and preview is not None:
                    log_hand_collision_boxes(ik, current_q)
    except KeyboardInterrupt:
        _phase1_interrupted = True
        print("\n[SDKController] KeyboardInterrupt — exiting Phase 1 "
              "(skipping Phase 2, returning to default).")
    except RRTPlanningError as exc:
        _phase1_interrupted = True
        print(f"\n[SDKController] SAFETY ABORT: {exc!r} — not falling back to "
              "straight-line motion near the head. Exiting Phase 1 "
              "(skipping Phase 2, returning to default).")
        if _HAS_RERUN and preview is not None:
            # Show the actual hand boxes at both the state the arm was in
            # when planning started and the standoff goal it couldn't reach --
            # this logging never ran before (it lived inside
            # execute_joint_path, which this exception means never executed),
            # so the abort used to be a text message with nothing to look at.
            log_hand_collision_boxes(ik, current_q)
            if "goal_q" in locals():
                log_hand_collision_boxes(ik, goal_q)

    if not _phase1_interrupted:
        print("[SDKController] At grasp pose.")

        # ---------------------------------------------------------------
        # Phase 1.5: Close hands to a light grip
        # ---------------------------------------------------------------
        if use_hands:
            print("[SDKController] Phase 1.5: closing hands to grip pose...")
            try:
                _lerp_hand_pose(robot, control_dt, ctrl_l.open_pose, ctrl_l.close_pose, _HAND_CLOSE_TIME)
                time.sleep(_HAND_SETTLE_TIME)
                if finger_force:
                    state_l = robot.hand.get_state("left")
                    state_r = robot.hand.get_state("right")
                    ctrl_l.arm(np.asarray(state_l["q"], dtype=float))
                    ctrl_r.arm(np.asarray(state_r["q"], dtype=float))
                if palm_press_enabled:
                    # Capture the no-contact palm baseline now: the fingers hold
                    # the object but the palms are still at nominal separation
                    # (squeeze=0), i.e. not yet pressing. See
                    # docs/palm_force_control.md.
                    reader.capture_baseline()
                print("[SDKController] Hands gripped.")
            except KeyboardInterrupt:
                print("\n[SDKController] KeyboardInterrupt — exiting hand-close early.")

        if tracking_available:
            # ---------------------------------------------------------------
            # Phase 2: Track target
            # ---------------------------------------------------------------
            print(f"[SDKController] Phase 2: tracking target (duration={duration}s)…")
            t_start = time.monotonic()
            _last_log_t = t_start - 1.0   # fire on first step
            _obj_lost_warned = False
            _contact_lost_warned = False
            _wrench_ref_captured = False
            if recorder is not None:
                recorder.start()
            try:
                while True:
                    elapsed = time.monotonic() - t_start
                    if elapsed >= duration:
                        break

                    box_center, theta = source.step()
                    if getattr(source, 'object_lost', False) and not _obj_lost_warned:
                        print("[SDKController] WARNING: object marker lost — box may have fallen! (SDK-6: stop here)")
                        _obj_lost_warned = True

                    obj_obs = getattr(source, 'last_obj_obs', None)
                    target_obs = getattr(source, 'last_tgt_obs', None)
                    R_torso = _obs_rotation(target_obs)

                    # F/T sensor: capture the gravity reference on the first step
                    # (now that a target/torso rotation is available), then read
                    # raw + gravity-cancelled wrench each step. No-op if disabled.
                    wrench_raw = wrench_comp = None
                    if wrench_source is not None:
                        if not _wrench_ref_captured:
                            wrench_source.capture_reference(R_torso)
                            _wrench_ref_captured = True
                        wrench_raw, wrench_comp = wrench_source.read(R_torso)

                    if recorder is not None:
                        recorder.record(
                            t=elapsed,
                            object_obs=obj_obs,
                            target_obs=target_obs,
                            theta=theta,
                            load_cell_wrench_raw=wrench_raw,
                            load_cell_wrench_grav_comp=wrench_comp,
                        )

                    # Palm-force grip: pick a press DEPTH (position mode) or a press
                    # WRENCH (jacobian mode). Both share the S0 contact-loss guard. In
                    # dry-run the reader returns 0 N, so nothing arms/presses and the
                    # arm behaves exactly as the pre-palm-press tracking loop.
                    squeeze = seat_squeeze if palm_press_jacobian else 0.0
                    F_cmd = 0.0
                    if palm_press_enabled:
                        F_palm  = reader.target_force()
                        F_guard = reader.guard_force()
                        # Contact-loss safety guard (see palm_force.py). Update first;
                        # a trip FREEZES the press (position: hold last depth; jacobian:
                        # zero the wrench) and holds pose stiffly via kp -- it does NOT
                        # retreat or end the run.
                        watchdog.update(F_palm)
                        frozen = watchdog.press_frozen
                        if frozen and watchdog.tripped and not _contact_lost_warned:
                            print("[SDKController] [SAFETY] palm contact lost — press "
                                  "frozen, holding pose stiffly (head may have dropped/slipped).")
                            _contact_lost_warned = True
                        if palm_press_jacobian:
                            # Hold the fixed seat separation; the grip force comes from
                            # the torque feedforward below, not from a position error.
                            F_cmd = 0.0 if frozen else force_ctrl.step(F_palm)
                        elif frozen:
                            squeeze = press_ctrl.s        # position mode: hold last depth
                        else:
                            squeeze = press_ctrl.step(F_palm, F_guard)

                    L_target, R_target = compute_hand_targets(
                        box_center, box_half_width, theta, squeeze=squeeze,
                    )

                    current_dq = np.zeros(14)
                    sol_q, sol_tauff = ik.solve_ik(L_target, R_target, current_q, current_dq)

                    # Torque feedforward: gravity comp, plus (jacobian mode) the press
                    # wrench mapped to joint torque tau = J_L^T W_L + J_R^T W_R at the
                    # just-solved config. The SDK does NOT clamp tau_ff in software, so
                    # tau_press is saturated here. tau_press is added AFTER solve_ik's
                    # tauff EMA (which only smooths gravity), so the press responds
                    # promptly.
                    tau_ff = sol_tauff.copy() if gravity_comp_enabled else np.zeros(14)
                    tau_press_peak = 0.0
                    if palm_press_jacobian:
                        W_L, W_R = compute_press_wrenches(theta, F_cmd)
                        J_L, J_R = ik.ee_jacobians(sol_q)
                        tau_press = J_L.T @ W_L + J_R.T @ W_R
                        tau_press = np.clip(tau_press, -tau_press_max, tau_press_max)
                        tau_press_peak = float(np.max(np.abs(tau_press)))
                        tau_ff = tau_ff + tau_press
                    send_tauff = tau_ff if (gravity_comp_enabled or palm_press_jacobian) else None
                    robot.arm.set_joint_targets(sol_q, send_tauff)
                    current_q = sol_q

                    if finger_force:
                        state_l = robot.hand.get_state("left")
                        state_r = robot.hand.get_state("right")
                        q_cmd_l = ctrl_l.step(np.asarray(state_l["force"], dtype=float),
                                              np.asarray(state_l["q"], dtype=float), control_dt)
                        q_cmd_r = ctrl_r.step(np.asarray(state_r["force"], dtype=float),
                                              np.asarray(state_r["q"], dtype=float), control_dt)
                        robot.hand.set_joint_targets(q_left=q_cmd_l, q_right=q_cmd_r)
                    # (palm-press mode leaves the fingers at the grasp pose commanded
                    # in Phase 1.5; the SDK's 250 Hz publisher re-sends it for free.)

                    if cam_for_preview is not None:
                        cam_for_preview.capture()
                    time.sleep(control_dt)

                    if time.monotonic() - _last_log_t >= 1.0:
                        _last_log_t = time.monotonic()
                        _log_marker_info(source, theta, elapsed, preview,
                                          L_target=L_target, R_target=R_target)
                        if palm_press_enabled:
                            _wd = ""
                            if watchdog.tripped:
                                _wd = "  [CONTACT LOST — FROZEN]"
                            elif watchdog.press_frozen:
                                _wd = "  [contact dip — press held]"
                            elif not watchdog.armed:
                                _wd = "  [pre-contact]"
                            if palm_press_jacobian:
                                print(f"  Palm force[jacobian]: F_cmd={F_cmd:.2f}N  "
                                      f"F_meas={F_palm:.2f}N  (setpoint={force_ctrl.force_target:.2f}N)  "
                                      f"|tau_press|max={tau_press_peak:.2f}Nm  "
                                      f"seat={squeeze * 1000:.1f}mm{_wd}")
                            else:
                                print(f"  Palm press: s={squeeze * 1000:.1f}mm  "
                                      f"F_target={F_palm:.2f}N  F_guard={F_guard:.2f}N  "
                                      f"(setpoint={press_ctrl.force_target:.2f}N){_wd}")
                            if _HAS_RERUN and preview is not None:
                                if palm_press_jacobian:
                                    rr.log("palm_press/F_cmd_N", rr.Scalars(float(F_cmd)))
                                    rr.log("palm_press/F_meas_N", rr.Scalars(float(F_palm)))
                                    rr.log("palm_press/tau_press_peak_Nm", rr.Scalars(float(tau_press_peak)))
                                else:
                                    rr.log("palm_press/squeeze_mm", rr.Scalars(float(squeeze * 1000)))
                                    rr.log("palm_press/force_target_N", rr.Scalars(float(F_palm)))
                                    rr.log("palm_press/force_guard_N", rr.Scalars(float(F_guard)))
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
            finally:
                # Guaranteed regardless of how Phase 2 exited (normal duration
                # timeout, the KeyboardInterrupt above, or any other exception
                # propagating through) -- save() is a pure pickle write with no
                # hardware calls, so it can't itself raise or need a further
                # guard. Simpler than grasp_sequence.py's pending_exc-reraise
                # pattern, which exists there to guarantee MULTIPLE sequential
                # hardware side effects (release, then return, then settle),
                # each resilient to a further interrupt; here there's exactly
                # one guaranteed side effect.
                if recorder is not None:
                    recorder.save()

        # ---------------------------------------------------------------
        # Guaranteed hand release -- runs whenever grip was engaged,
        # regardless of how Phase 1.5/2 ended, before Phase 3 begins.
        # ---------------------------------------------------------------
        if use_hands:
            print("[SDKController] Releasing hands...")
            try:
                if finger_force:
                    ctrl_l.reset()
                    ctrl_r.reset()
                if palm_press_enabled:
                    press_ctrl.reset()
                    if force_ctrl is not None:
                        force_ctrl.reset()
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

    if reader is not None:
        reader.close()

    settle_time = cfg.get("shutdown", {}).get("settle_time", 0.4)
    print(f"[SDKController] Settling for {settle_time:.2f}s before shutdown...")
    time.sleep(settle_time)
    print("[SDKController] Done.")
