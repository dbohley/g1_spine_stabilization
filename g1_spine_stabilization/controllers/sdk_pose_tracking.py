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
  2. If a target marker was found at startup: drive both hands via the
     trajectory source until duration expires or KeyboardInterrupt (use None
     duration for marker-driven interactive mode). Grip runs alongside the arm
     every step, in one of two modes:
       - palm_press.enabled: an outer Cartesian press loop reads palm force and
         drives a press depth `s` that shrinks the hand-target separation (the
         IK moves the wrist/elbow to press the palms together); fingers stay at
         the grasp pose. See controllers/palm_force.py, docs/palm_force_control.md.
       - else (finger-force mode): the per-finger TransportGraspController loop
         nudges the fingers toward a target force.
     Otherwise (no target marker -- e.g. a lone object marker for real-world
     grip-strength testing): hold the grasp indefinitely with the SAME grip
     loop above still running live (so palm-press force keeps converging/
     holding and its contact-loss watchdog stays armed), until a human presses
     SPACE or Ctrl-C. See utils/keyboard.py's SpacebarWatcher.
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
import pinocchio as pin
from scipy.spatial.transform import Rotation as _Rot

try:
    import rerun as rr
    _HAS_RERUN = True
except ImportError:
    _HAS_RERUN = False

from g1_spine_stabilization.controllers.grip_force import GraspController
from g1_spine_stabilization.controllers.palm_force import (
    PalmForceReader,
    WristWrenchForceReader,
    PalmPressController,
    PalmContactWatchdog,
    PalmForceController,
    PressSeatDetector,
)
from g1_spine_stabilization.controllers.joint_path_execution import (
    execute_joint_path,
    close_phase_steps,
    log_head_sphere,
    log_floor_plane,
    log_hand_collision_boxes,
    log_hand_goal_targets,
    log_hand_goal_targets_actual,
)
from g1_spine_stabilization.kinematics.collision import (
    head_sphere_center_from_marker,
    build_head_obstacle_stack,
)
from g1_spine_stabilization.kinematics.rrt_planner import (
    plan_collision_free_approach,
    RRTPlanningError,
)
from g1_spine_stabilization.diagnostics.run_log import RUN_LOG
from g1_spine_stabilization.utils.keyboard import SpacebarWatcher
from g1_spine_stabilization.utils.transforms import make_se3, slerp_rotation, smoothstep, as_rotation
from g1_spine_stabilization.trajectory.targets import (
    compute_hand_targets,
    apply_palm_tilt,
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


def _log_marker_info(source, rotation, elapsed, preview, L_target=None, R_target=None,
                     verbose=False):
    """Print augmented per-step marker diagnostics and optionally push to Rerun.

    L_target/R_target (optional): the actual computed compute_hand_targets()
    output for this step, logged as a distinctly-colored Rerun entity
    (targets/L_hand, targets/R_hand) separate from the raw marker triads
    (markers/object, markers/target) and the robot's own rendered mesh --
    lets a human directly compare "where is the IK target" vs. "where is the
    marker" vs. "where does the arm actually end up" in the live Rerun view.

    Also logs markers/applied_shift: source.box_center (+ the tracked
    rotation), the corrected/tracked object-center pose compute_hand_targets
    is actually built from -- distinct from the raw object marker's own pose.
    With box_half_height > 0 this should sit visibly below markers/object,
    confirming the marker-sits-on-top correction is applied as intended.

    `rotation` (renamed from `theta`) is the tracked rotation fed to
    compute_hand_targets this step -- scalar or (3,3), same convention as
    compute_hand_targets's own theta arg. Since the fix in
    docs/six_dof_tracking_fix.md, it's normally a full (3,3) rotation, so
    the diagnostic below prints full roll/pitch/yaw + total angle instead of
    a single X-axis degree value.
    """
    _R = as_rotation(rotation)
    _rpy = _rpy_deg(_R)
    _angle_deg = math.degrees(np.linalg.norm(_Rot.from_matrix(_R).as_rotvec()))
    print(f"[SDKController] t={elapsed:.1f}s  "
          f"tracked rotation (pelvis frame): rpy=[{_rpy[0]:+.1f}°, {_rpy[1]:+.1f}°, "
          f"{_rpy[2]:+.1f}°]  |angle|={_angle_deg:.1f}°")

    obj_obs = getattr(source, 'last_obj_obs', None)
    tgt_obs = getattr(source, 'last_tgt_obs', None)

    # Raw per-marker pose dump -- verbose only, same rationale as the
    # transform-chain block below: it is a calibration/perception debugging
    # aid, and at ~1Hz it buries the lines that matter during a live run
    # (press SEATED, GP ARMED, safety warnings). The tracked-rotation header
    # above and the Tracking delta below stay unconditional -- those ARE the
    # tracking signal, not raw sensor spam.
    if verbose:
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
    # Printed only under verbose (--verbose); off by default since it is a
    # calibration-debugging aid, not routine run output.
    # Only available when source wraps a real MarkerTracker with
    # get_diagnostic_pose (not SineSource, and not MockMarkerTracker, which
    # has no transform chain to report -- --source mock/sine simply won't
    # print this section).
    tracker = getattr(source, 'tracker', None) if verbose else None
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
                          wrench_source=None, test_start_pose=None,
                          pause_after_test_pose=False,
                          log_joint_path=None, release_watcher=None, verbose=False):
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
        log_joint_path:  optional file path -- if given, Phase 1a's execute_joint_path
                         writes a per-tick CSV log of every RRT*-approach joint command
                         there (see execute_joint_path's log_path docstring). None (default)
                         = no logging, current behavior.
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
        release_watcher: optional object/callable exposing .pressed() -> bool,
                         used ONLY when no target marker was found at startup
                         (see the no-target-marker hold phase above). None
                         (default) constructs a real SpacebarWatcher lazily --
                         only if that branch is actually entered, so the
                         common tracking_available=True path never touches
                         the terminal. Tests inject a fake here instead of
                         needing a real TTY.
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
    # that shrinks the hand-target separation in compute_hand_targets.
    # force_source selects the sensing input -- palm_sensor (PalmForceReader,
    # a pure DDS consumer, dry_run -> 0N) or wrist_wrench (WristWrenchForceReader,
    # derived from measured joint torque -- see controllers/palm_force.py for
    # why: no hardware wrist F/T sensor exists on this robot/SDK). Both expose
    # the same target_force()/guard_force()/capture_baseline()/update()/close()
    # interface, so nothing downstream branches on which one is active.
    reader = None
    press_ctrl = None
    watchdog = None
    force_ctrl = None
    seat_detector = None
    palm_press_hold_mode = palm_press_cfg.get("hold_mode", "seat")

    # GP feedforward arming (see trajectory/sources.py's PredictiveSource).
    # Read here with the other config; consumed at the top of Phase 2.
    _gp_cfg = cfg.get("trajectory", {}).get("gp_predict", {})
    _gp_arm_after_seat = _gp_cfg.get("arm_after_seat", True)
    _gp_arm_timeout = _gp_cfg.get("arm_timeout_s", None)
    if palm_press_enabled:
        force_source = palm_press_cfg.get("force_source", "palm_sensor")
        if force_source == "wrist_wrench":
            reader = WristWrenchForceReader(
                robot, ik, signal=palm_press_cfg.get("signal", "mean"),
            )
        else:
            reader = PalmForceReader(
                ["left", "right"],
                dry_run=bool(getattr(robot, "dry_run", False)),
                counts_per_newton=palm_press_cfg.get("counts_per_newton", 1.0),
                baseline_frames=palm_press_cfg.get("baseline_frames", 20),
                noise_threshold=palm_press_cfg.get("noise_threshold", 0.0),
                signal=palm_press_cfg.get("signal", "mean"),
            )
        # Position-mode press loop (the default / fallback). Always constructed so
        # the position path works and its force_target is available for logging.
        press_ctrl = PalmPressController(
            kappa=palm_press_cfg.get("kappa", 0.0),
            s_max=palm_press_cfg.get("squeeze_max", 0.0),
            force_min=palm_press_cfg.get("force_min", 0.0),
            force_target=palm_press_cfg.get("force_target", 0.0),
            force_max=palm_press_cfg.get("force_max", 0.0),
            # Step-and-settle: advance the press in bounded increments with a
            # pause between them, so each increment is judged against a force
            # reading that reflects the previous one rather than stale data.
            # None (config default) keeps the original continuous law.
            step_increment=palm_press_cfg.get("step_increment", None),
            settle_time=palm_press_cfg.get("settle_time", 0.0),
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
        # Gated by palm_press.watchdog_enabled so the guard can be turned off
        # WITHOUT disabling the whole palm-press loop (palm_press.enabled would
        # kill the press itself too). watchdog_latch=False (the config default)
        # makes a confirmed loss recoverable rather than ending the run's press.
        # See palm_force.py.
        if palm_press_cfg.get("watchdog_enabled", True):
            watchdog = PalmContactWatchdog(
                touch_threshold=palm_press_cfg.get("touch_threshold", 0.5),
                loss_floor=palm_press_cfg.get("loss_floor", 0.2),
                loss_frames=palm_press_cfg.get("loss_frames", 3),
                latch=palm_press_cfg.get("watchdog_latch", False),
            )
        else:
            print("[SDKController] Palm contact-loss watchdog DISABLED "
                  "(palm_press.watchdog_enabled=false) -- the press will not be "
                  "frozen if contact is lost.")
        # Seat-and-freeze: the press loop's job is to ESTABLISH the grip, not to
        # keep regulating force while Phase 2 is trying to track. Because the
        # press depth s feeds compute_hand_targets directly, continuous
        # regulation fights the tracker on the press axis (with noise, under
        # force_source: wrist_wrench). Once seated, s is held fixed and tracking
        # owns the hand targets. hold_mode: continuous restores the old
        # always-regulating behavior for hardware A/B. See palm_force.py.
        if palm_press_hold_mode == "seat":
            seat_detector = PressSeatDetector(
                force_target=press_ctrl.force_target,
                tolerance=palm_press_cfg.get("seat_tolerance", 0.5),
                frames=palm_press_cfg.get("seat_frames", 5),
                # Both hands must individually be loaded before the grip can
                # seat -- the aggregate signal alone cannot distinguish a
                # balanced press from one hand doing all the work.
                per_hand_min=palm_press_cfg.get("seat_per_hand_min", None),
            )
        print(f"[SDKController] Palm-press grip: mode={palm_press_mode}, "
              f"hold_mode={palm_press_hold_mode}"
              + (f" (tol={seat_detector.tolerance:.2f}N x{seat_detector.frames} frames)"
                 if seat_detector is not None else "")
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
        if pause_after_test_pose:
            # The SDK's 250 Hz publisher thread keeps re-publishing the last
            # set_joint_targets() call, so the arm holds here for free.
            print("[SDKController] Holding test start pose. "
                  "Press Enter to continue into tracking (Ctrl-C to abort)...")
            input()

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
              "normally, then hold with the grip loop active until SPACE/Ctrl-C "
              "instead of tracking (Phase 2).")

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
    head_center_offset = cfg.get("collision_avoidance", {}).get("head_center_offset_m", [0.0, 0.0, 0.0])
    head_anchor_distance = cfg.get("collision_avoidance", {}).get("head_anchor_distance_m", None)
    marker_pose = getattr(source, "object_marker_pose", None)
    if marker_pose is not None:
        sphere_center = head_sphere_center_from_marker(
            *marker_pose, head_radius, head_center_offset, head_anchor_distance,
        )
        # Captured ONCE here (not re-read per-tick) since the object's
        # physical orientation doesn't change mid-grasp -- only the tracked
        # rotation for Phase 2 (theta) does, which is separate. Passed to
        # every compute_hand_targets call below so the standoff/grasp arc
        # follows the object's own left-right axis instead of the pelvis's
        # fixed one -- see compute_hand_targets's grasp_frame_R docstring.
        _, grasp_frame_R = marker_pose
        # Direction any configured head_stack_count spheres extend along,
        # past the primary sphere -- the SAME "into the object" depth axis
        # head_sphere_center_from_marker already offsets the primary sphere
        # along, so a stacked sphere continues into the same physical
        # volume (more head/neck/table) rather than an arbitrary direction.
        sphere_stack_axis = -grasp_frame_R[:, 2]
    else:
        sphere_center = box_center
        grasp_frame_R = None
        # No real marker orientation available (SineSource/no camera) --
        # fall back to world -Z (gravity-down), matching grasp_sequence.py's
        # unconditional fallback for the same reason: a physically
        # reasonable default when there's no measured "into the object"
        # axis to use instead.
        sphere_stack_axis = np.array([0.0, 0.0, -1.0])
    ca_cfg = cfg.get("collision_avoidance", {})
    head_stack_count = ca_cfg.get("head_stack_count", 0)
    head_stack_spacing = ca_cfg.get("head_stack_spacing_m", 0.0)
    if _HAS_RERUN and preview is not None:
        # Pure geometry (no Pinocchio model construction) -- cheap to build
        # here just for the sphere centers, separately from the real
        # obstacle stack plan_collision_free_approach builds for planning
        # below (rebuilding is simpler than threading the same list through
        # both call sites, and this is not the hot planning loop).
        stack_obstacles = build_head_obstacle_stack(
            sphere_center, head_radius, ca_cfg.get("head_margin_m", 0.0),
            stack_count=head_stack_count, stack_spacing_m=head_stack_spacing,
            axis=sphere_stack_axis,
        )
        stack_centers = [obs.placement.translation for obs in stack_obstacles]
        log_head_sphere(stack_centers, head_radius, ca_cfg.get("head_margin_m", 0.0))
        floor_z = ca_cfg.get("floor_z_m", None)
        if floor_z is not None:
            log_floor_plane(floor_z, ca_cfg.get("floor_margin_m", 0.0),
                             center_xy=sphere_center[:2])

    # ---------------------------------------------------------------
    # Shared per-tick grip regulation + IK hold. Used by BOTH Phase 2's
    # tracking loop and the no-target-marker hold phase below, so the
    # safety-critical press/crush-guard/watchdog logic exists in exactly one
    # place -- two independently-maintained copies of a crush guard would be
    # a real risk of the two drifting apart. Defined here (a closure, not a
    # free function) so it can read the grip objects/config already
    # constructed above without a long parameter list.
    # ---------------------------------------------------------------
    # Edge-triggered contact-loss logging: prints on each loss->regain
    # transition rather than once per run, so the log shows the real history
    # when the watchdog is non-latching (palm_press.watchdog_latch=false).
    _contact_state = {"tripped": False}
    # Same dict-in-closure pattern for the one-shot "press seated" announcement.
    _seat_state = {"seated": False}

    def _grip_and_hold_tick(tick_box_center, theta, tick_current_q):
        """One tick: palm-press or finger-force regulation, then IK + arm
        (+ finger) command. Returns (new_current_q, diag) where diag carries
        the latest force/press readings and this tick's L/R hand targets, for
        periodic printing/logging by the caller."""
        squeeze = seat_squeeze if palm_press_jacobian else 0.0
        F_cmd = 0.0
        F_palm = F_guard = 0.0
        if palm_press_enabled:
            # Refresh this tick's reading. No-op for PalmForceReader (fed by
            # its own background DDS callback); required for
            # WristWrenchForceReader, which has no background thread and
            # needs this tick's theta/grasp_frame_R to compute anything.
            reader.update(theta, grasp_frame_R)
            F_palm  = reader.target_force()
            F_guard = reader.guard_force()
            # Per-hand readings, kept alongside the aggregate: the combined
            # signal alone cannot tell a balanced grip from a lopsided one
            # (left +6N / right 0N averages to the same value as +3/+3), and
            # the seat gate below must reject the lopsided case. Both reader
            # types expose force(side), so this needs no internals.
            F_left  = reader.force("left")
            F_right = reader.force("right")
            # Contact-loss safety guard (see palm_force.py). Update first;
            # a trip FREEZES the press (position: hold last depth; jacobian:
            # zero the wrench) and holds pose stiffly via kp -- it does NOT
            # retreat or end the run. None when palm_press.watchdog_enabled
            # is false, in which case the press is never frozen at all.
            frozen = False
            if watchdog is not None:
                watchdog.update(F_palm)
                frozen = watchdog.press_frozen
                if watchdog.tripped and not _contact_state["tripped"]:
                    print("[SDKController] [SAFETY] palm contact lost — press "
                          "frozen, holding pose stiffly (head may have dropped/"
                          "slipped). Tracking CONTINUES; Ctrl-C to stop.")
                    _contact_state["tripped"] = True
                elif _contact_state["tripped"] and not watchdog.tripped:
                    print("[SDKController] [SAFETY] palm contact regained — "
                          "press resuming.")
                    _contact_state["tripped"] = False
            # Seat-and-freeze, checked AHEAD of the watchdog freeze branch: a
            # seated press is held regardless of watchdog state, and stays held
            # even if contact is later lost. Losing contact must flag loudly
            # (the [SAFETY] prints above) but must NOT hand the hand targets
            # back to the force loop mid-tracking. None in hold_mode:
            # continuous, where the press regulates for the whole run as before.
            if seat_detector is not None:
                seat_detector.update(F_palm, per_hand=(F_left, F_right))
                if seat_detector.seated and not _seat_state["seated"]:
                    _seat_state["seated"] = True
                    if palm_press_jacobian:
                        print(f"[SDKController] Palm press SEATED at "
                              f"F_cmd={force_ctrl.F_cmd:.2f}N (F={F_palm:.2f}N, "
                              f"L={F_left:.2f}N R={F_right:.2f}N, "
                              f"target={force_ctrl.force_target:.2f}N) — commanded "
                              f"press force now HELD; tracking owns the hand targets.")
                    else:
                        print(f"[SDKController] Palm press SEATED at "
                              f"s={press_ctrl.s * 1000:.1f}mm (F={F_palm:.2f}N, "
                              f"L={F_left:.2f}N R={F_right:.2f}N, "
                              f"target={press_ctrl.force_target:.2f}N) — press depth "
                              f"now HELD; tracking owns the hand targets.")

            seated = seat_detector is not None and seat_detector.seated
            if palm_press_jacobian:
                # Seated: hold the converged commanded wrench (the force-mode
                # equivalent of a frozen depth) instead of re-stepping the loop.
                if frozen:
                    F_cmd = 0.0
                else:
                    F_cmd = force_ctrl.F_cmd if seated else force_ctrl.step(F_palm)
            elif seated or frozen:
                # press_ctrl.s already holds the converged (or last) depth.
                squeeze = press_ctrl.s
            else:
                squeeze = press_ctrl.step(F_palm, F_guard)

        L_target, R_target = compute_hand_targets(
            tick_box_center, box_half_width, theta, squeeze=squeeze, grasp_frame_R=grasp_frame_R,
        )
        L_target, R_target = apply_palm_tilt(L_target, R_target, tilt_L_deg, tilt_R_deg)

        current_dq = np.zeros(14)
        sol_q, sol_tauff = ik.solve_ik(L_target, R_target, tick_current_q, current_dq)

        # Torque feedforward: gravity comp, plus (jacobian mode) the press
        # wrench mapped to joint torque tau = J_L^T W_L + J_R^T W_R at the
        # just-solved config. The SDK does NOT clamp tau_ff in software, so
        # tau_press is saturated here.
        tau_ff = sol_tauff.copy() if gravity_comp_enabled else np.zeros(14)
        tau_press_peak = 0.0
        if palm_press_jacobian:
            W_L, W_R = compute_press_wrenches(theta, F_cmd, grasp_frame_R=grasp_frame_R)
            J_L, J_R = ik.ee_jacobians(sol_q)
            tau_press = J_L.T @ W_L + J_R.T @ W_R
            tau_press = np.clip(tau_press, -tau_press_max, tau_press_max)
            tau_press_peak = float(np.max(np.abs(tau_press)))
            tau_ff = tau_ff + tau_press
        send_tauff = tau_ff if (gravity_comp_enabled or palm_press_jacobian) else None
        robot.arm.set_joint_targets(sol_q, send_tauff)

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

        diag = {
            "squeeze": squeeze, "F_palm": F_palm, "F_guard": F_guard,
            "F_cmd": F_cmd, "tau_press_peak": tau_press_peak,
            "seated": _seat_state["seated"],
            "L_target": L_target, "R_target": R_target, "sol_tauff": sol_tauff,
        }
        return sol_q, diag

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
    tilt_L_deg      = approach_cfg.get("palm_tilt_deg_left", [0.0, 0.0, 0.0])
    tilt_R_deg      = approach_cfg.get("palm_tilt_deg_right", [0.0, 0.0, 0.0])

    steps_interp = int(interpolation_time / control_dt)
    steps_close  = close_phase_steps(
        close_time, pregrasp_offset, control_dt,
        cfg.get("collision_avoidance", {}).get("max_hand_speed_m_s", None),
    )

    current_q = _get_q(robot)
    L_current_pose, R_current_pose = ik.fk(current_q)
    L_start_pos = L_current_pose.translation.copy()
    R_start_pos = R_current_pose.translation.copy()
    L_start_rot = L_current_pose.rotation.copy()
    R_start_rot = R_current_pose.rotation.copy()

    L_pre, R_pre = compute_hand_targets(box_center, box_half_width, 0.0,
                                        squeeze=-pregrasp_offset, grasp_frame_R=grasp_frame_R)
    L_pre, R_pre = apply_palm_tilt(L_pre, R_pre, tilt_L_deg, tilt_R_deg)
    if _HAS_RERUN and preview is not None:
        # Lets a human directly compare "where is Phase 1a/1b's IK/RRT goal"
        # against the head obstacle sphere (log_head_sphere, logged above)
        # in Rerun -- previously invisible during Phase 1a/1b (targets/
        # L_hand/R_hand only existed inside Phase 2's _log_marker_info).
        log_hand_goal_targets(L_pre, R_pre)

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
            # Uses solve_ik_refined (not a single solve_ik() call): a
            # one-shot solve from a warm start far from the target (a large
            # RRT-scale reach, or an extreme start pose) can land in a
            # locally-optimal solution several cm off the real target even
            # though IPOPT reports success -- confirmed via an isolated
            # repro (see G1HandIK.solve_ik_refined's docstring). The RRT
            # planner then faithfully reaches that wrong goal_q, so any
            # error here becomes a real Cartesian gap Phase 1b inherits.
            goal_q = ik.solve_ik_refined(L_pre, R_pre, current_q)

            # Reachability observability ONLY -- no gating/abort. solve_ik()
            # never signals when a target is unreachable (confirmed via an
            # isolated test: a target 1.5m out of reach still reports IPOPT
            # convergence, ipopt_exhaustion_count untouched), and
            # RRTStarPlanner.plan()'s only preconditions on q_goal are
            # collision checks against the head sphere -- it has no idea
            # whether q_goal's FK actually matches L_pre/R_pre, so it will
            # happily plan a perfectly smooth, safe path straight to a
            # silently-wrong pose. This measures and surfaces that gap so it
            # can be observed across different marker poses in Rerun/the
            # diagnostic log, without changing control flow.
            L_goal_actual, R_goal_actual = ik.fk(goal_q)
            L_goal_pos_err = float(np.linalg.norm(L_goal_actual.translation - L_pre[:3, 3]))
            R_goal_pos_err = float(np.linalg.norm(R_goal_actual.translation - R_pre[:3, 3]))
            L_goal_rot_err = float(np.linalg.norm(pin.log3(L_goal_actual.rotation @ L_pre[:3, :3].T)))
            R_goal_rot_err = float(np.linalg.norm(pin.log3(R_goal_actual.rotation @ R_pre[:3, :3].T)))
            goal_pos_err = max(L_goal_pos_err, R_goal_pos_err)
            goal_rot_err = max(L_goal_rot_err, R_goal_rot_err)
            print(f"[SDKController] Pregrasp standoff reachability: "
                  f"pos_err={goal_pos_err * 100:.1f}cm, rot_err={math.degrees(goal_rot_err):.1f}° "
                  f"(worst of L/R)")
            print(f"  L: pos_err={L_goal_pos_err * 100:.1f}cm, rot_err={math.degrees(L_goal_rot_err):.1f}°")
            print(f"  R: pos_err={R_goal_pos_err * 100:.1f}cm, rot_err={math.degrees(R_goal_rot_err):.1f}°")
            if _HAS_RERUN and preview is not None:
                log_hand_goal_targets_actual(L_goal_actual, R_goal_actual)

            # Logged BEFORE the plan call, not after: plan() raises
            # StartInCollisionError/GoalInCollisionError from its own
            # preconditions, so the "plan" record below (which carries the
            # same q_start/q_goal) never executes on an abort -- exactly the
            # run you most need the inputs for. This record is the only
            # capture of the planner's inputs on that path.
            RUN_LOG.log("phase1a", "plan_inputs",
                        q_start=current_q.tolist(), q_goal=goal_q.tolist(),
                        sphere_center=np.asarray(sphere_center).tolist(),
                        sphere_stack_axis=np.asarray(sphere_stack_axis).tolist(),
                        head_radius=float(head_radius),
                        head_margin_m=ca_cfg.get("head_margin_m"),
                        head_stack_count=ca_cfg.get("head_stack_count"),
                        head_stack_spacing_m=ca_cfg.get("head_stack_spacing_m"),
                        floor_z_m=ca_cfg.get("floor_z_m"),
                        floor_margin_m=ca_cfg.get("floor_margin_m"),
                        box_center=np.asarray(box_center).tolist(),
                        goal_pos_err=float(goal_pos_err), goal_rot_err=float(goal_rot_err))
            waypoints = plan_collision_free_approach(
                ik, current_q, goal_q, sphere_center, head_radius, cfg,
                axis=sphere_stack_axis,
            )
            print(f"[SDKController] RRT* path: {len(waypoints)} waypoints.")
            _seg_lengths = np.linalg.norm(np.diff(waypoints, axis=0), axis=1) if len(waypoints) > 1 else np.array([])
            RUN_LOG.log("phase1a", "plan",
                        n_waypoints=len(waypoints),
                        fast_path=(len(waypoints) == 2),
                        waypoints=waypoints.tolist(),
                        seg_lengths_rad=_seg_lengths.tolist(),
                        collision_avoidance_enabled=collision_avoidance_enabled,
                        q_start=current_q.tolist(), q_goal=goal_q.tolist(),
                        goal_pos_err=float(goal_pos_err), goal_rot_err=float(goal_rot_err),
                        L_goal_pos_err=L_goal_pos_err, R_goal_pos_err=R_goal_pos_err,
                        L_goal_rot_err=L_goal_rot_err, R_goal_rot_err=R_goal_rot_err)
            current_q = execute_joint_path(
                robot, ik, control_dt, waypoints, interpolation_time,
                gravity_comp_enabled, cam_for_preview,
                log_collision_geometry=(_HAS_RERUN and preview is not None),
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
            # is therefore frozen at whatever it held before Phase 1a
            # started. Without this reset, Phase 1b's first solve_ik() call
            # blends a correct fresh solution against 3 stale, pre-Phase-1a
            # samples, producing a large spurious corrective jump. See
            # G1HandIK.reset_smoothing's docstring.
            ik.reset_smoothing(current_q)
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

        # Phase transition diagnostic: compare what Phase 1a handed off
        # against a fresh robot.get_state() read taken at the same instant,
        # to directly measure any gap between "what the controller thinks
        # current_q is" and what the SDK's own dry-run clipper (_sim_q) has
        # actually reached -- see diagnostics/run_log.py's module docstring.
        try:
            _fresh_state_q = np.asarray(robot.get_state()["arm"]["q"], dtype=float)
            _gap_norm = float(np.linalg.norm(current_q - _fresh_state_q))
        except Exception:
            _fresh_state_q = None
            _gap_norm = None
        RUN_LOG.log("phase_transition", "1a_to_1b",
                    handoff_q=current_q.tolist(),
                    fresh_state_q=(None if _fresh_state_q is None else _fresh_state_q.tolist()),
                    gap_norm=_gap_norm)

        # --- Phase 1b: straight-in lateral close (squeeze -offset -> 0) ---
        print(f"[SDKController] Phase 1b: closing straight in along the inter-palm "
              f"axis over {close_time}s…")
        _prev_phase1b_q = current_q.copy()
        for i in range(steps_close):
            alpha = smoothstep(i / steps_close)
            squeeze = (1 - alpha) * (-pregrasp_offset)   # -offset -> 0

            L_target, R_target = compute_hand_targets(
                box_center, box_half_width, 0.0, squeeze=squeeze, grasp_frame_R=grasp_frame_R,
            )
            L_target, R_target = apply_palm_tilt(L_target, R_target, tilt_L_deg, tilt_R_deg)
            current_dq = np.zeros(14)
            sol_q, sol_tauff = ik.solve_ik(L_target, R_target, current_q, current_dq)
            robot.arm.set_joint_targets(sol_q, sol_tauff if gravity_comp_enabled else None)
            RUN_LOG.log("phase1b", "tick", i=i, alpha=alpha, squeeze=squeeze,
                        sol_q=sol_q.tolist(), prev_q=_prev_phase1b_q.tolist(),
                        delta_norm=float(np.linalg.norm(sol_q - _prev_phase1b_q)))
            _prev_phase1b_q = sol_q.copy()
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
        RUN_LOG.log("phase1a", "rrt_abort",
                    error_type=type(exc).__name__, error=str(exc),
                    q_at_abort=current_q.tolist(),
                    goal_q=goal_q.tolist() if "goal_q" in locals() else None)
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
            _wrench_ref_captured = False
            # GP feedforward arming (no-op unless the source is a
            # PredictiveSource -- sine/mock and plain marker runs have no
            # arm()). The GP anchors on the marker pose at the instant arm()
            # is called, and its training clips were all anchored at the START
            # of a motion segment, so anchoring must wait until the grip is
            # established and the human is about to move -- NOT at
            # source.startup(), which runs before Phase 1a.
            _gp_arm = getattr(source, "arm", None) if _gp_arm_after_seat else None
            if recorder is not None:
                recorder.start()
            try:
                while True:
                    elapsed = time.monotonic() - t_start
                    if elapsed >= duration:
                        break

                    box_center, theta = source.step()
                    if getattr(source, 'object_lost', False) and not _obj_lost_warned:
                        print("[SDKController] WARNING: object marker lost — box may have "
                              "fallen! Holding last known position; tracking CONTINUES "
                              "(Ctrl-C to stop).")
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
                    # arm behaves exactly as the pre-palm-press tracking loop. Shared
                    # with the no-target-marker hold phase below -- see
                    # _grip_and_hold_tick's docstring.
                    current_q, _diag = _grip_and_hold_tick(box_center, theta, current_q)
                    squeeze, F_palm, F_guard, F_cmd, tau_press_peak, L_target, R_target, sol_tauff = (
                        _diag["squeeze"], _diag["F_palm"], _diag["F_guard"],
                        _diag["F_cmd"], _diag["tau_press_peak"],
                        _diag["L_target"], _diag["R_target"], _diag["sol_tauff"],
                    )

                    # Arm the GP feedforward once the grip is established.
                    #
                    # This runs in the SAME tick seating is detected, so the
                    # GP's anchor (gp.reset(), which captures the marker pose)
                    # is taken at the seat instant -- not a tick later. What
                    # is offset by one tick is only when the lead first
                    # REACHES the IK: source.step() ran earlier in this same
                    # iteration and _grip_and_hold_tick has already consumed
                    # this tick's box_center/theta. That offset is structural
                    # to the tick order (moving this check earlier would read
                    # the PREVIOUS tick's seat state, i.e. arm later;
                    # re-stepping the source would double-step the tracker,
                    # which MarkerSource documents against). It is also moot:
                    # the GP needs two accepted samples at its own ~4.4Hz
                    # before twist() returns anything, so ~0.5s of warmup
                    # follows arming regardless and swallows the ~125ms.
                    #
                    # Seating is the physical signal that the hands are loaded
                    # on the object and the human is about to start moving the
                    # torso -- the motion-segment start the GP's training
                    # clips were all anchored at.
                    if _gp_arm is not None:
                        _timed_out = (_gp_arm_timeout is not None
                                      and elapsed >= _gp_arm_timeout)
                        if _seat_state["seated"] or _timed_out:
                            if _gp_arm():
                                _why = ("press seated" if _seat_state["seated"]
                                        else f"arm_timeout_s={_gp_arm_timeout}s elapsed "
                                             f"without the press seating")
                                print(f"[SDKController] GP feedforward ARMED ({_why}) "
                                      f"— anchored on the target marker's current pose; "
                                      f"start moving the torso now.")

                    if cam_for_preview is not None:
                        cam_for_preview.capture()
                    time.sleep(control_dt)

                    if time.monotonic() - _last_log_t >= 1.0:
                        _last_log_t = time.monotonic()
                        _log_marker_info(source, theta, elapsed, preview,
                                          L_target=L_target, R_target=R_target,
                                          verbose=verbose)
                        # Reachability observability ONLY -- no gating/abort, same
                        # spirit as Phase 1a's own goal_pos_err/goal_rot_err check
                        # above. Now that theta can be a genuine 3-DoF rotation
                        # (docs/six_dof_tracking_fix.md), a commanded orientation
                        # may simply be unreachable under the shoulder-roll self-
                        # collision bounds or the wrist limits, with solve_ik()
                        # silently converging to a poor solution -- this surfaces
                        # that gap so trajectory.tracked_rotation.max_angle_deg can
                        # be tuned against real data instead of guessed.
                        L_act, R_act = ik.fk(current_q)
                        _rot_err = max(
                            float(np.linalg.norm(pin.log3(L_act.rotation @ L_target[:3, :3].T))),
                            float(np.linalg.norm(pin.log3(R_act.rotation @ R_target[:3, :3].T))),
                        )
                        _rot_err_deg = math.degrees(_rot_err)
                        _warn = "  [WARNING: possibly unreachable -- see trajectory.tracked_rotation.max_angle_deg]" if _rot_err_deg > 10.0 else ""
                        print(f"  IK orientation residual: {_rot_err_deg:.1f}°  "
                              f"ipopt_exhaustion_count={ik.ipopt_exhaustion_count}{_warn}")
                        if palm_press_enabled:
                            _wd = ""
                            if watchdog is None:
                                _wd = "  [watchdog off]"
                            elif watchdog.tripped:
                                _wd = "  [CONTACT LOST — press frozen, tracking continues]"
                            elif watchdog.press_frozen:
                                _wd = "  [contact dip — press held]"
                            elif not watchdog.armed:
                                _wd = "  [pre-contact]"
                            # Seat state: is the press still converging, or is
                            # the depth now held with tracking owning the targets?
                            if seat_detector is not None:
                                _wd = ("  [SEATED — depth held]" if _diag["seated"]
                                       else "  [seating…]") + _wd
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
        else:
            # ---------------------------------------------------------------
            # No target marker found: hold the grasp with the grip loop still
            # LIVE (palm-press keeps converging/holding its target force, and
            # the contact-loss watchdog stays armed) until a human presses
            # SPACE or Ctrl-C. Real-world grip-strength testing scenario --
            # only the object marker is needed. See utils/keyboard.py.
            # ---------------------------------------------------------------
            print("[SDKController] Holding grasp — no tracking target. "
                  "Press SPACE to release and shut down (Ctrl-C also works).")
            _owns_watcher = release_watcher is None
            watcher = release_watcher
            try:
                if _owns_watcher:
                    watcher = SpacebarWatcher()
                    watcher.__enter__()
                t_start = time.monotonic()
                _last_log_t = t_start - 1.0   # fire on first step
                while not watcher.pressed():
                    current_q, _diag = _grip_and_hold_tick(box_center, 0.0, current_q)
                    time.sleep(control_dt)

                    if time.monotonic() - _last_log_t >= 1.0:
                        _last_log_t = time.monotonic()
                        held_s = time.monotonic() - t_start
                        if palm_press_enabled:
                            if palm_press_jacobian:
                                print(f"  holding {held_s:.0f}s  F_cmd={_diag['F_cmd']:.2f}N  "
                                      f"F_meas={_diag['F_palm']:.2f}N  "
                                      f"(target={force_ctrl.force_target:.2f}N)  "
                                      f"(SPACE to release)")
                            else:
                                _seated = "  [SEATED]" if _diag["seated"] else ""
                                print(f"  holding {held_s:.0f}s  s={_diag['squeeze'] * 1000:.1f}mm  "
                                      f"F_target={_diag['F_palm']:.2f}N  F_guard={_diag['F_guard']:.2f}N  "
                                      f"(setpoint={press_ctrl.force_target:.2f}N){_seated}  (SPACE to release)")
                        else:
                            print(f"  holding {held_s:.0f}s  (SPACE to release)")
            except KeyboardInterrupt:
                print("\n[SDKController] KeyboardInterrupt — exiting hold, releasing.")
            finally:
                if _owns_watcher:
                    watcher.__exit__(None, None, None)

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
                    if seat_detector is not None:
                        seat_detector.reset()
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
    _prev_phase3_q = current_q.copy()
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
            RUN_LOG.log("phase3", "tick", i=i, alpha=alpha,
                        sol_q=sol_q.tolist(), prev_q=_prev_phase3_q.tolist(),
                        delta_norm=float(np.linalg.norm(sol_q - _prev_phase3_q)))
            _prev_phase3_q = sol_q.copy()
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
