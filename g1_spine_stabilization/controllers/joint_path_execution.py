"""
Executes a sequence of already-valid joint-space waypoints (e.g. from
kinematics.rrt_planner.plan_collision_free_approach) on the real/dry-run
robot.

Unlike the Cartesian smoothstep+slerp approach used elsewhere in this
codebase, waypoints here are already valid joint configs -- no IK re-solve
per step, just direct joint-space interpolation.
"""

import csv
import math
import time

import numpy as np
import pinocchio as pin

try:
    import rerun as rr
    _HAS_RERUN = True
except ImportError:
    _HAS_RERUN = False

from g1_spine_stabilization.utils.transforms import smoothstep
from g1_spine_stabilization.diagnostics.run_log import RUN_LOG

# Cubic smoothstep (3t^2 - 2t^3) has derivative 6t(1-t), peaking at t=0.5
# with value 1.5 -- i.e. the per-tick delta at the MIDDLE of a segment is
# 1.5x the segment's naive average-per-tick delta (length / n_steps), even
# though the average across all n_steps ticks matches that average exactly.
# A per-tick delta cap must be derived against this peak, not the average,
# or the two ticks straddling a segment's midpoint can still exceed the cap.
_SMOOTHSTEP_PEAK_SLOPE = 1.5

# The only two hppfcl.Box-type collision geometries in the reduced model
# (everything else -- shoulders, elbows, wrists, every finger segment -- is a
# mesh-derived BVHModelOBBRSS, out of scope for log_hand_collision_boxes; see
# its docstring). Confirmed by direct enumeration of ik.reduced.collision_model.
_HAND_COLLISION_BOX_NAMES = (
    ("L_hand_hand_base_link_0", "collision_geometry/L_hand"),
    ("R_hand_hand_base_link_0", "collision_geometry/R_hand"),
)


def log_head_sphere(center, radius, margin_m):
    """Log the RRT* head obstacle to Rerun as a translucent sphere, so a
    human watching the dry-run/live preview can see exactly what volume the
    collision-avoiding approach is planning around. Logs both the true
    (exact radius) and padded (+margin) extents when margin_m > 0.
    No-op if rerun isn't installed -- callers should still guard on
    preview-enabled state themselves to avoid the (cheap) call entirely.

    `center` may be a single [x,y,z] (today's original single-sphere usage,
    unchanged) or a list of centers -- e.g. the primary sphere plus any
    configured collision_avoidance.head_stack_count additional spheres
    (see kinematics.collision.build_head_obstacle_stack) -- in which case
    EVERY center gets the same radius/margin_m, logged together as one
    obstacles/head_true (and one obstacles/head_padded) multi-sphere
    Ellipsoids3D entity, so the whole stack renders as a single logical
    group in the viewer rather than needing per-sphere entity paths."""
    if not _HAS_RERUN:
        return
    centers = [center] if np.asarray(center).ndim == 1 else list(center)
    rr.log("obstacles/head_true", rr.Ellipsoids3D(
        centers=centers, radii=[float(radius)] * len(centers),
        colors=[[255, 80, 80, 90]], fill_mode="solid",
    ))
    if margin_m > 0:
        rr.log("obstacles/head_padded", rr.Ellipsoids3D(
            centers=centers, radii=[float(radius) + margin_m] * len(centers),
            colors=[[255, 200, 80, 40]], fill_mode="solid",
        ))


def log_floor_plane(z_min, margin_m, center_xy=(0.0, 0.0), half_extent=1.0):
    """Log the RRT* floor/table obstacle (collision_avoidance.floor_z_m) to
    Rerun as a large, thin, translucent box at z=z_min (pelvis frame) --
    same true (red)/padded (orange) convention as log_head_sphere, so the
    two obstacle types read consistently in the viewer. The actual
    collision check is an infinite half-space (X/Y don't matter to it at
    all) -- `center_xy`/`half_extent` are PURE rendering choices, letting
    the visualized patch sit near the tracked object/workspace (pass
    sphere_center[:2] or box_center[:2]) instead of defaulting to the
    pelvis origin, which would render far from where the arm actually
    works. No preview parameter (matches log_head_sphere's convention) --
    callers gate the call themselves. No-op if rerun isn't installed."""
    if not _HAS_RERUN:
        return
    _THICKNESS = 0.001
    cx, cy = float(center_xy[0]), float(center_xy[1])
    rr.log("obstacles/floor_true", rr.Boxes3D(
        centers=[[cx, cy, float(z_min)]],
        half_sizes=[[half_extent, half_extent, _THICKNESS]],
        colors=[[255, 80, 80, 90]], fill_mode="solid",
    ))
    if margin_m > 0:
        rr.log("obstacles/floor_padded", rr.Boxes3D(
            centers=[[cx, cy, float(z_min) + margin_m]],
            half_sizes=[[half_extent, half_extent, _THICKNESS]],
            colors=[[255, 200, 80, 40]], fill_mode="solid",
        ))


def log_hand_goal_targets(L_target, R_target):
    """Log the Cartesian SE3 targets Phase 1a/1b's IK/RRT are solving
    toward (targets/L_hand, targets/R_hand -- same entity names and
    cyan/magenta convention Phase 2's _log_marker_info already uses for its
    own per-step targets, sdk_pose_tracking.py), so they can be visually
    compared against the head obstacle sphere (log_head_sphere) in Rerun.
    Distinct from log_hand_collision_boxes, which shows the actual
    collision GEOMETRY at a joint config -- this shows the point/frame IK
    is aiming for. No preview parameter (matches log_head_sphere's
    convention) -- callers gate the call themselves. No-op if rerun isn't
    installed."""
    if not _HAS_RERUN:
        return
    _AXIS_LEN = 0.05
    for name, T, color in (
        ("targets/L_hand", L_target, [0, 255, 255, 255]),
        ("targets/R_hand", R_target, [255, 0, 255, 255]),
    ):
        t, R = T[:3, 3], T[:3, :3]
        rr.log(name, rr.Arrows3D(
            origins=[t, t, t],
            vectors=[R[:, 0] * _AXIS_LEN, R[:, 1] * _AXIS_LEN, R[:, 2] * _AXIS_LEN],
            colors=[color, color, color],
        ))


def log_hand_goal_targets_actual(L_actual_pose, R_actual_pose):
    """Log where solve_ik()/solve_ik_refined() ACTUALLY landed
    (targets/L_hand_actual, targets/R_hand_actual, muted gray) -- distinct
    from log_hand_goal_targets's cyan/magenta (where IK was ASKED to go).
    Reachability-observability companion: solve_ik() never signals when a
    target is unreachable (a target 1.5m out of reach still reports IPOPT
    convergence), so this lets a human visually compare "requested" vs.
    "achieved" pose directly in Rerun -- a visible gap between the colored
    and gray arrows means the goal was unreachable, no other change to
    control flow. Takes pin.SE3 poses (as returned by ik.fk()), not raw 4x4
    arrays, since that's the natural type at this call site. No preview
    parameter (matches log_head_sphere's convention) -- callers gate the
    call themselves. No-op if rerun isn't installed."""
    if not _HAS_RERUN:
        return
    _AXIS_LEN = 0.05
    _GRAY = [160, 160, 160, 200]
    for name, pose in (
        ("targets/L_hand_actual", L_actual_pose),
        ("targets/R_hand_actual", R_actual_pose),
    ):
        t, R = pose.translation, pose.rotation
        rr.log(name, rr.Arrows3D(
            origins=[t, t, t],
            vectors=[R[:, 0] * _AXIS_LEN, R[:, 1] * _AXIS_LEN, R[:, 2] * _AXIS_LEN],
            colors=[_GRAY, _GRAY, _GRAY],
        ))


def log_marker_observations(obj_obs, tgt_obs):
    """Log the raw object/target marker triads to Rerun.

    Intended as the body of MarkerSource's on_observations callback
    (trajectory/sources.py) -- called from inside MarkerSource itself the
    instant each tracker.step() call detects something (Phase A/B/C's
    wait/countdown loops AND step()), so markers appear in Rerun immediately
    on detection, independent of whatever the controller does with that
    detection afterward (a Phase 1 collision-avoidance abort, for example,
    must not retroactively un-show a marker that was already detected).
    Deliberately does NOT log targets/L_hand/R_hand (the IK hand targets) --
    those need Phase-2-only data (L_target/R_target); Phase 2 keeps calling
    the fuller _log_marker_info for that. No preview parameter (matches
    log_head_sphere's convention) -- callers gate the call themselves on
    their own preview-enabled state."""
    if not _HAS_RERUN:
        return
    _AXIS_LEN = 0.05
    for name, obs in (("markers/object", obj_obs), ("markers/target", tgt_obs)):
        if obs is None:
            continue
        t = obs.pose_in_pelvis[:3, 3]
        R = obs.pose_in_pelvis[:3, :3]
        rr.log(name, rr.Arrows3D(
            origins=[t, t, t],
            vectors=[R[:, 0] * _AXIS_LEN, R[:, 1] * _AXIS_LEN, R[:, 2] * _AXIS_LEN],
            colors=[[255, 0, 0, 255], [0, 255, 0, 255], [0, 0, 255, 255]],
        ))


def log_hand_collision_boxes(ik, q):
    """Log the two hand-base collision geometries (the ACTUAL boxes the RRT*
    planner checks against the head obstacle, not the rendered hand mesh) to
    Rerun, positioned at the given joint config q.

    These boxes (assets/g1_with_hands.urdf's simplified single-box hand
    collision, [0.035, 0.15, 0.07] at xyz="0 -0.1136 0") are visibly larger
    than the rendered hand mesh -- a collision check against them can fail
    while the visible mesh looks clear, which is otherwise invisible in the
    preview. Every OTHER arm/hand collision geometry (shoulder_yaw, elbow,
    both wrists, all finger segments) is a mesh-derived BVHModelOBBRSS, not a
    simple primitive -- visualizing those needs extracting BVH triangle data
    into rr.Mesh3D, a separate and meaningfully larger task, not attempted
    here. No preview parameter (matches log_head_sphere's convention) --
    callers gate the call themselves. No-op if rerun isn't installed.
    """
    if not _HAS_RERUN:
        return
    gmodel = ik.reduced.collision_model
    gdata = pin.GeometryData(gmodel)
    pin.updateGeometryPlacements(ik.reduced.model, ik.reduced.data, gmodel, gdata,
                                  np.asarray(q, dtype=float))
    for name, entity in _HAND_COLLISION_BOX_NAMES:
        if not gmodel.existGeometryName(name):
            continue
        idx = gmodel.getGeometryId(name)
        shape = gmodel.geometryObjects[idx].geometry
        half_side = getattr(shape, "halfSide", None)
        if half_side is None:
            continue  # not a Box (shouldn't happen for these two names)
        placement = gdata.oMg[idx]
        quat_xyzw = _rotation_matrix_to_quat_xyzw(placement.rotation)
        rr.log(entity, rr.Boxes3D(
            centers=[placement.translation],
            half_sizes=[np.asarray(half_side)],
            quaternions=[quat_xyzw],
            colors=[[80, 160, 255, 90]], fill_mode="solid",
        ))


def _rotation_matrix_to_quat_xyzw(R):
    """3x3 rotation matrix -> [x, y, z, w] quaternion (no scipy dependency
    here, to keep this module's import list minimal)."""
    m00, m01, m02 = R[0]
    m10, m11, m12 = R[1]
    m20, m21, m22 = R[2]
    trace = m00 + m11 + m22
    if trace > 0:
        s = 0.5 / math.sqrt(trace + 1.0)
        w = 0.25 / s
        x = (m21 - m12) * s
        y = (m02 - m20) * s
        z = (m10 - m01) * s
    elif m00 > m11 and m00 > m22:
        s = 2.0 * math.sqrt(1.0 + m00 - m11 - m22)
        w = (m21 - m12) / s
        x = 0.25 * s
        y = (m01 + m10) / s
        z = (m02 + m20) / s
    elif m11 > m22:
        s = 2.0 * math.sqrt(1.0 + m11 - m00 - m22)
        w = (m02 - m20) / s
        x = (m01 + m10) / s
        y = 0.25 * s
        z = (m12 + m21) / s
    else:
        s = 2.0 * math.sqrt(1.0 + m22 - m00 - m11)
        w = (m10 - m01) / s
        x = (m02 + m20) / s
        y = (m12 + m21) / s
        z = 0.25 * s
    return [x, y, z, w]


def close_phase_steps(close_time, pregrasp_offset, control_dt,
                       max_hand_speed_m_s=None):
    """Step count for the Phase 1b/0b straight-in close (sdk_pose_tracking.py
    / grasp_sequence.py -- the final squeeze -pregrasp_offset->0 onto the
    object, immediately after execute_joint_path's RRT*-planned gross reach).

    Mirrors execute_joint_path's max_step_rad floor: that loop is IK-driven
    rather than joint-space-interpolated (the resulting sol_q step size isn't
    known until after solve_ik() runs), but pregrasp_offset IS already the
    exact Cartesian distance each palm travels (compute_hand_targets's
    squeeze parameter, trajectory/targets.py) -- so unlike
    execute_joint_path's Cartesian cap, no ik.fk() call is needed to convert
    a joint-space segment into a Cartesian one; this "segment" is Cartesian
    -native already.

    Root cause this fixes: close_time (config: grasp_approach.close_time,
    default 1.5s) was previously the SOLE determinant of step count,
    completely independent of collision_avoidance.max_hand_speed_m_s -- a
    live-hardware incident confirmed a violent motion at the very end of the
    approach persisted even with that cap configured (down to 0.01 m/s),
    because this phase never read it at all: with defaults
    (pregrasp_offset=0.08, close_time=1.5), the true peak Cartesian speed
    (smoothstep's 1.5x-average peak, same _SMOOTHSTEP_PEAK_SLOPE factor used
    below) is 1.5*0.08/1.5 = 0.08 m/s regardless of any configured cap -- 8x
    over a 0.01 m/s target. NOTE: this bounds Cartesian speed only, not
    joint-space speed -- a joint-space overshoot is still theoretically
    possible if the IK solution moves disproportionately for a small
    Cartesian step (e.g. near a singularity), same best-effort caveat as
    execute_joint_path's own Cartesian cap.
    """
    steps = max(1, int(close_time / control_dt))
    if max_hand_speed_m_s is not None and max_hand_speed_m_s > 0:
        steps = max(steps, int(math.ceil(
            _SMOOTHSTEP_PEAK_SLOPE * pregrasp_offset
            / (max_hand_speed_m_s * control_dt))))
    return steps


def execute_joint_path(robot, ik, control_dt, waypoints, total_time,
                        gravity_comp_enabled, cam_for_preview=None,
                        log_collision_geometry=False, max_speed_rad_s=None,
                        max_cartesian_speed_m_s=None, arrival_tolerance_rad=None,
                        arrival_timeout_s=2.0, arrival_poll_interval_s=0.02,
                        max_step_rad=None, log_path=None,
                        cartesian_arrival_tolerance_m=None,
                        cartesian_arrival_tolerance_rad=None):
    """Smoothstep-blended, per-segment linear joint-space interpolation
    between consecutive waypoints.

    Per-segment time is proportional to that segment's joint-space arc
    length, so overall motion is roughly constant-velocity across the whole
    path rather than giving equal time to a long detour segment and a short
    direct one.

    `max_speed_rad_s`, if given, caps that constant velocity (combined
    14-DOF joint-space vector norm per second -- same convention as
    step_size_rad/rewire_radius_rad, not a true per-joint physical limit).
    `max_cartesian_speed_m_s`, if given, additionally caps Cartesian
    hand-tip speed (worst case of L/R hand, via ik.fk()) -- joint-space
    rad/s alone doesn't bound this, since the same joint-space rate can move
    the hand much faster through space if it comes from large proximal
    joints (shoulder/elbow) rather than distal ones (wrist). Checked
    PER SEGMENT (not just in aggregate like the joint-space cap): since
    per-segment time is allocated proportional to joint-space length, a
    segment with small joint-space length but large Cartesian displacement
    could otherwise still exceed this cap even after an aggregate-only
    extension.

    BOTH caps' required-time derivation is scaled by `_SMOOTHSTEP_PEAK_SLOPE`
    (1.5x): `seg_times[i]/control_dt` (or the cap-derived equivalent) sets
    each segment's AVERAGE per-tick rate, but smoothstep's peak slope, at
    its own midpoint, is 1.5x that average. Confirmed on a live-hardware
    incident: WITH max_speed_rad_s/max_cartesian_speed_m_s both actively
    configured, a dangerous fast/violent motion still occurred, because the
    caps (pre-fix) only bounded the average, silently allowing a true peak
    1.5x higher. Fixed here so the configured cap values are the actual
    peak bound, not the average.

    If EITHER cap is given, `total_time` is IGNORED and replaced entirely by
    whatever the cap(s) require (pure path_length / speed, taking the larger
    of the two caps' requirements) -- not blended/floored against the
    passed-in `total_time`. There is no fixed time budget once a cap exists:
    a path that the cap would let finish faster than `total_time` finishes
    faster, not padded out to fill it. Only when NEITHER cap is configured
    does `total_time` remain the sole determinant of pacing (there's no
    other information to derive it from).

    `arrival_tolerance_rad`, if given, adds a post-playback wait: after the
    final commanded step, poll `robot.get_state()["arm"]["q"]` (the same
    real-state readback `sdk_pose_tracking.py`'s `_get_q` uses) until it's
    within this combined joint-space tolerance of the final waypoint, or
    `arrival_timeout_s` elapses. This exists because NEITHER a fixed
    `total_time` NOR a cap-derived one is a guarantee -- both are still
    predictions of how long the real robot/sim needs, not confirmation it
    actually got there. Disabled by default (`None`) so behavior is
    unchanged unless explicitly configured. On timeout, prints a warning and
    proceeds rather than hard-aborting.

    `cartesian_arrival_tolerance_m` / `cartesian_arrival_tolerance_rad`, if
    given, add a SECOND, independent check to that same poll loop and share
    its `arrival_timeout_s` budget (not a separate sequential wait): once
    `robot.get_state()` is readable, compute `ik.fk(actual_q)` and
    `ik.fk(waypoints[-1])` for both hands, and require position error
    (meters, worst of L/R) under `cartesian_arrival_tolerance_m` AND
    rotation error (`pin.log3(R_actual @ R_goal.T)` norm, radians, worst of
    L/R -- same rotation-error convention `ik_solver.py`'s own IK cost
    function uses) under `cartesian_arrival_tolerance_rad`. Motivation:
    `arrival_tolerance_rad` alone only confirms joint-space closeness,
    which does not guarantee the thing that actually matters before
    Phase 1b's straight-in close begins -- the hand's actual Cartesian
    pose. On a live-hardware run where the real arm can lag behind
    commanded targets, Phase 1b's first `solve_ik()` call warm-starts from
    whatever `current_q` this function returns; a stale Cartesian pose at
    that handoff is a plausible source of a large first-tick jump in
    Phase 1b that a joint-space-only check could miss (small joint-space
    tolerance does not bound Cartesian error uniformly, since the same
    joint-space delta produces different Cartesian displacement depending
    on which joints it's concentrated in -- shoulder/elbow vs. wrist).
    Each of the three checks (joint-space, Cartesian position, Cartesian
    rotation) is independently optional (`None` = "not required, always
    considered satisfied") and the loop breaks only once every configured
    one passes, or the shared timeout elapses -- whichever configured
    check(s) are still unsatisfied at timeout are named in the warning
    print, not just a generic message.

    Gravity comp uses ik.gravity_torque(q) routed through ik.tauff_filter
    (when ik.tauff_filter_enabled) so its EMA state stays warm into whatever
    solve_ik()-driven phase follows this one, instead of resuming from
    several-calls-stale smoothing state.

    log_collision_geometry (throttled to ~1Hz, matching the rest of the
    codebase's Rerun-logging cadence): when True, logs the hand collision
    boxes (log_hand_collision_boxes) at the live commanded q -- this is the
    ONE place in the whole run where the arm is actually moving through the
    RRT*-planned path, i.e. the scenario worth watching most closely. Marker
    visualization is NOT handled here -- it lives in MarkerSource itself
    (trajectory/sources.py's on_observations hook), fired the instant each
    marker is detected, independent of whether this function (or the rest of
    Phase 1) ever runs at all.

    `max_step_rad`, if given, floors each segment's `n_steps` so that no
    single tick ever commands a joint-space delta larger than this many
    radians, REGARDLESS of how `seg_times` (proportional-to-length, then
    time- or cap-derived) sliced up that segment's share of `total_time`.
    This exists because per-segment time being proportional to length does
    NOT guarantee a safe per-tick delta: a segment that happens to receive a
    thin time slice (e.g. because other segments consumed most of a fixed
    `total_time`, or because `kinematics.rrt_planner.shortcut_path` left an
    uneven mix of long/short segments) can otherwise discretize down to
    `n_steps=1` -- one command at q_from, the very next command (one
    control_dt later) already at q_to -- a near-instant full-amplitude jump
    with no filtering to catch it (unlike solve_ik()-driven phases, which
    all route through set_ee_targets's low-pass filter; this function calls
    set_joint_targets directly). The floor is computed against smoothstep's
    PEAK per-tick slope (1.5x the segment's average, at its midpoint -- see
    `_SMOOTHSTEP_PEAK_SLOPE`), not the average, so every tick anywhere in the
    segment (not just on average) stays under `max_step_rad`. If None
    (default), behavior is unchanged from before this floor existed --
    n_steps is bounded only by `seg_times[seg_idx] / control_dt`, which is
    what allowed the collapse described above.

    `log_path`, if given, writes one CSV row per commanded tick (wall
    timestamp, segment index, step index, that segment's n_steps/length/
    time, q_cmd, and the per-tick delta norm) plus a header summarizing
    every segment's length/time/n_steps up front -- diagnostic instrumentation
    for tracing a sudden-motion incident back to the exact waypoint pair and
    tick that produced it. Deliberately independent of Rerun (a plain file
    write), since it must work even when the visualizer is unavailable or
    suspected of misrepresenting real commands.

    Returns the final known joint config -- the real robot/sim readback if
    the arrival check ran and could read state, else the last commanded q
    (== waypoints[-1]).
    """
    waypoints = np.asarray(waypoints, dtype=float)
    if len(waypoints) < 2:
        return waypoints[-1].copy() if len(waypoints) else None

    seg_lengths = np.linalg.norm(np.diff(waypoints, axis=0), axis=1)
    total_length = float(seg_lengths.sum())
    cap_required_time = None  # None = no cap configured -> total_time stays as passed in

    if max_speed_rad_s is not None and max_speed_rad_s > 0 and total_length > 0.0:
        # Scaled by _SMOOTHSTEP_PEAK_SLOPE: seg_times[i]/control_dt sets each
        # segment's AVERAGE per-tick rate, but smoothstep's peak slope (at
        # its midpoint) is 1.5x that average -- without this factor,
        # max_speed_rad_s=0.5 would silently allow a true peak of 0.75 rad/s.
        cap_required_time = _SMOOTHSTEP_PEAK_SLOPE * total_length / max_speed_rad_s

    if max_cartesian_speed_m_s is not None and max_cartesian_speed_m_s > 0 and total_length > 0.0:
        hand_positions = [ik.fk(q) for q in waypoints]  # cheap: framesForwardKinematics, no IPOPT
        cart_lengths = np.array([
            max(np.linalg.norm(hand_positions[i + 1][0].translation - hand_positions[i][0].translation),
                np.linalg.norm(hand_positions[i + 1][1].translation - hand_positions[i][1].translation))
            for i in range(len(waypoints) - 1)
        ])
        nonzero = seg_lengths > 0
        if np.any(nonzero):
            # seg_times[i] scales with total_time; solve for the total_time
            # that keeps EVERY segment's cart_len[i]/seg_times[i] under the
            # cap, not just the aggregate (worst-case segment, worst-case
            # hand) -- scaled by _SMOOTHSTEP_PEAK_SLOPE for the same reason
            # as the joint-space cap above (bound the true peak, not the
            # segment average).
            needed = (_SMOOTHSTEP_PEAK_SLOPE * cart_lengths[nonzero] * total_length
                      / (max_cartesian_speed_m_s * seg_lengths[nonzero]))
            cart_required_time = float(needed.max())
            cap_required_time = (cart_required_time if cap_required_time is None
                                  else max(cap_required_time, cart_required_time))

    if cap_required_time is not None:
        if abs(cap_required_time - total_time) > 1e-9:
            print(f"[execute_joint_path] Duration set by speed cap(s): {cap_required_time:.2f}s "
                  f"(pure distance/speed -- the {total_time:.2f}s default is ignored once a "
                  f"cap is configured).")
        total_time = cap_required_time

    if total_length == 0.0:
        seg_times = np.full(len(seg_lengths), total_time / len(seg_lengths))
    else:
        seg_times = seg_lengths / total_length * total_time

    _last_log_t = time.monotonic() - 1.0  # fire on first step

    log_file = None
    log_writer = None
    if log_path is not None:
        log_file = open(log_path, "w", newline="")
        log_writer = csv.writer(log_file)
        log_writer.writerow(["# seg_idx", "seg_length_rad", "seg_time_s", "n_steps"])
        for i, (length, seg_time) in enumerate(zip(seg_lengths, seg_times)):
            planned_n_steps = max(1, int(seg_time / control_dt))
            if max_step_rad is not None and max_step_rad > 0:
                planned_n_steps = max(planned_n_steps,
                                      int(math.ceil(_SMOOTHSTEP_PEAK_SLOPE * length / max_step_rad)))
            log_writer.writerow([i, float(length), float(seg_time), planned_n_steps])
        log_writer.writerow([])
        log_writer.writerow(["timestamp", "seg_idx", "s", "n_steps", "seg_length_rad",
                              "seg_time_s", "per_tick_delta_norm_rad_s"] +
                             [f"q_cmd_{i}" for i in range(waypoints.shape[1])])

    try:
        current_q = waypoints[0].copy()
        prev_q_cmd = current_q.copy()
        for seg_idx in range(len(waypoints) - 1):
            q_from = waypoints[seg_idx]
            q_to = waypoints[seg_idx + 1]
            n_steps = max(1, int(seg_times[seg_idx] / control_dt))
            if max_step_rad is not None and max_step_rad > 0:
                n_steps = max(n_steps,
                              int(math.ceil(_SMOOTHSTEP_PEAK_SLOPE * seg_lengths[seg_idx] / max_step_rad)))
            for s in range(n_steps + 1):
                t = smoothstep(s / n_steps)
                q_cmd = (1 - t) * q_from + t * q_to

                tau_ff = None
                if gravity_comp_enabled:
                    raw = ik.gravity_torque(q_cmd)
                    if ik.tauff_filter_enabled:
                        ik.tauff_filter.add_data(raw)
                        raw = ik.tauff_filter.filtered_data.copy()
                    tau_ff = raw

                robot.arm.set_joint_targets(q_cmd, tau_ff)
                current_q = q_cmd
                if cam_for_preview is not None:
                    cam_for_preview.capture()

                if log_collision_geometry and time.monotonic() - _last_log_t >= 1.0:
                    _last_log_t = time.monotonic()
                    log_hand_collision_boxes(ik, q_cmd)

                per_tick_delta = float(np.linalg.norm(q_cmd - prev_q_cmd)) / control_dt
                RUN_LOG.log("phase1a", "tick", seg_idx=seg_idx, s=s, n_steps=n_steps,
                            seg_length_rad=float(seg_lengths[seg_idx]),
                            seg_time_s=float(seg_times[seg_idx]),
                            per_tick_delta=per_tick_delta, q_cmd=q_cmd.tolist())

                if log_writer is not None:
                    log_writer.writerow([time.time(), seg_idx, s, n_steps,
                                          float(seg_lengths[seg_idx]), float(seg_times[seg_idx]),
                                          per_tick_delta] + list(q_cmd))

                prev_q_cmd = q_cmd.copy()

                time.sleep(control_dt)
    finally:
        if log_file is not None:
            log_file.close()

    final_q = current_q
    _joint_check = arrival_tolerance_rad is not None and arrival_tolerance_rad > 0
    _cartesian_check = (cartesian_arrival_tolerance_m is not None and cartesian_arrival_tolerance_m > 0
                         and cartesian_arrival_tolerance_rad is not None and cartesian_arrival_tolerance_rad > 0)
    if _joint_check or _cartesian_check:
        target_q = waypoints[-1]
        L_goal = R_goal = None
        if _cartesian_check:
            L_goal, R_goal = ik.fk(target_q)
        t_wait_start = time.monotonic()
        while True:
            try:
                actual_q = np.asarray(robot.get_state()["arm"]["q"], dtype=float)
                final_q = actual_q  # prefer the real reading once we have one
            except Exception:
                break  # can't read real state -- nothing more to verify

            joint_ok = (not _joint_check) or (np.linalg.norm(actual_q - target_q) <= arrival_tolerance_rad)

            cartesian_ok = True
            if _cartesian_check:
                L_actual, R_actual = ik.fk(actual_q)
                pos_err = max(
                    np.linalg.norm(L_actual.translation - L_goal.translation),
                    np.linalg.norm(R_actual.translation - R_goal.translation),
                )
                rot_err = max(
                    np.linalg.norm(pin.log3(L_actual.rotation @ L_goal.rotation.T)),
                    np.linalg.norm(pin.log3(R_actual.rotation @ R_goal.rotation.T)),
                )
                cartesian_ok = (pos_err <= cartesian_arrival_tolerance_m
                                 and rot_err <= cartesian_arrival_tolerance_rad)

            if joint_ok and cartesian_ok:
                break
            if time.monotonic() - t_wait_start >= arrival_timeout_s:
                failed = []
                if not joint_ok:
                    failed.append("joint-space (collision_avoidance.arrival_tolerance_rad)")
                if not cartesian_ok:
                    failed.append("Cartesian (collision_avoidance.cartesian_arrival_tolerance_m/_rad)")
                print(f"[execute_joint_path] WARNING: robot did not converge to the "
                      f"final target within {arrival_timeout_s:.1f}s -- failed check(s): "
                      f"{', '.join(failed)} -- proceeding anyway.")
                break
            time.sleep(arrival_poll_interval_s)

    return final_q
