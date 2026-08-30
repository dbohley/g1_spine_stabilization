"""
Executes a sequence of already-valid joint-space waypoints (e.g. from
kinematics.rrt_planner.plan_collision_free_approach) on the real/dry-run
robot.

Unlike the Cartesian smoothstep+slerp approach used elsewhere in this
codebase, waypoints here are already valid joint configs -- no IK re-solve
per step, just direct joint-space interpolation.
"""

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
    preview-enabled state themselves to avoid the (cheap) call entirely."""
    if not _HAS_RERUN:
        return
    rr.log("obstacles/head_true", rr.Ellipsoids3D(
        centers=[center], radii=[float(radius)],
        colors=[[255, 80, 80, 90]], fill_mode="solid",
    ))
    if margin_m > 0:
        rr.log("obstacles/head_padded", rr.Ellipsoids3D(
            centers=[center], radii=[float(radius) + margin_m],
            colors=[[255, 200, 80, 40]], fill_mode="solid",
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


def execute_joint_path(robot, ik, control_dt, waypoints, total_time,
                        gravity_comp_enabled, cam_for_preview=None,
                        log_collision_geometry=False):
    """Smoothstep-blended, per-segment linear joint-space interpolation
    between consecutive waypoints.

    Per-segment time is proportional to that segment's joint-space arc
    length, so overall motion is roughly constant-velocity across the whole
    path rather than giving equal time to a long detour segment and a short
    direct one.

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

    Returns the final commanded q (== waypoints[-1]).
    """
    waypoints = np.asarray(waypoints, dtype=float)
    if len(waypoints) < 2:
        return waypoints[-1].copy() if len(waypoints) else None

    seg_lengths = np.linalg.norm(np.diff(waypoints, axis=0), axis=1)
    total_length = float(seg_lengths.sum())
    if total_length == 0.0:
        seg_times = np.full(len(seg_lengths), total_time / len(seg_lengths))
    else:
        seg_times = seg_lengths / total_length * total_time

    _last_log_t = time.monotonic() - 1.0  # fire on first step

    current_q = waypoints[0].copy()
    for seg_idx in range(len(waypoints) - 1):
        q_from = waypoints[seg_idx]
        q_to = waypoints[seg_idx + 1]
        n_steps = max(1, int(seg_times[seg_idx] / control_dt))
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

            time.sleep(control_dt)

    return current_q
