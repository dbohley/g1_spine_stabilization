#!/usr/bin/env python3
"""Live gravity-compensation debug tool: camera view + torso pose delta + F/T
wrench, side by side, no pickle written.

Isolates one thing out of record_human_trial.py's full loop: track ONLY the
torso (target) marker, show a live annotated video window (like
visualize_markers.py/marker_tracking_debug.py) so tracking can be visually
confirmed, print how far the torso has moved/rotated from its first reading,
and print load-cell readings side by side -- wrench_tared (plain DC-offset +
startup-gravity subtraction, orientation-blind), wrench_grav_comp
(wrench_tared additionally re-projected for the torso's current orientation
via GravityCompensator's rotation-matrix math), and a third, EXPERIMENTAL
sin_model_force (tared force minus a configurable reference_force_n *
sin(theta) fit -- see configs/default.yaml's force_sensor.debug_sin_model and
sensors/force_torque.py's sin_model_gravity_residual) -- each as a single
total force (N) [and total torque (Nm) magnitude for the first two] rather
than per-axis components -- so you can rotate the torso by hand and judge
which correction actually cancels the residual, before trusting a real
recorded trial. sin_model_force is debug-only/unvalidated -- it is NOT used
by the real recording pipeline.

Tracking is read directly per-frame (no MarkerTracker/get_latest() caching):
if the torso marker drops out of view mid-run, the last-known pose keeps
being used for the gravity-comp math (compensation shouldn't stop over a
brief dropout), but the terminal clearly flags STALE with how long it's been
since the last real detection, instead of silently pretending nothing
changed.

Camera + F/T sensor are both passive reads (no robot, no IK, no actuation),
so -- like sensor_test.py -- this is safe to run directly in a Claude Code
session.
"""
import argparse
import os
import sys
import time

_THIS_DIR = os.path.dirname(os.path.abspath(__file__))
_PROJECT_ROOT = os.path.abspath(os.path.join(_THIS_DIR, ".."))
if _PROJECT_ROOT not in sys.path:
    sys.path.insert(0, _PROJECT_ROOT)

import cv2
import numpy as np

from g1_spine_stabilization.perception.detector import AprilTagDetector, CameraIntrinsics
from g1_spine_stabilization.perception.frame_sources import RealSenseSource
from g1_spine_stabilization.perception.frames import (
    optical_to_d435_from_config, d435_to_pelvis_from_pinocchio, compose_to_pelvis,
)
from g1_spine_stabilization.data_recording.recorder import angle_diff_rpy, rotation_magnitude
from g1_spine_stabilization.sensors.force_torque import ForceTorqueSource, sin_model_gravity_residual
from g1_spine_stabilization.utils.config import load_config

_RPY_AXIS_INDEX = {"roll": 0, "pitch": 1, "yaw": 2}
_VALID_AXES = set(_RPY_AXIS_INDEX) | {"magnitude"}


def _force_total(w):
    """Total force magnitude from a (6,) wrench, or None if w is None."""
    return None if w is None else float(np.linalg.norm(w[:3]))


def _fmt_wrench(w, unavailable_reason):
    """Total force/torque magnitude (not per-axis) -- a terser single-number
    summary for by-hand comparison while rotating the torso."""
    if w is None:
        return f"n/a ({unavailable_reason})"
    t_total = np.linalg.norm(w[3:])
    return f"F_total={_force_total(w):.2f} N   T_total={t_total:.3f} Nm"


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", default=None, help="Path to YAML config")
    parser.add_argument(
        "--dry-run", action="store_true",
        help="Synthetic camera frames, no real camera required",
    )
    parser.add_argument(
        "--force-sensor", dest="force_sensor", action="store_true", default=None,
        help="Force-enable the ATI F/T sensor (overrides force_sensor.enabled in config).",
    )
    parser.add_argument(
        "--no-force-sensor", dest="force_sensor", action="store_false",
        help="Disable the F/T sensor for this run (e.g. sensor unplugged).",
    )
    parser.add_argument(
        "--rate", type=float, default=5.0,
        help="Terminal print rate in Hz (default 5). The video window updates every frame regardless.",
    )
    args = parser.parse_args()

    cfg = load_config(args.config)
    perception = cfg["perception"]
    torso_id = perception["target_marker_id"]
    print(f"[gravity_comp_debug] Tracking torso marker (target_marker_id={torso_id} "
          f"per configs/default.yaml's human-neck-trial convention).")

    sin_model_cfg = cfg.get("force_sensor", {}).get("debug_sin_model", {})
    sin_model_ref_force_n = sin_model_cfg.get("reference_force_n", 7.5)
    sin_model_axis = sin_model_cfg.get("axis", "magnitude")
    if sin_model_axis not in _VALID_AXES:
        raise ValueError(
            f"force_sensor.debug_sin_model.axis={sin_model_axis!r} must be one "
            f"of {sorted(_VALID_AXES)}"
        )
    print(f"[gravity_comp_debug] Experimental sin_model_force: "
          f"reference_force_n={sin_model_ref_force_n} N, axis={sin_model_axis} "
          f"(debug-only, not used by the real recording pipeline).")

    source = RealSenseSource(dry_run=args.dry_run)
    native = source.intrinsics_native
    intrinsics = CameraIntrinsics(
        fx=native["fx"], fy=native["fy"], cx=native["cx"], cy=native["cy"],
        distortion=np.asarray(native["distortion"]),
    )
    detector = AprilTagDetector(perception["marker_dict"], perception["marker_size"])
    T_opt_to_d435 = optical_to_d435_from_config(perception["d435_link_to_optical"])
    T_d435_to_pelvis = d435_to_pelvis_from_pinocchio()

    wrench_source = ForceTorqueSource.from_config(cfg, enabled_override=args.force_sensor)
    wrench_source.start()

    pos_ref = None
    R_ref_pelvis = None
    reference_captured = False

    # Last-known torso state, updated only on an ACTUAL per-frame detection --
    # kept separate from "reference" (captured once) so the gravity-comp math
    # can keep running through a brief dropout using the last real reading,
    # while the terminal still clearly flags when that's what's happening.
    last_pose_pelvis = None
    last_R_optical = None
    last_fresh_elapsed = None
    last_reproj_err = None

    print("[gravity_comp_debug] Waiting for first torso marker detection to "
          "capture the position/orientation/gravity reference. Hold the torso "
          "still for ~1s once seen. Ctrl-C / 'q' in the video window to stop.")

    t_start = time.monotonic()
    period = 1.0 / max(args.rate, 0.1)
    _last_print_t = t_start - period
    try:
        while True:
            read = source.read()
            if read is None:
                break
            frame, ts = read
            raw = detector.detect(frame, intrinsics)
            elapsed = time.monotonic() - t_start

            fresh_det = next((d for d in raw if d.marker_id == torso_id), None)
            is_fresh = fresh_det is not None

            for det in raw:
                R = det.pose_in_optical[:3, :3]
                t_vec = det.pose_in_optical[:3, 3]
                rvec, _ = cv2.Rodrigues(R)
                cv2.drawFrameAxes(
                    frame, intrinsics.camera_matrix,
                    np.asarray(intrinsics.distortion), rvec, t_vec,
                    perception["marker_size"] * 0.5,
                )
                label = "TORSO" if det.marker_id == torso_id else f"id={det.marker_id} (unlabeled)"
                cv2.putText(
                    frame, f"{label} err={det.reprojection_error:.2f}px",
                    tuple(det.corners[0].astype(int)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2,
                )
            cv2.imshow("Gravity comp debug", frame)

            if is_fresh:
                last_pose_pelvis = compose_to_pelvis(fresh_det.pose_in_optical, T_opt_to_d435, T_d435_to_pelvis)
                last_R_optical = fresh_det.pose_in_optical[:3, :3].copy()
                last_fresh_elapsed = elapsed
                last_reproj_err = fresh_det.reprojection_error

            if last_pose_pelvis is None:
                if time.monotonic() - _last_print_t >= period:
                    _last_print_t = time.monotonic()
                    print(f"[gravity_comp_debug] t={elapsed:.1f}s  torso NOT SEEN (waiting for first detection)")
                if cv2.waitKey(1) & 0xFF == ord("q"):
                    break
                continue

            if not reference_captured:
                pos_ref = last_pose_pelvis[:3, 3].copy()
                R_ref_pelvis = last_pose_pelvis[:3, :3].copy()
                wrench_source.capture_reference(last_R_optical)
                reference_captured = True
                print(f"[gravity_comp_debug] Reference captured at t={elapsed:.1f}s "
                      f"(pos={pos_ref.round(3).tolist()}).")
                _last_print_t = t_start - period
                if cv2.waitKey(1) & 0xFF == ord("q"):
                    break
                continue

            if time.monotonic() - _last_print_t < period:
                if cv2.waitKey(1) & 0xFF == ord("q"):
                    break
                continue
            _last_print_t = time.monotonic()

            pos_cur = last_pose_pelvis[:3, 3]
            R_cur_pelvis = last_pose_pelvis[:3, :3]
            pos_delta = pos_cur - pos_ref
            orient_delta_rad = angle_diff_rpy(R_ref_pelvis, R_cur_pelvis)
            orient_delta_deg = np.degrees(orient_delta_rad)

            _, wrench_tared, wrench_comp = wrench_source.read_all(last_R_optical)

            tared_f_total = _force_total(wrench_tared)
            sin_model_force = None
            theta_rad = None
            if tared_f_total is not None:
                if sin_model_axis == "magnitude":
                    theta_rad = rotation_magnitude(R_ref_pelvis, R_cur_pelvis)
                else:
                    theta_rad = orient_delta_rad[_RPY_AXIS_INDEX[sin_model_axis]]
                residual = sin_model_gravity_residual(theta_rad, sin_model_ref_force_n)
                sin_model_force = tared_f_total - residual

            staleness = (
                "FRESH" if is_fresh
                else f"STALE (last seen {elapsed - last_fresh_elapsed:.1f}s ago) -- using cached pose"
            )

            print(f"[gravity_comp_debug] t={elapsed:.1f}s  torso tracking: {staleness}")
            print(f"  torso pos_delta=[{pos_delta[0]:+.3f}, {pos_delta[1]:+.3f}, {pos_delta[2]:+.3f}] m")
            print(f"  torso orient_delta (rpy, deg) = "
                  f"[{orient_delta_deg[0]:+.1f}, {orient_delta_deg[1]:+.1f}, {orient_delta_deg[2]:+.1f}]")
            if theta_rad is None:
                print(f"  theta_used({sin_model_axis})   n/a   reproj_err={last_reproj_err:.2f}px")
            else:
                print(f"  theta_used({sin_model_axis})={np.degrees(theta_rad):+.1f} deg   "
                      f"reproj_err={last_reproj_err:.2f}px")
            print(f"  wrench_tared      {_fmt_wrench(wrench_tared, 'no tare reference yet')}  "
                  f"(DC offset + startup gravity only)")
            print(f"  wrench_grav_comp  {_fmt_wrench(wrench_comp, 'no gravity reference yet')}  "
                  f"(+ orientation tracking)")
            if sin_model_force is None:
                print(f"  sin_model_force   n/a (no tare reference yet)  [EXPERIMENTAL]")
            else:
                print(f"  sin_model_force   F_corrected={sin_model_force:+.2f} N   "
                      f"(tared_F_total - {sin_model_ref_force_n}*sin(theta_{sin_model_axis}))  "
                      f"[EXPERIMENTAL]")

            if cv2.waitKey(1) & 0xFF == ord("q"):
                break
    except KeyboardInterrupt:
        print("\n[gravity_comp_debug] KeyboardInterrupt -- stopping.")
    finally:
        wrench_source.close()
        source.close()
        cv2.destroyAllWindows()
    print("[gravity_comp_debug] Done.")


if __name__ == "__main__":
    main()
