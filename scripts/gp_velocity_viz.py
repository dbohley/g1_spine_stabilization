#!/usr/bin/env python3
"""Visualize the torso GP's predicted velocity against live marker motion.

NO ROBOT. Camera + Rerun only. The control loop is faked with time.sleep()
at the configured control rate, so this shows what the predictor would hand
the servo at our real loop frequency -- without commanding anything.

    python scripts/gp_velocity_viz.py            # live camera
    python scripts/gp_velocity_viz.py --replay data/runs/pelvis_marker_*.pkl

Press Enter to start (then move the torso marker), Ctrl-C to stop.

What you are looking at, in Rerun:
  gp/marker            measured marker pose (RGB axes)
  gp/velocity_measured GRAY arrow  -- finite-difference velocity of the marker
  gp/velocity_gp       CYAN arrow  -- GP-predicted linear velocity
  gp/predicted_pose    CYAN axes   -- where the GP says the marker will be
                                      one GP-step ahead
  plots/...            scalar time series: |v| measured vs GP, |omega|,
                                           and the GP's own sigma

Arrows are drawn at ARROW_SCALE seconds of travel (a 0.1 m/s velocity with
ARROW_SCALE=1.0 draws a 10 cm arrow), so measured and predicted are directly
comparable by eye.
"""
import argparse
import glob
import os
import pickle
import sys
import time

_THIS_DIR = os.path.dirname(os.path.abspath(__file__))
_PROJECT_ROOT = os.path.abspath(os.path.join(_THIS_DIR, ".."))
if _PROJECT_ROOT not in sys.path:
    sys.path.insert(0, _PROJECT_ROOT)
_GP_EXPORT = os.path.join(_PROJECT_ROOT, "gp_predict", "export")
if _GP_EXPORT not in sys.path:
    sys.path.insert(0, _GP_EXPORT)

import numpy as np
import rerun as rr
from scipy.spatial.transform import Rotation as Rot

from g1_spine_stabilization.perception.detector import AprilTagDetector, CameraIntrinsics
from g1_spine_stabilization.perception.frame_sources import RealSenseSource
from g1_spine_stabilization.perception.frames import (
    optical_to_d435_from_config, d435_to_pelvis_from_pinocchio,
)
from g1_spine_stabilization.perception.tracker import MarkerTracker
from g1_spine_stabilization.utils.config import load_config

from torso_gp_predictor import TorsoGP

MARKER_ID = 2               # the torso/pelvis marker the GP was trained on
MODEL_DIR = os.path.join(_GP_EXPORT, "models_robot")
ARROW_SCALE = 1.0           # seconds of travel drawn per unit velocity
_AXIS_LEN = 0.05

_CYAN = [0, 220, 220, 255]
_GRAY = [150, 150, 150, 255]


def log_pose_axes(name, p, R, colors):
    rr.log(name, rr.Arrows3D(
        origins=[p, p, p],
        vectors=[R[:, 0] * _AXIS_LEN, R[:, 1] * _AXIS_LEN, R[:, 2] * _AXIS_LEN],
        colors=colors,
    ))


def build_tracker(cfg):
    """Live RealSense + AprilTag tracker, wired exactly like the other
    perception scripts so poses land in the same pelvis frame the GP was
    trained on."""
    perception = cfg["perception"]
    cam = perception.get("camera", {})
    frame_source = RealSenseSource(
        width=cam.get("width", 640), height=cam.get("height", 480),
        fps=cam.get("fps", 30), dry_run=False,
        stream=cam.get("stream", "color"),
        emitter_enabled=cam.get("emitter_enabled", False),
    )
    native = frame_source.intrinsics_native
    intrinsics = CameraIntrinsics(
        fx=native["fx"], fy=native["fy"], cx=native["cx"], cy=native["cy"],
        distortion=np.asarray(native["distortion"]),
    )
    detector = AprilTagDetector(perception["marker_dict"], perception["marker_size"])
    T_opt_to_d435 = optical_to_d435_from_config(perception["d435_link_to_optical"])
    if frame_source.ir_to_color_extrinsic is not None:
        T_opt_to_d435 = T_opt_to_d435 @ frame_source.ir_to_color_extrinsic
        print("[gp-viz] stream=infrared: composed with live IR->color extrinsic.")
    tracker = MarkerTracker(frame_source, detector, intrinsics,
                            T_opt_to_d435, d435_to_pelvis_from_pinocchio())
    return tracker, frame_source


def replay_stream(path):
    """(t, p, R) from a recorded pelvis_marker pickle, paced in real time."""
    with open(path, "rb") as f:
        payload = pickle.load(f)
    samples = payload.get("samples", [])
    print(f"[gp-viz] Replaying {len(samples)} samples from {os.path.basename(path)}")
    for s in samples:
        T = s["pose_in_pelvis"]
        yield s["t"], T[:3, 3].copy(), T[:3, :3].copy()


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--replay", default=None,
                    help="replay a recorded pelvis_marker .pkl instead of the camera")
    ap.add_argument("--rate", type=float, default=None,
                    help="faked control rate in Hz (default: 1/control.dt from config)")
    ap.add_argument("--config", default=None)
    args = ap.parse_args()

    cfg = load_config(args.config)
    control_dt = 1.0 / args.rate if args.rate else float(cfg["control"]["dt"])
    print(f"[gp-viz] Faked control loop: {1.0 / control_dt:.1f} Hz "
          f"(dt={control_dt * 1000:.0f}ms, sleep only -- no robot)")

    gp = TorsoGP(MODEL_DIR)
    print(f"[gp-viz] GP loaded: horizons {gp.hset} "
          f"= {[round(h * gp.step_dt, 2) for h in gp.hset]}s, "
          f"step_dt={gp.step_dt * 1000:.0f}ms, accept_dt={gp.accept_dt * 1000:.0f}ms")
    print(f"[gp-viz] GP refreshes at {1.0 / gp.step_dt:.1f} Hz; "
          f"the loop runs at {1.0 / control_dt:.1f} Hz -- "
          f"the GP twist is held between refreshes.")

    rr.init("gp_velocity_viz", spawn=True)
    rr.log("/", rr.ViewCoordinates.RIGHT_HAND_Z_UP, static=True)

    tracker = frame_source = None
    if args.replay is None:
        tracker, frame_source = build_tracker(cfg)
        print(f"[gp-viz] Tracking marker id={MARKER_ID} "
              f"(marker_size={cfg['perception']['marker_size']:.4f}m)")
        input("[gp-viz] Press Enter to START, then move the torso marker…")
    else:
        input("[gp-viz] Press Enter to START replay…")

    gp.reset()
    stream = replay_stream(args.replay) if args.replay else None

    t0 = time.monotonic()
    pending = None            # replay: next sample not yet due
    prev_t = prev_p = prev_R = None
    tw_mean = None            # last GP twist, held between refreshes
    n_accept = 0
    n_tick = 0

    try:
        while True:
            tick_start = time.monotonic()

            # ---- get one pose sample --------------------------------
            if stream is not None:
                # Consume every recorded sample whose timestamp has come due,
                # keeping the newest -- so replay advances in real time at the
                # faked control rate, exactly like the live path, instead of
                # one sample per tick (which would run a 30 Hz recording
                # 3.75x too fast at an 8 Hz loop).
                t_now = time.monotonic() - t0
                got = None
                try:
                    while pending is None or pending[0] <= t_now:
                        if pending is not None:
                            got = pending
                        pending = next(stream)
                except StopIteration:
                    if got is None:
                        print("[gp-viz] Replay finished.")
                        break
                if got is None:
                    time.sleep(max(0.0, control_dt - (time.monotonic() - tick_start)))
                    continue
                t, p, R = got
            else:
                tracker.step()
                obs = tracker.get_latest(MARKER_ID)
                t = time.monotonic() - t0
                if obs is None:
                    time.sleep(max(0.0, control_dt - (time.monotonic() - tick_start)))
                    continue
                p = obs.pose_in_pelvis[:3, 3].copy()
                R = obs.pose_in_pelvis[:3, :3].copy()

            n_tick += 1
            rr.set_time("t", duration=t)

            # ---- measured velocity (finite difference) ---------------
            v_meas = np.zeros(3)
            w_meas = np.zeros(3)
            if prev_t is not None and t > prev_t:
                dt = t - prev_t
                v_meas = (p - prev_p) / dt
                w_meas = Rot.from_matrix(prev_R.T @ R).as_rotvec() / dt
            prev_t, prev_p, prev_R = t, p.copy(), R.copy()

            # ---- feed the GP; it accepts at its own ~4.4 Hz ----------
            if gp.update(t, p, R):
                n_accept += 1
                tw = gp.twist()
                if tw is not None:
                    tw_mean = tw[0]

            # ---- visualize -------------------------------------------
            log_pose_axes("gp/marker", p, R,
                          [[255, 0, 0, 255], [0, 255, 0, 255], [0, 0, 255, 255]])
            rr.log("gp/velocity_measured", rr.Arrows3D(
                origins=[p], vectors=[v_meas * ARROW_SCALE], colors=[_GRAY]))
            rr.log("plots/speed_measured", rr.Scalars(float(np.linalg.norm(v_meas))))
            rr.log("plots/omega_measured", rr.Scalars(float(np.linalg.norm(w_meas))))

            if tw_mean is not None:
                # twist() is in the marker BODY frame at the current step:
                # rotate linear into pelvis, right-multiply the angular part.
                v_body, w_body = tw_mean[:3], tw_mean[3:]
                v_pelvis = R @ v_body
                rr.log("gp/velocity_gp", rr.Arrows3D(
                    origins=[p], vectors=[v_pelvis * ARROW_SCALE], colors=[_CYAN]))
                rr.log("plots/speed_gp", rr.Scalars(float(np.linalg.norm(v_pelvis))))
                rr.log("plots/omega_gp", rr.Scalars(float(np.linalg.norm(w_body))))

                # Where the GP says the marker will be one GP-step ahead.
                T_ahead = gp.hset[0] * gp.step_dt
                p_pred = p + v_pelvis * T_ahead
                R_pred = R @ Rot.from_rotvec(w_body * T_ahead).as_matrix()
                log_pose_axes("gp/predicted_pose", p_pred, R_pred,
                              [_CYAN, _CYAN, _CYAN])

                out = gp.predict(horizons=(gp.hset[0],))
                if out is not None:
                    sig = out[gp.hset[0]]["sigma"]
                    rr.log("plots/sigma_pos", rr.Scalars(float(np.linalg.norm(sig[:3]))))
                    rr.log("plots/sigma_rot", rr.Scalars(float(np.linalg.norm(sig[3:]))))

            # ---- fake the control period -----------------------------
            time.sleep(max(0.0, control_dt - (time.monotonic() - tick_start)))

    except KeyboardInterrupt:
        print()
    finally:
        if frame_source is not None:
            frame_source.close()
        print(f"[gp-viz] {n_tick} control ticks, {n_accept} GP refreshes "
              f"({n_accept / n_tick * 100:.0f}% of ticks)" if n_tick else "")


if __name__ == "__main__":
    main()
