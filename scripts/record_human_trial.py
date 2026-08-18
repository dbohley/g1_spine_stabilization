#!/usr/bin/env python3
"""Record a human performing the same bimanual pose-tracking task the robot
does: hold an object marker (steady or moving), track a moving target marker
with it. NO robot, NO IK -- perception-only.

Reuses the exact same camera/marker stack and config as
run_pose_tracking_preview.py (RealSenseSource, AprilTagDetector,
MarkerTracker, configs/default.yaml's perception: section) and the SAME
RunRecorder/MarkerSample schema (g1_spine_stabilization.data_recording.
recorder) as the robot's Phase-2 recording, so pickle files from either
script are interchangeable inputs to a later comparison analysis.

The camera may need repositioning (no robot arm/hand to avoid), but marker
dictionary/size/IDs and the AprilTag detection stack are unchanged from the
robot pipeline -- this is required for the two datasets to be comparable.

This is camera-only code (no actuation), so unlike the robot scripts it is
safe to run live (not just --dry-run) in a Claude Code session.
"""
import argparse
import math
import os
import sys
import time

_THIS_DIR = os.path.dirname(os.path.abspath(__file__))
_PROJECT_ROOT = os.path.abspath(os.path.join(_THIS_DIR, ".."))
if _PROJECT_ROOT not in sys.path:
    sys.path.insert(0, _PROJECT_ROOT)

import numpy as np

from g1_spine_stabilization.perception.detector import AprilTagDetector, CameraIntrinsics
from g1_spine_stabilization.perception.frame_sources import RealSenseSource
from g1_spine_stabilization.perception.frames import (
    optical_to_d435_from_config, d435_to_pelvis_from_pinocchio,
)
from g1_spine_stabilization.perception.tracker import MarkerTracker
from g1_spine_stabilization.perception.types import MarkerObservation
from g1_spine_stabilization.data_recording.recorder import RunRecorder
from g1_spine_stabilization.sensors.force_torque import ForceTorqueSource
from g1_spine_stabilization.utils.config import load_config


def _obs_rotation(obs):
    """3x3 rotation from a MarkerObservation (optical frame preferred, pelvis
    fallback), or None -- the torso/target orientation for F/T gravity comp."""
    if obs is None:
        return None
    pose_opt = getattr(obs, "pose_in_optical", None)
    if pose_opt is not None:
        return pose_opt[:3, :3]
    return obs.pose_in_pelvis[:3, :3]


class _AnimatedHumanMockTracker:
    """Hardware-free smoke-test tracker: object fixed, target oscillates in
    rotation over wall-clock time. Script-local (mirrors
    run_pose_tracking_preview.py's _AnimatedMockTracker) since scripts/ is
    not part of the installed package."""

    def __init__(self, object_marker_id, target_marker_id,
                 box_center=(0.35, 0.0, 0.1), amplitude=0.4, freq=0.2):
        self._obj_id = object_marker_id
        self._tgt_id = target_marker_id
        self._box_center = np.asarray(box_center, dtype=float)
        self._amplitude = amplitude
        self._freq = freq
        self._latest: dict = {}
        self._t_start = time.monotonic()

    def step(self):
        t = time.monotonic() - self._t_start

        obj_pose = np.eye(4)
        obj_pose[:3, 3] = self._box_center
        obs_obj = MarkerObservation(
            marker_id=self._obj_id, pose_in_pelvis=obj_pose,
            timestamp=t, reprojection_error=0.0,
            # No real camera -- reuse the synthetic pose as a stand-in
            # "optical frame" reading so --record exercises the recorder's
            # camera-frame angular-velocity path in this hardware-free mode.
            pose_in_optical=obj_pose,
        )

        theta = self._amplitude * math.sin(2 * math.pi * self._freq * t)
        ct, st = math.cos(theta), math.sin(theta)
        tgt_pose = np.eye(4)
        tgt_pose[:3, :3] = np.array([[1, 0, 0], [0, ct, -st], [0, st, ct]])
        tgt_pose[:3, 3] = self._box_center
        obs_tgt = MarkerObservation(
            marker_id=self._tgt_id, pose_in_pelvis=tgt_pose,
            timestamp=t, reprojection_error=0.0,
            pose_in_optical=tgt_pose,
        )

        self._latest[self._obj_id] = obs_obj
        self._latest[self._tgt_id] = obs_tgt
        return [obs_obj, obs_tgt]

    def get_latest(self, marker_id):
        return self._latest.get(marker_id)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", default=None, help="Path to YAML config")
    parser.add_argument(
        "--source", choices=["realsense", "mock"], default="realsense",
        help="Camera source (default: realsense)",
    )
    parser.add_argument(
        "--dry-run", action="store_true",
        help="Synthetic camera frames, no real camera required (--source realsense only)",
    )
    parser.add_argument(
        "--tracking", choices=["orient-only", "6dof"], default="orient-only",
        help="Label only (no compute_hand_targets math here) -- records which "
             "physical setup this trial used, for filename/metadata parity with "
             "the robot script",
    )
    parser.add_argument(
        "--record", nargs="?", const="__AUTO__", default=None, metavar="PATH",
        help="Save this trial to a pickle file (same schema as the robot recorder). "
             "Omit PATH to auto-generate under data/runs/. If --record is omitted "
             "entirely, the script still runs (e.g. for a camera-positioning "
             "rehearsal) but nothing is saved.",
    )
    parser.add_argument(
        "--duration", type=float, default=None,
        help="Seconds to record. Default: run until Ctrl-C.",
    )
    parser.add_argument(
        "--force-sensor", dest="force_sensor", action="store_true", default=None,
        help="Force-enable the ATI F/T sensor (overrides force_sensor.enabled in config).",
    )
    parser.add_argument(
        "--no-force-sensor", dest="force_sensor", action="store_false",
        help="Disable the F/T sensor for this run (e.g. sensor unplugged).",
    )
    args = parser.parse_args()

    cfg = load_config(args.config)
    perception = cfg["perception"]
    obj_id = perception["object_marker_id"]
    tgt_id = perception["target_marker_id"]

    print("[record_human_trial] Marker roles (configs/default.yaml):")
    print(f"  Object marker  ID={obj_id}  -> attach to the held object")
    print(f"  Target marker  ID={tgt_id}  -> the reference to track")

    frame_source = None
    if args.source == "realsense":
        frame_source = RealSenseSource(dry_run=args.dry_run)
        native = frame_source.intrinsics_native
        intrinsics = CameraIntrinsics(
            fx=native["fx"], fy=native["fy"], cx=native["cx"], cy=native["cy"],
            distortion=np.asarray(native["distortion"]),
        )
        detector = AprilTagDetector(perception["marker_dict"], perception["marker_size"])
        T_opt_to_d435 = optical_to_d435_from_config(perception["d435_link_to_optical"])
        # No robot/IK object in scope -- pure Pinocchio FK on the URDF for a
        # camera-mount-frame constant, same pattern as visualize_markers.py's
        # run_realsense(). No actuation; this is the SAME reference frame the
        # robot pipeline uses, which is what makes the two datasets comparable.
        T_d435_to_pelvis = d435_to_pelvis_from_pinocchio()
        tracker = MarkerTracker(frame_source, detector, intrinsics, T_opt_to_d435, T_d435_to_pelvis)
    else:
        tracker = _AnimatedHumanMockTracker(obj_id, tgt_id)

    recorder = None
    if args.record is not None:
        data_recording_cfg = cfg.get("data_recording", {})
        if not data_recording_cfg.get("enabled", True):
            print("[record_human_trial] --record passed but data_recording.enabled=false; ignoring.")
        else:
            output_path = None if args.record == "__AUTO__" else args.record
            recorder = RunRecorder(
                origin="human", source_label=args.source, tracking_mode=args.tracking,
                output_path=output_path,
                output_dir=data_recording_cfg.get("output_dir"),
                filename_template=data_recording_cfg.get("filename_template"),
                object_marker_id=obj_id, target_marker_id=tgt_id,
                metadata={"perception": dict(perception)},
            )
            print(f"[record_human_trial] Recording enabled -> {recorder.output_path}")
    else:
        print("[record_human_trial] --record not passed -- running WITHOUT saving "
              "(use for camera-positioning rehearsal only).")

    # ATI F/T sensor (only meaningful when recording). Degrades to None
    # wrenches if unreachable, so this script always runs. Same camera/marker/
    # sensor stack as the robot path -- see docs/force_sensor_setup.md.
    wrench_source = None
    if recorder is not None:
        wrench_source = ForceTorqueSource.from_config(cfg, enabled_override=args.force_sensor)
        wrench_source.start()

    print("[record_human_trial] Ready. Hold the object marker steady (or move it), "
          "move the target marker to perform the tracking task. Ctrl-C to stop and save.")
    if recorder is not None:
        recorder.start()

    t_start = time.monotonic()
    _last_print_t = t_start - 1.0
    _wrench_ref_captured = False
    try:
        while True:
            elapsed = time.monotonic() - t_start
            if args.duration is not None and elapsed >= args.duration:
                break

            tracker.step()
            obj_obs = tracker.get_latest(obj_id)
            tgt_obs = tracker.get_latest(tgt_id)
            R_torso = _obs_rotation(tgt_obs)

            wrench_raw = wrench_comp = None
            if wrench_source is not None:
                if not _wrench_ref_captured:
                    wrench_source.capture_reference(R_torso)
                    _wrench_ref_captured = True
                wrench_raw, wrench_comp = wrench_source.read(R_torso)

            if recorder is not None:
                recorder.record(t=elapsed, object_obs=obj_obs, target_obs=tgt_obs,
                                load_cell_wrench_raw=wrench_raw,
                                load_cell_wrench_grav_comp=wrench_comp)

            if time.monotonic() - _last_print_t >= 1.0:
                _last_print_t = time.monotonic()
                print(f"[record_human_trial] t={elapsed:.1f}s  "
                      f"object={'seen' if obj_obs is not None else 'NOT SEEN'}  "
                      f"target={'seen' if tgt_obs is not None else 'NOT SEEN'}  "
                      f"samples={len(recorder) if recorder is not None else 0}")

            if args.source == "mock":
                time.sleep(0.02)   # avoid pegging a CPU core; no actuation, not timing-critical
    except KeyboardInterrupt:
        print("\n[record_human_trial] KeyboardInterrupt -- stopping.")
    finally:
        if recorder is not None:
            recorder.save()
        if wrench_source is not None:
            wrench_source.close()
        if frame_source is not None:
            frame_source.close()
    print("[record_human_trial] Done.")


if __name__ == "__main__":
    main()
