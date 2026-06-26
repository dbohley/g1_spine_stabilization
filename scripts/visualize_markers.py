#!/usr/bin/env python3
"""Visualize AprilTag detections from the head-mounted RealSense.
Hold up markers, see if poses look right. Used to validate the perception
pipeline against real hardware."""
import argparse
import os
import sys
import time

# Make the package importable when running this script directly without
# `pip install -e .`.
_THIS_DIR = os.path.dirname(os.path.abspath(__file__))
_PROJECT_ROOT = os.path.abspath(os.path.join(_THIS_DIR, '..'))
if _PROJECT_ROOT not in sys.path:
    sys.path.insert(0, _PROJECT_ROOT)

import cv2
import numpy as np

from g1_spine_stabilization.perception.detector import (
    AprilTagDetector, CameraIntrinsics
)
from g1_spine_stabilization.perception.frame_sources import RealSenseSource
from g1_spine_stabilization.perception.frames import (
    optical_to_d435_from_config, d435_to_pelvis_from_pinocchio,
    compose_to_pelvis,
)
from g1_spine_stabilization.perception.tracker import (
    MarkerTracker, MockMarkerTracker
)
from g1_spine_stabilization.perception.types import MarkerObservation
from g1_spine_stabilization.utils.config import load_config


def run_realsense(config, dry_run=False):
    perception = config["perception"]
    source = RealSenseSource(dry_run=dry_run)
    native = source.intrinsics_native
    intrinsics = CameraIntrinsics(
        fx=native["fx"], fy=native["fy"], cx=native["cx"], cy=native["cy"],
        distortion=np.asarray(native["distortion"]),
    )
    detector = AprilTagDetector(
        perception["marker_dict"], perception["marker_size"]
    )
    T_opt_to_d435 = optical_to_d435_from_config(perception["d435_link_to_optical"])
    T_d435_to_pelvis = d435_to_pelvis_from_pinocchio()

    tracker = MarkerTracker(
        source, detector, intrinsics, T_opt_to_d435, T_d435_to_pelvis
    )

    print("[visualize_markers] Streaming. Press 'q' to quit.")
    try:
        while True:
            read = source.read()
            if read is None:
                break
            frame, ts = read
            # Re-detect on the same frame for visualization
            raw = detector.detect(frame, intrinsics)
            for det in raw:
                # Recover rvec/tvec from the SE3 pose for drawFrameAxes
                R = det.pose_in_optical[:3, :3]
                t = det.pose_in_optical[:3, 3]
                rvec, _ = cv2.Rodrigues(R)
                cv2.drawFrameAxes(
                    frame, intrinsics.camera_matrix,
                    np.asarray(intrinsics.distortion), rvec, t,
                    perception["marker_size"] * 0.5,
                )
                


               
                cv2.putText(
                    frame, f"id={det.marker_id} err={det.reprojection_error:.2f}px",
                    tuple(det.corners[0].astype(int)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2,
                )
                # Also compose into pelvis frame and print
                pose_pelvis = compose_to_pelvis(
                    det.pose_in_optical, T_opt_to_d435, T_d435_to_pelvis
                )
                pos = pose_pelvis[:3, 3]
                print(f"[visualize_markers] id={det.marker_id} "
                      f"pelvis_pos=[{pos[0]:.3f}, {pos[1]:.3f}, {pos[2]:.3f}] m  "
                      f"reproj_err={det.reprojection_error:.2f} px")
                R = pose_pelvis[:3, :3]
                print(f"  marker_X_in_pelvis = [{R[0,0]:+.2f}, {R[1,0]:+.2f}, {R[2,0]:+.2f}]")
                print(f"  marker_Y_in_pelvis = [{R[0,1]:+.2f}, {R[1,1]:+.2f}, {R[2,1]:+.2f}]")
                print(f"  marker_Z_in_pelvis = [{R[0,2]:+.2f}, {R[1,2]:+.2f}, {R[2,2]:+.2f}]")
            cv2.imshow("AprilTag detections", frame)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break
    finally:
        source.close()
        cv2.destroyAllWindows()


def run_mock(config):
    # Three deterministic observations
    obs = []
    for i, x in enumerate([0.3, 0.32, 0.34]):
        pose = np.eye(4); pose[:3, 3] = [x, 0.0, 0.5]
        obs.append(MarkerObservation(
            marker_id=config["perception"]["target_marker_id"],
            pose_in_pelvis=pose, timestamp=float(i), reprojection_error=0.3,
        ))
    tracker = MockMarkerTracker(obs)
    print("[visualize_markers] Mock mode: 3 scripted observations.")
    while True:
        result = tracker.step()
        if not result:
            break
        for o in result:
            pos = o.pose_in_pelvis[:3, 3]
            print(f"[visualize_markers] (mock) id={o.marker_id} "
                  f"pos=[{pos[0]:.3f}, {pos[1]:.3f}, {pos[2]:.3f}]")
        time.sleep(0.2)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--source", choices=["realsense", "mock"], default="mock")
    parser.add_argument("--config", default=None)
    parser.add_argument("--dry-run", action="store_true",
                        help="Use synthetic camera frames (no hardware needed)")
    args = parser.parse_args()
    config = load_config(args.config)
    if args.source == "realsense":
        run_realsense(config, dry_run=args.dry_run)
    else:
        run_mock(config)


if __name__ == "__main__":
    main()
