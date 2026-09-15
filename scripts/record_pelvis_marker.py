#!/usr/bin/env python3
"""Record one AprilTag marker's pelvis-frame trajectory between two Enters.

Minimal, perception-only: no robot connection, no IK, no force sensor.
Enter starts recording, Enter stops it, and the samples are written to a
.pkl next to every other run under data/runs/.

Camera-only code (no actuation), so this is safe to run live.

    python scripts/record_pelvis_marker.py --marker-id 1
    python scripts/record_pelvis_marker.py --marker-id 1 --source mock
"""
import argparse
import os
import pickle
import sys
import threading
import time
from datetime import datetime

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
from g1_spine_stabilization.utils.config import load_config

SCHEMA_VERSION = 1

# The pelvis marker. Same tag as perception.target_marker_id in
# configs/default.yaml.
MARKER_ID = 2


def build_tracker(cfg, dry_run):
    """Live RealSense + AprilTag tracker, wired exactly like the other
    perception scripts so this records in the same pelvis frame they do."""
    perception = cfg["perception"]
    cam = perception.get("camera", {})

    frame_source = RealSenseSource(
        width=cam.get("width", 640),
        height=cam.get("height", 480),
        fps=cam.get("fps", 30),
        dry_run=dry_run,
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
        # Infrared mode: detection runs in the IR-left optical frame, but
        # d435_link_to_optical is calibrated for color -- bridge with the
        # camera's own factory extrinsic (same composition as the preview).
        T_opt_to_d435 = T_opt_to_d435 @ frame_source.ir_to_color_extrinsic
        print("[pelvis-rec] stream=infrared: composed with live IR->color extrinsic.")

    # Zero-waist assumption: no robot connection here, so there is no live
    # waist angle to read (the preview script corrects this post-connect).
    T_d435_to_pelvis = d435_to_pelvis_from_pinocchio()

    tracker = MarkerTracker(
        frame_source, detector, intrinsics, T_opt_to_d435, T_d435_to_pelvis,
    )
    return tracker, frame_source


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--source", choices=["realsense", "mock"], default="realsense",
                    help="mock = synthetic frames, no camera needed")
    ap.add_argument("--rate", type=float, default=30.0,
                    help="target sample rate in Hz (default: 30)")
    ap.add_argument("--out", default=None, help="output .pkl path")
    ap.add_argument("--config", default=None)
    args = ap.parse_args()

    cfg = load_config(args.config)
    print(f"[pelvis-rec] Tracking marker ID={MARKER_ID} "
          f"(marker_size={cfg['perception']['marker_size']:.4f}m)")

    tracker, frame_source = build_tracker(cfg, dry_run=(args.source == "mock"))

    input("[pelvis-rec] Press Enter to START recording…")

    samples = []
    stop = threading.Event()
    period = 1.0 / args.rate

    def loop():
        t0 = time.monotonic()
        misses = 0
        while not stop.is_set():
            tracker.step()
            obs = tracker.get_latest(MARKER_ID)
            now = time.monotonic()
            if obs is not None and obs.timestamp is not None:
                samples.append({
                    "t": now - t0,
                    "pose_in_pelvis": obs.pose_in_pelvis.copy(),
                    "pose_in_optical": (None if obs.pose_in_optical is None
                                        else obs.pose_in_optical.copy()),
                    "reprojection_error": obs.reprojection_error,
                })
            else:
                misses += 1
            stop.wait(period)
        if misses:
            print(f"[pelvis-rec] ({misses} frames without a detection)")

    worker = threading.Thread(target=loop, daemon=True)
    worker.start()
    print("[pelvis-rec] Recording… press Enter to STOP.")

    try:
        input()
    except KeyboardInterrupt:
        print()
    finally:
        stop.set()
        worker.join(timeout=2.0)

    out = args.out
    if out is None:
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        out = os.path.join(_PROJECT_ROOT, "data", "runs",
                           f"pelvis_marker_{stamp}_id{MARKER_ID}.pkl")
    os.makedirs(os.path.dirname(out), exist_ok=True)

    payload = {
        "schema_version": SCHEMA_VERSION,
        "origin": "pelvis_marker",
        "marker_id": MARKER_ID,
        "source_label": args.source,
        "frame": "pelvis",
        "marker_size": cfg["perception"]["marker_size"],
        "camera_stream": cfg["perception"].get("camera", {}).get("stream", "color"),
        "samples": samples,
    }
    with open(out, "wb") as f:
        pickle.dump(payload, f)

    if samples:
        span = samples[-1]["t"] - samples[0]["t"]
        pos = np.array([s["pose_in_pelvis"][:3, 3] for s in samples])
        travel = float(np.linalg.norm(pos.max(axis=0) - pos.min(axis=0)))
        print(f"[pelvis-rec] {len(samples)} samples over {span:.1f}s "
              f"({len(samples) / span if span > 0 else 0:.1f} Hz), "
              f"bounding-box travel {travel * 100:.1f}cm")
    else:
        print(f"[pelvis-rec] WARNING: no detections recorded — "
              f"marker id={MARKER_ID} not visible?")
    print(f"[pelvis-rec] Saved: {out}")

    frame_source.close()


if __name__ == "__main__":
    main()
