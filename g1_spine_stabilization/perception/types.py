"""Data types shared across the perception module."""
from dataclasses import dataclass
from typing import Optional
import numpy as np


@dataclass
class MarkerObservation:
    """A single marker detection, with pose expressed in the pelvis frame."""
    marker_id: int
    pose_in_pelvis: np.ndarray   # 4x4 SE3
    timestamp: float             # seconds, monotonic
    reprojection_error: float    # pixels, from solvePnP refinement; lower is better
    # 4x4 SE3, camera optical frame (OpenCV/REP-103: X right, Y down, Z
    # forward) -- the raw solvePnP output, before T_optical_to_d435/
    # T_d435_to_pelvis composition. Optional/None for observations that
    # don't come from a real camera (e.g. synthetic MarkerObservations built
    # by hand in tests, or mock trackers that don't populate it) -- real
    # detections via MarkerTracker.step() always populate this. Used by
    # data_recording.recorder.RunRecorder for object/target angular
    # velocity, specifically to avoid depending on the pelvis-frame
    # composition (see docs/data_recording_conventions.md).
    pose_in_optical: Optional[np.ndarray] = None
