"""Data types shared across the perception module."""
from dataclasses import dataclass
import numpy as np


@dataclass
class MarkerObservation:
    """A single marker detection, with pose expressed in the pelvis frame."""
    marker_id: int
    pose_in_pelvis: np.ndarray   # 4x4 SE3
    timestamp: float             # seconds, monotonic
    reprojection_error: float    # pixels, from solvePnP refinement; lower is better
