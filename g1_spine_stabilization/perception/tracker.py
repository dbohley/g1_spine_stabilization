"""Marker trackers: real (combines frame source + detector + transforms)
and mock (deterministic poses, no hardware)."""
from typing import Optional
import numpy as np

from g1_spine_stabilization.perception.types import MarkerObservation
from g1_spine_stabilization.perception.detector import (
    AprilTagDetector, CameraIntrinsics
)


class MarkerTracker:
    """Combines a frame source, detector, and camera-to-pelvis transform.
    Returns observations in the pelvis frame."""

    def __init__(
        self,
        frame_source,
        detector: AprilTagDetector,
        intrinsics: CameraIntrinsics,
        T_optical_to_d435: np.ndarray,
        T_d435_to_pelvis: np.ndarray,
    ):
        self._source = frame_source
        self._detector = detector
        self._intrinsics = intrinsics
        self._T_optical_to_d435 = T_optical_to_d435
        self._T_d435_to_pelvis = T_d435_to_pelvis
        self._latest: dict[int, MarkerObservation] = {}
        # Intermediate transform-chain stages, kept per marker for diagnostics
        # (see get_diagnostic_pose) -- isolating a real-world calibration
        # error (e.g. the ~10cm marker-height discrepancy) needs comparing
        # physical measurements against each stage, not just the final pose.
        self._last_optical: dict[int, np.ndarray] = {}
        self._last_d435_mount: dict[int, np.ndarray] = {}

    def step(self) -> list[MarkerObservation]:
        """Read one frame, detect markers, return current-frame observations."""
        read_result = self._source.read()
        if read_result is None:
            return []
        frame, timestamp = read_result
        raw = self._detector.detect(frame, self._intrinsics)
        observations = []
        for det in raw:
            pose_d435_mount = self._T_optical_to_d435 @ det.pose_in_optical
            pose_pelvis = self._T_d435_to_pelvis @ pose_d435_mount
            self._last_optical[det.marker_id] = det.pose_in_optical
            self._last_d435_mount[det.marker_id] = pose_d435_mount
            obs = MarkerObservation(
                marker_id=det.marker_id,
                pose_in_pelvis=pose_pelvis,
                timestamp=timestamp,
                reprojection_error=det.reprojection_error,
            )
            observations.append(obs)
            self._latest[det.marker_id] = obs
        return observations

    def get_latest(self, marker_id: int) -> Optional[MarkerObservation]:
        return self._latest.get(marker_id)

    def get_diagnostic_pose(self, marker_id: int) -> Optional[dict]:
        """Return {'optical', 'd435_mount', 'pelvis'} 4x4 poses for the most
        recent detection of marker_id, or None if never seen. Lets a human
        compare a physical measurement against each transform stage to
        isolate which one introduces a calibration error, rather than only
        seeing the final composed pelvis-frame pose."""
        if marker_id not in self._latest:
            return None
        return {
            "optical": self._last_optical[marker_id],
            "d435_mount": self._last_d435_mount[marker_id],
            "pelvis": self._latest[marker_id].pose_in_pelvis,
        }


class MockMarkerTracker:
    """Returns deterministic poses without any camera or detector.

    Useful for unit testing the controller integration (Session 3) and
    for smoke-testing visualize_markers.py without hardware.
    """

    def __init__(self, scripted_observations: list[MarkerObservation]):
        """scripted_observations: returned in order, one per step() call.
        After exhaustion, step() returns []."""
        self._script = list(scripted_observations)
        self._index = 0
        self._latest: dict[int, MarkerObservation] = {}

    def step(self) -> list[MarkerObservation]:
        if self._index >= len(self._script):
            return []
        obs = self._script[self._index]
        self._index += 1
        self._latest[obs.marker_id] = obs
        return [obs]

    def get_latest(self, marker_id: int) -> Optional[MarkerObservation]:
        return self._latest.get(marker_id)
