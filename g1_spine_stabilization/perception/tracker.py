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
    Returns observations in the pelvis frame.

    Real-world fault handling -- step() never raises because of the camera:
      * Dropped/failed frames: if frame_source.read() raises (e.g. a
        librealsense wait_for_frames timeout, a USB hiccup) or returns an
        empty frame, the frame is counted as dropped and step() returns [] --
        the same "nothing seen this tick" result as a frame with no markers
        in view. Downstream (MarkerSource) already treats that as "hold the
        last known pose and track staleness by wall-clock", so a dropped frame
        degrades gracefully instead of killing the control loop mid-grasp.
        Logged edge-triggered (first drop, periodic reminder while the outage
        lasts, recovery) so a sustained outage is loud but doesn't spam.
      * Bad pose fits: with max_reprojection_error set, detections whose
        solvePnP reprojection error exceeds it (motion blur, glare, a partly
        occluded tag) are rejected rather than passed downstream as a
        confidently-wrong pose. None (default) disables the gate.
    Counters are exposed via `stats` for post-run diagnostics.
    """

    # Re-print the "frames are dropping" warning every this many consecutive
    # drops while an outage lasts (~1 s at 30 fps).
    DROP_WARN_EVERY = 30

    def __init__(
        self,
        frame_source,
        detector: AprilTagDetector,
        intrinsics: CameraIntrinsics,
        T_optical_to_d435: np.ndarray,
        T_d435_to_pelvis: np.ndarray,
        pelvis_offset: Optional[np.ndarray] = None,
        max_reprojection_error: Optional[float] = None,
    ):
        self._source = frame_source
        self._detector = detector
        self._intrinsics = intrinsics
        self._T_optical_to_d435 = T_optical_to_d435
        self._T_d435_to_pelvis = T_d435_to_pelvis
        self._max_reproj_err = max_reprojection_error
        self._frames_ok = 0
        self._frames_dropped = 0
        self._consecutive_dropped = 0
        self._detections_rejected = 0
        # Applied directly in pelvis frame, AFTER the full optical->mount->
        # pelvis composition below -- this is the one place "pelvis frame"
        # is unambiguous and final, so e.g. [0, 0.05, 0] means exactly "+5cm
        # in pelvis Y" with no rotation in between to complicate it. Meant
        # for a deliberate, easily-reversible empirical probe (see
        # configs/default.yaml perception.pelvis_offset_hotfix), NOT for the
        # real camera-mount calibration -- that lives in T_optical_to_d435.
        self._pelvis_offset = (
            np.zeros(3) if pelvis_offset is None else np.asarray(pelvis_offset, dtype=float)
        )
        self._latest: dict[int, MarkerObservation] = {}
        # Intermediate transform-chain stages, kept per marker for diagnostics
        # (see get_diagnostic_pose) -- isolating a real-world calibration
        # error (e.g. the ~10cm marker-height discrepancy) needs comparing
        # physical measurements against each stage, not just the final pose.
        self._last_optical: dict[int, np.ndarray] = {}
        self._last_d435_mount: dict[int, np.ndarray] = {}

    def step(self) -> list[MarkerObservation]:
        """Read one frame, detect markers, return current-frame observations.

        Returns [] (never raises) on end-of-stream or a dropped frame -- see
        the class docstring."""
        try:
            read_result = self._source.read()
        except Exception as exc:  # camera faults only; KeyboardInterrupt still propagates
            self._record_drop(f"{type(exc).__name__}: {exc}")
            return []
        if read_result is None:
            return []
        frame, timestamp = read_result
        if frame is None or frame.size == 0:
            self._record_drop("empty frame")
            return []
        if self._consecutive_dropped:
            print(f"[MarkerTracker] Camera recovered after "
                  f"{self._consecutive_dropped} dropped frame(s).")
            self._consecutive_dropped = 0
        self._frames_ok += 1

        raw = self._detector.detect(frame, self._intrinsics)
        observations = []
        for det in raw:
            if (self._max_reproj_err is not None
                    and det.reprojection_error > self._max_reproj_err):
                self._detections_rejected += 1
                continue
            pose_d435_mount = self._T_optical_to_d435 @ det.pose_in_optical
            pose_pelvis = self._T_d435_to_pelvis @ pose_d435_mount
            pose_pelvis[:3, 3] += self._pelvis_offset
            self._last_optical[det.marker_id] = det.pose_in_optical
            self._last_d435_mount[det.marker_id] = pose_d435_mount
            obs = MarkerObservation(
                marker_id=det.marker_id,
                pose_in_pelvis=pose_pelvis,
                timestamp=timestamp,
                reprojection_error=det.reprojection_error,
                pose_in_optical=det.pose_in_optical,
            )
            observations.append(obs)
            self._latest[det.marker_id] = obs
        return observations

    def _record_drop(self, reason: str) -> None:
        self._frames_dropped += 1
        self._consecutive_dropped += 1
        if self._consecutive_dropped == 1:
            print(f"[MarkerTracker] WARNING: dropped frame ({reason}) -- "
                  f"holding last known marker poses.")
        elif self._consecutive_dropped % self.DROP_WARN_EVERY == 0:
            print(f"[MarkerTracker] WARNING: {self._consecutive_dropped} consecutive "
                  f"dropped frames (latest: {reason}).")

    @property
    def stats(self) -> dict:
        """Frame/detection health counters since construction."""
        return {
            "frames_ok": self._frames_ok,
            "frames_dropped": self._frames_dropped,
            "consecutive_dropped": self._consecutive_dropped,
            "detections_rejected": self._detections_rejected,
        }

    def get_latest(self, marker_id: int) -> Optional[MarkerObservation]:
        return self._latest.get(marker_id)

    def set_T_d435_to_pelvis(self, T: np.ndarray) -> None:
        """Update the mount-to-pelvis transform in place, e.g. once the
        robot's actual live waist angles are known post-connect (the
        constructor-time value assumes zero, since it's computed before the
        robot is even connected)."""
        self._T_d435_to_pelvis = T

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
