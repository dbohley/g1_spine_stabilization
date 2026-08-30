"""Tests for MarkerTracker (real tracker: frame_source + detector +
camera->pelvis composition). Hardware-free -- uses a stub frame_source and
stub detector, no real camera or cv2 detection. (MockMarkerTracker itself
is tested separately in test_mock_tracker.py.)"""
import numpy as np

from g1_spine_stabilization.perception.detector import RawDetection
from g1_spine_stabilization.perception.tracker import MarkerTracker


class _StubFrameSource:
    def __init__(self):
        self._t = 0.0

    def read(self):
        self._t += 0.1
        return np.zeros((4, 4, 3), dtype=np.uint8), self._t


class _StubDetector:
    def __init__(self, pose_in_optical, marker_id=0):
        self._pose_in_optical = pose_in_optical
        self._marker_id = marker_id

    def detect(self, frame, intrinsics):
        return [RawDetection(
            marker_id=self._marker_id,
            pose_in_optical=self._pose_in_optical,
            corners=np.zeros((4, 2)),
            reprojection_error=0.3,
        )]


def test_marker_tracker_step_populates_pose_in_optical():
    """MarkerObservation.pose_in_optical must match the raw detection's
    pose_in_optical exactly -- this is what lets RunRecorder compute
    angular velocity from the camera frame without pelvis composition."""
    pose_optical = np.eye(4)
    pose_optical[:3, 3] = [0.1, 0.2, 0.3]
    detector = _StubDetector(pose_optical)
    tracker = MarkerTracker(
        _StubFrameSource(), detector, intrinsics=None,
        T_optical_to_d435=np.eye(4), T_d435_to_pelvis=np.eye(4),
    )

    observations = tracker.step()

    assert len(observations) == 1
    obs = observations[0]
    np.testing.assert_allclose(obs.pose_in_optical, pose_optical)


def test_marker_tracker_get_latest_includes_pose_in_optical():
    pose_optical = np.eye(4)
    pose_optical[:3, 3] = [0.4, 0.0, 0.0]
    detector = _StubDetector(pose_optical, marker_id=2)
    tracker = MarkerTracker(
        _StubFrameSource(), detector, intrinsics=None,
        T_optical_to_d435=np.eye(4), T_d435_to_pelvis=np.eye(4),
    )

    tracker.step()

    obs = tracker.get_latest(2)
    assert obs is not None
    np.testing.assert_allclose(obs.pose_in_optical, pose_optical)


def test_set_t_d435_to_pelvis_updates_subsequent_steps():
    """set_T_d435_to_pelvis() must take effect on the next step() -- used to
    correct T_d435_to_pelvis once a robot's real, live waist angle is known
    post-connect (the constructor-time value assumes zero-waist)."""
    pose_optical = np.eye(4)
    pose_optical[:3, 3] = [1.0, 0.0, 0.0]
    detector = _StubDetector(pose_optical)
    tracker = MarkerTracker(
        _StubFrameSource(), detector, intrinsics=None,
        T_optical_to_d435=np.eye(4), T_d435_to_pelvis=np.eye(4),
    )

    obs_before = tracker.step()[0]
    np.testing.assert_allclose(obs_before.pose_in_pelvis[:3, 3], [1.0, 0.0, 0.0])

    T_shifted = np.eye(4)
    T_shifted[:3, 3] = [0.0, 0.0, 0.5]   # e.g. a corrected pelvis-to-mount offset
    tracker.set_T_d435_to_pelvis(T_shifted)

    obs_after = tracker.step()[0]
    np.testing.assert_allclose(obs_after.pose_in_pelvis[:3, 3], [1.0, 0.0, 0.5])


def test_pelvis_offset_hotfix_applied_after_full_composition():
    """pelvis_offset is added directly in pelvis frame, after T_d435_to_pelvis
    @ T_optical_to_d435 @ pose_in_optical -- an empirical, reversible probe
    (configs/default.yaml perception.pelvis_offset_hotfix), distinct from the
    real T_optical_to_d435 calibration."""
    pose_optical = np.eye(4)
    pose_optical[:3, 3] = [1.0, 2.0, 3.0]
    detector = _StubDetector(pose_optical)
    tracker = MarkerTracker(
        _StubFrameSource(), detector, intrinsics=None,
        T_optical_to_d435=np.eye(4), T_d435_to_pelvis=np.eye(4),
        pelvis_offset=[0.0, 0.05, 0.0],
    )

    obs = tracker.step()[0]
    np.testing.assert_allclose(obs.pose_in_pelvis[:3, 3], [1.0, 2.05, 3.0])


def test_no_pelvis_offset_by_default():
    """Omitting pelvis_offset must reproduce the exact prior behavior --
    every existing caller relies on this."""
    pose_optical = np.eye(4)
    pose_optical[:3, 3] = [1.0, 2.0, 3.0]
    detector = _StubDetector(pose_optical)
    tracker = MarkerTracker(
        _StubFrameSource(), detector, intrinsics=None,
        T_optical_to_d435=np.eye(4), T_d435_to_pelvis=np.eye(4),
    )

    obs = tracker.step()[0]
    np.testing.assert_allclose(obs.pose_in_pelvis[:3, 3], [1.0, 2.0, 3.0])
