"""Detector tests using synthesized AprilTag images.
No hardware required."""
import numpy as np
import pytest

cv2 = pytest.importorskip("cv2")

from g1_spine_stabilization.perception.detector import (
    AprilTagDetector, CameraIntrinsics
)


def _synthesize_marker_frame(marker_id: int, dict_name: str, size_px: int,
                             frame_shape=(480, 640)):
    """Render an AprilTag at the center of a blank frame."""
    dict_id = getattr(cv2.aruco, dict_name)
    dictionary = cv2.aruco.getPredefinedDictionary(dict_id)
    marker_img = cv2.aruco.generateImageMarker(dictionary, marker_id, size_px)

    frame = np.full((frame_shape[0], frame_shape[1], 3), 255, dtype=np.uint8)
    cy, cx = frame_shape[0] // 2, frame_shape[1] // 2
    half = size_px // 2
    marker_bgr = cv2.cvtColor(marker_img, cv2.COLOR_GRAY2BGR)
    frame[cy - half : cy + half, cx - half : cx + half] = marker_bgr
    return frame


def _synthesize_marker_frame_grayscale(marker_id: int, dict_name: str, size_px: int,
                                       frame_shape=(480, 640)):
    """Like _synthesize_marker_frame but returns a genuine single-channel
    (H, W) frame -- exercises detector.py's `frame.ndim == 2` branch
    directly, the path a raw infrared (y8) frame takes."""
    dict_id = getattr(cv2.aruco, dict_name)
    dictionary = cv2.aruco.getPredefinedDictionary(dict_id)
    marker_img = cv2.aruco.generateImageMarker(dictionary, marker_id, size_px)

    frame = np.full(frame_shape, 255, dtype=np.uint8)
    cy, cx = frame_shape[0] // 2, frame_shape[1] // 2
    half = size_px // 2
    frame[cy - half : cy + half, cx - half : cx + half] = marker_img
    return frame


@pytest.fixture
def detector():
    return AprilTagDetector("DICT_APRILTAG_36h11", marker_size_m=0.04)


@pytest.fixture
def intrinsics():
    return CameraIntrinsics(
        fx=615.0, fy=615.0, cx=320.0, cy=240.0,
        distortion=np.zeros(5),
    )


def test_detector_finds_synthesized_marker(detector, intrinsics):
    frame = _synthesize_marker_frame(0, "DICT_APRILTAG_36h11", size_px=200)
    detections = detector.detect(frame, intrinsics)
    assert len(detections) == 1
    assert detections[0].marker_id == 0


def test_detector_recovers_id_correctly(detector, intrinsics):
    frame = _synthesize_marker_frame(7, "DICT_APRILTAG_36h11", size_px=200)
    detections = detector.detect(frame, intrinsics)
    assert len(detections) == 1
    assert detections[0].marker_id == 7


def test_detector_reprojection_error_is_small(detector, intrinsics):
    """A well-resolved synthetic marker should have sub-pixel reprojection error."""
    frame = _synthesize_marker_frame(0, "DICT_APRILTAG_36h11", size_px=200)
    detections = detector.detect(frame, intrinsics)
    assert len(detections) == 1
    assert detections[0].reprojection_error < 1.0


def test_detector_returns_pose_with_positive_z(detector, intrinsics):
    """The marker is in front of the camera, so the recovered tvec.z should be > 0
    in optical-frame convention (Z forward into the scene)."""
    frame = _synthesize_marker_frame(0, "DICT_APRILTAG_36h11", size_px=200)
    detections = detector.detect(frame, intrinsics)
    assert detections[0].pose_in_optical[2, 3] > 0.0


def test_detector_finds_synthesized_marker_on_grayscale_frame(detector, intrinsics):
    """A genuine (H, W) single-channel frame -- the shape a raw infrared
    (y8) frame comes back as -- must detect identically to the 3-channel
    path, with no conversion needed by the caller."""
    frame = _synthesize_marker_frame_grayscale(0, "DICT_APRILTAG_36h11", size_px=200)
    assert frame.ndim == 2
    detections = detector.detect(frame, intrinsics)
    assert len(detections) == 1
    assert detections[0].marker_id == 0


def test_detector_returns_empty_on_blank_frame(detector, intrinsics):
    frame = np.full((480, 640, 3), 255, dtype=np.uint8)
    assert detector.detect(frame, intrinsics) == []
