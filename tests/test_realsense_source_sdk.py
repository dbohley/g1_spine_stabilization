"""Tests for the SDK-backed RealSenseSource.

All tests use dry_run=True so no camera hardware is required.
"""
import numpy as np
import pytest

from g1_spine_stabilization.perception.frame_sources import RealSenseSource
from g1_spine_stabilization.perception.detector import AprilTagDetector, CameraIntrinsics


def _make_source():
    return RealSenseSource(width=640, height=480, fps=30, dry_run=True)


def test_dry_run_read_returns_frame_and_timestamp():
    src = _make_source()
    result = src.read()
    assert result is not None
    frame, ts = result
    assert isinstance(frame, np.ndarray)
    assert frame.dtype == np.uint8
    assert frame.shape == (480, 640, 3)
    assert isinstance(ts, float)
    src.close()


def test_intrinsics_native_has_required_keys():
    src = _make_source()
    intr = src.intrinsics_native
    for key in ("fx", "fy", "cx", "cy", "distortion", "resolution"):
        assert key in intr, f"missing key: {key}"
    assert len(intr["distortion"]) == 5
    assert intr["resolution"] == [640, 480]
    assert intr["fx"] > 0 and intr["fy"] > 0
    src.close()


def test_intrinsics_native_returns_copy():
    src = _make_source()
    a = src.intrinsics_native
    b = src.intrinsics_native
    assert a is not b
    src.close()


def test_close_does_not_raise():
    src = _make_source()
    src.read()
    src.close()  # should not raise


def test_second_instance_after_close():
    src1 = _make_source()
    src1.close()
    src2 = _make_source()
    frame, _ = src2.read()
    assert frame.shape == (480, 640, 3)
    src2.close()


def test_detector_smoke_on_dry_run_frame():
    src = _make_source()
    intr_dict = src.intrinsics_native
    intrinsics = CameraIntrinsics(
        fx=intr_dict["fx"], fy=intr_dict["fy"],
        cx=intr_dict["cx"], cy=intr_dict["cy"],
        distortion=np.asarray(intr_dict["distortion"]),
    )
    detector = AprilTagDetector("DICT_APRILTAG_36h11", marker_size_m=0.04)
    frame, _ = src.read()
    detections = detector.detect(frame, intrinsics)
    # Synthetic gradient frame has no real markers — expect empty list, no exception.
    assert isinstance(detections, list)
    src.close()
