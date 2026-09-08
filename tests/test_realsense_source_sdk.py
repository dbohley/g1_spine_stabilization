"""Tests for the SDK-backed RealSenseSource.

All tests use dry_run=True so no camera hardware is required.
"""
import numpy as np
import pytest

from g1_spine_stabilization.perception.frame_sources import RealSenseSource, _extrinsics_to_se3
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


def test_non_default_resolution_flows_through_to_frame_and_intrinsics():
    src = RealSenseSource(width=1280, height=720, fps=30, dry_run=True)
    frame, _ = src.read()
    assert frame.shape == (720, 1280, 3)
    assert src.intrinsics_native["resolution"] == [1280, 720]
    src.close()


def test_infrared_dry_run_read_returns_2d_frame():
    src = RealSenseSource(width=640, height=480, fps=30, dry_run=True, stream="infrared")
    frame, ts = src.read()
    assert isinstance(frame, np.ndarray)
    assert frame.dtype == np.uint8
    assert frame.shape == (480, 640)   # single-channel -- no 3rd dim
    assert isinstance(ts, float)
    src.close()


def test_infrared_dry_run_intrinsics_reflect_resolution():
    src = RealSenseSource(width=640, height=480, fps=30, dry_run=True, stream="infrared")
    intr = src.intrinsics_native
    assert intr["resolution"] == [640, 480]
    assert intr["fx"] > 0 and intr["fy"] > 0
    src.close()


def test_color_mode_has_no_ir_extrinsic():
    src = _make_source()
    assert src.ir_to_color_extrinsic is None
    src.close()


def test_infrared_mode_extrinsic_is_valid_se3():
    src = RealSenseSource(width=640, height=480, fps=30, dry_run=True, stream="infrared")
    T = src.ir_to_color_extrinsic
    assert T is not None
    assert T.shape == (4, 4)
    R = T[:3, :3]
    np.testing.assert_allclose(R @ R.T, np.eye(3), atol=1e-8)
    np.testing.assert_allclose(T[3, :], [0, 0, 0, 1])
    src.close()


def test_unknown_stream_mode_raises():
    with pytest.raises(ValueError):
        RealSenseSource(dry_run=True, stream="depth")


def test_extrinsics_to_se3_reshapes_column_major():
    # Deliberately asymmetric so row-major vs. column-major reshaping give
    # different, distinguishable results -- pyrealsense2's rs.extrinsics.
    # rotation is column-major (confirmed against the installed package and
    # the librealsense C header); a naive row-major reshape would silently
    # return the transpose of R_true instead.
    R_true = np.array([
        [1.0, 2.0, 3.0],
        [4.0, 5.0, 6.0],
        [7.0, 8.0, 9.0],
    ])
    t_true = [0.01, -0.02, 0.03]
    rotation_flat = list(R_true.flatten(order="F"))
    T = _extrinsics_to_se3(rotation_flat, t_true)
    np.testing.assert_allclose(T[:3, :3], R_true)
    np.testing.assert_allclose(T[:3, 3], t_true)


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
