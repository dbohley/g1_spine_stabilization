"""Frame sources for the perception pipeline.

Each source produces (frame, timestamp) tuples. `read()` returns None at EOS.
"""
import time
import cv2
import numpy as np

from g1_spine_stabilization.utils.transforms import make_se3


class ImageFileSource:
    """Single-image source, useful for tests and one-off visual checks."""

    def __init__(self, path: str):
        self._frame = cv2.imread(path)
        if self._frame is None:
            raise FileNotFoundError(f"Could not read image: {path}")
        self._consumed = False

    def read(self):
        if self._consumed:
            return None
        self._consumed = True
        return self._frame, time.monotonic()

    def close(self):
        pass


class VideoFileSource:
    """Read frames sequentially from a video file."""

    def __init__(self, path: str):
        self._cap = cv2.VideoCapture(path)
        if not self._cap.isOpened():
            raise FileNotFoundError(f"Could not open video: {path}")

    def read(self):
        ok, frame = self._cap.read()
        if not ok:
            return None
        return frame, time.monotonic()

    def close(self):
        self._cap.release()


def _extrinsics_to_se3(rotation_flat, translation) -> np.ndarray:
    """Build a 4x4 SE3 from a pyrealsense2 rs.extrinsics-shaped rotation +
    translation.

    rs.extrinsics.rotation is COLUMN-major -- confirmed directly against the
    installed pyrealsense2 package (`help(rs.extrinsics)` ->
    "Column-major 3x3 rotation matrix") and the vendored librealsense C
    header (rs_sensor.h: "Column-major 3x3 rotation matrix"). A naive
    row-major `np.array(rotation).reshape(3, 3)` silently returns the
    TRANSPOSE of the real rotation -- a valid-looking but wrong matrix.
    """
    R = np.array(rotation_flat, dtype=float).reshape(3, 3, order="F")
    t = np.array(translation, dtype=float)
    return make_se3(t, R)


class RealSenseSource:
    """Live frames from an Intel RealSense (D435 expected).

    stream="color" (default): backed by the g1_inspire_sdk RealSenseCamera.
    Lazy-imports the SDK so this module is importable on machines without
    the SDK installed. Pass dry_run=True to get synthetic gradient frames
    without a camera.

    stream="infrared": bypasses the vendored SDK entirely (it has no
    infrared/emitter support and no extension point -- see docs/followups.md)
    and talks to pyrealsense2 directly. The D435's IR/depth stereo module has
    a genuinely wider FOV (87x58 deg) than the color sensor (69x42 deg) --
    a fixed optical/hardware difference, not a resolution setting. Since the
    IR-left sensor sits at a different physical spot than the color sensor,
    marker poses detected in this mode need an extra correction on top of
    the existing (color-frame) perception.d435_link_to_optical value -- see
    `ir_to_color_extrinsic`, queried live from the camera's own factory
    calibration rather than requiring a second manual calibration.
    """

    def __init__(self, width=640, height=480, fps=30, dry_run=False, preview=None,
                 pointcloud=False, stream="color", emitter_enabled=False):
        self._stream = stream
        if stream == "color":
            from g1_inspire_sdk.camera import RealSenseCamera   # lazy
            self._cam = RealSenseCamera(width=width, height=height, fps=fps,
                                        dry_run=dry_run, preview=preview,
                                        pointcloud=pointcloud)
            self._cam.connect()
            # Remap SDK intrinsics (ppx/ppy/coeffs) to our convention (cx/cy/distortion).
            intr = self._cam.intrinsics
            self._intrinsics_native = {
                "fx": intr["fx"], "fy": intr["fy"],
                "cx": intr["ppx"], "cy": intr["ppy"],
                "distortion": list(intr["coeffs"]),
                "resolution": [intr["width"], intr["height"]],
            }
            self._ir_to_color_extrinsic = None
        elif stream == "infrared":
            self._connect_infrared(width, height, fps, dry_run, preview, emitter_enabled)
        else:
            raise ValueError(f"Unknown stream mode: {stream!r} (expected 'color' or 'infrared')")

    def _connect_infrared(self, width, height, fps, dry_run, preview, emitter_enabled):
        self._preview = preview
        if dry_run:
            self._ir_dry_run = True
            row = np.linspace(0, 255, width, dtype=np.uint8)
            self._ir_synthetic_frame = np.tile(row, (height, 1))
            self._intrinsics_native = {
                "fx": 0.9 * width, "fy": 0.9 * width,
                "cx": width / 2.0, "cy": height / 2.0,
                "distortion": [0.0, 0.0, 0.0, 0.0, 0.0],
                "resolution": [width, height],
            }
            self._ir_to_color_extrinsic = np.eye(4)
            return

        self._ir_dry_run = False
        import pyrealsense2 as rs   # lazy
        self.pipe = rs.pipeline()
        cfg = rs.config()
        cfg.enable_stream(rs.stream.infrared, 1, width, height, rs.format.y8, fps)
        cfg.enable_stream(rs.stream.color, width, height, rs.format.bgr8, fps)
        self.profile = self.pipe.start(cfg)

        device = self.profile.get_device()
        depth_sensor = device.first_depth_sensor()
        if depth_sensor.supports(rs.option.emitter_enabled):
            depth_sensor.set_option(rs.option.emitter_enabled,
                                    1.0 if emitter_enabled else 0.0)

        ir_profile = self.profile.get_stream(rs.stream.infrared, 1).as_video_stream_profile()
        color_profile = self.profile.get_stream(rs.stream.color).as_video_stream_profile()
        intr = ir_profile.get_intrinsics()
        self._intrinsics_native = {
            "fx": intr.fx, "fy": intr.fy,
            "cx": intr.ppx, "cy": intr.ppy,
            "distortion": list(intr.coeffs),
            "resolution": [intr.width, intr.height],
        }
        extrinsics = ir_profile.get_extrinsics_to(color_profile)
        self._ir_to_color_extrinsic = _extrinsics_to_se3(
            extrinsics.rotation, extrinsics.translation)

    @property
    def intrinsics_native(self) -> dict:
        """Intrinsics as queried from the camera itself (whichever stream is active)."""
        return dict(self._intrinsics_native)

    @property
    def ir_to_color_extrinsic(self):
        """4x4 SE3 mapping a pose in the IR-left optical frame into the color
        optical frame, queried live from the camera's own factory calibration
        at connect time. None when stream="color" (no correction needed --
        detection already runs in the color optical frame that
        perception.d435_link_to_optical is calibrated for)."""
        if self._ir_to_color_extrinsic is None:
            return None
        return self._ir_to_color_extrinsic.copy()

    def read(self):
        if self._stream == "color":
            frame = self._cam.capture()
            return frame["color"], frame["timestamp"]
        # infrared
        if self._ir_dry_run:
            return self._ir_synthetic_frame, time.monotonic()
        frames = self.pipe.wait_for_frames()
        ir = frames.get_infrared_frame(1)
        if not ir:
            raise RuntimeError("missing infrared frame")
        frame = np.asanyarray(ir.get_data())
        if self._preview is not None:
            fn = getattr(self._preview, "update_camera", None)
            if fn is not None:
                try:
                    fn(color=frame)
                except Exception:
                    pass
        return frame, time.monotonic()

    def close(self):
        if self._stream == "color":
            self._cam.shutdown()
        elif not self._ir_dry_run:
            self.pipe.stop()
