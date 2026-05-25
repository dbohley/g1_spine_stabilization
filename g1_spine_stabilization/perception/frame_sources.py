"""Frame sources for the perception pipeline.

Each source produces (frame, timestamp) tuples. `read()` returns None at EOS.
"""
import time
import cv2
import numpy as np


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


class RealSenseSource:
    """Live frames from an Intel RealSense (D435 expected).

    Lazy-imports pyrealsense2 so this module is importable on machines
    without the librealsense stack.
    """

    def __init__(self, width=640, height=480, fps=30):
        import pyrealsense2 as rs   # lazy
        self._rs = rs
        self._pipeline = rs.pipeline()
        cfg = rs.config()
        cfg.enable_stream(rs.stream.color, width, height, rs.format.bgr8, fps)
        profile = self._pipeline.start(cfg)
        # Cache the intrinsics so the controller can ask for them.
        color_profile = profile.get_stream(rs.stream.color)
        intr = color_profile.as_video_stream_profile().get_intrinsics()
        self._intrinsics_native = {
            "fx": intr.fx, "fy": intr.fy, "cx": intr.ppx, "cy": intr.ppy,
            "distortion": list(intr.coeffs),
            "resolution": [intr.width, intr.height],
        }

    @property
    def intrinsics_native(self) -> dict:
        """Intrinsics as queried from the camera itself."""
        return dict(self._intrinsics_native)

    def read(self):
        frames = self._pipeline.wait_for_frames()
        color = frames.get_color_frame()
        if not color:
            return None
        return np.asanyarray(color.get_data()), time.monotonic()

    def close(self):
        self._pipeline.stop()
