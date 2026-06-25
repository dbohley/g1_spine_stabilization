"""Frame sources for the perception pipeline.

Each source produces (frame, timestamp) tuples. `read()` returns None at EOS.
"""
import time
import cv2


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

    Backed by the g1_inspire_sdk RealSenseCamera. Lazy-imports the SDK so
    this module is importable on machines without the SDK installed.
    Pass dry_run=True to get synthetic gradient frames without a camera.
    """

    def __init__(self, width=640, height=480, fps=30, dry_run=False):
        from g1_inspire_sdk.camera import RealSenseCamera   # lazy
        self._cam = RealSenseCamera(width=width, height=height, fps=fps, dry_run=dry_run)
        self._cam.connect()
        # Remap SDK intrinsics (ppx/ppy/coeffs) to our convention (cx/cy/distortion).
        intr = self._cam.intrinsics
        self._intrinsics_native = {
            "fx": intr["fx"], "fy": intr["fy"],
            "cx": intr["ppx"], "cy": intr["ppy"],
            "distortion": list(intr["coeffs"]),
            "resolution": [intr["width"], intr["height"]],
        }

    @property
    def intrinsics_native(self) -> dict:
        """Intrinsics as queried from the camera itself."""
        return dict(self._intrinsics_native)

    def read(self):
        frame = self._cam.capture()
        return frame["color"], frame["timestamp"]

    def close(self):
        self._cam.shutdown()
