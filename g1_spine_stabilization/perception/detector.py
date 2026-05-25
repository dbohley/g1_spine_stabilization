"""AprilTag detector wrapping cv2.aruco + cv2.solvePnP."""
from dataclasses import dataclass
import cv2
import numpy as np


@dataclass
class CameraIntrinsics:
    fx: float
    fy: float
    cx: float
    cy: float
    distortion: np.ndarray   # shape (5,) for plumb-bob

    @property
    def camera_matrix(self) -> np.ndarray:
        return np.array([
            [self.fx, 0.0,     self.cx],
            [0.0,     self.fy, self.cy],
            [0.0,     0.0,     1.0    ],
        ], dtype=np.float64)


@dataclass
class RawDetection:
    """A detected marker before pelvis-frame composition."""
    marker_id: int
    pose_in_optical: np.ndarray   # 4x4 SE3, optical frame
    corners: np.ndarray           # shape (4, 2), pixels — for visualization
    reprojection_error: float     # pixels


class AprilTagDetector:
    """Detect AprilTag markers and recover their 6-DoF pose via solvePnP."""

    def __init__(self, dict_name: str, marker_size_m: float):
        # dict_name is the string version of the cv2.aruco constant, e.g.
        # "DICT_APRILTAG_36h11". Look it up via getattr for config flexibility.
        if not hasattr(cv2.aruco, dict_name):
            raise ValueError(f"Unknown ArUco dictionary: {dict_name}")
        dict_id = getattr(cv2.aruco, dict_name)
        self._dictionary = cv2.aruco.getPredefinedDictionary(dict_id)
        self._params = cv2.aruco.DetectorParameters()
        self._detector = cv2.aruco.ArucoDetector(self._dictionary, self._params)
        self._marker_size = marker_size_m

        # Object points in marker frame (Z out of marker, X right, Y up).
        # Corner order must match what cv2.aruco returns:
        # top-left, top-right, bottom-right, bottom-left.
        half = marker_size_m / 2.0
        self._object_points = np.array([
            [-half,  half, 0.0],
            [ half,  half, 0.0],
            [ half, -half, 0.0],
            [-half, -half, 0.0],
        ], dtype=np.float32)

    def detect(self, frame: np.ndarray, intrinsics: CameraIntrinsics) -> list:
        """Return a list of RawDetection for every recognized marker in the frame."""
        if frame.ndim == 3:
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        else:
            gray = frame

        corners, ids, _rejected = self._detector.detectMarkers(gray)
        if ids is None:
            return []

        results = []
        K = intrinsics.camera_matrix
        D = intrinsics.distortion.astype(np.float64)
        for i, marker_id in enumerate(ids.flatten()):
            image_points = corners[i][0].astype(np.float32)
            ok, rvec, tvec = cv2.solvePnP(
                self._object_points, image_points, K, D,
                flags=cv2.SOLVEPNP_IPPE_SQUARE,
            )
            if not ok:
                continue
            R, _ = cv2.Rodrigues(rvec)
            T = np.eye(4)
            T[:3, :3] = R
            T[:3, 3]  = tvec.flatten()

            # Reprojection error
            projected, _ = cv2.projectPoints(self._object_points, rvec, tvec, K, D)
            err = float(np.linalg.norm(projected.reshape(-1, 2) - image_points, axis=1).mean())

            results.append(RawDetection(
                marker_id=int(marker_id),
                pose_in_optical=T,
                corners=image_points,
                reprojection_error=err,
            ))
        return results
