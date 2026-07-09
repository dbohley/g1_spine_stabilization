"""Pure math for calibrating T_optical_to_d435 from a known marker position.

Given a marker placed at a physically-measured, known pose in the pelvis
frame (T_marker_true_pelvis) and the raw camera-frame pose the perception
pipeline actually reads for it (pose_in_optical), solve for the
T_optical_to_d435 correction that would make the two agree:

    pose_in_pelvis = T_d435_to_pelvis @ T_optical_to_d435 @ pose_in_optical
    =>  T_optical_to_d435 = T_d435_to_pelvis^-1 @ T_marker_true_pelvis @ pose_in_optical^-1

T_d435_to_pelvis (from Pinocchio FK against the URDF) is trusted and not
solved for here. No hardware or I/O in this module -- see
scripts/calibrate_camera_extrinsics.py for the camera-facing wrapper.
"""
import numpy as np
from scipy.spatial.transform import Rotation


def solve_optical_to_d435(
    T_d435_to_pelvis: np.ndarray,
    T_marker_true_pelvis: np.ndarray,
    pose_in_optical: np.ndarray,
) -> np.ndarray:
    """Closed-form solve for the T_optical_to_d435 that reconciles a known
    ground-truth marker pose with what the camera actually reports for it."""
    return (
        np.linalg.inv(T_d435_to_pelvis)
        @ T_marker_true_pelvis
        @ np.linalg.inv(pose_in_optical)
    )


def se3_to_xyz_rpy(T: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Decompose a 4x4 SE3 into (xyz, rpy), inverse of the construction in
    perception/frames.py's optical_to_d435_from_config (URDF/ROS fixed-axis
    convention: R = Rz(yaw) @ Ry(pitch) @ Rx(roll), scipy 'ZYX' extrinsic)."""
    xyz = T[:3, 3].copy()
    yaw, pitch, roll = Rotation.from_matrix(T[:3, :3]).as_euler("ZYX")
    rpy = np.array([roll, pitch, yaw])
    return xyz, rpy


def pose_error(T_a: np.ndarray, T_b: np.ndarray) -> tuple[float, float]:
    """Position error (m) and rotation error (deg) between two SE3 poses.

    Same rotation-error construction already used in sdk_pose_tracking.py's
    _log_marker_info (R_diff = R_a.T @ R_b, angle from the rotvec norm).
    """
    pos_err = float(np.linalg.norm(T_a[:3, 3] - T_b[:3, 3]))
    R_diff = T_a[:3, :3].T @ T_b[:3, :3]
    rot_err = float(np.degrees(np.linalg.norm(Rotation.from_matrix(R_diff).as_rotvec())))
    return pos_err, rot_err
