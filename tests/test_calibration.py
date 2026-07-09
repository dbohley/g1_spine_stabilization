"""Hardware-free tests for perception/calibration.py's pure math."""
import numpy as np
from scipy.spatial.transform import Rotation

from g1_spine_stabilization.perception.calibration import (
    pose_error, se3_to_xyz_rpy, solve_optical_to_d435,
)
from g1_spine_stabilization.perception.frames import optical_to_d435_from_config
from g1_spine_stabilization.utils.transforms import make_se3


def test_solve_optical_to_d435_recovers_known_transform():
    """Given a known 'true' T_optical_to_d435 and the synthetic pose_in_optical
    a perfect camera would report for a marker at a known ground-truth pose,
    the solver must recover the original transform."""
    rng = np.random.default_rng(0)

    T_true_optical_to_d435 = make_se3(
        np.array([0.02, -0.01, 0.09]),
        Rotation.from_euler("xyz", [-1.55, 0.03, -1.57]).as_matrix(),
    )
    T_d435_to_pelvis = make_se3(
        np.array([0.1, 0.0, 0.5]),
        Rotation.from_euler("xyz", [0.1, -0.2, 0.05]).as_matrix(),
    )
    T_marker_true_pelvis = make_se3(np.array([0.4, 0.0, 0.0]), np.eye(3))

    # What a perfect camera would report, inverting the known chain:
    # pose_in_pelvis = T_d435_to_pelvis @ T_true_optical_to_d435 @ pose_in_optical
    pose_in_optical = (
        np.linalg.inv(T_true_optical_to_d435)
        @ np.linalg.inv(T_d435_to_pelvis)
        @ T_marker_true_pelvis
    )

    T_recovered = solve_optical_to_d435(T_d435_to_pelvis, T_marker_true_pelvis, pose_in_optical)

    np.testing.assert_allclose(T_recovered, T_true_optical_to_d435, atol=1e-9)


def test_se3_to_xyz_rpy_round_trips_against_optical_to_d435_from_config():
    xyz_in = [0.02, -0.01, 0.09]
    rpy_in = [-1.5498, 0.0134, -1.5601]
    T = optical_to_d435_from_config({"xyz": xyz_in, "rpy": rpy_in})

    xyz_out, rpy_out = se3_to_xyz_rpy(T)

    np.testing.assert_allclose(xyz_out, xyz_in, atol=1e-9)
    np.testing.assert_allclose(rpy_out, rpy_in, atol=1e-6)

    # Rebuilding from the decomposed values must reproduce the same SE3.
    T_rebuilt = optical_to_d435_from_config({"xyz": xyz_out.tolist(), "rpy": rpy_out.tolist()})
    np.testing.assert_allclose(T_rebuilt, T, atol=1e-9)


def test_pose_error_zero_for_identical_poses():
    T = make_se3(np.array([0.3, 0.1, 0.2]), Rotation.from_euler("xyz", [0.1, 0.2, 0.3]).as_matrix())
    pos_err, rot_err = pose_error(T, T)
    assert pos_err < 1e-9
    assert rot_err < 1e-9


def test_pose_error_matches_known_position_offset():
    T_a = make_se3(np.array([0.0, 0.0, 0.0]), np.eye(3))
    T_b = make_se3(np.array([0.10, 0.0, 0.0]), np.eye(3))
    pos_err, rot_err = pose_error(T_a, T_b)
    assert abs(pos_err - 0.10) < 1e-9
    assert rot_err < 1e-9


def test_pose_error_matches_known_rotation_offset():
    T_a = make_se3(np.array([0.0, 0.0, 0.0]), np.eye(3))
    R_b = Rotation.from_euler("z", 10, degrees=True).as_matrix()
    T_b = make_se3(np.array([0.0, 0.0, 0.0]), R_b)
    pos_err, rot_err = pose_error(T_a, T_b)
    assert pos_err < 1e-9
    assert abs(rot_err - 10.0) < 1e-6
