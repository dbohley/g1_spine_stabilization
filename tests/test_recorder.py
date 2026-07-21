"""Tests for the shared robot/human trial-data recorder.

Hardware-free -- uses hand-built MarkerObservations, no camera or robot.
"""
import os
import pickle
import re

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from g1_spine_stabilization.data_recording.recorder import (
    MarkerSample,
    RunRecorder,
    angle_diff_rpy,
    angular_velocity_rotvec,
    default_run_path,
)
from g1_spine_stabilization.perception.types import MarkerObservation


def _obs(marker_id, R=None, pos=(0.0, 0.0, 0.0), timestamp=0.0):
    pose = np.eye(4)
    if R is not None:
        pose[:3, :3] = R
    pose[:3, 3] = pos
    return MarkerObservation(
        marker_id=marker_id, pose_in_pelvis=pose,
        timestamp=timestamp, reprojection_error=0.5,
    )


# ---------------------------------------------------------------------
# angle_diff_rpy
# ---------------------------------------------------------------------

def test_angle_diff_rpy_known_rotation():
    R_reference = np.eye(3)
    R_moving = Rotation.from_euler('xyz', [0.1, 0.2, 0.3]).as_matrix()
    result = angle_diff_rpy(R_reference, R_moving)
    np.testing.assert_allclose(result, [0.1, 0.2, 0.3], atol=1e-9)


def test_angle_diff_rpy_is_relative_not_subtracted_eulers():
    R_reference = Rotation.from_euler('xyz', [0.3, -0.5, 0.8]).as_matrix()
    R_moving = Rotation.from_euler('xyz', [1.2, 0.4, -0.6]).as_matrix()

    result = angle_diff_rpy(R_reference, R_moving)
    expected_relative = Rotation.from_matrix(R_reference.T @ R_moving).as_euler('xyz')
    np.testing.assert_allclose(result, expected_relative, atol=1e-9)

    naive_subtracted = (
        Rotation.from_matrix(R_moving).as_euler('xyz')
        - Rotation.from_matrix(R_reference).as_euler('xyz')
    )
    assert not np.allclose(result, naive_subtracted, atol=1e-3)


def test_angle_diff_rpy_argument_order_is_not_a_sign_flip():
    """Swapping which matrix is the reference is a real modeling choice, not
    cosmetic -- angle_diff_rpy(A, B) is NOT simply -angle_diff_rpy(B, A) for
    non-trivial rotations (only approximately true for small angles), since
    each decomposes in a DIFFERENT frame (whichever argument is first).
    This is the exact swap RunRecorder.record() relies on: target passed as
    reference, object as moving (see recorder.py's module docstring)."""
    R_a = Rotation.from_euler('xyz', [0.9, -0.6, 1.1]).as_matrix()
    R_b = Rotation.from_euler('xyz', [-0.4, 1.3, 0.2]).as_matrix()

    forward = angle_diff_rpy(R_a, R_b)
    backward = angle_diff_rpy(R_b, R_a)

    assert not np.allclose(forward, -backward, atol=1e-3)
    # Sanity: backward really does match decomposing in R_b's own frame.
    np.testing.assert_allclose(
        backward, Rotation.from_matrix(R_b.T @ R_a).as_euler('xyz'), atol=1e-9
    )


# ---------------------------------------------------------------------
# angular_velocity_rotvec
# ---------------------------------------------------------------------

def test_angular_velocity_finite_difference_known_rate():
    omega = 0.5   # rad/s about Z
    dt = 0.1
    R_prev = np.eye(3)
    R_cur = Rotation.from_rotvec([0, 0, omega * dt]).as_matrix()
    result = angular_velocity_rotvec(R_prev, 0.0, R_cur, dt)
    np.testing.assert_allclose(result, [0.0, 0.0, omega], atol=1e-6)


def test_angular_velocity_dt_zero_guard():
    R = np.eye(3)
    assert angular_velocity_rotvec(R, 1.0, R, 1.0) is None


def test_angular_velocity_dt_negative_guard():
    R = np.eye(3)
    assert angular_velocity_rotvec(R, 1.0, R, 0.5) is None


# ---------------------------------------------------------------------
# RunRecorder.record()
# ---------------------------------------------------------------------

def test_recorder_first_sample_has_no_velocity():
    rec = RunRecorder(origin="robot", source_label="mock", output_path="unused.pkl")
    obs = _obs(0, timestamp=0.0)
    sample = rec.record(t=0.0, object_obs=obs, target_obs=obs)
    assert sample.object_angular_velocity is None
    assert sample.target_angular_velocity is None


def test_recorder_second_sample_computes_velocity():
    rec = RunRecorder(origin="robot", source_label="mock", output_path="unused.pkl")
    omega = 1.0
    dt = 0.05
    R0 = np.eye(3)
    R1 = Rotation.from_rotvec([0, 0, omega * dt]).as_matrix()
    obs0 = _obs(0, R=R0, timestamp=0.0)
    obs1 = _obs(0, R=R1, timestamp=dt)

    rec.record(t=0.0, object_obs=obs0, target_obs=None)
    sample = rec.record(t=dt, object_obs=obs1, target_obs=None)

    assert sample.object_angular_velocity is not None
    np.testing.assert_allclose(sample.object_angular_velocity, [0.0, 0.0, omega], atol=1e-6)
    assert sample.target_angular_velocity is None


def test_recorder_handles_none_observations():
    rec = RunRecorder(origin="robot", source_label="sine", output_path="unused.pkl")
    sample = rec.record(t=0.0, object_obs=None, target_obs=None, theta=0.3)
    assert sample.object_pose is None
    assert sample.target_pose is None
    assert sample.angle_diff_rpy is None
    assert sample.object_angular_velocity is None
    assert sample.target_angular_velocity is None
    assert sample.theta == 0.3
    assert len(rec) == 1


def test_recorder_angle_diff_only_when_both_markers_present():
    rec = RunRecorder(origin="robot", source_label="mock", output_path="unused.pkl")
    obj_obs = _obs(0, R=np.eye(3), timestamp=0.0)
    sample = rec.record(t=0.0, object_obs=obj_obs, target_obs=None)
    assert sample.angle_diff_rpy is None


def test_recorder_angle_diff_anchored_to_target_not_object():
    """RunRecorder.record() must pass target as the reference (biomechanics
    convention: torso, in a neck trial) and object as moving (head) -- not
    the reverse. Uses non-trivial rotations so the two conventions produce
    measurably different numbers (see test_angle_diff_rpy_argument_order_
    is_not_a_sign_flip)."""
    rec = RunRecorder(origin="human", source_label="mock", output_path="unused.pkl")
    R_head = Rotation.from_euler('xyz', [0.9, -0.6, 1.1]).as_matrix()
    R_torso = Rotation.from_euler('xyz', [-0.4, 1.3, 0.2]).as_matrix()
    obj_obs = _obs(0, R=R_head, timestamp=0.0)      # object_marker_id = head
    tgt_obs = _obs(2, R=R_torso, timestamp=0.0)      # target_marker_id = torso

    sample = rec.record(t=0.0, object_obs=obj_obs, target_obs=tgt_obs)

    np.testing.assert_allclose(
        sample.angle_diff_rpy, angle_diff_rpy(R_torso, R_head), atol=1e-9
    )
    assert not np.allclose(sample.angle_diff_rpy, angle_diff_rpy(R_head, R_torso), atol=1e-3)


# ---------------------------------------------------------------------
# RunRecorder.save() -- pickle round trip, idempotency, directory creation
# ---------------------------------------------------------------------

def test_recorder_pickle_round_trip(tmp_path):
    out_path = str(tmp_path / "run.pkl")
    rec = RunRecorder(
        origin="human", source_label="human", tracking_mode="orient-only",
        output_path=out_path, object_marker_id=0, target_marker_id=2,
        metadata={"note": "test"},
    )
    obj_obs = _obs(0, R=np.eye(3), pos=(0.1, 0.2, 0.3), timestamp=0.0)
    tgt_obs = _obs(2, R=Rotation.from_euler('xyz', [0.1, 0, 0]).as_matrix(),
                   pos=(0.4, 0.0, 0.3), timestamp=0.0)
    rec.record(t=0.0, object_obs=None, target_obs=None)
    rec.record(t=0.02, object_obs=obj_obs, target_obs=tgt_obs)
    rec.save()

    with open(out_path, "rb") as f:
        payload = pickle.load(f)

    assert payload["schema_version"] == RunRecorder.SCHEMA_VERSION
    assert payload["origin"] == "human"
    assert payload["source_label"] == "human"
    assert payload["tracking_mode"] == "orient-only"
    assert payload["object_marker_id"] == 0
    assert payload["target_marker_id"] == 2
    assert payload["metadata"] == {"note": "test"}
    assert len(payload["samples"]) == 2

    s0, s1 = payload["samples"]
    assert isinstance(s0, MarkerSample)
    assert s0.object_pose is None
    np.testing.assert_allclose(s1.object_pose, obj_obs.pose_in_pelvis)
    np.testing.assert_allclose(s1.target_pose, tgt_obs.pose_in_pelvis)
    # target is the reference (biomechanics-style: torso/tracking-goal frame),
    # object is the moving segment -- see recorder.py's module docstring.
    np.testing.assert_allclose(
        s1.angle_diff_rpy,
        angle_diff_rpy(tgt_obs.pose_in_pelvis[:3, :3], obj_obs.pose_in_pelvis[:3, :3]),
    )


def test_recorder_save_is_idempotent(tmp_path):
    out_path = str(tmp_path / "run.pkl")
    rec = RunRecorder(origin="robot", source_label="mock", output_path=out_path)
    rec.record(t=0.0, object_obs=None, target_obs=None)
    first_path = rec.save()
    rec.record(t=0.02, object_obs=None, target_obs=None)
    second_path = rec.save()

    assert first_path == second_path
    with open(out_path, "rb") as f:
        payload = pickle.load(f)
    assert len(payload["samples"]) == 1


def test_recorder_save_creates_nested_output_dir(tmp_path):
    out_path = str(tmp_path / "nested" / "dir" / "run.pkl")
    rec = RunRecorder(origin="robot", source_label="mock", output_path=out_path)
    rec.record(t=0.0, object_obs=None, target_obs=None)
    rec.save()
    assert os.path.exists(out_path)


# ---------------------------------------------------------------------
# default_run_path
# ---------------------------------------------------------------------

def test_default_run_path_filename_format(tmp_path):
    path = default_run_path("marker", "6dof", origin="robot", output_dir=str(tmp_path))
    assert re.search(r"robot_run_\d{8}_\d{6}_marker_6dof\.pkl$", path)
    assert path.startswith(str(tmp_path))


def test_default_run_path_omits_tracking_mode_when_none(tmp_path):
    path = default_run_path("sine", None, origin="robot", output_dir=str(tmp_path))
    basename = os.path.basename(path)
    assert "_None" not in basename
    assert "__" not in basename
    assert not basename.startswith("_")
    assert re.search(r"robot_run_\d{8}_\d{6}_sine\.pkl$", path)


def test_default_run_path_prefixes_human_origin(tmp_path):
    path = default_run_path("realsense", "orient-only", origin="human", output_dir=str(tmp_path))
    basename = os.path.basename(path)
    assert re.search(r"^human_run_\d{8}_\d{6}_realsense_orient-only\.pkl$", basename)


def test_default_run_path_omits_prefix_when_origin_none(tmp_path):
    """Backward compatible: a caller that doesn't pass origin gets the same
    unprefixed filenames as before the robot_run_/human_run_ prefix existed."""
    path = default_run_path("marker", "6dof", output_dir=str(tmp_path))
    basename = os.path.basename(path)
    assert "_None" not in basename
    assert not basename.startswith("_")
    assert re.search(r"^run_\d{8}_\d{6}_marker_6dof\.pkl$", basename)


# ---------------------------------------------------------------------
# RunRecorder constructor validation
# ---------------------------------------------------------------------

def test_recorder_rejects_invalid_origin():
    with pytest.raises(ValueError):
        RunRecorder(origin="alien", source_label="mock", output_path="unused.pkl")


def test_recorder_auto_path_prefixed_by_origin(tmp_path):
    robot_rec = RunRecorder(origin="robot", source_label="marker", tracking_mode="6dof",
                            output_dir=str(tmp_path))
    human_rec = RunRecorder(origin="human", source_label="realsense", tracking_mode="orient-only",
                            output_dir=str(tmp_path))
    assert os.path.basename(robot_rec.output_path).startswith("robot_run_")
    assert os.path.basename(human_rec.output_path).startswith("human_run_")
