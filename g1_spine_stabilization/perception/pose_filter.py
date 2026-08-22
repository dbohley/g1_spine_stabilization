"""Median-of-N then EMA smoothing for a single marker's raw SE3 pose stream.

Rejects an occasional single-frame solvePnP misread (AprilTag pose
instability -- see docs/gravity_comp_recording_plan.md and
docs/data_recording_conventions.md) before it reaches ANY downstream
consumer: the recorded object_pose/target_pose/angle_diff_rpy/angular-
velocity fields (data_recording/recorder.py), the F/T gravity-comp
reprojection (sensors/force_torque.py's GravityCompensator.compensate()),
and the experimental sin-model theta (data_recording/recorder.py's
sin_model_theta_rad()).

MUST be applied to pose_in_optical (the camera-native frame every
MarkerObservation carries) exactly once, with pose_in_pelvis then re-derived
from the FILTERED optical pose via perception/frames.py's compose_to_pelvis
-- never filtered independently in both frames. GravityCompensator.
compensate()'s own docstring notes the relative rotation R_cur.T @ R_ref is
invariant to which fixed frame it's expressed in; that invariance only holds
because compose_to_pelvis is a single fixed left-multiplication (a linear
map). Median-of-N is a nonlinear, order-statistics operation and is NOT
equivariant under an arbitrary fixed rotation (only EMA, being linear, is)
-- so filtering translation/rotation separately in optical vs. pelvis frame
would silently produce two different, inconsistent answers instead of one.
"""
import numpy as np
from scipy.spatial.transform import Rotation


class PoseNoiseFilter:
    """Smooths one marker's raw pose_in_optical stream. One instance per
    marker id -- do not share between object and target (each has its own
    independent noise history).

    Position: median-of-N then EMA on the (3,) translation vector -- same
    recipe as trajectory/sources.py's NoiseFilteredSource.

    Rotation: converted to a quaternion each call; sign-flipped for
    hemisphere continuity against the previous FILTERED quaternion (q and
    -q represent the identical physical rotation -- without this check, a
    filter would see consecutive same-rotation readings that happen to land
    in opposite quaternion hemispheres as a huge, spurious jump); the (4,)
    quaternion history is then median-of-N filtered then EMA'd
    component-wise, and the result re-normalized to unit length before
    converting back to a rotation matrix.
    """

    def __init__(self, median_window=5, ema_alpha=0.3):
        if median_window < 1 or median_window % 2 == 0:
            raise ValueError(
                f"median_window must be a positive odd integer, got {median_window}"
            )
        self._median_window = int(median_window)
        self._ema_alpha = float(ema_alpha)
        self._pos_hist = []
        self._quat_hist = []
        self._ema_pos = None
        self._ema_quat = None

    def filter(self, pose_in_optical: np.ndarray) -> np.ndarray:
        pos = np.asarray(pose_in_optical[:3, 3], dtype=float)
        R = np.asarray(pose_in_optical[:3, :3], dtype=float)
        quat = Rotation.from_matrix(R).as_quat()

        if self._quat_hist and np.dot(quat, self._quat_hist[-1]) < 0.0:
            quat = -quat

        self._pos_hist.append(pos.copy())
        self._quat_hist.append(quat.copy())
        if len(self._pos_hist) > self._median_window:
            self._pos_hist.pop(0)
            self._quat_hist.pop(0)

        if len(self._pos_hist) < self._median_window:
            med_pos, med_quat = pos, quat
        else:
            med_pos = np.median(np.stack(self._pos_hist, axis=0), axis=0)
            med_quat = np.median(np.stack(self._quat_hist, axis=0), axis=0)
            med_quat = med_quat / np.linalg.norm(med_quat)

        if self._ema_pos is None:
            self._ema_pos, self._ema_quat = med_pos.copy(), med_quat.copy()
        else:
            self._ema_pos = self._ema_alpha * med_pos + (1.0 - self._ema_alpha) * self._ema_pos
            ema_quat = self._ema_alpha * med_quat + (1.0 - self._ema_alpha) * self._ema_quat
            self._ema_quat = ema_quat / np.linalg.norm(ema_quat)

        out = np.eye(4)
        out[:3, :3] = Rotation.from_quat(self._ema_quat).as_matrix()
        out[:3, 3] = self._ema_pos
        return out
