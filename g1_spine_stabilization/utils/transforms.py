"""
SE3 math helpers for the G1 pose-tracking controller.
All poses are in the robot pelvis frame unless otherwise noted.
"""

import math
import numpy as np
from scipy.spatial.transform import Rotation, Slerp


def make_se3(xyz, R):
    """Build 4x4 homogeneous transform from position and rotation matrix."""
    T        = np.eye(4)
    T[:3, :3] = R
    T[:3,  3] = np.array(xyz)
    return T


def se3_from_xyz_rpy(xyz, rpy):
    """Build a 4x4 SE3 from position + fixed-axis roll/pitch/yaw (radians).

    URDF/ROS convention: R = Rz(yaw) @ Ry(pitch) @ Rx(roll) -- matches the
    convention already used independently in perception/frames.py's
    optical_to_d435_from_config() and scripts/calibrate_camera_extrinsics.py.
    """
    rpy = np.asarray(rpy, dtype=float)
    R = Rotation.from_euler("ZYX", [rpy[2], rpy[1], rpy[0]]).as_matrix()
    return make_se3(xyz, R)


def slerp_rotation(R1, R2, t):
    """Smoothly interpolate between two rotation matrices."""
    key_rots = Rotation.from_matrix(np.stack([R1, R2]))
    slerp    = Slerp([0, 1], key_rots)
    return slerp(float(t)).as_matrix()


def smoothstep(t):
    """Smooth interpolation with zero velocity at endpoints."""
    t = max(0.0, min(1.0, t))
    return t * t * (3 - 2 * t)


def Rx(theta):
    """Rotation matrix around x-axis by theta radians."""
    c, s = math.cos(theta), math.sin(theta)
    return np.array([
        [1, 0,  0],
        [0, c, -s],
        [0, s,  c],
    ])


def is_scalar_theta(x) -> bool:
    """True for a plain float/int/numpy scalar or 0-d array -- the "old-style"
    single-axis theta representation. False for anything array-like with a
    shape (e.g. a (3,3) rotation matrix)."""
    if isinstance(x, (int, float)):
        return True
    arr = np.asarray(x)
    return arr.ndim == 0


def as_rotation(x) -> np.ndarray:
    """Normalize a scalar-or-rotation `theta` into a (3,3) rotation matrix.

    A plain float/int/0-d value is interpreted as a roll about pelvis X
    (``Rx(theta)``) -- the original, single-axis representation. A (3,3)
    array is returned as-is (copied to a plain float ndarray) -- a full
    pelvis-frame rotation. Anything else raises, since silently accepting
    e.g. a 4x4 SE3 here would be a nasty, easy-to-make mistake at these call
    sites (several 4x4 SE3s are in scope nearby).
    """
    if is_scalar_theta(x):
        return Rx(float(x))
    arr = np.asarray(x, dtype=float)
    if arr.shape != (3, 3):
        raise ValueError(
            f"as_rotation expects a scalar or a (3,3) rotation matrix, got shape {arr.shape}"
        )
    return arr


def rotation_angle(R) -> float:
    """Total rotation angle (radians) of a (3,3) rotation matrix -- the norm
    of its axis-angle (rotvec) representation."""
    return float(np.linalg.norm(Rotation.from_matrix(R).as_rotvec()))


def clamp_rotation(R, max_angle=None, enabled_axes=None) -> np.ndarray:
    """Project a (3,3) rotation onto an allowed subset of axes and/or clamp
    its total angle, both expressed via its rotvec (axis-angle) form.

    Args:
        R:             (3,3) rotation matrix.
        max_angle:     if given, the rotvec is scaled down (direction
                       preserved) so its norm never exceeds this (radians).
                       None disables the angle clamp.
        enabled_axes:  optional 3-tuple of bools/0-1, e.g. [True, False, False].
                       Disabled axes have their rotvec component zeroed
                       BEFORE the angle clamp. None (default) enables all
                       three. This zeroes components of R's OWN minimal
                       rotvec representation, not an independent per-axis
                       Euler decomposition -- the two only coincide when R
                       is itself already a pure single-axis rotation on an
                       enabled axis (e.g. masking Y/Z on a pure roll-about-X
                       input reproduces it exactly, byte-for-byte). For a
                       general multi-axis rotation this is a deterministic,
                       well-behaved (norm-non-increasing) suppression of the
                       disabled axes' contribution -- adequate for a safety
                       clamp, not a physically exact per-axis split.

    Both args None is an exact identity passthrough (returns R unchanged,
    just copied to a plain float ndarray).
    """
    R = np.asarray(R, dtype=float)
    if max_angle is None and enabled_axes is None:
        return R.copy()
    v = Rotation.from_matrix(R).as_rotvec()
    if enabled_axes is not None:
        mask = np.array([1.0 if a else 0.0 for a in enabled_axes])
        v = v * mask
    if max_angle is not None:
        angle = np.linalg.norm(v)
        if angle > max_angle and angle > 0.0:
            v = v * (float(max_angle) / angle)
    return Rotation.from_rotvec(v).as_matrix()