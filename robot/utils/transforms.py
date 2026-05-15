# utils/transforms.py
"""
SE3 math helpers for the G1 box rotation controller.
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