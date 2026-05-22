# controller/trajectory.py
"""
Trajectory generation for box rotation task.

Values measured from Pinocchio FK at zero config with the
final g1_with_hands.urdf (EE frames at inner palm surface):

  L_ee position: [0.3549,  0.1341, 0.0952]
  R_ee position: [0.3549, -0.1281, 0.0952]

  L_ee rotation: [[0,-1,0],[1,0,0],[0,0,1]]
  R_ee rotation: [[0,-1,0],[-1,0,0],[0,0,-1]]

  Box center:    [0.3549,  0.003,  0.0952]
  Box half width: 0.1311
"""

import math
import numpy as np
from utils.transforms import make_se3, Rx

# EE positions at zero config — palms naturally facing each other
L_EE_DEFAULT = np.array([0.3549,  0.1341, 0.0952])
R_EE_DEFAULT = np.array([0.3549, -0.1281, 0.0952])

# Box geometry derived from EE positions
BOX_CENTER_DEFAULT     = (L_EE_DEFAULT + R_EE_DEFAULT) / 2
BOX_HALF_WIDTH_DEFAULT = abs(L_EE_DEFAULT[1] - BOX_CENTER_DEFAULT[1])

# EE base orientations at zero config
# These are the "natural grasp" orientations — what the EE looks like
# when palms are facing each other in the rest pose
R_L_BASE = np.array([
    [ 0, -1,  0],
    [ 1,  0,  0],
    [ 0,  0,  1],
])

R_R_BASE = np.array([
    [ 0, -1,  0],
    [-1,  0,  0],
    [ 0,  0, -1],
])


def compute_hand_targets(box_center, box_half_width, theta):
    """
    Compute left and right hand SE3 targets for a given box rotation.

    Args:
        box_center:     [x, y, z] in robot base/pelvis frame
        box_half_width: half distance between hands in y (meters)
        theta:          rotation angle around robot x-axis (radians)

    Returns:
        L_target, R_target: 4x4 SE3 homogeneous matrices
    """
    R = Rx(theta)

    # Hand positions arc around box center in the y-z plane
    L_pos = np.array(box_center) + R @ np.array([0.0,  box_half_width, 0.0])
    R_pos = np.array(box_center) + R @ np.array([0.0, -box_half_width, 0.0])

    # Apply box rotation on top of base orientations
    return make_se3(L_pos, R @ R_L_BASE), make_se3(R_pos, R @ R_R_BASE)


def sine_trajectory(t, amplitude, freq):
    """
    Sine wave rotation trajectory.
    Replace with camera-driven theta when integrating with real perception.
    """
    return amplitude * math.sin(2 * math.pi * freq * t)