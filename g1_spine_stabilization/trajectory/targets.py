"""
Hand-target geometry for the bimanual pose-tracking controller.

Values measured from Pinocchio FK at zero config with the
final assets/g1_with_hands.urdf (EE frames at inner palm surface):

  L_ee position: [0.3549,  0.1341, 0.0952]
  R_ee position: [0.3549, -0.1281, 0.0952]

  L_ee rotation: [[0,-1,0],[1,0,0],[0,0,1]]
  R_ee rotation: [[0,-1,0],[-1,0,0],[0,0,-1]]

  Box center:    [0.3549,  0.003,  0.0952]
  Box half width: 0.1311
"""

import numpy as np

from g1_spine_stabilization.utils.transforms import make_se3, Rx

# EE positions at zero config — palms naturally facing each other
L_EE_DEFAULT = np.array([0.3549,  0.1341, 0.0952])
R_EE_DEFAULT = np.array([0.3549, -0.1281, 0.0952])

# Box geometry derived from EE positions
BOX_CENTER_DEFAULT     = (L_EE_DEFAULT + R_EE_DEFAULT) / 2
BOX_HALF_WIDTH_DEFAULT = abs(L_EE_DEFAULT[1] - BOX_CENTER_DEFAULT[1])

# EE base orientations at zero config — the "natural grasp" orientations:
# palms facing each other in the rest pose.
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


def compute_hand_targets(box_center, box_half_width, theta, squeeze=0.0):
    """
    Compute left and right hand SE3 targets for a given object rotation.

    Args:
        box_center:     [x, y, z] in robot base/pelvis frame
        box_half_width: half distance between hands in y (meters)
        theta:          rotation angle around robot x-axis (radians)
        squeeze:        inward press depth s (meters). Both palms translate
                        toward the object center by this amount along the
                        (rotated) inter-palm axis, so the hand-to-hand
                        separation becomes 2*(box_half_width - squeeze). This
                        is the actuation knob for the palm-force press loop
                        (see controllers/palm_force.py, docs/palm_force_control.md).
                        squeeze=0.0 (default) reproduces the original geometry
                        exactly.

    Returns:
        L_target, R_target: 4x4 SE3 homogeneous matrices
    """
    R = Rx(theta)

    # Effective half-width after the inward press. Clamp at 0 so a squeeze
    # larger than the half-width never sends the palms past the center (the
    # press loop's squeeze_max should prevent this upstream, but clamp anyway).
    w = max(box_half_width - squeeze, 0.0)

    # Hand positions arc around object center in the y-z plane
    L_pos = np.array(box_center) + R @ np.array([0.0,  w, 0.0])
    R_pos = np.array(box_center) + R @ np.array([0.0, -w, 0.0])

    # Apply object rotation on top of base orientations
    return make_se3(L_pos, R @ R_L_BASE), make_se3(R_pos, R @ R_R_BASE)
