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
from scipy.spatial.transform import Rotation

from g1_spine_stabilization.utils.transforms import make_se3, as_rotation

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


def compute_hand_targets(box_center, box_half_width, theta, squeeze=0.0,
                          grasp_frame_R=None):
    """
    Compute left and right hand SE3 targets for a given object rotation.

    Args:
        box_center:     [x, y, z] in robot base/pelvis frame
        box_half_width: half distance between hands in y (meters)
        theta:          the tracked object rotation, pelvis frame. Either a
                        scalar (interpreted as a roll about pelvis X --
                        Rx(theta), the original single-axis representation,
                        still used by SineSource and the theta=0.0 approach-
                        phase call sites) or a full (3,3) rotation matrix
                        (from a MarkerSource tracking the target's genuine
                        3-DoF orientation -- see trajectory/sources.py).
                        Normalized via utils.transforms.as_rotation.
        squeeze:        inward press depth s (meters). Both palms translate
                        toward the object center by this amount along the
                        (rotated) inter-palm axis, so the hand-to-hand
                        separation becomes 2*(box_half_width - squeeze). This
                        is the actuation knob for the palm-force press loop
                        (see controllers/palm_force.py, docs/palm_force_control.md).
                        squeeze=0.0 (default) reproduces the original geometry
                        exactly.
        grasp_frame_R:  optional 3x3 rotation (pelvis frame), default None
                        (== identity -- reproduces the original pelvis-Y
                        behavior exactly, byte-for-byte). Represents the
                        object's own basis: column 1 (grasp_frame_R[:, 1])
                        becomes the inter-palm arc axis, replacing the
                        hardcoded world/pelvis Y [0,1,0], and R_L_BASE/
                        R_R_BASE are expressed in this frame too
                        (grasp_frame_R @ R_L_BASE) so palm ORIENTATION
                        rotates along with palm POSITION -- both position
                        and orientation must move together, since applying
                        this rotation to position alone would arc the palms
                        to the right spot while leaving them facing the old
                        pelvis-relative direction instead of each other.
                        Pass the object marker's own measured rotation here
                        so a yawed/tilted object still gets grasped along
                        its own true left-right axis, not the robot's fixed
                        one. theta's existing Rx(theta) tracking rotation
                        composes on top of this, unchanged.

    Returns:
        L_target, R_target: 4x4 SE3 homogeneous matrices
    """
    G = np.eye(3) if grasp_frame_R is None else np.asarray(grasp_frame_R, dtype=float)
    R = as_rotation(theta)

    # Effective half-width after the inward press. Clamp at 0 so a squeeze
    # larger than the half-width never sends the palms past the center (the
    # press loop's squeeze_max should prevent this upstream, but clamp anyway).
    w = max(box_half_width - squeeze, 0.0)

    # Hand positions arc around object center in the (grasp_frame_R-rotated)
    # y-z plane -- G[:, 1] is [0,1,0] itself when grasp_frame_R is identity.
    axis = G[:, 1]
    L_pos = np.array(box_center) + R @ (w * axis)
    R_pos = np.array(box_center) + R @ (-w * axis)

    # Apply object rotation on top of base orientations, themselves first
    # re-expressed in the object's own frame (G @ R_L_BASE) -- see
    # grasp_frame_R's docstring above for why this must move together with
    # the position arc axis rather than independently.
    return make_se3(L_pos, R @ G @ R_L_BASE), make_se3(R_pos, R @ G @ R_R_BASE)


def apply_palm_tilt(L_target, R_target, tilt_left_deg=(0.0, 0.0, 0.0),
                     tilt_right_deg=(0.0, 0.0, 0.0)):
    """Right-multiply each hand's target rotation by a small LOCAL/body-frame
    tilt -- a rotation about the hand's OWN current axes, appended as the
    rightmost factor, so it composes safely on top of whatever
    theta/grasp_frame_R/squeeze already produced (positions are untouched).

    tilt_*_deg is [roll, pitch, yaw] in degrees, intrinsic (each successive
    angle rotates about the hand's own axis as reoriented by the previous
    one):
      roll  = local X = the inter-palm press/squeeze axis. Same real-world
              sense for both hands.
      pitch = local Y = the forward/reach axis. Same real-world sense for
              both hands.
      yaw   = local Z = up/down -- flips real-world sign between hands (+Z
              for left, -Z for right; see CLAUDE.md's Frame conventions).
    So a real-world-symmetric tilt needs OPPOSITE signs between hands for
    roll/yaw but the SAME sign for pitch -- tilt_left_deg/tilt_right_deg are
    independent, not auto-mirrored, since that asymmetry is easy to get
    backwards; tune each by observation.

    All-zero tilts (the default) are an exact identity no-op.

    Added to diagnose/mitigate a live finding: the Inspire hand's thumb tip
    sits well past the palm's own L_ee_link/R_ee_link contact point along
    the press axis, so it can contact the grasped object before the palm
    sensor does. See docs/palm_force_control.md.
    """
    L_tilt = Rotation.from_euler("XYZ", tilt_left_deg, degrees=True).as_matrix()
    R_tilt = Rotation.from_euler("XYZ", tilt_right_deg, degrees=True).as_matrix()
    L_out = np.array(L_target, dtype=float, copy=True)
    R_out = np.array(R_target, dtype=float, copy=True)
    L_out[:3, :3] = L_target[:3, :3] @ L_tilt
    R_out[:3, :3] = R_target[:3, :3] @ R_tilt
    return L_out, R_out


def press_axis(theta, grasp_frame_R=None):
    """The (rotated) inter-palm press axis in pelvis frame -- the direction a
    positive squeeze/press-force acts along. Same quantity as
    ``compute_hand_targets``'s internal ``R @ (G[:, 1])``, factored out here
    so ``compute_press_wrenches`` and the wrist-wrench force estimator
    (``controllers/palm_force.py``'s ``WristWrenchForceReader``) can share it
    instead of each hardcoding their own copy.

    grasp_frame_R=None (default) reproduces the original pelvis-Y axis
    exactly -- see compute_hand_targets's own grasp_frame_R docstring for why
    an object's own measured basis should be used instead when available.

    theta: scalar (roll about pelvis X) or (3,3) rotation matrix -- same
    convention as compute_hand_targets's theta; see its docstring.
    """
    G = np.eye(3) if grasp_frame_R is None else np.asarray(grasp_frame_R, dtype=float)
    return as_rotation(theta) @ G[:, 1]


def compute_press_wrenches(theta, F_cmd, grasp_frame_R=None):
    """Desired inter-palm press wrenches for the Jacobian force controller.

    Builds a pure-force, zero-moment wrench per hand pointing INWARD toward the
    object center along the (rotated) inter-palm axis -- the same axis the
    ``squeeze`` knob in ``compute_hand_targets`` translates along. These feed the
    torque feedforward via ``tau = J^T W`` (see G1HandIK.ee_jacobians), where W is
    the wrench the palm exerts ON the object, expressed in the pelvis frame.

    Geometry: the left palm sits at ``+w`` along ``u = press_axis(theta, grasp_frame_R)``
    (see ``compute_hand_targets``), so to press toward center it exerts ``-F_cmd*u``;
    the right palm sits at ``-w`` and exerts ``+F_cmd*u``. Both wrenches are in
    the SAME pelvis frame as the LOCAL_WORLD_ALIGNED Jacobians they multiply.

    Args:
        theta: scalar (roll about robot X) or (3,3) rotation matrix -- the
               same theta driving ``compute_hand_targets`` this step.
        F_cmd: commanded press-force magnitude (N), >= 0.
        grasp_frame_R: optional 3x3 rotation (pelvis frame), default None
               (== identity, reproduces the original pelvis-Y axis exactly).
               Pass the same value given to compute_hand_targets so the press
               wrench follows the object's own measured left-right axis
               instead of the robot's fixed one -- this was previously
               hardcoded to pelvis-Y here even when compute_hand_targets was
               already using a real grasp_frame_R, a stale-axis gap fixed
               alongside the wrist-wrench force estimator (which needs the
               same axis for its own projection).

    Returns:
        (W_L, W_R): two length-6 numpy arrays [fx, fy, fz, mx, my, mz], zero moment.
    """
    u = press_axis(theta, grasp_frame_R)
    W_L = np.zeros(6)
    W_R = np.zeros(6)
    W_L[:3] = -float(F_cmd) * u                  # left presses toward center (-u)
    W_R[:3] = +float(F_cmd) * u                  # right presses toward center (+u)
    return W_L, W_R
