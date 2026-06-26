"""
Camera-frame transforms for the head-mounted RealSense.

pose_in_pelvis = T_d435_to_pelvis @ T_optical_to_d435 @ pose_in_optical
"""
import numpy as np
from scipy.spatial.transform import Rotation

from g1_spine_stabilization.utils.transforms import make_se3


def optical_to_d435_from_config(config_section) -> np.ndarray:
    """Build T_optical_to_d435 from the YAML config section.

    Expects keys 'xyz' (list of 3) and 'rpy' (list of 3, fixed-axis).
    """
    xyz = np.asarray(config_section["xyz"], dtype=float)
    rpy = np.asarray(config_section["rpy"], dtype=float)
    # URDF/ROS convention: R = Rz(yaw) @ Ry(pitch) @ Rx(roll)
    R = Rotation.from_euler("ZYX", [rpy[2], rpy[1], rpy[0]]).as_matrix()
    return make_se3(xyz, R)


def d435_to_pelvis_from_pinocchio(locked_joint_config=None, ik=None) -> np.ndarray:
    """Compute T_d435_to_pelvis via Pinocchio FK at the given (or zero) joint config.

    For the current controller, head/waist joints are locked at zero, so this
    is a constant computed once. If they move later, this function must be
    called per-frame.

    Pass an already-loaded G1HandIK instance via `ik` to avoid the ~5-15 s
    Pinocchio/CasADi cold-start when one is already available.
    """
    import pinocchio as pin

    if ik is None:
        from g1_spine_stabilization.kinematics.ik_solver import G1HandIK
        ik = G1HandIK()
    model, data = ik.get_full_model_and_data()
    q = locked_joint_config if locked_joint_config is not None else np.zeros(model.nq)
    pin.forwardKinematics(model, data, q)
    pin.updateFramePlacements(model, data)
    frame_id = model.getFrameId("d435_link")
    placement = data.oMf[frame_id]
    T = np.eye(4)
    T[:3, :3] = placement.rotation
    T[:3, 3] = placement.translation
    return T


def compose_to_pelvis(
    pose_in_optical: np.ndarray,
    T_optical_to_d435: np.ndarray,
    T_d435_to_pelvis: np.ndarray,
) -> np.ndarray:
    """Apply the full transform chain to take a marker pose to the pelvis frame."""
    return T_d435_to_pelvis @ T_optical_to_d435 @ pose_in_optical
