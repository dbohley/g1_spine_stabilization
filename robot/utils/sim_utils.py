# utils/sim_utils.py
"""
CoppeliaSim helpers for the G1 controller.
"""

import numpy as np
from coppeliasim_zmqremoteapi_client import RemoteAPIClient

# Joint names in reduced model order (matches IK solver sol_q indices 0-13)
PREFIX = '/pelvis_respondable'

ARM_JOINT_NAMES = [
    f'{PREFIX}/left_shoulder_pitch_joint',
    f'{PREFIX}/left_shoulder_roll_joint',
    f'{PREFIX}/left_shoulder_yaw_joint',
    f'{PREFIX}/left_elbow_joint',
    f'{PREFIX}/left_wrist_roll_joint',
    f'{PREFIX}/left_wrist_pitch_joint',
    f'{PREFIX}/left_wrist_yaw_joint',
    f'{PREFIX}/right_shoulder_pitch_joint',
    f'{PREFIX}/right_shoulder_roll_joint',
    f'{PREFIX}/right_shoulder_yaw_joint',
    f'{PREFIX}/right_elbow_joint',
    f'{PREFIX}/right_wrist_roll_joint',
    f'{PREFIX}/right_wrist_pitch_joint',
    f'{PREFIX}/right_wrist_yaw_joint',
]


def get_arm_handles(sim):
    return [sim.getObject(name) for name in ARM_JOINT_NAMES]


def read_joint_state(sim, handles):
    q  = np.array([sim.getJointPosition(h) for h in handles])
    dq = np.array([sim.getJointVelocity(h)  for h in handles])
    return q, dq


def set_joint_positions(sim, handles, q):
    for h, qi in zip(handles, q):
        sim.setJointTargetPosition(h, float(qi))


# utils/sim_utils.py - add this helper

# utils/sim_utils.py - update create_box to make it static by default

def create_box(sim, size, position, color=None, mass=0.1, static=True):
    """
    Create a box in the scene.

    Args:
        sim:      CoppeliaSim API client
        size:     [x, y, z] full dimensions in meters
        position: [x, y, z] world position
        color:    [r, g, b] each in [0,1]
        mass:     mass in kg
        static:   if True, box is unaffected by gravity (default)
    """
    if color is None:
        color = [0.8, 0.3, 0.2]

    box_handle = sim.createPrimitiveShape(
        sim.primitiveshape_cuboid,
        size,
        0,
    )

    sim.setObjectAlias(box_handle, 'rotation_box')
    sim.setObjectPosition(box_handle, sim.handle_world, position)

    sim.setObjectInt32Param(box_handle, sim.shapeintparam_respondable, 1)
    sim.setObjectInt32Param(
        box_handle, sim.shapeintparam_static, 1 if static else 0
    )

    sim.setShapeMass(box_handle, mass)
    sim.computeMassAndInertia(box_handle, 1000.0)
    sim.setShapeColor(box_handle, None, sim.colorcomponent_ambient_diffuse, color)

    return box_handle


def attach_box_to_hand(sim, box_handle, hand_handle):
    """Make the box a child of the hand so it moves with the hand."""
    # keepInPlace=True preserves the box's world position when reparenting
    sim.setObjectParent(box_handle, hand_handle, True)
    # Make it static so physics doesn't fight us
    sim.setObjectInt32Param(box_handle, sim.shapeintparam_static, 1)


def detach_box(sim, box_handle):
    """Release the box back to world parent and enable physics."""
    sim.setObjectParent(box_handle, -1, True)  # -1 = world
    sim.setObjectInt32Param(box_handle, sim.shapeintparam_static, 0)