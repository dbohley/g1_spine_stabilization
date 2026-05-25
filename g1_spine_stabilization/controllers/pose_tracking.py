"""
Bimanual pose-tracking controller (CoppeliaSim, sine placeholder).

Three-phase routine:
  1. Interpolate EE targets from the default standing pose to the grasp pose.
  2. Drive both hands together via the trajectory source (sine wave for now,
     ArUco-driven in Session 3).
  3. Interpolate EE targets back to the default standing pose.
"""

import math
import os
import time

import numpy as np
import yaml
from coppeliasim_zmqremoteapi_client import RemoteAPIClient

from g1_spine_stabilization.kinematics.ik_solver import G1HandIK
from g1_spine_stabilization.sim.coppelia_utils import (
    get_arm_handles, read_joint_state, set_joint_positions,
    create_box, attach_box_to_hand,
)
from g1_spine_stabilization.utils.transforms import make_se3, slerp_rotation, smoothstep
from g1_spine_stabilization.trajectory.targets import (
    compute_hand_targets,
    R_L_BASE,
    R_R_BASE,
    L_EE_DEFAULT,
    R_EE_DEFAULT,
)
from g1_spine_stabilization.trajectory.sources import sine_trajectory


ARM_DEFAULT_Q = np.array([
    -0.07641,  0.3355,  -0.38365, -0.04522,  1.18319,  0.3721,  0.10582,
    -0.0737,  -0.3477,   0.37916, -0.03927, -1.17787,  0.37036, -0.0962,
])

# Path to the default YAML config, resolved relative to the project root.
PROJECT_ROOT = os.path.abspath(
    os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..')
)
DEFAULT_CONFIG_PATH = os.path.join(PROJECT_ROOT, 'configs', 'default.yaml')


def _load_config(config_path):
    with open(config_path, 'r') as f:
        return yaml.safe_load(f)


def step_realtime(sim, control_dt):
    t_start = time.perf_counter()
    sim.step()
    elapsed = time.perf_counter() - t_start
    sleep_time = control_dt - elapsed
    if sleep_time > 0:
        time.sleep(sleep_time)


def run_pose_tracking_sim(
    config_path        = None,
    box_size           = None,
    rotation_amplitude = None,
    rotation_freq      = None,
    duration           = None,
    control_dt         = None,
    interpolation_time = None,
):
    """
    Run the bimanual pose-tracking controller against the CoppeliaSim sim.

    YAML config provides defaults; explicit kwargs override.
    """
    # ---------------------------------------------------
    # Load config (defaults) and apply kwarg overrides
    # ---------------------------------------------------
    if config_path is None:
        config_path = DEFAULT_CONFIG_PATH
    cfg = _load_config(config_path)

    control_dt         = control_dt         if control_dt         is not None else cfg['control']['dt']
    interpolation_time = interpolation_time if interpolation_time is not None else cfg['control']['interpolation_time']
    duration           = duration           if duration           is not None else cfg['control']['rotation_duration']
    rotation_amplitude = rotation_amplitude if rotation_amplitude is not None else cfg['trajectory']['sine']['amplitude']
    rotation_freq      = rotation_freq      if rotation_freq      is not None else cfg['trajectory']['sine']['frequency']
    box_size           = list(box_size)     if box_size           is not None else list(cfg['box']['size'])
    box_color          = list(cfg['box']['color'])
    box_mass           = cfg['box']['mass']
    pelvis_world_x     = cfg['frames']['pelvis_world_x']
    pelvis_world_z     = cfg['frames']['pelvis_world_z']

    # Half width is half the box width in y direction
    box_half_width = box_size[1] / 2

    # Box center in pelvis frame — palms need to be at ±box_half_width
    # in y, at the same x and z as where hands naturally meet
    box_center_pelvis = np.array([
        L_EE_DEFAULT[0],   # x: where palms naturally are
        0.0,               # y: centered between hands
        L_EE_DEFAULT[2],   # z: at palm height
    ])

    # Box position in world frame for creating it in CoppeliaSim
    box_center_world = [
        box_center_pelvis[0] + pelvis_world_x,
        box_center_pelvis[1],
        box_center_pelvis[2] + pelvis_world_z,
    ]

    print(f"Box size:        {box_size}")
    print(f"Box half width:  {box_half_width:.4f}")
    print(f"Box center (pelvis): {np.round(box_center_pelvis, 4)}")
    print(f"Box center (world):  {np.round(box_center_world, 4)}")

    # ---------------------------------------------------
    # Initialise
    # ---------------------------------------------------
    print("\nInitialising IK solver...")
    ik = G1HandIK()

    client  = RemoteAPIClient()
    sim     = client.require('sim')
    handles = get_arm_handles(sim)

    # Get hand handle that we'll attach the box to
    # We attach to the left hand base — could be either, just pick one
    left_hand_handle = sim.getObject(
        '/pelvis_respondable/L_hand_hand_base_link_respondable'
    )

    # ---------------------------------------------------
    # Create the box BEFORE starting simulation
    # ---------------------------------------------------
    print("\nCreating box in scene...")
    # Place the box above where the hands will grasp it
    # so it's clearly visible before simulation starts
    box_handle = create_box(
        sim,
        size=box_size,
        position=box_center_world,
        color=box_color,
        mass=box_mass,
    )
    print(f"Box created, handle: {box_handle}")

    # ---------------------------------------------------
    # Start simulation
    # ---------------------------------------------------
    sim.setStepping(True)
    sim.startSimulation()
    print("Simulation started.")

    # ---------------------------------------------------
    # Phase 1: Interpolate to grasp pose
    # ---------------------------------------------------
    print(f"\nPhase 1: Interpolating to grasp pose over {interpolation_time}s...")
    L_grasp, R_grasp = compute_hand_targets(box_center_pelvis, box_half_width, 0.0)
    steps_interp = int(interpolation_time / control_dt)

    for i in range(steps_interp):
        alpha = smoothstep(i / steps_interp)

        L_pos = (1 - alpha) * L_EE_DEFAULT + alpha * L_grasp[:3, 3]
        R_pos = (1 - alpha) * R_EE_DEFAULT + alpha * R_grasp[:3, 3]
        L_R   = slerp_rotation(R_L_BASE, L_grasp[:3, :3], alpha)
        R_R   = slerp_rotation(R_R_BASE, R_grasp[:3, :3], alpha)

        current_q, current_dq = read_joint_state(sim, handles)
        sol_q, _ = ik.solve_ik(
            make_se3(L_pos, L_R),
            make_se3(R_pos, R_R),
            current_q, current_dq,
        )
        set_joint_positions(sim, handles, sol_q)
        step_realtime(sim, control_dt)

        if i % 50 == 0:
            print(f"  {alpha*100:.0f}%")

    print("At grasp pose.")

    # ---------------------------------------------------
    # ATTACH BOX TO HAND
    # ---------------------------------------------------
    print("\nAttaching box to left hand...")
    attach_box_to_hand(sim, box_handle, left_hand_handle)

    # ---------------------------------------------------
    # Phase 2: Track target trajectory
    # ---------------------------------------------------
    print(f"\nPhase 2: Tracking target for {duration}s...")
    steps_rotate = int(duration / control_dt)

    for i in range(steps_rotate):
        t     = i * control_dt
        theta = sine_trajectory(t, rotation_amplitude, rotation_freq)

        current_q, current_dq = read_joint_state(sim, handles)
        L_target, R_target    = compute_hand_targets(
            box_center_pelvis, box_half_width, theta
        )
        sol_q, _ = ik.solve_ik(
            L_target, R_target, current_q, current_dq
        )
        set_joint_positions(sim, handles, sol_q)
        step_realtime(sim, control_dt)

        if i % 50 == 0:
            print(f"  t={t:.1f}s  theta={math.degrees(theta):.1f} deg")

    # ---------------------------------------------------
    # Phase 3: Return to default pose using IK
    # This brings EE back to the natural standing positions
    # ---------------------------------------------------
    print("\nPhase 3: Returning to default pose...")
    steps_return = int(interpolation_time / control_dt)

    # Get current EE state via FK on current joint config
    current_q, _ = read_joint_state(sim, handles)
    L_current_pose, R_current_pose = ik.fk(current_q)
    L_start_pos = L_current_pose.translation.copy()
    R_start_pos = R_current_pose.translation.copy()
    L_start_rot = L_current_pose.rotation.copy()
    R_start_rot = R_current_pose.rotation.copy()

    for i in range(steps_return):
        alpha = smoothstep(i / steps_return)

        # Interpolate EE position back to default
        L_pos = (1 - alpha) * L_start_pos + alpha * L_EE_DEFAULT
        R_pos = (1 - alpha) * R_start_pos + alpha * R_EE_DEFAULT
        L_R = slerp_rotation(L_start_rot, R_L_BASE, alpha)
        R_R = slerp_rotation(R_start_rot, R_R_BASE, alpha)

        current_q, current_dq = read_joint_state(sim, handles)
        sol_q, _ = ik.solve_ik(
            make_se3(L_pos, L_R),
            make_se3(R_pos, R_R),
            current_q, current_dq,
        )
        set_joint_positions(sim, handles, sol_q)
        step_realtime(sim, control_dt)

    sim.stopSimulation()
    print("\nDone.")
