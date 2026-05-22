# controller/box_rotation.py
import math
import time
import numpy as np
from coppeliasim_zmqremoteapi_client import RemoteAPIClient

from ik_solver import G1HandIK
from utils.sim_utils import (
    get_arm_handles, read_joint_state, set_joint_positions,
    create_box, attach_box_to_hand, detach_box,
)
from utils.transforms import make_se3, slerp_rotation, smoothstep
from controller.trajectory import (
    compute_hand_targets,
    sine_trajectory,
    R_L_BASE,
    R_R_BASE,
    L_EE_DEFAULT,
    R_EE_DEFAULT,
)

ARM_DEFAULT_Q = np.array([
    -0.07641,  0.3355,  -0.38365, -0.04522,  1.18319,  0.3721,  0.10582,
    -0.0737,  -0.3477,   0.37916, -0.03927, -1.17787,  0.37036, -0.0962,
])

# Pelvis world Z so we can position the box correctly in world frame
PELVIS_WORLD_Z = 0.7923  # measured earlier
PELVIS_WORLD_X = -0.1934


def step_realtime(sim, control_dt):
    t_start = time.perf_counter()
    sim.step()
    elapsed = time.perf_counter() - t_start
    sleep_time = control_dt - elapsed
    if sleep_time > 0:
        time.sleep(sleep_time)


def rotate_box_controller(
    box_size           = [0.16, 0.16, 0.16],   # 16cm cube
    rotation_amplitude = 0.524,                 # ~30 degrees
    rotation_freq      = 0.3,                   # Hz
    duration           = 10.0,                  # seconds
    control_dt         = 0.02,                  # seconds per step
    interpolation_time = 3.0,                   # seconds
):
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
        box_center_pelvis[0] + PELVIS_WORLD_X,
        box_center_pelvis[1],
        box_center_pelvis[2] + PELVIS_WORLD_Z,
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
        color=[0.8, 0.3, 0.2],   # red-orange
        mass=0.2,
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
    # Phase 2: Rotate box
    # ---------------------------------------------------
    print(f"\nPhase 2: Rotating for {duration}s...")
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

    # In box_rotation.py, replace Phase 3 with this:

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


if __name__ == '__main__':
    rotate_box_controller(
        box_size           = [0.16, 0.16, 0.16],
        rotation_amplitude = 0.524,
        rotation_freq      = 0.3,
        duration           = 10.0,
        control_dt         = 0.02,
        interpolation_time = 3.0,
    )