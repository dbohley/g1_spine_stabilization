# G1 Spine Stabilization — Claude Reference

This file gives Claude (and Claude Code) the context needed to work effectively on this project. Read it fully at the start of any session.

## Project Goal

Build a control pipeline for a Unitree G1 humanoid robot to perform bimanual rigid-body pose tracking: the robot holds an object in both hands and matches its pose to a moving target reference. Currently developed in CoppeliaSim simulation; long-term goal is transfer to the real G1 robot.

The simulation today uses a sine wave to drive a single-axis (X) rotation as a placeholder. The intended real-world system uses two ArUco markers:

- **Object marker** (on top of the held object): gives the object's actual starting pose at grasp time, since real-world placement varies between trials
- **Target marker** (held by the user or attached to a moving jig): the pose the held object must track during the rotation phase

Object width and depth are passed as runtime arguments — ArUco is used for pose only, not dimensions.

## Current Status

**Working baseline:** Robot loads cleanly into CoppeliaSim. IK solves both arms simultaneously with self-collision avoidance via tightened shoulder_roll bounds. Box gets created in the scene, grasped, attached to the left hand, rotated around the X axis driven by a sine wave θ trajectory, then released back to the default pose. Three-phase controller: interpolate to grasp → rotate → interpolate back.

## Session Roadmap

- **Session 1:** Repo restructure, packaging, configs, tests — no behavior change
- **Session 2:** ArUco perception module (standalone, no robot integration yet)
- **Session 3:** Replace sine wave with ArUco-driven target source; perception-driven controller
- **Session 4:** Grip force controller (finger close + optional wrist press)
- **Session 5 (deferred):** Real-robot bringup — gated on providing the reference codebase

## File Structure

```
g1_spine_stabilization/
├── README.md
├── LICENSE                          # internal use notice (private repo)
├── .gitignore
├── requirements.txt
├── setup.py                         # minimal, for `pip install -e .`
├── CLAUDE.md
│
├── configs/
│   └── default.yaml                 # tunable params: box size, timing, sine params
│
├── assets/
│   ├── g1_with_hands.urdf
│   ├── inspire_hand/
│   │   ├── inspire_hand_left.urdf
│   │   ├── inspire_hand_right.urdf
│   │   └── meshes/                  # .obj for collision/visual
│   └── meshes/                      # G1 body STLs
│
├── g1_spine_stabilization/          # the package
│   ├── __init__.py
│   ├── kinematics/
│   │   ├── __init__.py
│   │   └── ik_solver.py             # G1HandIK (Pinocchio + CasADi)
│   ├── controllers/
│   │   ├── __init__.py
│   │   ├── pose_tracking.py         # was box_rotation.py
│   │   └── grip_force.py            # empty stub for Session 4
│   ├── trajectory/
│   │   ├── __init__.py
│   │   ├── targets.py               # compute_hand_targets — geometry
│   │   └── sources.py               # sine driver now; ArUco/mock later
│   ├── perception/                  # empty placeholder for Session 2
│   │   └── __init__.py
│   ├── sim/
│   │   ├── __init__.py
│   │   └── coppelia_utils.py        # was utils/sim_utils.py
│   └── utils/
│       ├── __init__.py
│       └── transforms.py
│
├── scripts/
│   ├── run_pose_tracking_sim.py     # main entry point — replaces __main__ block
│   ├── setup_scene.py               # one-time URDF import into CoppeliaSim
│   └── verify_ik.py                 # standalone IK self-test
│
├── tests/
│   ├── __init__.py
│   ├── test_kinematics.py           # FK at zero, FK at default, IK round-trip
│   ├── test_trajectory.py           # compute_hand_targets at θ=0 matches defaults
│   └── test_transforms.py           # SE3, slerp, smoothstep, Rx edge cases
│
└── docs/
    ├── architecture.md              # high-level data flow
    └── frame_conventions.md         # pelvis, EE, marker frames
```

## Hard Rules

- **Never modify `assets/g1_with_hands.urdf` without explicit approval.** It's been carefully constructed and tuned. Changes propagate to both Pinocchio and CoppeliaSim.
- **Never change the EE frame definitions** (`L_ee_link`, `R_ee_link`) without discussing first. They define the IK target convention.
- **Never change the shoulder_roll bounds** (`L_SHOULDER_ROLL_MIN = 0.15`, `R_SHOULDER_ROLL_MAX = -0.15`) without explicit approval. They prevent self-collision with the torso. These stay as code constants in `ik_solver.py`, NOT in the YAML config.
- **Never change the IK cost weights** (50 trans, 5 rot, 0.02 reg, 0.1 smooth) without approval. They stay as code constants.
- **Always preserve the joint ordering** in `ARM_JOINT_NAMES` — it must match the IK solver's reduced model order exactly.
- **Don't use `simURDF.import_`** in CoppeliaSim Python API — `import` is a Python keyword. Use `simURDF.importFile()` instead.
- **Don't add new dependencies without discussion.** Current stack: numpy, scipy, pinocchio (conda-forge), casadi, pyyaml, coppeliasim_zmqremoteapi_client, pytest. New libs need a justification.

## Reference Codebase Policy

A separate codebase for G1 teleoperation and data collection may be provided as a reference (the `g1_data_collection` repo or similar). It contains:

- Vive tracker integration
- A vendored `fairmotion` library (rotation/math helpers)
- An alternative IK solver (`G1_29_ArmIK`)
- DDS command init helpers, remote controller readers, IMU frame helpers
- Startup sequence and safety gating for the real robot

**Policy: this codebase is for patterns only — specifically, how to safely talk to G1 hardware, command structure, and what the startup sequence looks like.** Do not:

- Import or vendor any of its modules
- Copy its IK solver (we have our own — `G1HandIK`)
- Use its `fairmotion` rotation helpers (we use scipy)
- Pull in Vive tracker code (we use ArUco markers)

When in doubt: read the reference for patterns, then write our own version that uses our existing utilities. Document the inspiration in a comment, but the code must be ours.

## Pipeline Overview

### Per-timestep data flow (Phase 2 rotation, current baseline)

```
t (time)
↓
theta = sine_source(t, amplitude, freq)              [trajectory/sources.py]
↓
L_target, R_target = compute_hand_targets(...)       [trajectory/targets.py]
   4×4 SE3 matrices in pelvis frame
↓
current_q, current_dq = read_joint_state(...)        [sim/coppelia_utils.py]
   Read from CoppeliaSim
↓
sol_q, _ = ik.solve_ik(L_target, R_target,           [kinematics/ik_solver.py]
                       current_q, current_dq)
   IPOPT optimization with smoothing
↓
set_joint_positions(sim, handles, sol_q)             [sim/coppelia_utils.py]
↓
sim.step() + time.sleep()                            [controllers/pose_tracking.py]
```

### After Session 3 (ArUco-driven)

```
camera frame → ArUco detector → marker poses (object, target)
                                 ↓
                       (object marker only at startup) → object initial pose
                       (target marker, per timestep)   → desired object pose
                                 ↓
                       trajectory/sources.py (ArUcoSource)
                                 ↓
                       compute_hand_targets → IK → sim/robot
```

### Three control phases

1. **Phase 1 (3s):** Interpolate EE targets from default standing pose to grasp pose using smoothstep
2. **Phase 2 (10s, will become "while marker is moving"):** Box attached to left hand. Trajectory source (sine now, ArUco later) drives both hands together
3. **Phase 3 (3s):** Interpolate EE targets back to default standing pose

## Key Components

### IK Solver (`kinematics/ik_solver.py`)

**Class:** `G1HandIK`

Solves IK for both arms simultaneously given two SE3 targets in the pelvis frame. Loads the full G1 URDF (53 DOF) via Pinocchio, builds a reduced model locking legs/waist/finger joints down to 14 DOF (7 per arm), looks up `L_ee_link` and `R_ee_link` frame IDs, and runs IPOPT on a CasADi-compiled cost.

**Optimization problem:**
```
Minimize:   50 · ||pos_err||² + 5 · ||rot_err||² + 0.02 · ||q||² + 0.1 · ||q - q_prev||²
Subject to: lower_limits ≤ q ≤ upper_limits
            L_shoulder_roll ≥ 0.15
            R_shoulder_roll ≤ -0.15
Variables:  q ∈ ℝ¹⁴
```

**Reduced model joint order (matches `sol_q` indices 0-13):**
```
0: left_shoulder_pitch    7: right_shoulder_pitch
1: left_shoulder_roll     8: right_shoulder_roll
2: left_shoulder_yaw      9: right_shoulder_yaw
3: left_elbow            10: right_elbow
4: left_wrist_roll       11: right_wrist_roll
5: left_wrist_pitch      12: right_wrist_pitch
6: left_wrist_yaw        13: right_wrist_yaw
```

**Smoothing:** `WeightedMovingFilter` with weights `[0.4, 0.3, 0.2, 0.1]` over the last 4 solutions.

### Trajectory (`trajectory/targets.py` and `trajectory/sources.py`)

**`targets.py`** holds `compute_hand_targets(object_center, object_half_width, theta)` — the geometry that turns an object pose into hand SE3 targets. Hand positions arc on a circle of radius `object_half_width` around the object center in the Y-Z plane. Orientations are `Rx(theta) @ R_BASE`.

**`sources.py`** holds the drivers that produce θ (or, post-Session 3, a full target pose) over time. Currently just `sine_source(t, amplitude, freq)`. Session 3 adds `ArUcoSource` with the same interface so the controller doesn't need to change.

**Measured base values (from Pinocchio FK at zero config):**
```python
L_EE_DEFAULT = [0.3549,  0.1341, 0.0952]
R_EE_DEFAULT = [0.3549, -0.1281, 0.0952]

R_L_BASE = [[ 0, -1,  0], [ 1,  0,  0], [ 0,  0,  1]]
R_R_BASE = [[ 0, -1,  0], [-1,  0,  0], [ 0,  0, -1]]
```

### Controller (`controllers/pose_tracking.py`)

`run_pose_tracking(...)` orchestrates the three phases. Reads tunables from `configs/default.yaml`. Uses `step_realtime(sim, control_dt)` for consistent timing.

### URDF (`assets/g1_with_hands.urdf`)

Modified from the Unitree G1 29-DOF URDF. Key modifications:

- Removed floating base joint (CoppeliaSim incompatible)
- Removed `<mujoco>` tag
- Added dummy spheres to empty sensor links (`imu_in_torso`, `imu_in_pelvis`, `d435_link`, `mid360_link`, finger tips)
- Merged inspire hand kinematic chains with attachment offsets:
  - Left: `xyz="0.0415 0.003 0" rpy="0 0 1.5708"`
  - Right: `xyz="0.0415 -0.003 0" rpy="3.14159 0 -1.5708"`
- Replaced multi-element hand base collisions with single box `[0.035, 0.15, 0.07]` at `xyz="0 -0.1136 0"`
- Added `L_ee_link`/`R_ee_link` at `xyz="-0.0175 -0.1136 0"` (inner palm surface)
- Converted `.glb` visual meshes to `.obj` (CoppeliaSim doesn't support `.glb`)

## Critical Assumptions

These are baked into the current implementation. Changes here have wide impact:

1. **Robot is fixed in space.** Pelvis doesn't move. Pelvis frame = robot base frame for IK targets.
2. **World frame ≈ pelvis frame.** Since the robot doesn't move, no transformation needed *for IK targets*. ArUco markers will be in camera frame and will need transforming — that's a Session 2/3 problem.
3. **Object dimensions are known inputs.** Width and depth come from CLI args or config; only pose is sensed.
4. **Object is attached to left hand during rotation.** Bypasses the grip-force problem in sim. Right hand follows because IK keeps both in correct relative positions. Session 4 will replace this with actual finger control.
5. **Symmetric grasp.** Both hands target the same object center with mirrored Y offset.
6. **Rotation axis is robot X (forward).** Single-axis rotation only in the current baseline. Session 3 may extend to multi-axis depending on the target-marker tracking decision.
7. **Only shoulder_roll bounds for self-collision avoidance.** Arms can still collide with each other or external objects. No SDF-based collision check in IK yet.
8. **Position control only.** No torque control or impedance control yet.
9. **Sine wave θ is a placeholder.** Will be replaced by ArUco in Session 3.

## Open Design Questions

These are unresolved and will be decided in their respective sessions:

- **Orientation-only vs full 6-DoF tracking** (Session 3): when the target marker moves in space, does the held object follow that translation, or only match orientation? Probably a CLI flag.
- **Target filtering** (Session 3): how aggressively to smooth the target-marker pose to avoid IK twitching from camera noise.
- **Camera mounting** (Session 2): is the camera on the robot head, fixed external, or something else? Affects perception-to-pelvis frame transforms.
- **Grip pressure** (Session 4): closed-loop with force feedback, or just a fixed wrist offset baked into the grasp pose?

## Coding Conventions

- **Imports at top of file**, grouped: stdlib, third-party, local
- **Type hints are optional** but use them for public APIs
- **Print statements with `[ModuleName]` prefix** for traceability (e.g., `print("[G1HandIK] ...")`)
- **Constants in UPPER_SNAKE_CASE** at module level
- **Tunable params go in `configs/default.yaml`**, safety-critical constants stay in code (see Hard Rules)
- **Numpy preferred over lists** for vector/matrix data
- **SE3 represented as 4x4 numpy arrays** unless using `pin.SE3` for Pinocchio interop
- **All angles in radians**, all distances in meters
- **Pelvis frame is the default reference frame** for IK targets, hand positions, object positions
- **Real-time pacing** via `step_realtime(sim, control_dt)` — never bare `sim.step()` in control loops
- **Don't reach into private `_attrs`** of other modules; route through public functions

## Gotchas and Lessons Learned

### CoppeliaSim

- URDF import via `simURDF.importFile()` not `simURDF.import_()` (Python keyword conflict)
- `.glb` visual meshes not supported — must convert to `.obj`
- Empty links (no visual or collision) crash the URDF importer — add tiny sphere visual
- `floating` joint type not supported — remove before import
- `<mujoco>` tag confuses the importer — remove it
- `sim.removeObject` is deprecated — use `sim.removeObjects([h], False)`
- `sim.setObjectInt32Param(h, sim.objectproperty_depthinvisible, 1)` doesn't work — use `setObjectSpecialProperty` instead
- Shape orientations from `getObjectMatrix` are stored column-major: `[matrix[0], matrix[4], matrix[8]]` is the first column

### Pinocchio

- Version 4.x: pass package paths as list `[path]` not string `path` in `BuildFromURDF`
- Fixed joints get merged into parent body but frames remain accessible via `getFrameId`
- Joint frame positions DO change with parent joint rotations (frames are kinematic, not just anchors)
- `RobotWrapper.BuildFromURDF` only works with conda-forge Pinocchio (`conda install pinocchio -c conda-forge`), not the pip package named "pinocchio"
- The `RuntimeWarning: DeprecatedBool already registered` is harmless

### URDF

- URDF `rpy` is fixed-axis (extrinsic) rotation: `R = Rz(yaw) · Ry(pitch) · Rx(roll)`
- Joint origin xyz is in the PARENT link's local frame
- Collision shape origin xyz is in the LINK's local frame
- Mesh paths are relative to the directory containing the URDF
- Children inherit parent rotations transitively — local frame conventions accumulate through the chain

### Frame conventions

- Hand base local frame at zero config: `local_X = +Y_world`, `local_Y = -X_world`, `local_Z = +Z_world` (for left hand)
- "Forward in world" = `-local_Y` direction in the hand base frame
- The EE position offset of `xyz="-0.0175 -0.1136 0"` exists because `-0.1136` moves forward and `-0.0175` moves to the inner palm

## Standard Verification Workflow

When making changes to IK, trajectory, or URDF, run these checks before launching the full controller:

1. **Pinocchio FK at zero config:** `python scripts/verify_ik.py` — verify EE positions and orientations are as expected
2. **Pinocchio FK at default arm pose:** verify natural standing pose is reachable
3. **IK round-trip:** `pytest tests/test_kinematics.py` — solve IK for the default EE pose, check position/orientation errors are small
4. **Visual check in CoppeliaSim:** `python scripts/setup_scene.py` — import URDF, verify collision boxes align with visual meshes
5. **Then run the full controller:** `python scripts/run_pose_tracking_sim.py`

## When in Doubt

- Read this file first
- Check the relevant module's docstring at the top of the file
- Verify with Pinocchio FK before changing anything in `trajectory/`
- Run `python scripts/verify_ik.py` — it's the standalone IK self-test
- For CoppeliaSim issues, the diagnostic pattern is: get the object handle, get its position/orientation/matrix, print all values
- For any "should I use code from the reference repo?" question — the answer is no. See Reference Codebase Policy.
