# G1 Spine Stabilization — Claude Reference

This file gives Claude (and Claude Code) the context needed to work effectively on this project. Read it fully at the start of any session.

## Project Goal

Build a control pipeline for a Unitree G1 humanoid robot to perform bimanual rigid-body pose tracking: the robot holds an object in both hands and matches its pose to a moving target reference. Currently developed in CoppeliaSim simulation; long-term goal is transfer to the real G1 robot.

The intended real-world system uses two AprilTag markers (dictionary `DICT_APRILTAG_36h11`, 4 cm each) viewed by a head-mounted RealSense D435:

- **Object marker** (on top of the held object): gives the object's actual starting pose at grasp time, since real-world placement varies between trials
- **Target marker** (held by the user or attached to a moving jig): the pose the held object must track during the rotation phase

Object width and depth are passed as runtime arguments — markers are used for pose only, not dimensions.

## Current Status

**Working baseline:** Robot loads cleanly into CoppeliaSim. IK solves both arms simultaneously with self-collision avoidance via tightened shoulder_roll bounds. Box is created, grasped, attached to the left hand, rotated about X by a sine-wave θ trajectory, then released. Three-phase controller: interpolate to grasp → rotate → interpolate back.

The repo is a proper Python package (configs, tests, entry-point scripts). **Perception (Session 2) is complete**: AprilTag detector + frame-transform stack + hardware-free tests; `scripts/visualize_markers.py` runs in `--source realsense` and `--source mock`. **51 tests pass.**

**Direction shift (Session 0, SDK discovery — complete):** we are adopting the external `g1_inspire_sdk` as the hardware transport + preview + camera layer. Source-grounded analysis is in `docs/sdk_analysis.md`. The adoption is **additive** — CoppeliaSim stays as the sim of record; the SDK becomes a parallel hardware/preview path.

**SDK-2 — COMPLETE** (perception re-platform): `RealSenseSource` now wraps `g1_inspire_sdk.camera.RealSenseCamera`. 6 new hardware-free tests; 35 tests pass total. `T_optical_to_d435` hand-calibrated (`rpy: [3.14159, 0.0, 1.5708]` in `configs/default.yaml`).

**SDK-3+4 — COMPLETE** (IK transport + preview): `sdk_pose_tracking.py` drives `G1InspireRobot` via `arm.set_joint_targets(q14)`; `SineSource` + `MarkerSource` in `trajectory/sources.py`; `run_pose_tracking_preview.py` (3 source modes: `sine|mock|marker`, 2 tracking modes: `orient-only|6dof`) is the dry-run preview platform and the pre-flight gate before every live bringup. **51 tests pass.**

**SDK-5 — COMPLETE** (grip force baseline + grasp-demo jerk fix): `GraspController` in `grip_force.py`; `scripts/grasp_demo.py` for bimanual hand-only validation. Phase 0 jerk fixed: interpolation now starts from `ik.fk(current_q)` rather than hardcoded zero-config FK positions — mirrors the Phase R pattern. **51 tests pass.**

**SDK-5.5 — COMPLETE.** Gravity compensation for Inspire hands: runtime Pinocchio inertia correction in `G1HandIK.__init__` (1.0 kg measured hand, URDF was 0.14 kg); `sol_tauff` now routed to `robot.arm.set_joint_targets(sol_q, sol_tauff)` via the SDK tau feedforward channel. Gated by `gravity_comp.enabled` config flag. 5 new hardware-free tests (56 total). SDK-6 gate is now clear.

**SDK-6-safety — COMPLETE.** Guaranteed safe shutdown phase-out. Root cause: `grasp_demo.py`'s `main()` had no `except` clause at all, so a Ctrl-C/exception during the grasp/hold phases skipped hand-release and arm-return-to-default entirely, falling straight from an actively-loaded grasp pose into the SDK's fixed one-shot damping command (`G1Arm.shutdown()`: `kp=0, kd=8, tau=0`, unmodifiable — vendored SDK). Fix: grasp phase logic moved to `controllers/grasp_sequence.py` (`run_grasp_phases`), restructured so hand-release + arm-return-to-default always run, even after Ctrl-C/exceptions, and a brief settle pause (`shutdown.settle_time` in config, default 0.4s) holds the final pose immediately before `robot.shutdown()` — the SDK's 250 Hz publisher thread already re-publishes the last `set_joint_targets()` call for free, so the settle is a plain `time.sleep()`. `sdk_pose_tracking.py`'s Phase 1 got the same `KeyboardInterrupt` treatment Phase 2 already had, plus the same settle pause at the end of Phase 3. `run_pose_tracking_preview.py` gained `--dry-run`/`--no-dry-run`/`--net` flags (mirroring `grasp_demo.py`'s existing convention) so pose-tracking with `mock`/`marker` sources can now actually be run live — this replaces the not-yet-built `run_pose_tracking_sdk.py` named in earlier roadmap drafts. 6 new hardware-free tests (62 total).

**SDK-6 jerk/smoothness fixes — COMPLETE.** First live hardware test (`run_pose_tracking_preview.py --source mock --tracking 6dof --no-dry-run`) surfaced two bugs. (1) Startup jerk: `sdk_pose_tracking.py`'s Phase 1 interpolated from the hardcoded `L_EE_DEFAULT`/`R_EE_DEFAULT` constants instead of `ik.fk(current_q)` — same bug class already fixed in `grasp_sequence.py`'s Phase 0 during SDK-5, just never ported to this file; now fixed identically, mirroring Phase 3's existing pattern in the same file. (2) Jerky tracking: measured `ik.solve_ik()` at ~100ms/call, meaning the real Phase-2 loop runs at ~6-10Hz, not the assumed 50Hz — since `SineSource`/mock sample a real-time sine, this inflated per-step target deltas 5-9x. New `RateLimitedSource` (`trajectory/sources.py`) slew-rate-limits `(box_center, theta)` against the actual measured dt before it reaches the IK, resolving the long-standing "Target filtering" Open Design Question. Sine default frequency lowered `0.3`→`0.1` Hz. 10 new hardware-free tests (72 total). Speeding up the IK solve itself is deliberately deferred — see "Known timing floor" in the IK Solver section.

**Env (manual install — complete):** the unified env is named **`g1_inspire_sdk`** and was built by hand following the SDK's README. Our repo is installed into it. SDK examples **01 (preview), 06 (live pick-and-place), and 07 (live camera)** all run successfully — the live-hardware tier (CycloneDDS + `unitree_sdk2py` + `inspire_sdkpy` + `pyrealsense2`) is proven end-to-end. Two items remain from SDK-1: run our own 29 tests in this env (regression check), and write the per-machine setup record (`docs/sdk_setup.md`). Both are handled as preflight in SDK-2; no dedicated env session is required.

## Session Roadmap

**Original pipeline (CoppeliaSim track):**

- **Session 1 — COMPLETE.** Repo restructure, packaging, configs, tests.
- **Session 2 — COMPLETE.** AprilTag perception module (standalone). 29 tests.

**SDK adoption track (additive; built on `g1_inspire_sdk`):**

- **Session 0 — COMPLETE.** Read-only SDK discovery. See `docs/sdk_analysis.md`.
- **SDK-1 — COMPLETE (manual).** Env `g1_inspire_sdk` built by hand per the SDK README; our repo installed into it; SDK examples 01/06/07 verified (06 ran on the live robot). Test regression + setup record fold into SDK-2 preflight.
- **SDK-2 — COMPLETE.** Perception re-platform: `RealSenseSource` now wraps `g1_inspire_sdk.camera.RealSenseCamera`; 6 new tests; 35 total. `T_optical_to_d435` hand-calibrated.
- **SDK-3 + SDK-4 — COMPLETE (merged).** IK transport adapter + marker-driven source: `sdk_pose_tracking.py` feeds `G1HandIK` output to `arm.set_joint_targets(q14)`; `MarkerSource` + `SineSource` in `trajectory/sources.py`; `run_pose_tracking_preview.py` multi-mode preview script (`--source sine|marker|mock`, `--tracking orient-only|6dof`). 51 tests total.
- **SDK-5 — COMPLETE.** Grip force baseline: `GraspController` in `grip_force.py`; `scripts/grasp_demo.py` for bimanual hand-only validation (Phase 0 jerk fixed); `docs/ik_weight_analysis.md` IK weight/payload analysis doc.
- **SDK-5.5 — COMPLETE.** Gravity compensation: runtime Pinocchio inertia correction; `sol_tauff` wired through `robot.arm.set_joint_targets(sol_q, tau_ff)`; 5 new tests.
- **SDK-6-safety — COMPLETE.** Guaranteed safe shutdown: `grasp_demo.py` phases moved to `controllers/grasp_sequence.py` so hand-release + arm-return-to-default always run before `robot.shutdown()`, even on Ctrl-C/exceptions; settle pause (`shutdown.settle_time`) added before shutdown in both `grasp_demo.py` and `sdk_pose_tracking.py`; `run_pose_tracking_preview.py` gained `--dry-run`/`--no-dry-run`/`--net` flags. 6 new tests.
- **SDK-6 jerk/smoothness fixes — COMPLETE.** First live test surfaced a startup-jerk bug (Phase 1 in `sdk_pose_tracking.py` fixed to start from `ik.fk(current_q)`, matching Phase 3/`grasp_sequence.py`'s existing pattern) and jerky tracking (root-caused to `ik.solve_ik()` costing ~100ms/call — real loop rate ~6-10Hz vs. assumed 50Hz; mitigated with a new `RateLimitedSource` slew-rate limiter on the trajectory target, config in `trajectory.rate_limit`). Sine default frequency lowered to `0.1` Hz. 10 new tests (72 total).
- **SDK-6 = original Session 5.** Live-robot bringup: run `scripts/grasp_demo.py --no-dry-run` to validate both hands; complete the full pre-flight sequence in `run_pose_tracking_preview.py` (sine → mock → marker); then run `run_pose_tracking_preview.py --no-dry-run --source marker`. **Human-run only** — robot mounted, E-stop in reach.

## File Structure

Current state of the repo:

```
g1_spine_stabilization/
├── README.md
├── LICENSE
├── .gitignore
├── requirements.txt
├── setup.py
├── CLAUDE.md
│
├── configs/
│   └── default.yaml                 # tunable params; perception section added in Session 2
│
├── assets/
│   ├── g1_with_hands.urdf
│   ├── inspire_hand/
│   │   ├── inspire_hand_left.urdf
│   │   ├── inspire_hand_right.urdf
│   │   └── meshes/
│   └── meshes/                      # G1 body STLs
│
├── g1_spine_stabilization/          # the package
│   ├── __init__.py
│   ├── kinematics/
│   │   ├── __init__.py
│   │   └── ik_solver.py             # G1HandIK (Pinocchio + CasADi)
│   ├── controllers/
│   │   ├── __init__.py
│   │   ├── pose_tracking.py         # run_pose_tracking_sim, three-phase controller (CoppeliaSim)
│   │   ├── sdk_pose_tracking.py     # added SDK-3/4; run_pose_tracking_sdk, three-phase via G1InspireRobot
│   │   ├── grip_force.py            # GraspController; config-driven baseline hold (SDK-5)
│   │   └── grasp_sequence.py        # added SDK-6-safety; run_grasp_phases, guaranteed safe-return before shutdown
│   ├── trajectory/
│   │   ├── __init__.py
│   │   ├── targets.py               # compute_hand_targets + constants
│   │   └── sources.py               # SineSource, MarkerSource (SDK-3/4); RateLimitedSource (jerk/smoothness fix)
│   ├── perception/                  # populated in Session 2
│   │   ├── __init__.py
│   │   ├── types.py                 # MarkerObservation dataclass
│   │   ├── frame_sources.py         # RealSenseSource, VideoFileSource, ImageFileSource
│   │   ├── detector.py              # AprilTagDetector (cv2.aruco + solvePnP)
│   │   ├── tracker.py               # MarkerTracker, MockMarkerTracker
│   │   └── frames.py                # camera optical → d435_link → pelvis composition
│   ├── sim/
│   │   ├── __init__.py
│   │   ├── coppelia_utils.py        # joint handles, read/write state, box helpers
│   │   └── scene_setup.py           # import_g1_urdf()
│   └── utils/
│       ├── __init__.py
│       ├── transforms.py            # SE3 math (make_se3, slerp, smoothstep, Rx)
│       └── config.py                # load_config()
│
├── scripts/
│   ├── run_pose_tracking_sim.py     # main entry point (CoppeliaSim)
│   ├── run_pose_tracking_preview.py # added SDK-3/4; --source sine|marker|mock, --tracking orient-only|6dof; added SDK-6-safety: --dry-run/--no-dry-run/--net
│   ├── grasp_demo.py                # added SDK-5; bimanual hand open/close validation (--dry-run default); thin wrapper around grasp_sequence.run_grasp_phases (SDK-6-safety)
│   ├── setup_scene.py               # one-time URDF import
│   ├── verify_ik.py                 # FK + IK self-test (no sim needed)
│   └── visualize_markers.py         # added in Session 2; --source realsense|mock, --dry-run flag
│
├── tests/
│   ├── __init__.py
│   ├── test_kinematics.py
│   ├── test_trajectory.py
│   ├── test_transforms.py
│   ├── test_detector.py             # added in Session 2; uses synthetic markers
│   ├── test_perception_frames.py    # added in Session 2; pure math
│   ├── test_mock_tracker.py         # added in Session 2; mock API contract
│   ├── test_realsense_source_sdk.py # added SDK-2; 6 dry_run tests for SDK-backed RealSenseSource
│   ├── test_sdk_transport.py        # added SDK-3; dry_run lifecycle tests for G1InspireRobot
│   ├── test_marker_source.py        # added SDK-3/4; MarkerSource unit tests via MockMarkerTracker
│   ├── test_gravity_comp.py         # added SDK-5.5; 5 hardware-free gravity comp tests
│   ├── test_grasp_demo_safety.py    # added SDK-6-safety; 4 hardware-free safe-return/settle tests
│   └── test_rate_limited_source.py  # added SDK-6 jerk/smoothness fix; 9 hardware-free slew-limiter tests
│
└── docs/
    ├── architecture.md
    ├── frame_conventions.md
    ├── sdk_analysis.md              # SDK discovery notes (Session 0)
    ├── sdk_setup.md                 # per-machine env ledger (written in SDK-2 preflight)
    ├── ik_weight_analysis.md        # IK weight/payload brainstorm doc (SDK-5; paste into Claude chat)
    └── followups.md                 # living notes from each session
```

## Hard Rules

- **Never run live-robot code in a Claude Code session.** Any command that connects to or commands the physical robot — non-dry-run `connect()` / `initialize()` / `start_streaming()` / `step()`, DDS bring-up against a real network interface, any motor or hand command to hardware — is run by a **human only**, with the robot mounted and an E-stop in reach. `initialize()` moves the arm to a default pose and **counts as actuation**. Claude Code may *install* and *import-verify* the hardware bindings, and may run **dry-run** (`dry_run=True`) and **live-camera** checks (the camera does not actuate the robot). For anything live on the robot, Claude Code **writes** the script (e.g. `scripts/live_robot_check.py`) and stops; the human executes it.
- **Never modify `assets/g1_with_hands.urdf` without explicit approval.** It's been carefully constructed and tuned. Changes propagate to both Pinocchio and CoppeliaSim.
- **Never change the EE frame definitions** (`L_ee_link`, `R_ee_link`) without discussing first. They define the IK target convention.
- **Never change the shoulder_roll bounds** (`L_SHOULDER_ROLL_MIN = 0.15`, `R_SHOULDER_ROLL_MAX = -0.15`) without explicit approval. They prevent self-collision with the torso. These stay as code constants in `ik_solver.py`, NOT in the YAML config.
- **Never change the IK cost weights** (50 trans, 5 rot, 0.02 reg, 0.1 smooth) without approval. They stay as code constants.
- **Always preserve the joint ordering** in `ARM_JOINT_NAMES` — it must match the IK solver's reduced model order exactly.
- **Don't use `simURDF.import_`** in CoppeliaSim Python API — `import` is a Python keyword. Use `simURDF.importFile()` instead.
- **Don't add new dependencies without discussion.** The unified env is `g1_inspire_sdk` (one env for our repo + SDK + live hardware + camera). **No `environment.yml` is committed to this repo** — the live-hardware tier (source-built CycloneDDS, local-path `inspire_sdkpy`, git-installed `unitree_sdk2py`) is machine-specific and not portable via conda export. Instead, **`docs/sdk_setup.md`** records this machine's specifics: CycloneDDS install location, `inspire_sdkpy` path, resolved versions, and any deviations from the SDK README. The SDK README is the rebuild recipe; our setup doc is the per-machine ledger. Core stack in the env: numpy, scipy, pinocchio (**conda-forge**, with CasADi bindings), casadi, pyyaml, `coppeliasim_zmqremoteapi_client`, pytest, `opencv-contrib-python`, pyrealsense2, plus the SDK (`g1_inspire_sdk`) with its deps (`rerun-sdk`, `cyclonedds==0.10.2`, `unitree_sdk2py`, `inspire_sdkpy`, `pymodbus`). Pinocchio is conda-forge only. `pyrealsense2` stays lazy-imported. New libs need justification.
- **`pyrealsense2` should be lazy-imported** inside `RealSenseSource.__init__` so the perception module works on machines without it.
- **Keep `G1HandIK`; never adopt the SDK's IK.** Drive the arm by feeding our `solve_ik` output to `arm.set_joint_targets(q14)` (identity hand-off — our joint order matches the SDK's). **Never** use `arm.set_ee_targets(...)`: it runs the SDK IK (which lacks our shoulder-roll bounds, weakens orientation 5×, retargets the EE) and double-filters.
- **Do not import `g1_inspire_sdk._common.weighted_moving_filter`** — it pulls `matplotlib` at module top. Use our own filter.
- **Do not attempt SDK-6 live bringup until gravity compensation is implemented (SDK-5.5).** Without hand-mass gravity compensation, wrist joints under position-only control will droop under the Inspire hand's weight, and arm behavior at the live-robot stage may be unsafe.

## External SDK (dependency, not patterns)

The `g1_inspire_sdk` (cloned at `~/g1_inspire_sdk`, installed into the `g1_inspire_sdk` env) is a **dependency we build on** — unlike the `g1_data_collection` reference, which is patterns-only. We *do* import and call the SDK's arm/hand/camera/preview transport. We do **not** modify the clone (`git pull` only), and we do **not** use its IK or its `weighted_moving_filter` (see Hard Rules). Decisions and exact call surface: `docs/sdk_analysis.md`. Setup specifics for this machine: `docs/sdk_setup.md`.

## Reference Codebase Policy

**Note:** this policy covers `g1_data_collection` only. The `g1_inspire_sdk` is a dependency we build on, not a patterns-only reference — see "External SDK" above.

A separate codebase for G1 teleoperation and data collection may be provided as a reference (the `g1_data_collection` repo or similar). It contains Vive tracker integration, a vendored `fairmotion` library, an alternative IK solver, DDS command helpers, and startup/safety code for the real robot.

**Policy: this codebase is for patterns only — specifically, how to safely talk to G1 hardware, command structure, and what the startup sequence looks like.** Do not:

- Import or vendor any of its modules
- Copy its IK solver (we have our own — `G1HandIK`)
- Use its `fairmotion` rotation helpers (we use scipy)
- Pull in Vive tracker code (we use AprilTags)

When in doubt: read the reference for patterns, then write our own version that uses our existing utilities. The code must be ours.

## Pipeline Overview

### Per-timestep data flow (current baseline, sine-driven)

```
t (time)
↓
theta = sine_trajectory(t, amplitude, freq)          [trajectory/sources.py]
↓
L_target, R_target = compute_hand_targets(...)       [trajectory/targets.py]
   4×4 SE3 matrices in pelvis frame
↓
current_q, current_dq = read_joint_state(...)        [sim/coppelia_utils.py]
↓
sol_q, _ = ik.solve_ik(L_target, R_target,           [kinematics/ik_solver.py]
                       current_q, current_dq)
↓
set_joint_positions(sim, handles, sol_q)             [sim/coppelia_utils.py]
↓
sim.step() + step_realtime(...)                      [controllers/pose_tracking.py]
```

### Perception layer (Session 2 — standalone, not yet wired in)

```
Frame source (RealSense | VideoFile | ImageFile)
   produces (frame, timestamp)
       ↓
Detector (AprilTagDetector)
   wraps cv2.aruco.ArucoDetector + cv2.solvePnP
   returns [(marker_id, pose_in_camera_optical_frame), ...]
       ↓
Marker tracker (MarkerTracker)
   composes camera optical → d435_link → pelvis
   exposes: step() → list[MarkerObservation]
            get_latest(marker_id) → MarkerObservation | None

Mock variant (MockMarkerTracker)
   returns pre-set deterministic poses
   no frame source, no detector — for unit tests and Session 3 integration testing
```

The transform chain is `pose_in_pelvis = T_d435_to_pelvis @ T_optical_to_d435 @ pose_in_optical`. `T_d435_to_pelvis` comes from Pinocchio FK at the locked head/waist joint configuration. `T_optical_to_d435` is a fixed rotation in config — hand-calibrated: `rpy: [3.14159, 0.0, 1.5708]` in `configs/default.yaml`.

**Note:** `d435_to_pelvis_from_pinocchio()` in `frames.py` accepts an optional `ik` parameter. Pass the already-loaded `G1HandIK` instance to avoid re-instantiating Pinocchio on every call.

### After Session 3 (perception-driven controller)

```
camera frame → marker tracker → object pose (at startup) + target pose (per step)
                                    ↓
                           trajectory/sources.py (MarkerSource)
                                    ↓
                           compute_hand_targets → IK → sim/robot
```

### Three control phases

1. **Phase 1 (3s):** Interpolate EE targets from default standing pose to grasp pose using smoothstep
2. **Phase 2 (10s, will become "while marker is moving"):** Box attached to left hand. Trajectory source drives both hands together
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

**Known accuracy floor:** With the locked cost weights, IK round-trip position error settles at ~1.8 mm at the default pose due to the regularization term. This is documented in tests as the `5e-3` tolerance. It eats into the perception error budget — see Open Design Questions.

**Known timing floor:** `control.dt: 0.02` (50Hz) in `configs/default.yaml` is the configured control-period assumption, not an achieved one. Measured: `solve_ik()` costs ~100ms/call (CasADi/IPOPT re-solve from warm start; `n=55 mean=104.30ms median=100.89ms min=67.97ms max=161.97ms`), so the real Phase-2 loop rate is ~6-10Hz, and `time.sleep(control_dt)` is unconditional (no compute-time compensation). This is not fixed, only mitigated — see `RateLimitedSource` below and `docs/followups.md` for the measurement and reasoning. Speeding up the solve itself (IPOPT `max_iter`/tolerances) is deliberately out of scope — deferred to a separate future effort since it touches the documented accuracy floor above.

### Trajectory (`trajectory/targets.py` and `trajectory/sources.py`)

**`targets.py`** holds `compute_hand_targets(box_center, box_half_width, theta)` — the geometry that turns an object pose into hand SE3 targets. Hand positions arc on a circle of radius `box_half_width` around the object center in the Y-Z plane. Orientations are `Rx(theta) @ R_BASE`.

**`sources.py`** holds the drivers that produce θ over time. `SineSource` wraps `sine_trajectory(t, amplitude, freq)`. `MarkerSource` (added SDK-3/4) reads AprilTag poses from a `MarkerTracker` and supports `"orient-only"` and `"6dof"` modes. `RateLimitedSource` (added in the jerk/smoothness fix) wraps any of the above, slew-rate-limiting `(box_center, theta)` against the actual measured wall-clock dt before it reaches `compute_hand_targets`/`solve_ik` — see "Known timing floor" above and the Open Design Questions "Resolved" list.

**Measured base values (from Pinocchio FK at zero config):**
```python
L_EE_DEFAULT = [0.3549,  0.1341, 0.0952]
R_EE_DEFAULT = [0.3549, -0.1281, 0.0952]

R_L_BASE = [[ 0, -1,  0], [ 1,  0,  0], [ 0,  0,  1]]
R_R_BASE = [[ 0, -1,  0], [-1,  0,  0], [ 0,  0, -1]]
```

`ARM_DEFAULT_Q` (the natural standing pose) produces EE positions about 13 mm away from `L_EE_DEFAULT`/`R_EE_DEFAULT` — they aren't the same pose. The default pose is the resting configuration; the defaults are the zero-config FK output. Tests use a 2.5 cm tolerance to verify they're in the same neighborhood.

### Controller (`controllers/pose_tracking.py`)

`run_pose_tracking_sim(...)` orchestrates the three phases. Reads tunables from `configs/default.yaml`. Uses `step_realtime(sim, control_dt)` for consistent timing.

### Perception (`perception/`, populated in Session 2)

**Three-layer architecture, each independently replaceable:**

1. **Frame source** (`frame_sources.py`): produces `(frame, timestamp)`. Implementations include `RealSenseSource` (live D435, wraps `g1_inspire_sdk.camera.RealSenseCamera`; lazy-imported; `dry_run=True` returns synthetic gradient frames), `VideoFileSource` (mp4 replay), `ImageFileSource` (single still). Hardware-dependent code is isolated here.

2. **Detector** (`detector.py`): pure function from frame + camera intrinsics → list of `(marker_id, pose_in_camera_optical_frame)`. Wraps `cv2.aruco.ArucoDetector` with `DICT_APRILTAG_36h11`. Uses `cv2.solvePnP` for pose (the deprecated `estimatePoseSingleMarkers` is not used). Stateless.

3. **Marker tracker** (`tracker.py`): composes frame source + detector, applies camera→pelvis transform, exposes the API that Session 3 will consume. `MockMarkerTracker` is the no-hardware variant returning deterministic synthesized poses.

**Frame composition** (`frames.py`):
```
pose_in_pelvis = T_d435_to_pelvis @ T_optical_to_d435 @ pose_in_optical
```
`T_d435_to_pelvis` is computed once from Pinocchio FK at the locked joint configuration. `T_optical_to_d435` is a fixed rotation from config — hand-calibrated: `rpy: [3.14159, 0.0, 1.5708]` in `configs/default.yaml`.

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

1. **Robot is fixed in space.** Pelvis doesn't move. Pelvis frame = robot base frame for IK targets.
2. **Head and waist joints are locked at zero** in the current controller, so the camera-to-pelvis transform is a constant. If head/waist motion is added later, the perception module needs to recompute the transform per frame.
3. **Object dimensions are known inputs.** Width and depth come from CLI args or config; only pose is sensed.
4. **Object is attached to left hand during rotation.** Bypasses the grip-force problem in sim. Right hand follows because IK keeps both in correct relative positions. Session 4 will replace this with actual finger control.
5. **Symmetric grasp.** Both hands target the same object center with mirrored Y offset.
6. **Rotation axis is robot X (forward).** Single-axis rotation only in the current baseline. Session 3 may extend depending on tracking decision.
7. **Only shoulder_roll bounds for self-collision avoidance.** Arms can still collide with each other or external objects.
8. **Position control only.** No torque or impedance control yet.
9. **Sine wave θ is a placeholder.** Will be replaced by marker tracking in Session 3.

## Open Design Questions

These are unresolved and will be decided in their respective sessions:

- **Error budget**: expected end-to-end accuracy is roughly: ~2-5 mm from marker detection + calibration residual (calibration done; exact residual not yet characterized) + ~2 mm from IK round-trip floor = roughly ~5-15 mm at the EE. If the application requires tighter, the IK cost weights would need revisiting (Hard Rules).

**Resolved:**
- Camera mounting: head-mounted RealSense D435 (uses `d435_link` in URDF).
- Marker spec: AprilTag `DICT_APRILTAG_36h11`, 4 cm markers.
- **Orientation-only vs 6-DoF**: both supported via `--tracking orient-only|6dof` CLI flag in `run_pose_tracking_preview.py`. Orient-only: box center fixed from object marker at startup, theta from target marker Rx. 6-DoF: box center tracks target marker position each step.
- **`T_optical_to_d435` calibration**: done, `rpy: [3.14159, 0.0, 1.5708]` in `configs/default.yaml`.
- **Target filtering** (SDK-3/4, resolved in the jerk/smoothness fix): added `RateLimitedSource` (`trajectory/sources.py`) — a slew-rate clamp on `(box_center, theta)` applied before `compute_hand_targets`, using measured wall-clock dt (not assumed `control_dt`). Architecturally distinct from the IK's `WeightedMovingFilter` (rate limit on the input vs. moving average on the output), so no double-filtering. Config: `trajectory.rate_limit` in `configs/default.yaml`, applied uniformly across `--source sine|mock|marker`.
- **Grip pressure** (SDK-5): start from the SDK's `TransportGraspController` combined with our acceleration policy, rather than a fixed wrist offset.
- **Gravity compensation** (SDK-5.5): option (b) — runtime `pin.Inertia` update in `G1HandIK.__init__`. SDK torque feedforward confirmed via `G1Arm.set_joint_targets(q14, tau_ff14)`. Measured hand mass = 1.0 kg. Acceleration coupling deferred to post-SDK-6.

## Coding Conventions

- **Imports at top of file**, grouped: stdlib, third-party, local
- **Type hints are optional** but use them for public APIs
- **Print statements with `[ModuleName]` prefix** for traceability
- **Constants in UPPER_SNAKE_CASE** at module level
- **Tunable params go in `configs/default.yaml`**, safety-critical constants stay in code
- **Numpy preferred over lists** for vector/matrix data
- **SE3 represented as 4x4 numpy arrays** unless using `pin.SE3` for Pinocchio interop
- **All angles in radians**, all distances in meters
- **Pelvis frame is the default reference frame** for IK targets, hand positions, object positions
- **Real-time pacing** via `step_realtime(sim, control_dt)` — never bare `sim.step()` in control loops
- **Run pytest from the repo root** (`cd <repo>; pytest tests/`). Running from inside `tests/` will fail import resolution.
- **Hardware-dependent imports are lazy** (inside `__init__` of the class that uses them) so the package imports cleanly on machines without the hardware library installed.

## Gotchas and Lessons Learned

### CoppeliaSim

- URDF import via `simURDF.importFile()` not `simURDF.import_()` (Python keyword conflict)
- `.glb` visual meshes not supported — must convert to `.obj`
- Empty links (no visual or collision) crash the URDF importer — add tiny sphere visual
- `floating` joint type not supported — remove before import
- `<mujoco>` tag confuses the importer — remove it
- `sim.removeObject` is deprecated — use `sim.removeObjects([h], False)`
- `sim.setObjectInt32Param(h, sim.objectproperty_depthinvisible, 1)` doesn't work — use `setObjectSpecialProperty` instead
- Shape orientations from `getObjectMatrix` are stored column-major

### Pinocchio

- Version 4.x: pass package paths as list `[path]` not string `path` in `BuildFromURDF`
- Fixed joints get merged into parent body but frames remain accessible via `getFrameId`
- Joint frame positions DO change with parent joint rotations
- `RobotWrapper.BuildFromURDF` only works with conda-forge Pinocchio, not the pip package
- The `RuntimeWarning: DeprecatedBool already registered` is harmless

### URDF

- URDF `rpy` is fixed-axis (extrinsic) rotation: `R = Rz(yaw) · Ry(pitch) · Rx(roll)`
- Joint origin xyz is in the PARENT link's local frame
- Collision shape origin xyz is in the LINK's local frame
- Mesh paths are relative to the directory containing the URDF
- Children inherit parent rotations transitively

### Frame conventions

- Hand base local frame at zero config: `local_X = +Y_world`, `local_Y = -X_world`, `local_Z = +Z_world` (for left hand)
- "Forward in world" = `-local_Y` direction in the hand base frame
- The EE position offset of `xyz="-0.0175 -0.1136 0"` exists because `-0.1136` moves forward and `-0.0175` moves to the inner palm
- **Camera optical frame (OpenCV / REP-103):** X right, Y down, Z forward (into the scene)
- **Camera mount frame (ROS / REP-105):** X forward, Y left, Z up
- The transform between them is a fixed rotation. Standard convention has `rpy = [-π/2, 0, -π/2]` from mount to optical, but verify against the actual `d435_link` definition in our URDF.

### OpenCV ArUco

- Use `cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)` and `cv2.aruco.ArucoDetector(dict, params)` — the function-based API is deprecated
- `cv2.aruco.estimatePoseSingleMarkers` is deprecated in OpenCV ≥4.7 — construct object points manually and call `cv2.solvePnP`
- Marker corner order returned by detector: top-left, top-right, bottom-right, bottom-left when viewing the marker upright
- Object points for `solvePnP` must match that corner order

### Test tolerances (documented Session 1)

- `test_fk_at_default_arm_pose_close_to_defaults`: 2.5 cm tolerance. `ARM_DEFAULT_Q` is the natural standing pose; `L_EE_DEFAULT`/`R_EE_DEFAULT` are FK at zero config. They differ by ~13 mm by design.
- `test_ik_round_trip_at_default_pose`: 5 mm position tolerance. IK regularization pulls solutions ~1.8 mm from the exact answer; tightening would require changing cost weights (Hard Rules).

## Standard Verification Workflow

Run from the repo root:

1. **Pinocchio FK / IK self-test:** `python scripts/verify_ik.py` — verify EE positions and IK round-trip
2. **Unit tests:** `pytest tests/ -v` — 51 tests pass
3. **CoppeliaSim scene setup:** `python scripts/setup_scene.py` — import URDF, visual check
4. **Full controller:** `python scripts/run_pose_tracking_sim.py`
5. **(Session 2+, with hardware) Marker visualizer:** `python scripts/visualize_markers.py`
6. **Grasp demo dry-run:** `python scripts/grasp_demo.py` — validates hand control plumbing + arm IK interpolation. Run all three sides: `--side both`, `--side left`, `--side right`.
7. **Pose tracking preview — no camera:**
   ```
   python scripts/run_pose_tracking_preview.py --source sine
   python scripts/run_pose_tracking_preview.py --source mock --tracking orient-only
   python scripts/run_pose_tracking_preview.py --source mock --tracking 6dof
   ```
   All three must complete without error or visible arm discontinuity in Rerun.
8. **Pose tracking preview — live camera:**
   `python scripts/run_pose_tracking_preview.py --source marker --tracking orient-only`
   Run with two AprilTag markers (IDs matching `configs/default.yaml`) in the D435's field of view. Arms must respond smoothly to target marker motion. Ctrl+C to exit.
9. **(SDK-6, human-run only, after SDK-5.5 gravity comp and SDK-6-safety are complete)** Run `python scripts/grasp_demo.py --no-dry-run` first; confirm hands actuate safely and both the hand-release and arm-return-to-default phases complete even after a mid-hold Ctrl-C. Then run `python scripts/run_pose_tracking_preview.py --no-dry-run --source marker --tracking orient-only` live. Robot mounted, E-stop in reach.

## When in Doubt

- Read this file first
- Check the relevant module's docstring at the top of the file
- Look at `docs/followups.md` for notes from previous sessions
- Verify with Pinocchio FK before changing anything in `trajectory/`
- Run `python scripts/verify_ik.py` — it's the standalone IK self-test
- For CoppeliaSim issues, the diagnostic pattern is: get the object handle, get its position/orientation/matrix, print all values
- For any "should I use code from the reference repo?" question — the answer is no. See Reference Codebase Policy.
