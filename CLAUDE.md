# G1 Spine Stabilization — Claude Reference

This file gives Claude (and Claude Code) the context needed to work effectively on this project. Read it fully at the start of any session.

## Project Goal

Build a control pipeline for a Unitree G1 humanoid robot to perform bimanual rigid-body pose tracking: the robot holds an object in both hands and matches its pose to a moving target reference. Currently developed in CoppeliaSim simulation; long-term goal is transfer to the real G1 robot.

The intended real-world system uses two AprilTag markers (dictionary `DICT_APRILTAG_36h11`, 4 cm each) viewed by a head-mounted RealSense D435:

- **Object marker** (on top of the held object): gives the object's actual starting pose at grasp time, since real-world placement varies between trials
- **Target marker** (held by the user or attached to a moving jig): the pose the held object must track during the rotation phase

Object width and depth are set in `configs/default.yaml`'s `box:` block — an object is selected once and used for a while, so this is a config value, not a per-run CLI argument. Markers are used for pose only, not dimensions.

## Current Status

**Working baseline:** Robot loads cleanly into CoppeliaSim. IK solves both arms simultaneously with self-collision avoidance via tightened shoulder_roll bounds. Box is created, grasped, attached to the left hand, rotated about X by a sine-wave θ trajectory, then released. Three-phase controller: interpolate to grasp → rotate → interpolate back.

The repo is a proper Python package (configs, tests, entry-point scripts). **Perception (Session 2) is complete**: AprilTag detector + frame-transform stack + hardware-free tests; `scripts/visualize_markers.py` runs in `--source realsense` and `--source mock`. **51 tests pass.**

**Direction shift (Session 0, SDK discovery — complete):** we are adopting the external `g1_inspire_sdk` as the hardware transport + preview + camera layer. Source-grounded analysis is in `docs/sdk_analysis.md`. The adoption is **additive** — CoppeliaSim stays as the sim of record; the SDK becomes a parallel hardware/preview path.

**SDK-2 — COMPLETE** (perception re-platform): `RealSenseSource` now wraps `g1_inspire_sdk.camera.RealSenseCamera`. 6 new hardware-free tests; 35 tests pass total. `T_optical_to_d435` hand-calibrated (`rpy: [-1.5708, 0.0, -1.5708]` in `configs/default.yaml`).

**SDK-3+4 — COMPLETE** (IK transport + preview): `sdk_pose_tracking.py` drives `G1InspireRobot` via `arm.set_joint_targets(q14)`; `SineSource` + `MarkerSource` in `trajectory/sources.py`; `run_pose_tracking_preview.py` (3 source modes: `sine|mock|marker`, 2 tracking modes: `orient-only|6dof`) is the dry-run preview platform and the pre-flight gate before every live bringup. **51 tests pass.**

**SDK-5 — COMPLETE** (grip force baseline + grasp-demo jerk fix): `GraspController` in `grip_force.py`; `scripts/grasp_demo.py` for bimanual hand-only validation. Phase 0 jerk fixed: interpolation now starts from `ik.fk(current_q)` rather than hardcoded zero-config FK positions — mirrors the Phase R pattern. **51 tests pass.**

**SDK-5.5 — COMPLETE.** Gravity compensation for Inspire hands: runtime Pinocchio inertia correction in `G1HandIK.__init__` (1.0 kg measured hand, URDF was 0.14 kg); `sol_tauff` now routed to `robot.arm.set_joint_targets(sol_q, sol_tauff)` via the SDK tau feedforward channel. Gated by `gravity_comp.enabled` config flag. 5 new hardware-free tests (56 total). SDK-6 gate is now clear.

**SDK-6-safety — COMPLETE.** Guaranteed safe shutdown phase-out. Root cause: `grasp_demo.py`'s `main()` had no `except` clause at all, so a Ctrl-C/exception during the grasp/hold phases skipped hand-release and arm-return-to-default entirely, falling straight from an actively-loaded grasp pose into the SDK's fixed one-shot damping command (`G1Arm.shutdown()`: `kp=0, kd=8, tau=0`, unmodifiable — vendored SDK). Fix: grasp phase logic moved to `controllers/grasp_sequence.py` (`run_grasp_phases`), restructured so hand-release + arm-return-to-default always run, even after Ctrl-C/exceptions, and a brief settle pause (`shutdown.settle_time` in config, default 0.4s) holds the final pose immediately before `robot.shutdown()` — the SDK's 250 Hz publisher thread already re-publishes the last `set_joint_targets()` call for free, so the settle is a plain `time.sleep()`. `sdk_pose_tracking.py`'s Phase 1 got the same `KeyboardInterrupt` treatment Phase 2 already had, plus the same settle pause at the end of Phase 3. `run_pose_tracking_preview.py` gained `--dry-run`/`--no-dry-run`/`--net` flags (mirroring `grasp_demo.py`'s existing convention) so pose-tracking with `mock`/`marker` sources can now actually be run live — this replaces the not-yet-built `run_pose_tracking_sdk.py` named in earlier roadmap drafts. 6 new hardware-free tests (62 total).

**SDK-6 jerk/smoothness fixes — COMPLETE.** First live hardware test (`run_pose_tracking_preview.py --source mock --tracking 6dof --no-dry-run`) surfaced two bugs. (1) Startup jerk: `sdk_pose_tracking.py`'s Phase 1 interpolated from the hardcoded `L_EE_DEFAULT`/`R_EE_DEFAULT` constants instead of `ik.fk(current_q)` — same bug class already fixed in `grasp_sequence.py`'s Phase 0 during SDK-5, just never ported to this file; now fixed identically, mirroring Phase 3's existing pattern in the same file. (2) Jerky tracking: measured `ik.solve_ik()` at ~100ms/call, meaning the real Phase-2 loop runs at ~6-10Hz, not the assumed 50Hz — since `SineSource`/mock sample a real-time sine, this inflated per-step target deltas 5-9x. New `RateLimitedSource` (`trajectory/sources.py`) slew-rate-limits `(box_center, theta)` against the actual measured dt before it reaches the IK, resolving the long-standing "Target filtering" Open Design Question. Sine default frequency lowered `0.3`→`0.1` Hz. 10 new hardware-free tests (72 total). Speeding up the IK solve itself is deliberately deferred — see "Known timing floor" in the IK Solver section.

**SDK-6 pre-bringup hardening — COMPLETE.** Deeper analysis of jerk/jitter beyond the previous fixes, plus the mock-to-real-marker transition. Six items: (0) reverted `INSPIRE_HAND_MASS` from an accidentally-left `1.6` back to the documented/measured `1.0` kg (gravity-comp torque had been ~60% too strong system-wide); (1) fixed `WeightedMovingFilter`'s reversed recency weighting (it weighted the *oldest* sample heaviest, doubling intended lag — verified against the vendored SDK's own reference implementation using the identical weights); (2) documented (not fixed) that `ik.solve_ik()` holds the Python GIL for ~100% of its duration, stalling the SDK's own 250Hz publisher thread, which additionally uses a fixed assumed tick period rather than measured elapsed time — lives inside the vendored SDK, cannot be modified; (3) added `ipopt_exhaustion_count` diagnostics for `solve_ik()`'s `max_iter` fallback path; (4) added an `ExponentialMovingAverageFilter` smoothing `sol_tauff` before handoff (`gravity_comp.tauff_filter` config), since the SDK applies feedforward torque as an unfiltered step every publisher tick; (5) added `NoiseFilteredSource` (median-of-N + EMA, `trajectory.noise_filter` config) — `RateLimitedSource` only clamps deltas *larger* than its threshold, so real AprilTag sensor noise (small, sub-threshold) would pass through it unfiltered; applied only to `--source marker`, since sine/mock are noiseless by construction. 24 new hardware-free tests (96 total).

**SDK-6 marker-tracking fix — COMPLETE.** A live test with real AprilTag markers using `--tracking 6dof` showed the hands positioning themselves around the target marker instead of the object marker. Root cause (confirmed via direct code read, not a marker-ID mixup): `MarkerSource`'s 6dof mode set `box_center` to the target marker's raw absolute position every step, instead of the object's own starting position shifted by however far the target had moved from *its own* starting position. Wrong whenever object and target are physically in different locations (the realistic setup — object in the robot's hands, target held separately). Fixed: both tracking modes now track the target's *displacement* from its own start (position delta for 6dof, rotation delta for both modes), composed onto the object's actual starting position — see `trajectory/sources.py`'s `MarkerSource`. Also added a Rerun diagnostic (`targets/L_hand`/`targets/R_hand`) showing the actual computed IK hand-target, distinct from the raw marker and the robot's rendered mesh, and diagnostic prints of the marker pose at intermediate transform stages to help investigate a separately-reported ~10cm marker-height discrepancy (still unresolved, needs physical measurement against real hardware). Along the way, ruled out a wrist-vs-palm IK targeting bug (confirmed `solve_ik()`'s cost function targets `L_ee_link`/`R_ee_link`, not the wrist) and corrected several stale doc claims (see Open Design Questions and File Structure). 3 tests rewritten in `tests/test_marker_source.py` to cover the delta semantics.

**Data recording — COMPLETE.** Added the ability to record trial data from both the robot's live tracking runs and a human performing the same physical task, into pickle files sharing one schema so the two can be compared. New `g1_spine_stabilization/data_recording/recorder.py`: `RunRecorder` accumulates per-sample `MarkerSample`s (both markers' poses, a per-axis angle-diff, finite-difference angular velocities, time, plus a `load_cell_wrench` field reserved for the future neck-mounted load cell — always `None` until that sensor exists) and pickles them via an idempotent `save()`. `run_pose_tracking_sdk`'s Phase 2 (`sdk_pose_tracking.py`) gained an optional `recorder` parameter: `record()` runs once per iteration, and `save()` runs in a `finally` guaranteed regardless of how Phase 2 exits (normal timeout or `KeyboardInterrupt`) — additive only, Phase 2's existing try/except is untouched. `run_pose_tracking_preview.py` gained a `--record [PATH]` flag (works with any `--source`; `--source sine` records time/theta only, no real markers). New standalone `scripts/record_human_trial.py` — perception-only (no robot/IK), reuses the same camera/marker stack and `RunRecorder` import as the robot path, so its output is directly comparable. New `data_recording:` config section; `data/runs/` gitignored. Auto-generated filenames carry an explicit `robot_run_...`/`human_run_...` prefix so origin is visible at a glance; the human script's `source_label` is the camera source (`realsense`/`mock`), not a fixed `"human"`, to avoid a redundant name. **`angle_diff_rpy(R_reference, R_moving)` decomposes `R_reference.T @ R_moving` — `RunRecorder.record()` always passes `target` as the reference and `object` as moving** (biomechanics convention: for the neck-trial mapping `target_marker_id`=torso, `object_marker_id`=head, so recorded roll/pitch/yaw describe head rotation in the torso's own axes, matching standard neck-kinematics reporting — this is NOT a symmetric difference and swapping the arguments is not just a sign flip for non-trivial rotations). **`object_angular_velocity`/`target_angular_velocity` are computed from each marker's own camera OPTICAL-frame readings (no pelvis/mount composition at all)**, since their purpose is magnitude-only cross-trial normalization (how much the torso/head was actually moving) rather than axis-level interpretation — `MarkerObservation` gained a `pose_in_optical` field (populated by `MarkerTracker.step()`) to carry this. The two rotation-derived fields deliberately live in different frames for different purposes. Full convention reference: `docs/data_recording_conventions.md`. 24 new hardware-free tests. **192 tests pass total.**

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
- **SDK-6 pre-bringup hardening — COMPLETE.** Deeper jerk/jitter analysis + mock-to-real-marker gap analysis. Reverted an accidentally-left `INSPIRE_HAND_MASS=1.6` back to `1.0`; fixed `WeightedMovingFilter`'s reversed recency-weighting bug (doubled lag); documented (not fixed) GIL contention stalling the SDK's own publisher thread during every IK solve; added `ipopt_exhaustion_count` diagnostics; added `ExponentialMovingAverageFilter` smoothing `sol_tauff` before handoff; added `NoiseFilteredSource` (median+EMA, marker-only) since `RateLimitedSource` structurally cannot catch real AprilTag sensor noise. 24 new tests (96 total).
- **SDK-6 marker-tracking fix — COMPLETE.** Found and fixed a real bug from live-marker testing: `MarkerSource`'s 6dof mode was teleporting `box_center` to the target marker's raw absolute position instead of tracking its displacement from its own starting pose, applied on top of the object's actual position. Both modes now track target displacement (position delta for 6dof, rotation delta for both). Added Rerun hand-target visualization and transform-chain diagnostics; corrected stale doc claims (calibration value, "runtime arguments" line, missing file references).
- **Data recording — COMPLETE.** New `g1_spine_stabilization/data_recording/recorder.py` (`RunRecorder`, `MarkerSample`, `angle_diff_rpy`, `angular_velocity_rotvec`) shared by both a `--record` flag on `run_pose_tracking_preview.py` (hooked into `sdk_pose_tracking.py`'s Phase 2, guaranteed-save via `finally`) and the new standalone `scripts/record_human_trial.py` (perception-only, no robot). Produces pickle files in one schema so robot and human trials are directly comparable, auto-named with an explicit `robot_run_...`/`human_run_...` prefix. `load_cell_wrench` reserved in the schema for the not-yet-built neck load cell. `angle_diff_rpy` anchors its roll/pitch/yaw decomposition to the `target` marker's frame (biomechanics convention); `object_angular_velocity`/`target_angular_velocity` use each marker's own camera optical frame instead (no pelvis composition — magnitude-only cross-trial normalization use, `MarkerObservation.pose_in_optical` added to carry this). See `docs/data_recording_conventions.md`. 24 new tests (192 total).
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
│   └── default.yaml                 # tunable params; perception section added in Session 2; data_recording section added in Data recording session
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
│   │   └── ik_solver.py             # G1HandIK (Pinocchio + CasADi); WeightedMovingFilter, ExponentialMovingAverageFilter
│   ├── controllers/
│   │   ├── __init__.py
│   │   ├── pose_tracking.py         # run_pose_tracking_sim, three-phase controller (CoppeliaSim)
│   │   ├── sdk_pose_tracking.py     # added SDK-3/4; run_pose_tracking_sdk, three-phase via G1InspireRobot
│   │   ├── grip_force.py            # GraspController; config-driven baseline hold (SDK-5)
│   │   └── grasp_sequence.py        # added SDK-6-safety; run_grasp_phases, guaranteed safe-return before shutdown
│   ├── trajectory/
│   │   ├── __init__.py
│   │   ├── targets.py               # compute_hand_targets + constants
│   │   └── sources.py               # SineSource, MarkerSource (SDK-3/4); RateLimitedSource (jerk/smoothness fix); NoiseFilteredSource (pre-bringup hardening)
│   ├── perception/                  # populated in Session 2
│   │   ├── __init__.py
│   │   ├── types.py                 # MarkerObservation dataclass; pose_in_optical field added Data recording session
│   │   ├── frame_sources.py         # RealSenseSource, VideoFileSource, ImageFileSource
│   │   ├── detector.py              # AprilTagDetector (cv2.aruco + solvePnP)
│   │   ├── tracker.py               # MarkerTracker, MockMarkerTracker
│   │   └── frames.py                # camera optical → d435_link → pelvis composition
│   ├── data_recording/              # added Data recording session
│   │   ├── __init__.py
│   │   └── recorder.py              # RunRecorder, MarkerSample, angle_diff_rpy, angular_velocity_rotvec, default_run_path; shared by sdk_pose_tracking.py Phase 2 and scripts/record_human_trial.py
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
│   ├── run_pose_tracking_preview.py # added SDK-3/4; --source sine|marker|mock, --tracking orient-only|6dof; added SDK-6-safety: --dry-run/--no-dry-run/--net; added Data recording session: --record [PATH]
│   ├── grasp_demo.py                # added SDK-5; bimanual hand open/close validation (--dry-run default); thin wrapper around grasp_sequence.run_grasp_phases (SDK-6-safety)
│   ├── setup_scene.py               # one-time URDF import
│   ├── verify_ik.py                 # FK + IK self-test (no sim needed)
│   ├── visualize_markers.py         # added in Session 2; --source realsense|mock, --dry-run flag
│   └── record_human_trial.py        # added Data recording session; perception-only human-trial recorder, --source realsense|mock, --record [PATH]
│
├── tests/
│   ├── __init__.py
│   ├── test_kinematics.py
│   ├── test_trajectory.py
│   ├── test_transforms.py
│   ├── test_detector.py             # added in Session 2; uses synthetic markers
│   ├── test_perception_frames.py    # added in Session 2; pure math
│   ├── test_mock_tracker.py         # added in Session 2; mock API contract
│   ├── test_tracker.py              # added Data recording session; real MarkerTracker, stub frame_source/detector, pose_in_optical wiring
│   ├── test_realsense_source_sdk.py # added SDK-2; 6 dry_run tests for SDK-backed RealSenseSource
│   ├── test_sdk_transport.py        # added SDK-3; dry_run lifecycle tests for G1InspireRobot
│   ├── test_marker_source.py        # added SDK-3/4; MarkerSource unit tests via MockMarkerTracker
│   ├── test_gravity_comp.py         # added SDK-5.5; 5 hardware-free gravity comp tests
│   ├── test_grasp_demo_safety.py    # added SDK-6-safety; 4 hardware-free safe-return/settle tests
│   ├── test_rate_limited_source.py  # added SDK-6 jerk/smoothness fix; 9 hardware-free slew-limiter tests
│   ├── test_weighted_moving_filter.py # added pre-bringup hardening; 5 tests, recency-weighting regression
│   ├── test_ipopt_diagnostics.py    # added pre-bringup hardening; 4 tests, exhaustion counter
│   ├── test_tauff_filter.py         # added pre-bringup hardening; 7 tests, EMA + solve_ik wiring
│   ├── test_noise_filtered_source.py # added pre-bringup hardening; 8 tests, median+EMA marker noise filter
│   └── test_recorder.py             # added Data recording session; 15 hardware-free tests for RunRecorder/angle_diff_rpy/angular_velocity_rotvec/default_run_path
│
└── docs/
    ├── sdk_analysis.md              # SDK discovery notes (Session 0)
    ├── sdk_setup.md                 # per-machine env ledger (written in SDK-2 preflight)
    ├── ik_weight_analysis.md        # IK weight/payload brainstorm doc (SDK-5; paste into Claude chat)
    ├── gravity_comp_reference.md    # gravity-comp pipeline reference (SDK-5.5)
    ├── data_recording_conventions.md # added Data recording session; pickle schema, angle_diff_rpy reference-frame convention, camera-invariance proof, marker axis convention
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

The transform chain is `pose_in_pelvis = T_d435_to_pelvis @ T_optical_to_d435 @ pose_in_optical`. `T_d435_to_pelvis` comes from Pinocchio FK at the locked head/waist joint configuration. `T_optical_to_d435` is a fixed rotation in config — hand-calibrated: `rpy: [-1.5708, 0.0, -1.5708]` in `configs/default.yaml`.

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

**Smoothing:** `WeightedMovingFilter` with weights `[0.4, 0.3, 0.2, 0.1]` over the last 4 `sol_q` solutions, correctly newest-sample-heaviest (fixed a reversed-weighting bug — see `docs/followups.md`). `ExponentialMovingAverageFilter` (single-pole EMA, `gravity_comp.tauff_filter` config, default `alpha=0.3`) separately smooths `sol_tauff` before it's returned, since the SDK's publisher thread applies feedforward torque as an unfiltered zero-order hold every tick.

**Known accuracy floor:** With the locked cost weights, IK round-trip position error settles at ~1.8 mm at the default pose due to the regularization term. This is documented in tests as the `5e-3` tolerance. It eats into the perception error budget — see Open Design Questions.

**Known timing floor:** `control.dt: 0.02` (50Hz) in `configs/default.yaml` is the configured control-period assumption, not an achieved one. Measured: `solve_ik()` costs ~100ms/call (CasADi/IPOPT re-solve from warm start; `n=55 mean=104.30ms median=100.89ms min=67.97ms max=161.97ms`), so the real Phase-2 loop rate is ~6-10Hz, and `time.sleep(control_dt)` is unconditional (no compute-time compensation). This is not fixed, only mitigated — see `RateLimitedSource` below and `docs/followups.md` for the measurement and reasoning. Speeding up the solve itself (IPOPT `max_iter`/tolerances) is deliberately out of scope — deferred to a separate future effort since it touches the documented accuracy floor above. **Additionally (confirmed empirically):** `solve_ik()`'s IPOPT solve holds the Python GIL for ~100% of its duration, stalling the SDK's own 250Hz publisher thread (same process, same GIL) for the ~70-200ms of every solve. Compounding this, the SDK's `G1Arm._clip_to_velocity_limit()` uses a fixed assumed tick period (`_motor_dt = 1/250`) rather than measured elapsed time — the same assumed-dt-vs-measured-dt bug class already fixed in-repo via `RateLimitedSource`, but this time inside the vendored SDK (`~/g1_inspire_sdk`), which cannot be modified (git-pull-only). **Documented only, deliberately not fixed** — same deferral category as the solve-speed item above.

**Diagnostics:** `G1HandIK.ipopt_exhaustion_count` counts how often `solve_ik()`'s `except` branch fires (IPOPT failing to converge within `max_iter`, falling back to an unconverged `opti.debug.value()`). Pure observability, no behavior change — check this after a live run if tracking looked unusually rough.

### Trajectory (`trajectory/targets.py` and `trajectory/sources.py`)

**`targets.py`** holds `compute_hand_targets(box_center, box_half_width, theta)` — the geometry that turns an object pose into hand SE3 targets. Hand positions arc on a circle of radius `box_half_width` around the object center in the Y-Z plane. Orientations are `Rx(theta) @ R_BASE`.

**`sources.py`** holds the drivers that produce θ over time. `SineSource` wraps `sine_trajectory(t, amplitude, freq)`. `MarkerSource` (added SDK-3/4) reads AprilTag poses from a `MarkerTracker` and supports `"orient-only"` and `"6dof"` modes. `RateLimitedSource` (added in the jerk/smoothness fix) wraps any of the above, slew-rate-limiting `(box_center, theta)` against the actual measured wall-clock dt before it reaches `compute_hand_targets`/`solve_ik` — see "Known timing floor" above and the Open Design Questions "Resolved" list. `NoiseFilteredSource` (added in pre-bringup hardening) wraps `MarkerSource` for `--source marker` only — median-of-N then EMA, cleaning real AprilTag sensor noise *before* `RateLimitedSource` sees it (composition: `RateLimitedSource(NoiseFilteredSource(MarkerSource(...)))`). Config: `trajectory.noise_filter` in `configs/default.yaml`.

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
`T_d435_to_pelvis` is computed once from Pinocchio FK at the locked joint configuration. `T_optical_to_d435` is a fixed rotation from config — hand-calibrated: `rpy: [-1.5708, 0.0, -1.5708]` in `configs/default.yaml`.

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
- **Orientation-only vs 6-DoF**: both supported via `--tracking orient-only|6dof` CLI flag in `run_pose_tracking_preview.py`. Both modes track the target marker's **displacement from its own starting pose** (position for 6-DoF, rotation for both), applied on top of the object marker's own actual starting position — not the target's raw absolute pose. Orient-only: box center fixed from object marker at startup, theta = target's rotation relative to its own start. 6-DoF: box center = object's start position + (target's current position − target's start position), theta the same relative-rotation tracking as orient-only. (Fixed a real bug here — see "SDK-6 marker-tracking fix" below and `docs/followups.md`: box_center used to be set directly to the target's raw absolute position, which sends the hands to wherever the target physically is rather than to the object shifted by how far the target has moved — wrong whenever object and target are in different physical locations, the realistic setup.)
- **`T_optical_to_d435` calibration**: done, `rpy: [-1.5708, 0.0, -1.5708]` in `configs/default.yaml`. (A prior session tried changing this to `[3.14159, 0.0, 1.5708]`; that was wrong and it was reverted. The current value is correct.)
- **Target filtering — jumps/slew** (SDK-3/4, resolved in the jerk/smoothness fix): added `RateLimitedSource` (`trajectory/sources.py`) — a slew-rate clamp on `(box_center, theta)` applied before `compute_hand_targets`, using measured wall-clock dt (not assumed `control_dt`). Architecturally distinct from the IK's `WeightedMovingFilter` (rate limit on the input vs. moving average on the output), so no double-filtering. Config: `trajectory.rate_limit` in `configs/default.yaml`, applied uniformly across `--source sine|mock|marker`. **This only ever addressed the jump/slew case** — it structurally cannot smooth small, sub-threshold sensor noise (see next item).
- **Target filtering — sensor noise** (resolved in pre-bringup hardening): added `NoiseFilteredSource` (`trajectory/sources.py`) — median-of-N then single-pole EMA, applied to the raw target *before* `RateLimitedSource`. Real AprilTag `solvePnP` detection produces small continuous frame-to-frame jitter that sits below the rate limiter's clamp threshold and would otherwise pass through unfiltered; `--source sine`/`--source mock` are noiseless by construction, so this is applied only to `--source marker`. Config: `trajectory.noise_filter` in `configs/default.yaml`, values are unmeasured placeholders pending real marker data.
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
2. **Unit tests:** `pytest tests/ -v` — 192 tests pass
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
10. **Recording pipeline (no hardware needed):**
    ```
    python scripts/run_pose_tracking_preview.py --source sine --record
    python scripts/run_pose_tracking_preview.py --source mock --tracking 6dof --record
    python scripts/record_human_trial.py --source mock --record
    ```
    Each must complete and leave a `.pkl` file under `data/runs/` that unpickles with the expected schema keys (`schema_version`, `origin`, `source_label`, `tracking_mode`, `object_marker_id`, `target_marker_id`, `metadata`, `samples`).

## When in Doubt

- Read this file first
- Check the relevant module's docstring at the top of the file
- Look at `docs/followups.md` for notes from previous sessions
- Verify with Pinocchio FK before changing anything in `trajectory/`
- Run `python scripts/verify_ik.py` — it's the standalone IK self-test
- For CoppeliaSim issues, the diagnostic pattern is: get the object handle, get its position/orientation/matrix, print all values
- For any "should I use code from the reference repo?" question — the answer is no. See Reference Codebase Policy.
