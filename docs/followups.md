# Follow-ups

Per-session log of small items that came out of each session — minor
cleanups, deviations from the spec, things deferred to a future session,
and rough edges that need attention before they bite. Newest sessions on
the bottom.

## Session 1 follow-ups

Session 1's followups file was not committed to the repo. Reconstructing
from what Session 2's task spec called out as outstanding cleanups (Task
9 of the Session 2 brief):

- **Unused import in `sim/coppelia_utils.py`** — `RemoteAPIClient` was
  imported but never referenced. The actual `RemoteAPIClient()` call
  lives in `controllers/pose_tracking.py`, which has its own import.
  → Addressed in Session 2.
- **Unused import in `controllers/pose_tracking.py`** — `detach_box` was
  imported but never called. The function definition stays in
  `coppelia_utils.py` for Session 4's grip-control work.
  → Addressed in Session 2.
- **No `pytest.ini`** — running `pytest` from the repo root without the
  `tests/` argument picked up nothing. Added a minimal config.
  → Addressed in Session 2.

## Session 2 follow-ups

### What was built

- `g1_spine_stabilization/perception/` — the five modules from the spec
  (`types.py`, `frames.py`, `detector.py`, `frame_sources.py`,
  `tracker.py`). All importable in isolation.
- `MockMarkerTracker` — works end-to-end with no camera / OpenCV calls
  needed beyond import.
- Hardware-free tests: `tests/test_detector.py` (synthesizes AprilTags
  in memory), `tests/test_perception_frames.py` (pure math + a
  Pinocchio FK smoke test), `tests/test_mock_tracker.py`. 13 new tests,
  all green.
- `scripts/visualize_markers.py` — `--source realsense` for live
  hardware testing, `--source mock` for hardware-free smoke testing.
- `configs/default.yaml` — new `perception:` block with marker config,
  placeholder D435 intrinsics, and the d435→optical-frame transform.
- `requirements.txt` — added `opencv-contrib-python`.
- README — new Perception section covering both visualizer modes, the
  `pyrealsense2` conda-forge install note, and the calibration
  requirement.

### Deviations from spec (small)

- **`G1HandIK.full_model` doesn't exist.** The IK class loads the full
  robot via `pin.RobotWrapper.BuildFromURDF` and stores the wrapper as
  `self.robot` (with public `.model` and `.data`). Per the spec's
  instruction not to reach into private state, added a public accessor
  `G1HandIK.get_full_model_and_data()` that returns `(model, data)`.
  `frames.d435_to_pelvis_from_pinocchio()` uses that accessor.
- **`g1_spine_stabilization.utils.config.load_config` did not exist.**
  The Session 2 spec assumed it did. Added a minimal `utils/config.py`
  with a `load_config(path=None)` helper that resolves `None` to
  `configs/default.yaml`. The inline `_load_config` in
  `controllers/pose_tracking.py` is unchanged (any consolidation
  belongs in Session 3+).
- **`visualize_markers.py` needed a `sys.path` bootstrap.** Matches the
  pattern already used in `scripts/run_pose_tracking_sim.py` so the
  script works without `pip install -e .`.

### Environment notes

- The active conda env (`teleoperate`) has `opencv-python 4.13.0` rather
  than `opencv-contrib-python`. On this build `cv2.aruco` works anyway
  (`DICT_APRILTAG_36h11 == 20`), so the env is functional, but
  `requirements.txt` now declares `opencv-contrib-python` per spec.
  Switching to `opencv-contrib-python` would require uninstalling
  `opencv-python` first (they don't coexist) — the spec told Claude to
  stop rather than uninstall things, so this is left for the user.
- `pyrealsense2` imports fine in the env but does not expose
  `__version__`. The `pyrealsense2 as rs; rs.__version__` line in the
  spec's environment check fails for that reason — not a real problem,
  just a quirk of the binding. RealSenseSource only uses
  `rs.pipeline`, `rs.config`, and `rs.stream`/`rs.format`, all of which
  are present.

### Test results

```
pytest tests/ -v
29 passed, 1 warning in 6.31s
```

The 1 warning is the known harmless `pinocchio::DeprecatedBool` notice
documented in CLAUDE.md.

The Session 2 spec's `pytest.ini` skeleton included a `rootdir = .`
line. Modern pytest (9.x in this env) treats `rootdir` as a CLI-only
option, not an ini key, and emits a `PytestConfigWarning`. Dropped the
line — pytest auto-detects `rootdir` from the `pytest.ini` location.

### Rough edges for Session 3

- `T_optical_to_d435` in `configs/default.yaml` is the standard ROS
  optical-frame placeholder. **Needs calibration against the real
  D435** the first time the perception pipeline is wired into the
  controller. Don't ship it as-is for real-robot work.
- `d435_to_pelvis_from_pinocchio()` constructs a fresh `G1HandIK()` on
  every call. That's fine for one-shot startup queries but cripplingly
  expensive in a control loop — Session 3 should pass the already-loaded
  IK instance through to perception rather than re-instantiating.
- The detector currently runs `solvePnP` with `SOLVEPNP_IPPE_SQUARE` on
  every detection per frame. Worth measuring detection latency before
  Session 3 closes the loop on it.
- No marker filtering yet. Camera-noise smoothing is explicitly a
  Session 3 design question per CLAUDE.md.

### Out-of-scope items left for later

- Wiring perception into the controller — Session 3.
- `TargetSource` protocol / `ArUcoSource` driver — Session 3.
- Detection-rate / latency profiling against real hardware — user does
  this separately.
- Real-hardware tests — user does these separately.

### Things not modified (per spec hard rules)

- `assets/g1_with_hands.urdf` — untouched.
- IK weights, shoulder-roll bounds, default arm pose — untouched.
- `CLAUDE.md` — untouched.

## Session 0 follow-ups (SDK discovery & documentation)

Out-of-sequence session: read-only discovery of the `g1_inspire_sdk` to
decide how to build on it. No migration, no code, no install. Numbered
"Session 0" because it predates any SDK adoption — it sets up the
decisions Sessions 3–5 will act on.

### What was produced

- `docs/sdk_analysis.md` — source-grounded analysis (every claim cited
  to `path:line` in the SDK clone), answering questions A–F, with a
  "Decisions for the human" section at the top (IK keep-vs-adopt,
  CoppeliaSim retire-vs-keep, grip-force adopt-vs-build — each LEFT
  OPEN), a module-by-module mapping table, and exact API signatures for
  everything we'd call.
- This followups entry.

### Environment notes

- SDK cloned to `~/reference/g1_inspire_sdk` — **outside** the package
  tree, deliberately, so it can never be accidentally imported or
  vendored. Same isolation policy as the `g1_data_collection` reference
  repo. 125 files.
- **Not installed.** Per spec, the SDK was not pip-installed into the
  active `teleoperate` env. All analysis is from reading source, not
  running it.
- SDK targets conda-forge Pinocchio 3.9 / casadi 3.7, python 3.12
  (`environment.yml`). Pinocchio is intentionally absent from its
  `pyproject.toml` deps (PyPI `pin` lacks `pinocchio.casadi`) — same
  constraint we already live with.

### Surprises / discrepancies worth remembering

- **The SDK IK is NOT a drop-in for ours.** It has no shoulder-roll
  self-collision bounds (`_ik/ik.py:178-184` are joint-limit
  constraints only), weights rotation at 1 vs. our 5
  (`_ik/ik.py:185-190`), and synthesizes EE frames at `[0.05,0,0]` off
  the wrist-yaw joints (`_ik/ik.py:108-123`) rather than our palm-surface
  `L_ee_link`/`R_ee_link`. Recommendation in the analysis: keep
  `G1HandIK`, feed `arm.set_joint_targets(q14)`. The joint ordering
  matches ours by construction, so that hand-off is the identity (no
  remap).
- **`set_ee_targets` double-filters.** It runs the SDK IK (which already
  has a `WeightedMovingFilter`) and then applies an *additional* output
  LPF (`arm_filter_tau=0.30`). Another reason to prefer
  `set_joint_targets`.
- **`__enter__` is not full bring-up.** `with G1InspireRobot(...) as r:`
  calls `connect()` only — not `initialize()` or `start_streaming()`
  (`robot.py:150-152`). Easy to misread.
- **RerunPreview is FK-only** — confirmed in source (`preview.py`: only
  `forwardKinematics` + frame/geometry updates + Rerun logging). It is a
  visualizer, NOT a physics replacement for CoppeliaSim. The
  retire-CoppeliaSim question is therefore real but deferred — surfaced
  in the analysis, **not decided** (per spec).
- **`_common/weighted_moving_filter.py` imports `matplotlib.pyplot` at
  module top.** Importing the SDK's filter drags in matplotlib as a hard
  dep. Use our own filter; don't import theirs.
- **README vs. code on IK rebuild.** README says rebuilding the IK cache
  on the bundled URDF "will fail"; the code actually filters out missing
  joint names (`_ik/ik.py:99-102`). Trust the source, not the README,
  here. We don't rebuild the cache either way.
- **`hand_mass_kg = 0.0` is a placeholder** (`_config/g1.yaml`, TODO
  ~0.54 kg; `_ik/admittance.py` default). Force/gravity-comp estimates
  are wrong until the Inspire hand is weighed — a Session 4 prerequisite.
- **Three URDFs in play:** SDK IK uses `g1_body29_hand14.urdf`,
  RerunPreview defaults to `g1_body29_inspire_hand.urdf`, ours is
  `assets/g1_with_hands.urdf`. If we keep our IK this is mostly moot for
  kinematics; the only visible effect is that a Rerun overlay would draw
  the SDK's geometry, not ours (cosmetic).

### Rough edges for the next SDK-adoption session

- The grip-force fork (Decision 3) needs a force signal. The Inspire
  modbus reader exposes per-finger `FORCE_ACT` via `get_state` →
  `{q, force}` (`inspire_hand.py`). `TransportGraspController` is
  close-only and a strong starting point, but the acceleration
  self-tightening (`tau_des = τ* + k_a·‖a‖`) is caller-supplied — we
  write that policy.
- Timing ownership is split: the SDK arm publishes at 250 Hz on its own
  thread; we set targets at our own cadence (live examples use ~60 Hz).
  In CoppeliaSim we keep `step_realtime`. Don't try to make our control
  loop run the motor rate.
- `unitree_sdk2py` + `inspire_sdkpy` are off-PyPI (install from checkout;
  `unitree_sdk2py` needs CycloneDDS). Live hardware bring-up will need
  these, which ties into the Session 5 real-robot gate.

### Out-of-scope items left for later (per spec)

- Any migration / new package code — none written this session.
- Installing the SDK into the env — deliberately not done.
- The three "Decisions for the human" — surfaced with evidence and a
  recommended default, but **left open** for a human call.
- Resolving the CoppeliaSim fork — explicitly not resolved.

### Things not modified (per spec hard rules)

- Only `docs/sdk_analysis.md` and `docs/followups.md` were written.
- `ik_solver.py`, the URDF, and everything under `assets/` — untouched.
- `CLAUDE.md` and all other repo code — untouched.

## SDK-1 follow-ups (manual env install)

Out-of-session: the unified env was built **by hand** following the SDK
README, not inside a Claude Code session, because the live-hardware
install is intricate enough that interactive control beat scripting it.
Recording the result here so SDK-2 has firm ground.

### What was built

- Conda env `g1_inspire_sdk` (Python 3.12.13, per the SDK README).
- SDK installed editable from git
  (`github.com/Daniel891116/g1_inspire_sdk`, commit `981e9e18`).
  Sub-packages `inspire_sdkpy` and `unitree_sdk2py` installed editable
  from the repo's `third_party/` subdirectories.
- Live-hardware tier: `cyclonedds==0.10.2` from PyPI (no source build
  required); `unitree_sdk2py` and `inspire_sdkpy` as editable sub-installs
  (above).
- Our repo (`g1_spine_stabilization`) importable from the env when run
  from the repo root; **not yet pip-installed** into the env — fold into
  SDK-2 preflight if needed.

### SDK examples verified

- **`examples/01_preview_joint_trajectory.py`** — preview path renders;
  Rerun visualizer works.
- **`examples/06_pick_and_place.py`** — ran on the **live robot**; arm
  and hand commands executed successfully; the SDK's arm + hand command
  path is proven end-to-end on this hardware.
- **`examples/07_camera_preview.py`** — RealSense capture works through
  the SDK's `RealSenseCamera`; live frames appeared in Rerun.

### Deliberate decisions

- **No `environment.yml` committed.** The SDK's README is the rebuild
  recipe; `docs/sdk_setup.md` is the per-machine ledger (written in
  SDK-2 preflight).

### Deviations from the SDK README

- Small CycloneDDS account-specific quirks (multi-user machine); not
  relevant to the repo or reproducible outside this machine. No
  recipe-level deviations.

### Environment notes

- `matplotlib` 3.11.0 IS installed in the env (pulled in as a transitive
  dep), but it does not leak at import time — confirmed by import sweep.
  The CLAUDE.md hard rule against importing
  `g1_inspire_sdk._common.weighted_moving_filter` remains critical: that
  import would pull matplotlib into the module namespace.
- `opencv-python` 4.13.0 is installed rather than `opencv-contrib-python`
  (same situation as the `teleoperate` env). `cv2.aruco` works on this
  build; `requirements.txt` still declares `opencv-contrib-python` per spec.
- `pytest` and `coppeliasim-zmqremoteapi-client` were not installed during
  the manual SDK-1 setup; both added at the start of SDK-2 preflight.

### Test results

- **Our 29 tests not run in this env during SDK-1** — both missing
  packages (`pytest`, `coppeliasim-zmqremoteapi-client`) were installed
  and all 29 tests run as SDK-2 Step 0 preflight. Result: **29 passed**.

### Rough edges for SDK-2

- `docs/sdk_setup.md` written in SDK-2 preflight (this session).
- `g1_spine_stabilization` not pip-installed into the env; works via
  sys.path from the repo root. Add `pip install -e .` to the setup doc
  and install if needed.
- The intrinsics shape returned by `RealSenseCamera.intrinsics` uses
  `ppx`/`ppy` rather than `cx`/`cy` — SDK-2's wrapper handles the remap.

### Out-of-scope items left for later

- Wiring the SDK into our perception pipeline — SDK-2.
- IK transport adapter — SDK-3.
- Controller / grip-force work — SDK-4 / SDK-5.

### Things not modified

- `assets/g1_with_hands.urdf` — untouched.
- IK weights, shoulder-roll bounds, default arm pose — untouched.
- The SDK clone — `git pull` only, no edits.

## SDK-2 follow-ups (perception re-platform)

First SDK code session. Swapped the camera backend in `RealSenseSource`;
everything downstream (`AprilTagDetector`, `MarkerTracker`, `frames.py`,
`scripts/visualize_markers.py`) is unchanged.

### What was built

- **`g1_spine_stabilization/perception/frame_sources.py`** — `RealSenseSource`
  internals replaced. Same public interface (constructor signature +
  optional `dry_run` kwarg, `.read()`, `.intrinsics_native`, `.close()`).
  Internally wraps `g1_inspire_sdk.camera.RealSenseCamera`; lazy-imports
  the SDK in `__init__`. Intrinsics remap: SDK `ppx`/`ppy`/`coeffs` →
  our `cx`/`cy`/`distortion`. Removed the now-unused `numpy` import.
- **`tests/test_realsense_source_sdk.py`** — 6 new hardware-free tests
  using `dry_run=True`: frame shape, dtype, timestamp, intrinsics keys,
  copy behavior, lifecycle (close + re-create), detector smoke on synthetic
  frame.
- **`scripts/visualize_markers.py`** — `--dry-run` flag added. When
  `--source realsense --dry-run` is used, `RealSenseSource(dry_run=True)`
  returns synthetic gradient frames from the SDK; no camera needed.
- **`docs/sdk_setup.md`** — per-machine env ledger (Step 0 deliverable):
  env name, Python version, SDK commit, full `conda list` + `pip freeze`.
- **`CLAUDE.md`** — patch v2 applied: SDK-1 marked complete, SDK-2 to
  NEXT, dependency hard rule updated (no `environment.yml`; sdk_setup.md
  is the ledger), External SDK section path corrected
  (`~/g1_inspire_sdk`), env name updated to `g1_inspire_sdk`.
- **`docs/followups.md`** — SDK-1 entry appended (this file).

### Deviations from spec

- **`pytest` and `coppeliasim-zmqremoteapi-client` were missing** from the
  `g1_inspire_sdk` env (not installed during SDK-1). Both are core-stack
  items. Added them at the start of Step 0 preflight; 29 tests then passed.
- **`g1_spine_stabilization` is not pip-installed** into the env. All
  script and test runs work via `sys.path` from the repo root. The setup
  doc notes this and gives the fix (`pip install -e .`).

### Environment notes

- New `hppfcl` deprecation warning appears in pytest output:
  `DeprecationWarning: Please update your 'hppfcl' imports to 'coal'`.
  Comes from Pinocchio's dependency; harmless. Not in our code.

### Test results

```
35 passed, 1 warning in 3.74s
```

29 original tests unchanged. 6 new SDK-source tests green.

### Human-run smoke check (not executed by Claude Code)

With a RealSense D435 connected:
```
conda activate g1_inspire_sdk
python scripts/visualize_markers.py --source realsense
```
Expected: live AprilTag detection at the same rate as before the swap.

The `--dry-run` variant (no camera required):
```
python scripts/visualize_markers.py --source realsense --dry-run
```
Expected: loop runs, prints nothing (synthetic gradient frame has no
markers), no exceptions.

### Rough edges for SDK-3

- SDK-3 will feed `G1HandIK` output to `arm.set_joint_targets(q14)`.
  The `G1InspireRobot` lifecycle (`connect → initialize → start_streaming
  → shutdown`) must be respected; `initialize()` actuates the arm (runs
  it to default pose) so it counts as live-robot code under the hard rule.
  SDK-3 validates against dry-run preview only; live bring-up is SDK-6.
- The timing split between the SDK's 250 Hz arm thread and our control
  cadence needs to be tested in dry-run; don't try to match the motor rate.

### Out-of-scope items left for later

- IK transport adapter (`G1HandIK` → `set_joint_targets`) — SDK-3.
- Marker-driven target source replacing the sine wave — SDK-4.
- Grip-force work — SDK-5.
- Live-robot bringup — SDK-6 (human-run only).

### Things not modified

- `detector.py`, `tracker.py`, `frames.py`, `types.py` — untouched.
- `ik_solver.py`, `assets/g1_with_hands.urdf`, IK weights, EE frames,
  shoulder-roll bounds — untouched.
- `docs/sdk_analysis.md` — untouched.
- `configs/default.yaml` — untouched (camera config keys unchanged).

## SDK-3 / SDK-4 follow-ups (IK transport adapter + marker-driven source, merged)

These two sessions were merged because the live-marker preview script is the
natural validation of the IK transport — no additional IK code is needed to
demonstrate it.

### What was built

- **`g1_spine_stabilization/perception/frames.py`** — `d435_to_pelvis_from_pinocchio`
  gains an optional `ik` parameter. Callers that already hold a `G1HandIK`
  instance can pass it in to avoid re-instantiating Pinocchio (~5–15 s cold
  start). Backward-compatible: no-arg calls still work.
- **`g1_spine_stabilization/trajectory/sources.py`** — two new classes:
  - `SineSource`: wraps `sine_trajectory` behind the same `.startup()` /
    `.step() → (box_center, theta)` interface so the SDK controller doesn't
    branch on source type.
  - `MarkerSource`: reads `MarkerTracker.step()` each control tick, extracts
    `theta = arctan2(R[2,1], R[2,2])` from the target marker's rotation, and
    returns `(box_center, theta)`. Two modes: `"orient-only"` (box_center
    fixed from object marker at startup) and `"6dof"` (box_center tracks target
    marker position). Falls back to last known values when no new detection.
- **`g1_spine_stabilization/controllers/sdk_pose_tracking.py`** — new module
  with `run_pose_tracking_sdk(robot, ik, cfg, source, duration=None)`. Mirrors
  the three-phase logic of `run_pose_tracking_sim` but drives
  `G1InspireRobot` instead of CoppeliaSim. Reads joint state from
  `robot.get_state()["arm"]["q"]`, sets targets via `robot.step(q_arm=sol_q)`,
  paces with `time.sleep(control_dt)`. Hand open/close replaces the
  CoppeliaSim box attach/detach. Caller owns the robot lifecycle.
- **`scripts/run_pose_tracking_preview.py`** — multi-mode entry point:
  - `--source sine` (default): `SineSource`, runs for `rotation_duration` seconds.
  - `--source mock`: `_AnimatedMockTracker` (time-varying poses, no camera),
    `MarkerSource`, runs for `rotation_duration` seconds.
  - `--source marker`: live `RealSenseSource` + `AprilTagDetector` +
    `MarkerTracker` + `MarkerSource`; runs until `KeyboardInterrupt`.
  - `--tracking orient-only|6dof` (for marker/mock modes).
  - Instantiates `RerunPreview(spawn=True)` and passes it to
    `G1InspireRobot(dry_run=True, preview=preview)` so all three phases appear
    in the Rerun viewer.
- **`tests/test_sdk_transport.py`** — 4 hardware-free `dry_run=True` tests:
  lifecycle, `get_state()["arm"]["q"]` shape, `step(q_arm=...)`, hand commands.
- **`tests/test_marker_source.py`** — 7 tests covering `MarkerSource`
  (orient-only, 6dof, last-known fallback, invalid mode) and `SineSource`.
- **`CLAUDE.md`** — patched: SDK-2 → COMPLETE, SDK-3+4 → NEXT/merged,
  test count 29 → 46, calibration status updated, file structure updated,
  `d435_to_pelvis_from_pinocchio()` note added, Open Design Questions revised.

### Deviations from spec

- **`robot.get_state()` key is `state["arm"]["q"]`**, not `"q_arm"`. Discovered
  at test time; fixed in `sdk_pose_tracking.py` and `test_sdk_transport.py`.
  The SDK's actual state dict is `{arm: {q, dq, tau}, arm_ee_left, arm_ee_right,
  hand_left, hand_right}`.

### Test results

```
46 passed, 1 warning in 3.63s
```

11 new tests (7 marker-source + 4 sdk-transport). All 46 green.

### Human-run smoke checks (not executed by Claude Code)

```bash
conda activate g1_inspire_sdk
# No camera, Rerun viewer:
python scripts/run_pose_tracking_preview.py --source sine
python scripts/run_pose_tracking_preview.py --source mock --tracking orient-only
python scripts/run_pose_tracking_preview.py --source mock --tracking 6dof

# With RealSense D435 + two AprilTag markers:
python scripts/run_pose_tracking_preview.py --source marker --tracking orient-only
python scripts/run_pose_tracking_preview.py --source marker --tracking 6dof
```

Expected for sine/mock: Rerun opens; robot arms move through all three phases;
script exits cleanly. Expected for marker: arms respond in real time to the
target AprilTag; Ctrl+C to exit.

### Rough edges for SDK-5

- Phase 2 duration logic: `duration=None` uses `cfg["control"]["rotation_duration"]`
  (10 s) for sine/mock; `float("inf")` is passed by the script for the live
  marker mode. This is implicit; consider an explicit `interactive=True` kwarg
  in a future refactor if the interface needs to be cleaner.
- `_AnimatedMockTracker` in `run_pose_tracking_preview.py` is script-local; if
  it needs reuse in other contexts, promote it to `perception/tracker.py`.
- `G1InspireRobot(hand_side="both")` — confirm this is the correct value for
  controlling both hands simultaneously when the real robot is connected in
  SDK-6. (In `dry_run=True` it is irrelevant.)
- Phase 3 reads FK from `ik.fk(current_q)` to get start-of-return EE poses.
  Verify this is correct after Phase 2 when `current_q` has been filtered by
  the IK's `WeightedMovingFilter` — the filter may cause a small discontinuity
  at the transition. Investigate if Phase 3 interpolation looks jerky in Rerun.

### Out-of-scope left for later

- SDK-5: grip force from `TransportGraspController` + our acceleration policy;
  prereq: weigh the Inspire hand, set `hand_mass_kg`.
- SDK-6 (human-run): flip `dry_run=False`, mount robot, E-stop in reach.
- CoppeliaSim path (`run_pose_tracking_sim.py`) — completely untouched this session.

### Things not modified

- `ik_solver.py`, `assets/g1_with_hands.urdf`, IK weights, EE frame defs,
  shoulder-roll bounds — untouched.
- `controllers/pose_tracking.py` (CoppeliaSim path) — untouched.
- `detector.py`, `tracker.py`, `types.py`, `frame_sources.py` — untouched.
- `docs/sdk_analysis.md` — untouched.

## SDK-5 follow-ups (grip force baseline + IK analysis)

Partial SDK-5 session: grip force controller and hand validation script delivered;
IK weight/payload analysis written as a standalone brainstorm doc; CLAUDE.md updated
to reflect SDK-3+4 completion and SDK-6 live-robot path.

### What was built

- **`g1_spine_stabilization/controllers/grip_force.py`** — `GraspController` class.
  Wraps `TransportGraspController` from `g1_inspire_sdk._ik.force_controller`.
  Per-hand (create one instance per side from the same config). Key methods:
  `arm(q_measured)` — snapshots the grasp pose once fingers settle;
  `step(force, q, dt, tau_des=None)` — runs force-feedback control, falls back to
  `close_pose` before arming; `reset()` — clears state before re-grasping.
  `TransportGraspController` is lazy-imported (same pattern as `pyrealsense2`) so
  the package imports cleanly without `g1_inspire_sdk` installed.
- **`configs/default.yaml`** — new `grip_force:` section: `tau_baseline=0.25 N`,
  `kappa=0.05`, `delta_q_max=1.0 rad`, `grasp_pose=[0.9, 0.9, 0.9, 0.9, 0.7, 0.35]`
  (soft close; below SDK hard limits `[1.47, 1.47, 1.47, 1.47, 1.2, 0.56]`).
- **`scripts/grasp_demo.py`** — bimanual open → close → hold → release demo.
  `--dry-run` (default `True`); `--no-dry-run` for live hardware (human-run only
  with E-stop); `--hold-time N` (default 3 s); `--side {both,left,right}`.
  Creates one `GraspController` per active hand. Hold loop calls `ctrl.step()` each
  tick and prints per-finger force when live; dry-run verifies command plumbing only.
- **`docs/ik_weight_analysis.md`** — self-contained analysis doc (paste into Claude
  chat for planning). Covers: IK cost formulation, `sol_tauff` computed-but-unused,
  hand mass in reduced model (needs verification), box payload unmodeled, impact
  table, proposed clean-session steps, and questions to resolve before coding.
- **`CLAUDE.md`** — patched: SDK-3+4 → COMPLETE; test count 35 → 46; SDK-5 →
  NEXT with updated description; SDK-6 description made concrete (grasp_demo first,
  then preview pre-flight gate, then live flip); file structure updated for new files;
  `sources.py` note updated to reflect `SineSource` + `MarkerSource` complete.

### Deviations from spec

- `GraspController.step()` exposes an optional `tau_des` override parameter. This
  is the hook for acceleration coupling (Phase 2) without requiring a subclass or
  reimplementing the core loop.

### Environment notes

- `TransportGraspController` is in `g1_inspire_sdk._ik.force_controller` (private
  subpackage, but stable — verified in SDK-0 analysis). Lazy import guard prevents
  import errors on machines without the SDK.
- `grasp_demo.py` passes `q_arm=_ARM_DEFAULT_Q` on every `robot.step()` call to
  keep arms still during the hand test. Avoids accidentally sending a `None` arm
  command that might be interpreted as "hold last" vs. "no command."

### Test results

```
51 passed, 1 warning in 3.93s
```
No new tests added this session — `GraspController` lazy-imports the SDK, making
hardware-free unit tests identical to the `RealSenseSource` pattern. Tests will be
added when the dry-run mode of `GraspController` is fleshed out (currently
`dry_run` plumbing lives in `G1InspireRobot`, not in `GraspController` itself).
Note: the count is 51 (not 46 as the SDK-3/4 summary stated) — the actual
test_marker_source.py had 10 tests, not the 7 cited in that entry.

### Human-run smoke checks (not executed by Claude Code)

```bash
conda activate g1_inspire_sdk

# Dry-run (safe, no robot):
python scripts/grasp_demo.py
python scripts/grasp_demo.py --side left
python scripts/grasp_demo.py --side right

# Live hardware (HUMAN-RUN ONLY, E-stop required):
python scripts/grasp_demo.py --no-dry-run
```

### Rough edges for SDK-5 Phase 2 / SDK-6

- **`hand_mass_kg` still `0.0`** in SDK `g1.yaml`. Must measure before acceleration
  coupling is calibrated. Record in `docs/sdk_setup.md` and update if URDF changes.
- **`sol_tauff` still discarded** in `sdk_pose_tracking.py`. No SDK feedforward
  torque channel exists in `G1InspireRobot`. Worth checking `unitree_sdk2py` DDS
  command types for a lower-level path before adding complexity.
- **Acceleration coupling** (`tau_des = tau_baseline + k_a * ||a_arm||`) requires
  an `a_arm` estimate. Best approach: finite-diff from the `WeightedMovingFilter`
  history buffer already in `G1HandIK` — no new state needed.
- **`G1InspireRobot(hand_side="both")`** for SDK-6: confirm this is correct for
  simultaneous bimanual control on live hardware (irrelevant in dry-run).

### Out-of-scope left for later

- SDK-5 Phase 2: acceleration coupling in `GraspController`.
- SDK-5 Phase 3: IK weight/payload changes (see `docs/ik_weight_analysis.md`).
- SDK-6 (human-run): `grasp_demo.py --no-dry-run`, then full preview pre-flight,
  then live `run_pose_tracking_sdk.py`.

### Things not modified

- `ik_solver.py`, `assets/g1_with_hands.urdf`, IK weights, EE frame defs,
  shoulder-roll bounds — untouched.
- `controllers/pose_tracking.py` (CoppeliaSim path) — untouched.
- All perception modules, trajectory modules, tests — untouched.
- `docs/sdk_analysis.md` — untouched.

## SDK-5 jerk fix + live-hardware bringup planning

Short follow-up session: fixed the Phase 0 jerk in `grasp_demo.py` and documented
the pre-flight gate sequence for live hardware bringup.

### What was fixed

- **`scripts/grasp_demo.py` Phase 0 jerk.** Phase 0 interpolated EE targets starting
  from `L_EE_DEFAULT`/`R_EE_DEFAULT` (FK at zero joint config) and `R_L_BASE`/`R_R_BASE`
  orientations. After `initialize()`, the robot is at the SDK default pose — a different
  joint configuration. The first IK command therefore targeted a position that didn't
  match where the arm actually was, causing a visible jerk.
- **Fix (line 209, `scripts/grasp_demo.py`):** Added `ik.fk(current_q)` call directly
  after `_get_q(robot)` to read the actual post-`initialize()` EE pose. Phase 0 now
  interpolates from those real positions and orientations to the grasp target. Exact
  mirror of the Phase R pattern already in the file (which was already correct).
- No test changes. 51 tests still pass.

### Pre-flight gate for live hardware runs (SDK-6)

The following sequence must all pass before any live bringup:

```bash
conda activate g1_inspire_sdk

# 1. Unit tests
pytest tests/ -v        # 51 must pass

# 2. Grasp demo dry-run (validates hand control + arm IK interpolation)
python scripts/grasp_demo.py
python scripts/grasp_demo.py --side left
python scripts/grasp_demo.py --side right

# 3. Sine-source preview (validates arm IK + SDK transport, no camera)
python scripts/run_pose_tracking_preview.py --source sine

# 4. Mock-source preview (validates marker→IK→SDK path headlessly)
python scripts/run_pose_tracking_preview.py --source mock --tracking orient-only
python scripts/run_pose_tracking_preview.py --source mock --tracking 6dof

# 5. Live-marker preview (validates full perception→IK→preview pipeline)
python scripts/run_pose_tracking_preview.py --source marker --tracking orient-only
# Move target marker in front of the D435; watch arms respond in Rerun; Ctrl+C to exit.
```

All stages must run without error or visible arm discontinuity in Rerun before
flipping `dry_run=False` for SDK-6.

### Rough edges / prerequisites before SDK-6

- **Gravity compensation not implemented.** Inspire hands are non-negligible mass
  hanging off the wrist joints. Under position-only control, IK solutions that ignore
  hand weight will droop at the wrist. Must be implemented before live bringup.
- **`hand_mass_kg` still `0.0`** in SDK `g1.yaml`. Needs the measured Inspire hand
  mass before acceleration coupling or any SDK gravity-comp feature is usable.
- **`sol_tauff` still discarded** in `sdk_pose_tracking.py`. Once hand inertial
  properties are added to the Pinocchio model, `sol_tauff` will carry correct
  feedforward torques — but whether the SDK transport exposes a DDS torque
  feedforward channel needs to be verified against `unitree_sdk2py` first.

### Out-of-scope left for next session (SDK-5.5)

- Gravity compensation implementation. **Plan in a Claude chat before handing to
  Claude Code.** Paste into the chat: `ik_solver.py`, `sdk_pose_tracking.py`,
  `grip_force.py`, the URDF hand-attachment joint section, `docs/ik_weight_analysis.md`,
  and the measured hand properties (mass, CoM in hand frame, inertia tensor,
  hand-to-wrist transform). Key questions to resolve in chat: runtime Pinocchio
  inertia update vs. URDF edit; SDK torque feedforward channel availability; where
  to wire gravity comp (inside `G1HandIK` vs. `sdk_pose_tracking.py`); how to
  estimate `a_arm` for acceleration coupling from the `WeightedMovingFilter` history.
- SDK-6 live bringup — human-run only; gated on gravity comp completion.

### Things not modified

- `ik_solver.py`, tests, perception modules, trajectory modules, sim controller — untouched.
- `docs/sdk_analysis.md`, `docs/ik_weight_analysis.md` — untouched.

---

## SDK-5.5 follow-ups

### What was done

Gravity compensation for the Inspire RH56 hands: runtime Pinocchio inertia
correction + tau feedforward wiring.

**Files changed:**
- `configs/default.yaml` — added `gravity_comp.enabled: true`
- `g1_spine_stabilization/kinematics/ik_solver.py` — module-level constants
  (`INSPIRE_HAND_MASS=1.0`, wrist/hand URDF inertia constants), `_make_composite_wrist_inertia()`
  helper, `gravity_comp_enabled` parameter in `G1HandIK.__init__`, `_apply_hand_inertia_correction()`
  method inserted before CasADi graph compilation
- `g1_spine_stabilization/controllers/sdk_pose_tracking.py` — all 3 phase loops
  now capture `sol_tauff`; `robot.step(q_arm=sol_q)` replaced with
  `robot.arm.set_joint_targets(sol_q, sol_tauff)` (SDK tau feedforward channel);
  Phase 2 logs `tauff/` scalars to Rerun
- `scripts/run_pose_tracking_preview.py` — passes `gravity_comp_enabled` from cfg to IK
- `tests/test_gravity_comp.py` — 5 new hardware-free tests (56 total)
- `docs/sdk_analysis.md`, `docs/ik_weight_analysis.md` — updated

**Key findings:**
- URDF hand_base_link is 0.141 kg; real Inspire RH56 is **1.0 kg** (scale measurement)
- `G1Arm.set_joint_targets(q14, tau_ff14=None)` already existed; SDK sends tau=0 by default (no SDK internal gravity comp)
- `G1InspireRobot.step()` doesn't forward tau — we bypass to `robot.arm.set_joint_targets()` directly
- Inertia correction confirmed by potential-energy gradient cross-check (Test 2)

### Out of scope for SDK-5.5

- Acceleration coupling (`tau_des = tau_baseline + k_a*||a_arm||`) for grip force — deferred
- SDK-6 live bringup — human-run only; gated on gravity comp

### Things not modified

- `assets/g1_with_hands.urdf` — runtime correction, no URDF change needed
- `~/g1_inspire_sdk` — SDK not modified
- EE frame defs, shoulder_roll bounds, IK cost weights — unchanged

## SDK-6-safety follow-ups

### What was done

Guaranteed safe shutdown phase-out, plus live-mode capability for
`run_pose_tracking_preview.py`. Trigger: reports that `grasp_demo.py`'s arms
fell to the table more aggressively at the end of a run than the vendored
SDK's own example scripts.

**Files changed:**
- `g1_spine_stabilization/controllers/grasp_sequence.py` — new file. Moved
  `grasp_demo.py`'s Phase 0/A/B/C/D/R logic into `run_grasp_phases(...)`,
  restructured so Phase D (hand release) and Phase R (arm return-to-default)
  always run — even after Ctrl-C/exceptions during Phase 0/A/B/C — followed
  by a settle pause before returning. Re-raises the first exception
  encountered after cleanup, so failures are never silently swallowed.
- `scripts/grasp_demo.py` — reduced to a thin wrapper: arg parsing, config/IK
  load, robot lifecycle, and a single call to `run_grasp_phases(...)`.
- `g1_spine_stabilization/controllers/sdk_pose_tracking.py` — Phase 1
  (interpolate to grasp) now catches `KeyboardInterrupt` and falls through to
  Phase 3, mirroring Phase 2's existing precedent. Phase 3 ends with a settle
  pause (`time.sleep(settle_time)`) before returning to the caller.
- `scripts/run_pose_tracking_preview.py` — added `--net`, `--dry-run`,
  `--no-dry-run` flags (mirroring `grasp_demo.py`'s convention) plus the same
  human confirmation gate, so `mock`/`marker` sources can now actually be run
  live. Also closes `frame_source` in the `finally` block for `--source
  marker` (was previously never closed on any exit path).
- `configs/default.yaml` — added `shutdown.settle_time: 0.4`.
- `tests/test_grasp_demo_safety.py` — new file, 4 hardware-free tests
  (baseline completion, fault-during-hold recovery, KeyboardInterrupt-during-
  Phase-0 recovery, config key present).
- `tests/test_sdk_transport.py` — 2 new tests (settle pause doesn't hang,
  Phase 1 KeyboardInterrupt falls through to Phase 3). 62 tests total.
- `CLAUDE.md` — Current Status, Session Roadmap, File Structure, Standard
  Verification Workflow step 9 updated; corrected stale references to a
  `run_pose_tracking_sdk.py` file that was never built.

**Key findings:**
- Root cause: `grasp_demo.py`'s `main()` had no `except` clause at all — any
  fault during the grasp/hold phases skipped hand-release and arm-return
  entirely, jumping straight from an actively-loaded grasp pose into the
  SDK's fixed damping cutoff.
- The SDK's own damping command (`G1Arm.shutdown()` → `create_damping_cmd`:
  `kp=0, kd=8, tau=0`, sent once) is fixed and cannot be changed (vendored
  SDK, git-pull-only). All of the SDK's own live examples (03/04/05/06)
  already follow `try: <motion> finally: robot.shutdown()`, matching our
  scripts' existing top-level structure — the gap was specifically the
  missing safe-return guarantee inside that structure, not the shutdown
  mechanism itself.
- Verified directly in `~/g1_inspire_sdk/g1_inspire_sdk/g1_arm.py`: the 250 Hz
  publisher thread continuously re-publishes whatever was last passed to
  `set_joint_targets()` until `stop_streaming()` runs (only reachable via
  `robot.shutdown()`). So the settle pause needs no repeated command loop —
  a plain `time.sleep()` holds the arm (with gravity-comp tau feedforward)
  for free.
- `G1Arm`/`InspireHand` raise `ValueError` if `dry_run=False` and
  `network_interface is None` — adding `--no-dry-run` to
  `run_pose_tracking_preview.py` therefore also required adding `--net`.
- `scripts/` is excluded from packaging (`setup.py`), so `grasp_demo.py`'s
  phase logic wasn't testable in place — moved to `controllers/` to match
  the pattern `sdk_pose_tracking.py` already established.

### Out of scope for SDK-6-safety

- No new "rest pose" with lower gravity torque was designed — Phase R still
  returns to the existing standing-default pose. The settle pause only
  guarantees the arm is stationary and gravity-comp-supported immediately
  before the SDK's fixed damping cutoff, not that the cutoff itself is gentler.
- No gradual tau-feedforward ramp-down was implemented (deliberately, per
  user decision — minimal fix scope).
- SDK-6 live bringup itself — still human-run only.

### Things not modified

- `~/g1_inspire_sdk` — SDK not modified.
- `assets/g1_with_hands.urdf`, EE frame defs, shoulder_roll bounds, IK cost
  weights — unchanged.
- `docs/gravity_comp_reference.md` — no change needed; the settle pause
  relies on the already-correctly-routed last `set_joint_targets()` call.

## SDK-6 jerk/smoothness fixes follow-ups

### What was done

First-ever live hardware test (`python scripts/run_pose_tracking_preview.py
--source mock --tracking 6dof --no-dry-run --net enx6c6e072d3ca1`) surfaced
two distinct problems: (1) a violent jerk right after startup settles, before
the arms converge on the mock object, and (2) visibly jumpy/jerky motion
during tracking, traced to the mock marker "moving in big jumps." Both are
fixed.

**Files changed:**
- `g1_spine_stabilization/controllers/sdk_pose_tracking.py` — Phase 1 now
  reads `ik.fk(current_q)` right after `current_q = _get_q(robot)` and
  interpolates from that actual current pose to the grasp target, instead of
  the hardcoded `L_EE_DEFAULT`/`R_EE_DEFAULT` constants. Mirrors Phase 3's
  existing pattern in the same file, and the identical fix already applied
  to `grasp_sequence.py`'s Phase 0 during SDK-5 (see "SDK-5 jerk fix" above)
  — that fix was simply never ported to this file until now.
- `g1_spine_stabilization/trajectory/sources.py` — new `RateLimitedSource`
  class: slew-rate-limits `(box_center, theta)` from any wrapped source
  (`SineSource`/`MarkerSource`), clamped against the ACTUAL measured
  wall-clock dt between `.step()` calls (via an injectable `clock`
  parameter, default `time.monotonic`), not the assumed `control_dt`.
  Duck-types the same `.startup()`/`.step()` contract; proxies unknown
  attributes (`object_lost`, `last_obj_obs`, etc.) via `__getattr__`.
- `configs/default.yaml` — new `trajectory.rate_limit` block
  (`enabled`, `max_theta_rate_deg_s: 60.0`, `max_pos_rate_m_s: 0.3`);
  `trajectory.sine.frequency` lowered `0.3` → `0.1` Hz (amplitude unchanged);
  comment added to `control.dt` cross-referencing the timing finding below.
- `scripts/run_pose_tracking_preview.py` — wraps `source` in
  `RateLimitedSource` (config-gated, default on) uniformly across all three
  `--source` modes, right after source construction and before
  `robot.connect()`.
- `tests/test_sdk_transport.py` — new regression test
  (`test_pose_tracking_sdk_phase1_starts_from_actual_current_pose`) proving
  Phase 1's first IK target equals `ik.fk(fake_q)`, not `L_EE_DEFAULT`; fails
  on the old code, passes now.
- `tests/test_rate_limited_source.py` — new file, 9 hardware-free tests
  (pass-through when under the limit, clamp when over — both theta and
  box_center with direction preservation, first-call-unclamped,
  measured-dt-not-assumed-dt, `__getattr__` proxying, `startup()` reseeding).
  72 tests total.
- `CLAUDE.md` — new "Known timing floor" note in the IK Solver section;
  "Target filtering" Open Design Question de-duplicated (was accidentally
  repeated verbatim) and moved to "Resolved"; Current Status, Session
  Roadmap, File Structure updated.

**Key findings:**
- Root cause 1 (startup jerk): exact same bug class as the already-fixed
  "SDK-5 jerk fix" (see above) — hardcoded interpolation start instead of
  the real current pose — just present in a second file (`sdk_pose_tracking.py`
  Phase 1) that never got the fix ported to it.
- Root cause 2 (jerky tracking), measured directly: `ik.solve_ik()` costs
  ~100ms/call (`n=55 mean=104.30ms median=100.89ms min=67.97ms max=161.97ms`,
  measured running the real code path in the `g1_inspire_sdk` env). Against
  the assumed `control_dt=0.02s` (50Hz) with an unconditional
  `time.sleep(control_dt)` (no compute-time compensation anywhere in the
  repo), the real Phase-2 loop rate is ~6-10Hz. `SineSource`/the mock both
  sample a real-time sine (`theta = amplitude*sin(2*pi*freq*(monotonic()-t0))`)
  — correct math, but each `.step()` call samples across a ~90-180ms gap
  instead of ~20ms, producing per-step deltas 5-9x bigger than intended,
  which IK's warm-started solve then chases abruptly.
- Confirmed this is not 6dof-specific: the mock's target marker position
  never moves in `_AnimatedMockTracker` (only its rotation does), so the
  jumpiness is purely rotational and would equally affect `--tracking
  orient-only` or a real live marker feed (a human moving a real marker
  "smoothly" would still get chopped into big jumps by a 6-10Hz effective
  loop rate).
- Confirmed the existing smoothing stack before this fix: only the IK's own
  output-side 4-tap `WeightedMovingFilter` (on `sol_q`, after the solve) and
  the SDK's joint-level velocity clip (`G1Arm._clip_to_velocity_limit`,
  20-30 rad/s in the 250Hz publisher thread) — nothing touched the target
  *before* the solve. `RateLimitedSource` is architecturally distinct from
  both (a rate limit on the input vs. a moving average on the output), so it
  resolves the "double-filtering" concern rather than adding to it.
- Phase 1/3's `smoothstep(i / steps_interp)` ramp is indexed by iteration
  count, not wall-clock time, so the slow-loop mechanism above does NOT
  affect those phases' jerkiness — only their total real-world duration
  (they just take longer than `interpolation_time` when the loop is slow).

### Out of scope for this fix (deliberate, per user decision)

- Speeding up `ik.solve_ik()` itself (IPOPT `max_iter`/tolerances) — the
  measured ~100ms/call cost is documented, not addressed. This touches core
  IK solver behavior with a documented ~1.8mm regularization floor and 5mm
  round-trip test tolerance; any change needs its own careful validation
  pass, deferred to a separate future effort.
- Tuning `max_theta_rate_deg_s`/`max_pos_rate_m_s` against real marker
  motion data — no real AprilTag marker speed measurement exists yet;
  current defaults are reasoned placeholders (see comments in
  `configs/default.yaml`).
- Live re-test on hardware to confirm the fix — human-run only.

### Things not modified

- `~/g1_inspire_sdk` — SDK not modified.
- `assets/g1_with_hands.urdf`, EE frame defs, shoulder_roll bounds, IK cost
  weights, IPOPT solver settings — unchanged.
- `g1_spine_stabilization/trajectory/targets.py` (`compute_hand_targets`) —
  left as pure stateless geometry; rate limiting lives one layer up, in the
  source wrapper, specifically to avoid injecting state into this function.

## SDK-6 pre-bringup hardening follow-ups

### What was done

User asked for a deeper analysis of remaining jerk/jitter causes beyond the
previous fix, and specifically how the pipeline changes once it moves from
the mock source to real AprilTag markers. Three parallel research agents
plus direct verification found five new issues; the user separately caught
a sixth (a leftover test edit to the modeled hand mass). All six addressed
except one (GIL contention), documented and deliberately deferred.

**Files changed:**
- `g1_spine_stabilization/kinematics/ik_solver.py`:
  - `INSPIRE_HAND_MASS` reverted from `1.6` (leftover local test edit) back
    to `1.0` kg (the measured value documented everywhere else, including
    this same file's own docstrings, which never got updated to match the
    `1.6` value).
  - `WeightedMovingFilter.add_data()` fixed: `zip(self.weights, self.queue)`
    → `zip(self.weights, reversed(self.queue))`. `self.queue` is FIFO
    (oldest at index 0); the old code paired the largest weight (0.4) with
    the *oldest* sample instead of the newest.
  - New `ExponentialMovingAverageFilter` class (single-pole EMA), and a new
    `self.tauff_filter` instance smoothing `sol_tauff` right before
    `solve_ik()` returns it (`tauff_filter_enabled`/`tauff_filter_alpha`
    constructor kwargs, default `alpha=0.3`).
  - New `self.ipopt_exhaustion_count` counter, incremented in `solve_ik()`'s
    `except` branch alongside an enriched print.
- `g1_spine_stabilization/trajectory/sources.py` — new `NoiseFilteredSource`
  class: median-of-N then EMA on `(box_center, theta)`, composed *before*
  `RateLimitedSource` in the wrapping chain.
- `configs/default.yaml` — new `gravity_comp.tauff_filter` block
  (`enabled`, `alpha: 0.3`) and `trajectory.noise_filter` block (`enabled`,
  `median_window: 5`, `ema_alpha: 0.3`).
- `scripts/run_pose_tracking_preview.py` / `scripts/grasp_demo.py` —
  `G1HandIK(...)` construction gains `tauff_filter_enabled`/
  `tauff_filter_alpha` read from config. `run_pose_tracking_preview.py`
  additionally wraps `source` in `NoiseFilteredSource` for `--source marker`
  only, before the existing `RateLimitedSource` wrapping.
- Four new test files: `tests/test_weighted_moving_filter.py` (5 tests),
  `tests/test_ipopt_diagnostics.py` (4 tests), `tests/test_tauff_filter.py`
  (7 tests), `tests/test_noise_filtered_source.py` (8 tests). 96 tests total.
- `CLAUDE.md` — IK Solver section ("Smoothing", "Known timing floor",
  new "Diagnostics" bullet), Open Design Questions (split "Target
  filtering" into jump/slew, now correctly scoped, and a new sensor-noise
  entry), Current Status, Session Roadmap, File Structure updated.
- `docs/sdk_analysis.md` — Section E's "~60 Hz achievable cadence" statement
  caveated (see below).

**Key findings:**
- **Hand-mass bug (found by the user, not the research agents):**
  `INSPIRE_HAND_MASS = 1.6` in `ik_solver.py` was a leftover local test
  edit — the file's own docstrings at lines 98/354/357 still said "1.0 kg
  measured hand," directly contradicting the constant two lines above them.
  Gravity-comp torque scales linearly with this constant, so `sol_tauff` had
  been ~60% too strong system-wide since whatever session this was changed
  in — a very plausible contributor to wrist jitter given the SDK's low
  wrist damping (`kd=1`).
- **`WeightedMovingFilter` reversed recency weighting (verified directly):**
  confirmed by reading the vendored SDK's own reference implementation
  (`~/g1_inspire_sdk/g1_inspire_sdk/_common/weighted_moving_filter.py`),
  which uses the identical weight vector `[0.4,0.3,0.2,0.1]` via
  `np.convolve(..., mode="valid")` — convolution flips the kernel, so its
  newest sample correctly gets weight 0.4. A background agent measured
  steady-state group delay on a ramp input: 2.0 samples (our buggy version)
  vs. 1.0 sample (correct). At the real ~6-10Hz loop rate this was
  ~250-300ms of unintended lag instead of ~125-150ms.
- **GIL contention (confirmed empirically, documented only — see "Out of
  scope" below):** a background agent ran a counting thread concurrently
  with a real IPOPT `opti.solve()` call; the thread's throughput dropped to
  2.2% of baseline during a 195ms solve, confirming CasADi/IPOPT holds the
  GIL for effectively the entire solve. Since `G1Arm`'s "250Hz" publisher
  thread runs in the same process, it's paused during every IK solve.
  Separately verified via grep: `G1Arm._clip_to_velocity_limit()`
  (`~/g1_inspire_sdk/g1_inspire_sdk/g1_arm.py:113,426`) uses a fixed
  `self._motor_dt = 1.0/250.0`, never measured elapsed time — the exact
  bug class already fixed in-repo via `RateLimitedSource`, but this
  instance lives inside the vendored SDK.
- **IPOPT `max_iter` exhaustion is plausible in practice:** a background
  agent empirically measured IPOPT's iteration count scaling with
  warm-start distance (4 iters at 0°, 7 at 10°, 8 at 20°, 11 at 45°, on a
  fixed warm start) — legitimate large tracking jumps could plausibly push
  iteration count toward `max_iter=30` in messier real bimanual geometry.
  No prior test/log had ever checked for `solve_ik()`'s `except` branch
  firing; there is now a counter.
- **SDK applies `tauff` as an unfiltered zero-order hold:** verified in
  `G1Arm._live_publisher_loop()` — `q_target` is velocity-clipped every
  250Hz tick, but `tauff_target` is passed straight through with no
  interpolation at all. Given gravity-comp torque ~1 N·m at the wrist and
  low wrist `kd=1`, an un-smoothed `sol_tauff` step every ~100-150ms could
  produce a small torque "kick" — mitigated by smoothing `sol_tauff`
  ourselves (fully in our own code, not the vendored SDK) before handoff.
- **`RateLimitedSource` cannot catch real sensor noise (this is the main
  finding tying directly to the mock-vs-real-marker question):** verified
  directly — its clamp only engages on deltas *larger* than the configured
  rate; anything smaller passes straight through by design (see its own
  test `test_theta_within_limit_passes_through_unchanged`). Real
  `solvePnP` detection jitter for a stationary marker is small and
  continuous, sitting below that threshold (which was deliberately set
  *above* the trajectory's own peak natural rate). `--source mock`/
  `--source sine` are perfectly noiseless and have never exercised this —
  it is a structural gap that only becomes visible with real markers.
  `NoiseFilteredSource` (median-of-N to reject single-frame misdetection
  spikes, then EMA to smooth continuous jitter) directly addresses this,
  applied *before* `RateLimitedSource` in the wrapping chain (clean the
  signal first, then bound the rate of the clean signal).

### Out of scope for this fix (deliberate, per user decision)

- **GIL contention / SDK publisher-thread starvation** — documented only,
  not fixed. This is architecture-level (would require running the IK
  solve without sharing the GIL with the publisher thread — a separate
  process, or a GIL-releasing extension — both much bigger changes) and
  lives partly inside the vendored SDK's own fixed-`_motor_dt` assumption,
  which cannot be modified (git-pull-only). Same deferral category as the
  IK-solve-speed item from the previous session.
- Tuning `tauff_filter.alpha`/`noise_filter.median_window`/
  `noise_filter.ema_alpha` against real hardware/marker data — all three
  are reasoned placeholders, not measurements (see comments in
  `configs/default.yaml`).
- Live re-test on hardware to confirm the mass fix and filter fix reduce
  visible jitter, and that the noise filter helps on real markers — human-
  run only.

### Things not modified

- `~/g1_inspire_sdk` — SDK not modified (the GIL-contention and
  fixed-`_motor_dt` findings live here but cannot be fixed without
  modifying the vendored clone).
- `assets/g1_with_hands.urdf`, EE frame defs, shoulder_roll bounds, IK cost
  weights, IPOPT solver settings — unchanged.
- `g1_spine_stabilization/trajectory/targets.py` (`compute_hand_targets`) —
  left as pure stateless geometry; noise filtering lives one layer up, in
  the source wrapper.

## SDK-6 marker-tracking fix follow-ups

### What was done

A live test with real AprilTag markers using `--tracking 6dof` showed the
robot's hands positioning themselves around the target marker instead of
the object marker. Investigation (three parallel research agents, direct
code verification, and back-and-forth with the user correcting several
initial hypotheses) found a real bug in `MarkerSource`, fixed it, and ruled
out two other suspected causes along the way.

**Files changed:**
- `g1_spine_stabilization/trajectory/sources.py` — `MarkerSource` restructured:
  new `self._object_start_pos` (captured once from the object marker,
  Phase A), `self._target_start_pos`/`self._target_start_rot` (captured once
  from the target marker, Phase B). New helpers `_theta_from_target_rotation()`
  (target's roll relative to its own starting orientation) and
  `_box_center_from_target_position()` (object's start position shifted by
  the target's displacement from its own start). Both `startup()`'s Phase C
  countdown and `step()` now use these helpers instead of assigning the
  target's raw absolute pose. Also added `tracker`/`obj_id`/`tgt_id`
  properties (used by the new diagnostics below).
- `g1_spine_stabilization/perception/tracker.py` — `MarkerTracker.step()` now
  keeps the intermediate `pose_in_optical`/`d435_mount`-frame poses (not just
  the final `pose_in_pelvis`) per marker, exposed via a new
  `get_diagnostic_pose(marker_id)` method. Internally computes the
  optical→d435→pelvis composition as two explicit matrix multiplies instead
  of one `compose_to_pelvis()` call (same math, exposes the intermediate);
  `compose_to_pelvis` itself is unchanged and still tested independently in
  `tests/test_perception_frames.py`.
- `g1_spine_stabilization/controllers/sdk_pose_tracking.py` — `_log_marker_info`
  gained two additions: (1) logs the actual computed `L_target`/`R_target`
  (already computed each Phase-2 iteration) to Rerun as `targets/L_hand`
  (cyan) / `targets/R_hand` (magenta), distinct from the raw marker triads
  and the robot's rendered mesh; (2) prints the marker's pose at each
  transform-chain stage (optical, d435-mount, pelvis) via the new
  `get_diagnostic_pose()`, to help isolate a separately-reported ~10cm
  marker-height discrepancy.
- `scripts/run_pose_tracking_preview.py` — marker-mode setup now prints the
  configured `perception.marker_size` with a reminder to verify it against
  the real printed marker's physical size (a mismatch would scale
  `solvePnP`'s recovered translation).
- `tests/test_marker_source.py` — three tests rewritten for delta semantics
  (`test_orient_only_theta_tracks_relative_rotation_not_absolute`,
  `test_6dof_box_center_tracks_target_displacement_not_absolute_position`,
  `test_startup_preseeds_nonzero_theta_delta_and_it_persists_when_stale`);
  new `_SequenceTracker` test helper (per-call scripted readings, needed
  since a static reading now correctly produces a zero delta). All other
  existing tests confirmed unaffected (staleness/monitoring behavior is
  orthogonal to absolute-vs-delta semantics) and still pass unchanged.
- `CLAUDE.md` — corrected four stale occurrences of the `d435_link_to_optical`
  calibration value, corrected the Project Goal's "runtime arguments" claim
  for object dimensions, rewrote the "Orientation-only vs 6-DoF" Resolved
  bullet to describe the corrected delta semantics, removed two File
  Structure entries for docs that don't exist (`frame_conventions.md`,
  `architecture.md`), added the missing `gravity_comp_reference.md` entry.

**Key findings:**
- **The actual bug**: `MarkerSource.step()`/`startup()` set `box_center =
  tgt_obs.pose_in_pelvis[:3, 3]` directly in 6dof mode — the target marker's
  raw absolute position, not a delta. If the object (in the robot's hands)
  and the target (e.g. held by a person) are in different physical
  locations — the realistic setup — this sends the hands to wherever the
  target physically is, not to the object shifted by how far the target has
  moved. Confirmed by the user: this is exactly what they saw, running
  `--tracking 6dof`. `orient-only` mode was unaffected (it never touches
  `box_center` after startup), which is why the bug only showed up in 6dof.
- **Ruled out: wrist-vs-palm IK targeting.** Verified directly that
  `solve_ik()`'s cost function computes error against
  `self.cdata.oMf[self.L_ee_id/self.R_ee_id].translation`
  (`ik_solver.py:295-296`), where those IDs are
  `self.reduced.model.getFrameId('L_ee_link'/'R_ee_link')`
  (`ik_solver.py:244-245`) — the palm frame, not the wrist joint. The user's
  observation that the marker sits exactly on the line between the two
  wrists is mathematically guaranteed by `compute_hand_targets`'s
  construction (`box_center` is always the midpoint of `L_pos`/`R_pos`)
  regardless of `box_half_width`, so it didn't indicate a targeting bug —
  the real issue was the 6dof position math above.
- **Ruled out: `d435_link_to_optical` calibration error.** The config's
  current rotation (`[-1.5708, 0.0, -1.5708]`) is correct — a prior Claude
  session had changed it to `[3.14159, 0.0, 1.5708]` and that turned out
  wrong, so it was reverted. CLAUDE.md was the stale one (still claiming the
  old value in four places), now corrected.
- **Ruled out: object dimensions as CLI runtime args.** CLAUDE.md's Project
  Goal claimed "object width and depth are passed as runtime arguments" —
  checked directly, this was never implemented anywhere (including the
  original CoppeliaSim entry point) and the user's actual preference is for
  box size to stay in `configs/default.yaml` (an object is selected once and
  used for a while). Corrected the doc to match.

### Out of scope for this fix

- **The ~10cm "markers appear lower in Rerun than physically" discrepancy
  remains unresolved.** With the calibration rotation now confirmed correct,
  remaining candidates (a `perception.marker_size` mismatch against the real
  printed marker, the URDF's fixed `d435_joint` mount origin not matching the
  physical bracket, or `d435_link_to_optical.xyz`'s translation, currently
  `[0,0,0]`) all require physical measurement against the real robot. The
  new transform-chain diagnostic print (`get_diagnostic_pose`) is meant to
  help the user isolate which stage is responsible, not a blind fix.
- No fix attempted for anything inside `~/g1_inspire_sdk` (none was needed
  here — the bug was entirely in our own `MarkerSource`).

### Things not modified

- `~/g1_inspire_sdk` — SDK not modified.
- `assets/g1_with_hands.urdf`, EE frame defs, shoulder_roll bounds, IK cost
  weights — unchanged.
- `g1_spine_stabilization/perception/frames.py` (`compose_to_pelvis`) —
  unchanged; `tracker.py` now computes the same composition as two explicit
  steps internally, but the pure-function utility itself and its tests are
  untouched.
- `configs/default.yaml`'s `d435_link_to_optical.rpy` — left as-is; confirmed
  correct by the user, only CLAUDE.md's documentation of it was wrong.

## Ctrl-C interrupt-swallowing fix

### What was done

Fixed a real responsiveness bug reported directly from a live-hardware run:
stopping `run_pose_tracking_preview.py` with Ctrl-C could take 8+ presses over
several seconds before the script actually entered its shutdown sequence,
each press instead printing `[G1HandIK] IK warning (exhaustion #N)` and the
control loop continuing as if nothing happened.

**The actual bug**: `solve_ik()`'s `except Exception as e:`
(`ik_solver.py:436`) was catching more than genuine IPOPT convergence
failures. CasADi's Ipopt interface checks for a pending SIGINT *during* a
long solve and aborts it by throwing its own C++ `KeyboardInterruptException`
(visible in the log as `CasADi ... WARNING("KeyboardInterruptException")`).
That crosses into Python as a plain exception — not a real Python
`KeyboardInterrupt` (a `BaseException`, so it would never have reached this
`except Exception` clause in the first place) — with a message containing
`return_status is 'NonIpopt_Exception_Thrown'`. The existing code treated
this identically to an ordinary solver failure: incremented
`ipopt_exhaustion_count`, printed a warning, fell back to
`opti.debug.value()`, and returned normally, completely absorbing the Ctrl-C.
Since `solve_ik()` costs ~70-160ms/call and dominates each loop iteration's
wall time (documented "Known timing floor"), most Ctrl-C presses land
*inside* an active solve and get swallowed this way — matching exactly what
was observed.

**Fix**: `solve_ik()` now checks for the `NonIpopt_Exception_Thrown` signature
before falling into the exhaustion-counting path. When present: skip the
counter, print a distinct "interrupted" message, and re-raise as a real
`KeyboardInterrupt` so it propagates to the caller's existing (already
correctly structured, per SDK-6-safety) interrupt handling. Any other
exception text is unaffected — genuine solver failures still fall back to
`opti.debug.value()` exactly as before.

Fixing this centrally in `ik_solver.py` exposed two related gaps in callers
that loop over `solve_ik()`, both fixed in the same pass:
- `sdk_pose_tracking.py`'s Phase 3 (return-to-default) had no
  `try/except KeyboardInterrupt` at all, unlike Phase 1/2 in the same file —
  added the same pattern, falling through unconditionally to the existing
  settle-sleep + "Done." lines.
- `grasp_sequence.py`'s Phase R wrapped the return-motion loop *and* the
  trailing settle-sleep in the same `try` — an interrupt during the loop
  jumped straight past the settle-sleep to the outer `except`. This was
  previously unreachable in practice only because the ik_solver.py bug was
  swallowing these interrupts too. Restructured so only the loop itself is
  wrapped, and the settle-sleep always runs afterward regardless of how the
  loop ended (mirrors the existing Phase D pattern already in this file).

### Tests

- New `tests/test_ik_interrupt_handling.py` (3 tests): the
  `NonIpopt_Exception_Thrown` signature raises `KeyboardInterrupt` and does
  *not* increment `ipopt_exhaustion_count`; an ordinary solver failure
  (different message) still increments the counter and falls back normally,
  unchanged.
- `tests/test_sdk_transport.py`: new test interrupting the 4th `solve_ik()`
  call (deterministically the first call inside Phase 3, given `cfg_fast`'s
  known Phase-1 step count and `duration=0.0` skipping Phase 2 entirely) —
  confirms `run_pose_tracking_sdk` doesn't raise and still settles.
- `tests/test_grasp_demo_safety.py`: new test interrupting the first
  `solve_ik()` call inside Phase R — confirms the settle-before-shutdown
  pause still runs even though the interruption happened inside Phase R's
  loop, not before it.

### Things not modified

- No signal-handling code (`signal.signal(SIGINT, ...)`) was added — CasADi
  already interrupts long solves responsively on its own; the bug was purely
  that our own broad `except Exception` re-labeled that interrupt as an
  ordinary solver failure instead of letting it propagate.
- `pose_tracking.py` (CoppeliaSim path) benefits from the `ik_solver.py` fix
  automatically (a real `KeyboardInterrupt` now propagates instead of being
  swallowed) but has no `try/except` of its own — left as-is, since it's the
  sim path, not the hardware-safety-critical one, and an uncaught interrupt
  there is a strict improvement (immediate stop) over the previous silent
  swallow-and-continue behavior.

## Light grip during pose tracking

### What was done

`run_pose_tracking_sdk` (`sdk_pose_tracking.py`) previously only drove the
arms — it never commanded the Inspire hands, even though
`run_pose_tracking_preview.py` already constructs the robot with
`hand_side="both"`. Grip control (`GraspController`, `grip_force.py`) existed
only in the separate, static `grasp_demo.py`/`grasp_sequence.py` hold-in-place
demo. Wired the same controller into the actual tracking flow so the hands
hold a light, config-driven grip (`grip_force.tau_baseline`, already
config-driven — the ask was to make sure it's actually applied during
tracking, not to add a new tunable) while the arms track a moving target.

New "Phase 1.5" between Phase 1 (arm to grasp) and Phase 2 (track): lerp both
hands from `open_pose` to `close_pose` (1.5s, matching `grasp_sequence.py`'s
Phase B), settle 0.5s, then `ctrl.arm(q_measured)` to snapshot the
force-feedback baseline. During Phase 2, `ctrl_l.step()`/`ctrl_r.step()` run
every iteration alongside the arm's `solve_ik()`/`set_joint_targets()`. A
guaranteed-release step runs immediately after Phase 2 (regardless of how it
ended — clean finish or `KeyboardInterrupt`), lerping both hands back to
`open_pose` over 1.0s, *before* Phase 3's arm-return begins — mirrors
`grasp_sequence.py`'s existing Phase D-before-R order (user's explicit
choice: release where tracking left off, don't carry the object through the
arm-return motion still gripped). All of this is gated by a new
`grip_force.enabled` config flag (default `true`), following the same
on/off convention as `trajectory.rate_limit.enabled` /
`trajectory.noise_filter.enabled` / `gravity_comp.enabled`.

`GraspController.step()` is already safe to call before `arm()` has been
reached (returns a static `close_pose` hold per its existing docstring), so
no extra state flag was needed to guard Phase 2's grip step against a
Ctrl-C that landed mid-close during Phase 1.5.

### Tests

Three new tests in `tests/test_sdk_transport.py`: grip engages (records a
closed-ish hand command) and releases (last command ≈ `HAND_OPEN_POSE`) on a
normal run; `grip_force.enabled=False` issues zero hand commands, preserving
today's arm-only behavior exactly; and a `KeyboardInterrupt` injected inside
Phase 2 still ends with the hands released.

### Things not modified

- Acceleration-coupled grip tightening (`tau_des = tau_baseline +
  k_a*||a_arm||`) — user explicitly chose a fixed light grip for now; the
  extension point already documented in `grip_force.py`'s docstring remains
  unused.
- Hand-close/settle/release durations (1.5s/0.5s/1.0s) are hardcoded in
  `sdk_pose_tracking.py`, matching `grasp_sequence.py`'s existing values
  exactly rather than adding new config keys — the user only asked for the
  grip *amount* to be tunable, which `grip_force.tau_baseline` already was.
- `grasp_demo.py`/`grasp_sequence.py` — unchanged; still the standalone
  static hold-in-place validation path, independent of this integration.

## d435_link_to_optical calibration (resolves the ~10cm marker-height discrepancy)

### What was done

Added `perception/calibration.py` (pure math: `solve_optical_to_d435`,
`se3_to_xyz_rpy`, `pose_error`) and `scripts/calibrate_camera_extrinsics.py`
(camera-only CLI, reuses the existing `RealSenseSource`/`AprilTagDetector`/
`MarkerTracker` stack) to solve for `perception.d435_link_to_optical`'s
xyz + rpy from one marker placed at a physically-measured, known pelvis-frame
position (assumed identity orientation). `T_d435_to_pelvis` (Pinocchio FK
against the URDF) is trusted and not solved for. Default mode prints the
computed correction for manual paste into the YAML (no auto-write, per the
existing caution around this value — see "SDK-6 marker-tracking fix" above).
A `--verify` mode checks the *current* config's residual error against a
second known point without doing any correction math, to catch a
`perception.marker_size` scale error masquerading as a mounting offset (the
two look identical at one distance, diverge at another).

**Result, run against the two markers already in the robot's field of view**
(object marker ID=0 at physically-measured pelvis position (0.40, 0, 0),
target marker ID=2 at (0.50, 0, 0), both close to identity orientation):

- Calibrating off marker ID=0: measured error with the old
  `xyz: [0,0,0]` / `rpy: [-1.5708, 0.0, -1.5708]` was **10.06cm position,
  9.19° rotation** — matches the previously-reported ~10cm discrepancy
  closely enough to confirm it as the root cause. Computed correction:
  `xyz: [-0.0860, -0.0002, -0.0384]`, `rpy: [-1.4118, 0.0076, -1.5896]`.
- Applied that correction to `configs/default.yaml`, then `--verify`'d
  against marker ID=2 (a different marker, different distance, never used
  in the fit): **6mm position error, 4.05° rotation error.** The position
  residual dropped by an order of magnitude and shows no sign of the
  distance-dependent growth a `marker_size` scale error would produce
  (50cm is 25% farther than 40cm; a proportional scale error would show up
  much larger than 6mm) — the position fix generalizes. The remaining 4°
  rotation residual is plausibly just physical marker-placement imprecision
  (the user's own caveat when placing them: "close to identity with some
  room for error, I can't set them perfectly"), not a code/calibration bug.

**Left applied in `configs/default.yaml`** pending the user's final review —
a large, consistent improvement, but flagged as tentative until confirmed
(see the config comment).

### Tests

5 new hardware-free tests in `tests/test_calibration.py`: `solve_optical_to_d435`
recovers a known synthetic transform exactly; `se3_to_xyz_rpy` round-trips
against `optical_to_d435_from_config`; `pose_error` matches known synthetic
position/rotation offsets.

### Out of scope / not modified

- `perception.marker_size` — not touched; the verify-mode result argues
  against it being a significant factor here, but wasn't ruled out with a
  dedicated distance-sweep test.
- URDF `d435_link` mount origin — not touched (Hard Rule: URDF changes need
  explicit approval); any real CAD mounting error is absorbed into the
  `d435_link_to_optical` correction instead, which is the intended,
  config-tunable place for it.

## box_half_height: correcting for the object marker sitting on top of the box

### What was done

The object marker is physically mounted on **top** of the held object, but
`MarkerSource` was treating the marker's raw measured position as the box's
geometric center directly. `compute_hand_targets` already offsets each hand
by `box_half_width` from `box_center` in Y — confirmed (via `grep`) to be
the *only* place `box.size` was read anywhere in the package; `box_size[2]`
(height) was read nowhere. So every hand target was too high by however far
the marker sits above the true center (8cm with the default `0.16` cube).

Fixed at the source: `MarkerSource.__init__` gained a `box_half_height`
kwarg (default `0.0`, preserving old behavior for any caller that doesn't
pass it), applied once in `startup()` Phase A where `self._object_start_pos`
is first captured — subtracting `box_half_height * marker_R[:, 2]` (the
marker's own measured "out of the surface" axis — `detector.py`'s solvePnP
object points are documented "Z out of marker" — expressed in the pelvis
frame, not a hardcoded world-Z). Using the marker's actual measured
orientation rather than assuming it's level was a deliberate choice
confirmed with the user, and costs nothing extra since the full pose
(including rotation) is already read at that moment. Fixing it here (not
just at the `run_pose_tracking_preview.py` call site) matters because
`6dof` mode's ongoing delta-tracking uses this same internal value as its
base reference for the rest of a session, not just the one-shot returned
`box_center`.

`run_pose_tracking_preview.py` now computes `box_half_height =
box_size[2] / 2` and passes it into both `MarkerSource(...)` construction
sites (`--source mock` and `--source marker`). `_AnimatedMockTracker` was
also updated to simulate a marker mounted on top of the box (offset by
`box_half_height` in Z) rather than handing back the ground-truth center
directly — makes the mock a more faithful stand-in for a real marker
reading and means `--source mock --tracking 6dof` dry-run testing actually
exercises the new correction path end-to-end (verified: with
`box.size=[0.16,0.16,0.08]` temporarily set, the dry-run round-trip
correctly reproduced the original intended `box_center`, confirming the
mock's added offset and `MarkerSource`'s new correction cancel out exactly
as designed).

### Tests

3 new tests in `tests/test_marker_source.py`: identity-rotation marker case
(offset subtracts along world Z); a 90°-tilted marker case (offset follows
the marker's measured Z axis into world -Y instead, proving it isn't
hardcoded to world-up); and `box_half_height=0.0` reproduces the exact
pre-change behavior (regression guard). All 12 pre-existing tests in the
file pass unchanged (none pass `box_half_height`, so they exercise the new
default).

### Things not modified

- `compute_hand_targets`/`trajectory/targets.py` — unchanged; the width
  offset there was already correct, this fix is orthogonal (corrects where
  `box_center` itself comes from, not how hands are offset from it).
- `pose_tracking.py` (CoppeliaSim path) and `grasp_sequence.py`'s Phase 0 —
  unchanged; both use a synthetic `box_center` (`[L_EE_DEFAULT[0], 0.0,
  L_EE_DEFAULT[2]]`), never derived from a marker reading, so there's no
  marker-on-top offset to correct there.

## Data recording (robot + human trial comparison)

### What was built

Goal: record trial data from the robot's live tracking runs, and from a
human performing the same physical task, into pickle files sharing one
schema, so the two can later be compared.

**Files changed:**
- `g1_spine_stabilization/data_recording/recorder.py` (new module, with
  empty `__init__.py`) — the shared schema/logic imported identically by
  both the robot controller and the human-trial script:
  - `angle_diff_rpy(R_object, R_target)` — per-axis [roll, pitch, yaw]
    angle difference, computed by decomposing the single relative rotation
    `R_object.T @ R_target` via `scipy.spatial.transform.Rotation.
    as_euler('xyz')` (not independent per-marker Euler angles subtracted).
  - `angular_velocity_rotvec(R_prev, t_prev, R_cur, t_cur)` — finite-
    difference angular velocity via `rotvec(R_cur @ R_prev.T) / dt`;
    returns `None` (not a divide-by-zero) when `dt <= 0`.
  - `MarkerSample` dataclass — one recorded instant: `t`, `wall_time`,
    both markers' `*_pose`/`*_obs_timestamp`/`*_reprojection_error`,
    `angle_diff_rpy`, both `*_angular_velocity`, `theta` (robot-only), and
    a `load_cell_wrench` field reserved for the future neck-mounted load
    cell (always `None` today — no sensor exists yet, this just keeps the
    schema forward-compatible).
  - `RunRecorder` — accumulates `MarkerSample`s via `start()`/`record()`,
    pickles them via `save()`. `save()` is idempotent (a second call is a
    no-op returning the same path) so it's safe to call unconditionally
    from a `finally` block.
  - `default_run_path()` — builds `data/runs/run_<timestamp>_<source>
    [_<tracking_mode>].pkl`, cleanly omitting the tracking-mode segment
    when it's `None` (e.g. `--source sine`).
- `g1_spine_stabilization/controllers/sdk_pose_tracking.py` —
  `run_pose_tracking_sdk` gained an optional `recorder=None` parameter.
  `recorder.start()` runs at the top of Phase 2; `recorder.record(...)`
  runs once per Phase-2 iteration, fed `getattr(source, 'last_obj_obs',
  None)`/`getattr(source, 'last_tgt_obs', None)` (works uniformly across
  `SineSource`, which has no such attributes, and `MarkerSource`/its
  `RateLimitedSource`/`NoiseFilteredSource` wrappers, which proxy them via
  `__getattr__`). `recorder.save()` runs in a `finally` attached to Phase
  2's existing `try/except KeyboardInterrupt` — the inner try/except is
  untouched (same print, same swallow-and-fall-through), so this is a
  strictly additive change. A comment at the `finally:` explains why a
  plain `try/finally` is used here instead of `grasp_sequence.py`'s
  `pending_exc`-reraise pattern (that pattern guarantees *multiple*
  sequential hardware side effects each resilient to a further interrupt;
  here there's exactly one guaranteed side effect, and `save()` is a pure
  pickle write that can't itself raise).
- `scripts/run_pose_tracking_preview.py` — new `--record [PATH]` flag
  (`nargs="?", const="__AUTO__"`: absent → disabled, bare `--record` →
  auto-generated path, `--record path.pkl` → explicit path). Constructs a
  `RunRecorder` after the source/rate-limit/noise-filter wrapping chain is
  finalized, gated by `data_recording.enabled` in config. `--source sine`
  stays recordable (useful for exercising the pipeline with zero hardware)
  but prints a warning that only `t`/`wall_time`/`theta` will be populated,
  since `SineSource` has no real markers.
- `scripts/record_human_trial.py` (new) — perception-only script: no
  robot, no IK, no `compute_hand_targets`. Reuses the same building blocks
  as `visualize_markers.py`'s `run_realsense()` (`RealSenseSource`,
  `AprilTagDetector`, `MarkerTracker`, `d435_to_pelvis_from_pinocchio()`
  with no `ik` arg — pure Pinocchio FK, no robot connection) and the same
  `configs/default.yaml` `perception:` section, so both scripts track
  identical markers. Imports `RunRecorder` from the same module as the
  robot script, making schema identity structural rather than
  conventional. `--source {realsense,mock}`, `--dry-run`, `--tracking`
  (label only, no math), `--record [PATH]`, `--duration`. No Rerun
  dependency (kept simple, matching `visualize_markers.py`'s style) — this
  also means, unlike `run_pose_tracking_preview.py`, it can be smoke-tested
  directly through the Bash tool without hitting the Rerun-spawn sandbox
  limitation noted below.
- `configs/default.yaml` — new `data_recording:` section (`enabled`,
  `output_dir: "data/runs"`, `filename_template`), inserted after
  `shutdown:`.
- `.gitignore` — added `data/runs/` (in addition to the existing generic
  `*.pkl` entry, to also cover the directory itself and any future
  non-`.pkl` artifact placed there).
- `tests/test_recorder.py` (new, 15 tests) — `angle_diff_rpy`/
  `angular_velocity_rotvec` correctness and guard conditions, `RunRecorder`
  first/second-sample velocity behavior, `None`-observation handling
  (the `--source sine` case), pickle round-trip, `save()` idempotency,
  nested-directory auto-creation, `default_run_path()` filename format
  (including the no-tracking-mode case), constructor validation.
- `tests/test_sdk_transport.py` — one new test,
  `test_pose_tracking_sdk_saves_recording_after_interrupt_during_tracking`,
  modeled directly on the existing
  `test_pose_tracking_sdk_releases_hands_after_interrupt_during_tracking`:
  injects a `KeyboardInterrupt` via `monkeypatch` on `ik.solve_ik`'s first
  Phase-2 call, runs `run_pose_tracking_sdk(..., recorder=recorder)`,
  confirms the pickle file exists afterward.

### Test results

184 tests pass (`pytest tests/ -v`), up from the count previously cited in
CLAUDE.md's Current Status — that number was already stale before this
session (a discrepancy the Plan agent flagged during design, consistent
with a similar stale-count note from SDK-5). CLAUDE.md has been updated to
cite 184 directly rather than propagate another soon-to-be-stale number.

### Human-run smoke checks (not executed by Claude Code)

- `scripts/record_human_trial.py --source mock --record --duration 3` WAS
  run directly (no Rerun dependency, so no sandbox issue) — produced a
  well-formed pickle; inspected manually: `angle_diff_rpy`/
  `object_angular_velocity`/`target_angular_velocity` all matched the mock
  tracker's known rotation-about-X motion.
- `scripts/run_pose_tracking_preview.py --source sine --record` /
  `--source mock --tracking 6dof --record` were **not** run directly, per
  the existing sandbox limitation (see `[[sdk6-jerk-fix-decisions]]` /
  memory): `RerunPreview(spawn=True)` spawns a detached GUI/server process
  that doesn't exit when the script finishes, causing pipe hangs. The
  `--record` code path this would exercise is validated instead by (1) the
  new `test_pose_tracking_sdk_saves_recording_after_interrupt_during_tracking`
  test, which runs the exact same `run_pose_tracking_sdk(..., recorder=...)`
  call the script makes, without spawning Rerun, and (2) `RunRecorder`
  itself being fully unit-tested. Still recommended before calling this
  session done: `python scripts/run_pose_tracking_preview.py --source sine
  --record` and `--source mock --tracking 6dof --record`, run by hand,
  confirming a `.pkl` lands under `data/runs/`.
- Real-hardware recording (`--source marker --no-dry-run --record` on the
  robot; `record_human_trial.py --source realsense` with a real camera) is
  human-run only, per CLAUDE.md's Hard Rules for the robot script and
  simply not yet exercised against real hardware for the human script.

### Rough edges for next session

- No comparison-analysis script yet (loading a robot pickle and a human
  pickle side-by-side and plotting/diffing them) — explicitly out of scope
  for this feature per the request; natural next step once real trial data
  exists.
- `load_cell_wrench` is a schema placeholder only; wiring it up is blocked
  on the neck-mounted load cell not existing yet.
- No noise/rate-limiting is applied to the *recorded* marker data — the
  recorder reads whatever `source.last_obj_obs`/`last_tgt_obs` are on the
  wrapped source, which for `--source marker` already passes through
  `NoiseFilteredSource`/`RateLimitedSource` before `compute_hand_targets`
  sees it, but the raw (pre-filter) marker reading is not separately
  available to the recorder without further plumbing. Worth deciding,
  once real marker data exists, whether the comparison analysis wants
  filtered or raw marker poses.

### Things not modified

- `assets/g1_with_hands.urdf`, EE frame defs, shoulder_roll bounds, IK cost
  weights — unchanged.
- `~/g1_inspire_sdk` — not modified.
- Phase 1/1.5/3 and their existing `KeyboardInterrupt` handling in
  `sdk_pose_tracking.py` — untouched; only Phase 2 gained recorder calls
  and a `finally`.
- `trajectory/sources.py` — unchanged; the recorder reads its existing
  `last_obj_obs`/`last_tgt_obs`/`object_lost` properties, no new ones
  needed.

### Addendum: robot_run_/human_run_ filename prefix

Follow-up requested right after the session above: origin (robot vs. human)
was only visible in an auto-generated filename via `source_label`
(`sine`/`mock`/`marker` vs. the then-hardcoded `"human"`), or by opening the
pickle and checking `origin`. Added an explicit prefix so it's visible at a
glance in `data/runs/`.

- `data_recording/recorder.py` — `DEFAULT_FILENAME_TEMPLATE` gained a
  leading `{origin}_run_` (was just `run_`); `default_run_path()` gained an
  `origin: Optional[str] = None` parameter. When `origin` is `None` the
  existing underscore-collapse regex strips the now-empty leading segment,
  so any caller that doesn't pass `origin` still gets the old unprefixed
  `run_...` filename — backward compatible by construction, not by a
  special case. `RunRecorder.__init__` passes `origin=self.origin`
  (already validated to `"robot"`/`"human"`) automatically, so neither
  `run_pose_tracking_preview.py` nor `record_human_trial.py` needed a
  call-site change for the prefix itself.
- `scripts/record_human_trial.py` — **`source_label` semantics changed**:
  was hardcoded `"human"` (which would have made the new prefix redundant,
  `human_run_..._human_...`); now `args.source` (`"realsense"` or
  `"mock"`), so filenames read e.g. `human_run_20260720_..._realsense_
  orient-only.pkl` — no redundancy, and it additionally tells you at a
  glance whether a given human-trial file came from a real camera trial or
  a hardware-free mock smoke test.
- `configs/default.yaml` — `data_recording.filename_template` default
  updated to match; inline comment explains the `{origin}` placeholder.
- `tests/test_recorder.py` — 3 new tests (prefixed-by-origin for human,
  the origin=None backward-compat fallback, `RunRecorder`'s
  auto-generated path carrying the right prefix for both origins); the
  two pre-existing `default_run_path` filename tests updated to pass
  `origin="robot"` and assert the new pattern. 187 tests pass total.

Things not modified: the pickle's internal `origin` field (already existed,
unchanged — the filename prefix is a convenience on top of it, not a
replacement); `MarkerSample` schema; the `--record` CLI flag itself on
either script.

### Addendum: angle_diff_rpy reference-frame convention (biomechanics alignment)

Follow-up discussion right after the session above surfaced a real
ambiguity: `angle_diff_rpy` decomposes a SINGLE relative rotation, so it's
anchored to whichever marker is passed as the first argument — this was
`object` in the original implementation, meaning the recorded roll/pitch/
yaw were expressed in the moving segment's own frame, not the reference
segment's, for anyone reusing this for the human neck trial (head =
`object_marker_id`, torso = `target_marker_id`, per the user's stated
convention: "the reference target is the torso while the head is the
object, since the object is what we are moving"). The user wants recorded
errors to match the standard biomechanics convention — distal/moving
segment (head) described relative to the proximal/stable reference segment
(torso), in the torso's own axes — so this maps cleanly to clinical
flexion/extension/rotation/lateral-bending terminology later, and asked
that the convention be very well documented since it's easy to get backward
(this conversation did, once, before the correction).

**Files changed:**
- `data_recording/recorder.py` — `angle_diff_rpy(R_object, R_target)`
  renamed to `angle_diff_rpy(R_reference, R_moving)` (same math,
  `R_reference.T @ R_moving`, intrinsic-xyz decomposition) — the rename
  itself is part of the fix: "object"/"target" role names don't by
  themselves indicate which one anchors the decomposition, which is
  exactly the confusion that prompted this addendum. `RunRecorder.record()`
  now calls `angle_diff_rpy(target_pose[...], object_pose[...])` —
  **target is always the reference, object is always moving** — correct
  for both the robot case (target = external tracking reference) and the
  human case (target_marker_id = torso) without any anatomy-specific
  special-casing, because `target` already plays the "stable reference"
  role by definition in both conventions this module serves. Module
  docstring rewritten with a prominent "Reference-frame convention" section
  covering: the math, the biomechanics rationale, the worked head/torso
  example, an explicit contrast with `MarkerSource._theta_from_target_
  rotation` (`trajectory/sources.py` — a different, single-axis,
  relative-to-target's-own-starting-pose computation used only to drive
  the IK, not to be confused with this field), and a caveat that
  `object_angular_velocity`/`target_angular_velocity` remain in the
  ambient pelvis frame (space-fixed), not body-relative like
  `angle_diff_rpy` — a known, deliberate asymmetry, not silently changed
  since the user's request was specifically about the angle metric.
- `docs/data_recording_conventions.md` (new) — consolidates everything
  worked out across the data-recording conversation into one authoritative
  reference: schema recap, filename convention, the angle_diff_rpy
  reference-frame convention in full (main content), the pelvis-frame
  invariance proof (angle_diff_rpy and angular-velocity *magnitude* are
  exactly invariant to camera placement/calibration error — proven
  algebraically via the shared-transform-cancels argument; absolute
  poses and angular-velocity *axis* direction are not), and the marker
  local-axis convention from `detector.py`'s `_object_points` (+X =
  rightward along the tag's canonical-upright top edge, +Y = upward, +Z =
  out of the printed face), with a pointer to verify it physically via
  `scripts/visualize_markers.py --source realsense`'s `drawFrameAxes`
  overlay on the **target** (reference) marker specifically.
- `configs/default.yaml` — one-line pointer comment on `data_recording:`
  to the new doc, matching the existing `palm_press` → `docs/
  palm_force_control.md` pattern.
- `tests/test_recorder.py` — 3 new tests: `test_angle_diff_rpy_argument_
  order_is_not_a_sign_flip` (proves swapping reference/moving is not just
  a sign flip for non-trivial rotations — a direct regression guard against
  silently reverting to the old, ambiguous convention), `test_recorder_
  angle_diff_anchored_to_target_not_object` (proves `RunRecorder.record()`
  itself, not just the pure function, anchors to target). The two
  pre-existing `angle_diff_rpy` tests renamed their local variables to
  `R_reference`/`R_moving` for consistency; `test_recorder_pickle_round_
  trip`'s expected value updated to the new `angle_diff_rpy(target, object)`
  call order. 2 new tests (189 total).

**Key finding worth restating:** swapping which matrix is the reference is
NOT a sign flip except to first order for small angles — `R.T`'s Euler
decomposition differs from `-euler(R)` elementwise for non-trivial
rotations, because the intrinsic-XYZ sequence is order-dependent (roll
first, then pitch about the already-rolled Y, then yaw about the
twice-rotated Z). This is why the old object-anchored data would NOT have
simply needed a sign flip to reinterpret as torso-anchored after the fact
— the fix had to happen at the source, and any data recorded before this
change should be treated as head-frame-anchored, not silently
reinterpreted.

Things not modified: `angular_velocity_rotvec`'s math or its
pelvis-frame (not body-relative) convention — flagged as a documented
caveat/open question per the user's decision to scope this fix to the
angle metric only; `MarkerSample`'s field list (no new fields, only the
`angle_diff_rpy` field's *meaning* changed); `theta`/`MarkerSource._theta_
from_target_rotation` (a deliberately separate, unrelated computation).

### Addendum: angular velocity moved to the camera optical frame (no pelvis composition)

Immediate follow-up to the addendum above. Once `angle_diff_rpy` was fixed
to be target/torso-anchored, the user asked what frame
`object_angular_velocity`/`target_angular_velocity` should use, since
"pelvis frame" isn't a real frame for the human trial (no G1, no pelvis —
`T_d435_to_pelvis` there is FK for a robot that isn't present). Discussion
landed on the actual intended use of these two fields: cross-trial
**normalization** — how much was the torso/head actually moving in a given
trial, since it won't be identical trial-to-trial, not axis-level
biomechanical interpretation (that's what the now-fixed `angle_diff_rpy` is
for). That's a magnitude-only need, and magnitude is exactly invariant to
any fixed rotation composition (proof: rotations preserve vector norm) —
so carrying `T_d435_to_pelvis`/`T_optical_to_d435` through this computation
added a dependency on transforms that are partly fictional (human trial) or
robot-specific-calibrated (`T_optical_to_d435`) for zero benefit. Decision:
remove the composition entirely — compute both fields directly from each
marker's own consecutive camera **optical-frame** readings.

**Files changed:**
- `perception/types.py` — `MarkerObservation` gained `pose_in_optical:
  Optional[np.ndarray] = None` (4x4 SE3, OpenCV/REP-103 convention: X
  right, Y down, Z forward). Optional with a `None` default specifically so
  every existing construction site (tests, mock trackers,
  `visualize_markers.py`'s `run_mock()`) keeps working unmodified — only
  sites that want the new angular-velocity behavior need to populate it.
- `perception/tracker.py` — `MarkerTracker.step()` now passes
  `pose_in_optical=det.pose_in_optical` into the `MarkerObservation` it
  constructs (one line; the data was already in scope, just never
  forwarded). This is the real/production path, so every live detection
  (robot and human, `--source marker`/`realsense`) gets it automatically.
- `data_recording/recorder.py` — `RunRecorder.record()`'s angular-velocity
  computation switched from `object_pose[:3,:3]`/`target_pose[:3,:3]`
  (pelvis-frame) to `object_obs.pose_in_optical[:3,:3]`/`target_obs.
  pose_in_optical[:3,:3]`. When `pose_in_optical` is `None`, the field is
  `None` for that sample — no fallback to pelvis-derived rotation, which
  would silently reintroduce the exact composition being removed.
  `_prev_obj`/`_prev_tgt` now track optical-frame rotations.
  `angular_velocity_rotvec` itself is unchanged (already frame-agnostic —
  only which rotation `RunRecorder` feeds it changed). Module docstring's
  caveat paragraph and the `MarkerSample` field comments rewritten to state
  the new convention and explicitly warn against cross-referencing these
  fields' axes with `angle_diff_rpy`'s (different frames, different
  purposes — magnitude-only normalization vs. axis-level clinical mapping).
- `scripts/run_pose_tracking_preview.py`'s `_AnimatedMockTracker` and
  `scripts/record_human_trial.py`'s `_AnimatedHumanMockTracker` — both now
  populate `pose_in_optical` (reusing the already-computed synthetic pose;
  there's no real camera to be accurate about, it just needs to be *some*
  consistent rotation) so `--source mock --record` smoke tests exercise
  the full schema instead of silently getting `None` for these two fields.
- `docs/data_recording_conventions.md` — §4 rewritten from "what's
  invariant to camera calibration" into "two different frames, on purpose":
  states plainly that `angle_diff_rpy` (target-anchored) and the angular
  velocities (camera optical-frame) now live in genuinely different frames
  for different reasons, with a table and the reasoning for each.
- `tests/test_recorder.py` — `_obs()` helper gained an optional
  `R_optical` parameter. `test_recorder_second_sample_computes_velocity`
  renamed to `..._from_optical_not_pelvis` and strengthened: `pose_in_
  pelvis` is now held IDENTICAL across both samples while `pose_in_optical`
  actually rotates, so the test would fail if the implementation
  accidentally read pelvis data (it would report zero velocity instead of
  the expected nonzero rate) — a direct proof, not just an assertion. New
  `test_recorder_angular_velocity_none_when_pose_in_optical_unavailable`
  (graceful degradation).
- `tests/test_tracker.py` (new) — no existing test file covered the real
  `MarkerTracker` (as opposed to `MockMarkerTracker`, covered by
  `test_mock_tracker.py`) at all, so two tests added with a stub frame
  source/detector (no real camera or cv2 detection): confirms `step()`/
  `get_latest()` populate `pose_in_optical` matching the raw detection
  exactly.

**Why this doesn't conflict with the target-frame decision for
`angle_diff_rpy`:** they're solving different problems. `angle_diff_rpy`
needs the pelvis-frame composition (or rather, needs to be invariant to
it, which target-anchoring provides) because its output is read
axis-by-axis for clinical mapping. The angular-velocity fields never
needed axis-level meaning — only a trial-comparable scalar — and magnitude
never depended on the composition in the first place, so removing it here
loses nothing and drops a dependency on two calibration transforms (one
partly fictional, one robot-specific) that weren't earning their keep for
this particular pair of fields.

Test count: 3 new tests (192 total, confirmed via `pytest tests/ -v`).

## ATI Axia80 F/T sensor integration

### What was built

The ATI Axia80-M20 6-axis force/torque sensor (Ethernet, RDT-over-UDP port
49152) is now read into every `--record` run — robot and human — populating
the previously-reserved load-cell field, in two forms (raw + gravity-
cancelled) per the user's request. `scripts/sensor_test.py` (a standalone UDP
print-loop with placeholder calibration) was the starting point.

**Files changed / added:**
- `g1_spine_stabilization/sensors/force_torque.py` (new `sensors/` subpackage)
  — `ForceTorqueSensor` (background-daemon-thread RDT/UDP reader, lock-guarded
  latest wrench, mirrors `PalmForceReader`'s pattern but owns its own recv
  thread since RDT is a raw stream; degrades to `read_raw()->None` on
  connect timeout with a printed diagnostic pointing at the setup doc);
  `GravityCompensator` (pure-numpy: orientation-aware force cancellation +
  tared torque); `ForceTorqueSource` (glue both scripts share:
  `from_config`/`start`/`capture_reference`/`read`/`close`). Dependency-light
  (socket/struct/threading/numpy/scipy) — no robot/IK/SDK import.
- `data_recording/recorder.py` — `SCHEMA_VERSION` 1→2; single reserved
  `load_cell_wrench` replaced by `load_cell_wrench_raw` +
  `load_cell_wrench_grav_comp` (both `(6,)` `[Fx,Fy,Fz,Mx,My,Mz]` N/N·m,
  sensor frame); `record()` gained the two matching keyword-only params.
- `controllers/sdk_pose_tracking.py` — `run_pose_tracking_sdk` gained an
  optional `wrench_source` param; Phase 2 captures the gravity reference on
  the first step (when a torso rotation is available) and reads raw+comp each
  iteration into `recorder.record(...)`. New `_obs_rotation` helper (optical-
  preferred torso rotation).
- `scripts/run_pose_tracking_preview.py` + `scripts/record_human_trial.py` —
  both construct a `ForceTorqueSource` (config-gated, `--force-sensor`/
  `--no-force-sensor` override), start it, capture the reference at the top of
  the tracking loop, read each iteration, and `close()` it in the existing
  `finally`. The human script stays perception-only (the reader has no robot
  dependency).
- `scripts/sensor_test.py` — refactored to use the shared `ForceTorqueSensor`
  (single source of truth for the RDT protocol) + read ip/cpf/cpt from config.
- `configs/default.yaml` — new `force_sensor:` section (ip, port, cpf/cpt
  placeholders, connect_timeout, baseline_frames, gravity_comp with
  `marker_to_sensor_rpy`).
- `docs/force_sensor_setup.md` (new) — networking fix for this machine
  (confirmed diagnosis: interface `enx00e04c6807dc` up but no IP, nmcli
  `disconnected`, `ping 192.168.1.1` 0 replies), the sudo `ip addr add` quick
  fix, the persistent `nmcli` profile, exactly what to hand an admin if sudo
  is unavailable, direct-connect vs USB-hub note, and how to read cpf/cpt from
  the web UI.
- `tests/test_force_torque.py` (new, 13 tests) — GravityCompensator math
  (cancels pure gravity at ref pose; orientation-aware force re-projection
  cancels at a NEW orientation; `M` threads through; torque always tared;
  plain-tare fallbacks; None cases) + ForceTorqueSensor over a localhost UDP
  loopback with a fake-sensor thread (parse/scale, disabled-returns-None).
  `tests/test_recorder.py` — new wrench-field + schema-v2 tests.

### Gravity-comp math (the crux)

`F_grav(t) = M · (R_cur.T @ R_ref) · M.T · F0`, subtract from raw force; tare
the torque. `F0`/`T0`/`R_ref` captured at startup at-rest; `R_cur` = torso
(target) marker rotation now; `M` = marker→sensor mounting (config, default I).
No payload weighing needed — the startup reading supplies the weight vector.
Relative rotation `R_cur.T @ R_ref` is frame-invariant (same reasoning as the
angular-velocity fields), so optical vs. pelvis torso rotation is equivalent.

### Deviations / limitations (documented, not silently assumed)

- `M` defaults to identity — accurate only if the torso marker is mounted
  axis-aligned with the sensor; otherwise one config rotation.
- Torque gravity-cancellation is tare-only (exact at the reference pose); full
  orientation-aware torque needs the payload COM offset `r` (future measurement).
- `ForceTorqueSensor.read_raw()` returns the reading even on a nonzero ATI
  status word (with a one-time warning) rather than nulling it — a status bit
  is a warning, not a reason to discard data (minor deviation from the plan's
  "None on status error").

### Networking / not completed by Claude Code

Making the sensor reachable on this machine needs a NIC IP assignment that
requires sudo/admin (this account may lack it). The code + `docs/
force_sensor_setup.md` are ready; the actual `ip addr`/`nmcli` change is the
user's step. Verified read-only: interface `enx00e04c6807dc` has no IP and the
sensor doesn't ping — matching the documented failure. Until fixed, both
scripts run fine with `load_cell_wrench_*` = `None`.

### Things not modified

- The arm `gravity_comp` (Pinocchio joint-torque feedforward) — unrelated to
  F/T gravity cancellation, untouched.
- `angle_diff_rpy` / angular-velocity conventions — unchanged.
- No new dependency added (socket/struct/threading are stdlib; numpy/scipy
  already core).

Test count: 15 new tests (13 in test_force_torque.py + 2 net in
test_recorder.py). Full count confirmed via `pytest tests/ -v` at 207.

## Debug tooling follow-ups (marker_tracking_debug.py + gravity_comp_debug.py)

### What was built

- `scripts/marker_tracking_debug.py` (new) — no-pickle intermediate between
  `visualize_markers.py` (visual only) and `record_human_trial.py` (data
  pipeline only, headless): annotated `cv2` window labeling detections
  `OBJECT`/`TARGET` per config, plus a live terminal printout of
  `angle_diff_rpy` and per-marker angular velocity — the same derived
  quantities `RunRecorder` computes, without saving a run.
- `scripts/gravity_comp_debug.py` (new) — isolates the F/T `GravityCompensator`
  from `record_human_trial.py`'s full loop: tracks only the torso
  (`target_marker_id`) marker, prints its position/orientation delta from its
  first reading, and prints the F/T wrench.

### Gap found and fixed: taring was gated behind orientation-tracking

While building `gravity_comp_debug.py`, tracing through `ForceTorqueSource`
surfaced a real bug: `capture_reference()` was a no-op whenever
`force_sensor.gravity_comp.enabled=false`, so **no tare was ever captured** in
that mode — `load_cell_wrench_grav_comp` stayed `None` for the entire run, not
just un-reprojected. The fix, scoped to `g1_spine_stabilization/sensors/
force_torque.py` (additive, no schema change):

- `GravityCompensator.tare(raw_wrench)` (new) — plain orientation-blind
  subtraction (`raw - [F0, T0]`), always available once a reference is set.
  `compensate()` is untouched (still the orientation-aware path with its
  existing no-orientation fallback).
- `ForceTorqueSource.capture_reference()` — dropped the
  `gravity_comp_enabled` early-out; now always captures the baseline whenever
  a compensator exists. Internal flag renamed
  `_gravity_comp_enabled` → `_orientation_tracking_enabled` to reflect its
  narrower new meaning: it now gates ONLY whether `R_torso` is forwarded into
  `compensate()` for reprojection (passing `None` when disabled correctly
  triggers `compensate()`'s pre-existing plain-tare fallback branch — no
  changes needed there).
- `ForceTorqueSource.read_all(R_torso)` (new) — returns
  `(raw, tared, grav_comp)` as a 3-tuple side by side, used only by
  `gravity_comp_debug.py`. The existing `read(R_torso)` (2-tuple) is
  UNCHANGED and still used by `record_human_trial.py`/`sdk_pose_tracking.py`
  Phase 2 — this was a deliberate scope decision (confirmed with the user) to
  avoid a pickle schema bump; recorded trials still only carry
  `load_cell_wrench_raw`/`_grav_comp`, not a third tared field.
- `configs/default.yaml` — clarified in a comment that `force_sensor.
  gravity_comp.enabled` now controls only the orientation-aware reprojection;
  the underlying tare is unconditional.
- `tests/test_force_torque.py` — 6 new tests: `GravityCompensator.tare()`
  (None-before-reference, None-when-raw-None, plain subtraction regardless of
  orientation) + `ForceTorqueSource` (new direct test coverage — previously
  untested as a class): capture_reference works with gravity_comp disabled
  (the regression test for the bug), `read_all()` returns a plain tare when
  disabled, `read_all()` still does full orientation reprojection when
  enabled. 213 tests total.

### Out of scope (deliberate, confirmed with user)

- Adding the tared reading to the recorded pickle schema
  (`MarkerSample.load_cell_wrench_tared`, `SCHEMA_VERSION` 2→3) — scoped out;
  only the standalone debug script got the third reading.

### Things not modified

- `assets/g1_with_hands.urdf`, IK weights, EE frame defs, shoulder-roll
  bounds — untouched.
- `data_recording/recorder.py`, `RunRecorder`/`MarkerSample` schema,
  `record_human_trial.py`, `controllers/sdk_pose_tracking.py` — untouched
  (still call the existing 2-tuple `read()`).
- `docs/data_recording_conventions.md` — untouched (describes the recorded
  schema, which didn't change).

## gravity_comp_debug.py follow-up: magnitude display + experimental sin(theta) model

### What was done

Two rounds of small changes to `scripts/gravity_comp_debug.py`, both scoped to
the debug script only (no schema/pipeline changes):

1. **Terser printout.** `wrench_raw` is no longer printed (dropped as noise
   for by-hand comparison). `wrench_tared`/`wrench_grav_comp` now each print a
   single total force (N) and total torque (Nm) magnitude
   (`np.linalg.norm(...)`) instead of 3-axis `[Fx,Fy,Fz]`/`[Mx,My,Mz]`
   components. `ForceTorqueSource.read_all()` itself is unchanged — this is
   presentation-only.
2. **Experimental sin(theta) force-residual model.** While comparing
   `wrench_tared` against `wrench_grav_comp` by hand, the user noticed: at the
   reference pose, `wrench_tared`'s total force is ~0 by construction; at a
   90-degree torso rotation, it read **7.5N**. Since `sin(0)=0` and
   `sin(90°)=1`, this fits `residual(theta) = reference_force_n * sin(theta)`.
   Added as a new module-level function
   `sin_model_gravity_residual(theta_rad, reference_force_n)` in
   `g1_spine_stabilization/sensors/force_torque.py` (pure, no instance state
   — not a `GravityCompensator` method), and wired into
   `gravity_comp_debug.py` as a **third**, clearly-labeled `[EXPERIMENTAL]`
   comparison line: `sin_model_force = wrench_tared's F_total -
   reference_force_n * sin(theta_axis)`, where `theta_axis` is one configured
   Euler axis (default `pitch`) of the same `angle_diff_rpy`-based
   `orient_delta` already computed for display. New `configs/default.yaml`
   block `force_sensor.debug_sin_model` (`reference_force_n: 7.5`,
   `axis: pitch`) makes both the fitted coefficient and the rotation axis
   tunable, since the coefficient was fit from a single observed data point
   and may not generalize. 3 new hardware-free tests for the pure function
   (sin(0)=0, sin(pi/2) recovers the exact 7.5N fit point, sin(pi/4) confirms
   it's a real sine and not a stand-in constant). 216 tests total.

### Key clarifications from the user (easy to get backwards)

- It's **sin**, not cos — their first message said cos but the observed data
  (0N at reference, 7.5N at 90°) only fits sin.
- `Force` in the formula is the **tared** total, not raw — "because at
  baseline we will have it at 0 anyways." This is a *second-stage* correction
  of the orientation-dependent residual taring alone can't remove, not an
  independent from-scratch computation off the raw signal.
- `theta` is **one specific Euler axis** (configurable, default pitch) of the
  existing roll/pitch/yaw `orient_delta`, not a combined rotation-angle
  magnitude.
- **Scope confirmed as debug-script-only** — this is explicitly unvalidated
  (fit from one data point) and not adopted into `GravityCompensator`/the real
  recording pipeline. Needs testing across a range of angles, not just 0° and
  90°, before any wider adoption is even considered.

### Out of scope / open

- Whether the sin(theta) model actually holds up across more than the two
  tested points (0° and 90°) — untested pending a human-run session rotating
  the torso through a range of angles and comparing all three printed values.
- If it does hold up, promoting it into the real `GravityCompensator`/
  recording pipeline would be a separate, bigger decision (schema/production
  code change) — not undertaken here.

### Things not modified

- `sensors/force_torque.py`'s `GravityCompensator`/`ForceTorqueSource` classes
  — only a new standalone function was added alongside them.
- `record_human_trial.py`, `controllers/sdk_pose_tracking.py`,
  `data_recording/recorder.py` — untouched.
- `docs/data_recording_conventions.md` — untouched.

## gravity_comp_debug.py follow-up: visualization + wrong-axis fix + stale-tracking visibility

### What was done

A real human-run of `gravity_comp_debug.py` (axis=pitch, the previous
default) surfaced two problems and prompted a third feature request, all
addressed in `scripts/gravity_comp_debug.py`:

1. **Added a live video window.** The camera loop no longer goes through
   `MarkerTracker` at all — it now calls `RealSenseSource.read()` +
   `AprilTagDetector.detect()` directly each frame (same pattern as
   `marker_tracking_debug.py`'s `run_realsense()`), drawing axes + a `TORSO`
   label on the tracked marker via `cv2.imshow`. This was necessary anyway
   (see #3) since `MarkerObservation` doesn't carry pixel corners for
   drawing, only the composed pose.

2. **Root-caused and fixed the wrong-rotation-axis bug.** The user reported
   that after a real run, `wrench_tared` and `sin_model_force` were
   "essentially the same" — meaning the `sin(theta)` correction was
   contributing ~nothing despite a visibly large torso rotation. Root cause:
   `angle_diff_rpy`'s roll/pitch/yaw decomposition is expressed in the
   AprilTag's own PRINTED local axes (an arbitrary direction fixed by how the
   physical tag happens to be mounted — see `docs/data_recording_
   conventions.md` §5), not necessarily the real-world axis the torso is
   actually rotated about. `axis: pitch` was a guess, and it was wrong for
   this rig — pitch read ~0 while the real rotation was happening on a
   different axis. Fix: new `rotation_magnitude(R_reference, R_moving)` in
   `g1_spine_stabilization/data_recording/recorder.py` (alongside
   `angle_diff_rpy`/`angular_velocity_rotvec` — same kind of generic
   rotation-comparison utility) computes the torso's TOTAL rotation angle
   since reference via `Rotation.from_matrix(R_ref.T @ R_cur).magnitude()` —
   correct regardless of which axis the physical rotation aligns with.
   `force_sensor.debug_sin_model.axis` now accepts `roll|pitch|yaw|magnitude`
   and defaults to `magnitude`. `tests/test_recorder.py` gained a test that
   demonstrates the failure mode directly: a pure 90° roll rotation reads
   `pitch ≈ 0` via `angle_diff_rpy` while `rotation_magnitude` correctly
   reports the full 90°.

3. **Fixed invisible marker dropout.** Tracing through why the user's third
   question ("how might losing the marker mess up the calculation") mattered,
   found: the OLD code called `tracker.step()` (discarding its return) then
   separately called `MarkerTracker.get_latest(torso_id)` — but
   `get_latest()` reads an internal cache (`perception/tracker.py:29,58,61-
   62`) that's set once a marker is first seen and NEVER cleared. So
   `torso_obs` could never be `None` again after the first sighting, even if
   the marker left the frame entirely for the rest of the run — a mid-run
   dropout was completely invisible, silently reusing a frozen pose forever
   with no indication. Confirmed with the user: gravity comp should KEEP
   running through a brief dropout using the last-known pose (don't stop
   compensating), but it must be clearly visible when that's happening. Fixed
   as part of the #1 rewrite: freshness is now a plain per-frame check
   (`torso_id` present in *this frame's* raw detections or not), with a local
   `last_pose_pelvis`/`last_R_optical`/`last_fresh_elapsed` state updated only
   on actual detections. Every printed tick now shows `torso tracking: FRESH`
   or `torso tracking: STALE (last seen N.Ns ago) -- using cached pose`.

### Key finding worth flagging loudly

**`record_human_trial.py` has the IDENTICAL latent staleness bug** — its main
loop uses the exact same `tracker.step()` + `get_latest()` pattern for BOTH
`object_obs` and `target_obs`. Once either marker is first seen, the script's
"object=seen"/"target=seen" status line will show "seen" for the rest of the
run even if that marker later drops entirely out of frame, and every
downstream computation (`angle_diff_rpy`, angular velocity, F/T gravity comp)
would silently keep using a stale pose. Confirmed with the user this is
**deliberately NOT fixed this session** — scoped to `gravity_comp_debug.py`
only, since fixing the real recording pipeline (and deciding how stale data
should be represented in the pickle schema — a new field? drop the sample?)
is a separate, bigger decision affecting real recorded data, not a quick
port.

### Out of scope / open

- Porting the freshness fix to `record_human_trial.py` (and by extension
  `controllers/sdk_pose_tracking.py` Phase 2, which has the same pattern for
  the robot-side recording) — flagged above, not undertaken.
- Whether `sin_model_force` under `axis=magnitude` now tracks visibly better
  than `wrench_tared` across a range of angles — untested pending the next
  human-run session (this was exactly the point of adding the video window +
  the axis fix together).

### Things not modified

- `perception/tracker.py`'s `MarkerTracker`/`get_latest()` — untouched (the
  fix is local to `gravity_comp_debug.py`, which stopped using it rather than
  changing its caching behavior for every caller).
- `record_human_trial.py`, `controllers/sdk_pose_tracking.py` — untouched
  (same latent staleness bug, deliberately left for a future session).
- `sensors/force_torque.py` — untouched this round (the axis fix lives in
  `recorder.py`/the script, not the F/T module).

## gravity_comp_debug.py follow-up: theta_used diagnostic (open anomaly, not yet resolved)

### What was found

A real run with the new `axis: magnitude` default (see previous entry) showed
`sin_model_force` reading ≈-7 at "startup" (right after reference capture,
torso presumed at rest) and ≈0 after rotating ~90° — the OPPOSITE of
expectation (should be ≈0 at BOTH points; `wrench_tared` itself correctly
goes from 0 → ~7, matching the original 90°-rotation data point the model was
fit from). The user also directly confirmed via the new video overlay that
the true physical rotation axis is X/roll (the red drawn axis), answering
their own question about whether the earlier `pitch` default was really
measuring the wrong (Y) axis — confirmed yes.

Worked through why the -7 anomaly can't be a bug in the sin-model math itself:
`wrench_tared` depends ONLY on the real F/T sensor's raw reading
(`GravityCompensator.tare()` takes no rotation argument at all), so its
correct 0→7 behavior means the SENSOR's own account of "did the orientation
really change" is trustworthy. And `theta` (whichever axis) is mathematically
guaranteed to equal exactly 0 at the reference instant
(`rotation_magnitude(R_ref, R_ref) = 0` identically). So `sin_model_force`
being -7 right at "startup" means the CAMERA-derived `theta` must have been
wrong (non-zero) at the moment the user read that value — either real/
apparent motion occurred between the "Reference captured" print and that
reading, or `solvePnP`'s pose estimate for a small planar AprilTag viewed
close to head-on is known to be less stable for the tilt/out-of-plane
rotation components (a real, physically-motivated hypothesis, not confirmed).

### What was done

Added a new printed line to `scripts/gravity_comp_debug.py`'s per-tick output,
purely diagnostic (no math changed):
```
theta_used(<axis>)=+68.4 deg   reproj_err=1.8px
```
Shows exactly what `theta_rad` value fed `sin_model_gravity_residual` this
tick (whichever axis is configured), plus the torso detection's own
`reprojection_error` (a leading indicator of an unstable/noisy pose
estimate). New `last_reproj_err` state tracked alongside the existing
`last_pose_pelvis`/`last_R_optical`/`last_fresh_elapsed`.

### Out of scope / open

- The -7 anomaly itself is NOT resolved — this session only added the
  visibility needed to diagnose it on the next run. Two competing
  hypotheses (timing/motion between reference capture and the "startup"
  reading vs. `solvePnP` pose instability for a near-frontal marker) need to
  be distinguished by reading the new `theta_used`/`reproj_err` line at the
  moment the anomaly reproduces.
- If pose instability turns out to be the real cause, a follow-up fix might
  involve improving the viewing angle/marker placement, adding a
  reprojection-error-based rejection threshold in `AprilTagDetector`, or
  averaging/filtering the pose over multiple frames — none of these were
  attempted, since the cause isn't confirmed yet.

### Things not modified

- `sensors/force_torque.py`, `data_recording/recorder.py`,
  `AprilTagDetector`/`MarkerTracker` — untouched; this was a print-only
  addition in `gravity_comp_debug.py`.

## Handoff doc written: gravity_comp_recording_plan.md

The user wants everything from this debugging arc (tare/gravity-comp
decoupling, the experimental sin(theta) model, the wrong-axis root-cause +
fix, the marker-dropout/staleness find, the still-open -7 anomaly) promoted
from `gravity_comp_debug.py`-only into the real recording pipeline
(`record_human_trial.py`, `sdk_pose_tracking.py` Phase 2) — but explicitly in
a **separate future session**, not this one. Rather than re-deriving all of
that from this diary (now 2000+ lines across many sessions), wrote a single
consolidated, self-contained spec: **`docs/gravity_comp_recording_plan.md`**
— findings with file:line evidence, the unresolved anomaly called out
prominently as a decision point to resolve first, a concrete schema-v3 +
pipeline-wiring implementation spec, and a summary checklist. Referenced from
`CLAUDE.md`'s Open Design Questions and File Structure, mirroring how
`docs/ik_weight_analysis.md` staged the SDK-5.5 gravity-comp work.

No source code changed this session — documentation only, per the user's
explicit request to defer implementation to a new session.

### Things not modified

- All source code — this was a docs-only session.

## Gravity-comp recording promotion follow-ups

### What was done

Implemented `docs/gravity_comp_recording_plan.md`'s spec (now marked
IMPLEMENTED at the top of that doc): promoted `gravity_comp_debug.py`'s
tared wrench + experimental sin-model force into the real `--record`
pipeline, and added a `'r'`-to-start recording gate to
`record_human_trial.py`.

**Files changed:**
- `g1_spine_stabilization/data_recording/recorder.py` — `SCHEMA_VERSION`
  2→3; `MarkerSample` gained `load_cell_wrench_tared`,
  `sin_model_theta_rad`, `sin_model_force_corrected` (the latter two
  EXPERIMENTAL); new shared helper `sin_model_theta_rad(axis, R_reference,
  R_current)` plus `RPY_AXIS_INDEX`/`VALID_SIN_MODEL_AXES` constants, so
  both real recording call sites compute this identically instead of each
  keeping their own copy (`gravity_comp_debug.py` keeps its pre-existing
  copy, untouched).
- `configs/default.yaml` — new `data_recording.reference_settle_frames: 5`;
  `force_sensor.debug_sin_model`'s comment updated (no longer
  debug-script-only).
- `scripts/record_human_trial.py` — rewritten. When `--record` is passed,
  opens a `cv2` window: waits for `'r'` before `recorder.start()` (nothing
  saved if `'q'` is pressed first), then requires
  `reference_settle_frames` consecutive fresh TARGET detections before
  locking the pose/gravity reference (used for both
  `wrench_source.capture_reference()` and the sin-model theta). New
  script-local `_DirectMultiMarkerTracker` replaces `MarkerTracker` for
  `--source realsense` — direct per-frame `AprilTagDetector` reads
  tracking both `object_marker_id`/`target_marker_id`, with
  `is_fresh()`/`seconds_since_fresh()` surfaced live as a `FRESH`/`STALE`
  overlay (fixes `MarkerTracker.get_latest()`'s sticky-cache staleness bug
  as a side effect, for this script only). `--source mock` gets a matching
  interface via `_build_mock_display_frame()` (synthetic canvas) so the
  same window/keypress flow works for both sources. When `--record` is
  omitted, behavior is unchanged (no window, immediate loop).
  `wrench_source.read()` → `read_all()`.
- `g1_spine_stabilization/controllers/sdk_pose_tracking.py` — Phase 2
  switched `read()` → `read_all()`, computes the same sin-model
  theta/force via the shared helper, and passes the 3 new fields to
  `recorder.record()`. No UI change — reference capture is still a single
  instant (no settle), since there's no interactive gating point on the
  robot path to attach one to.
- `scripts/run_pose_tracking_preview.py` — `RunRecorder` construction
  gains `metadata["sin_model_config"]` (reads the same
  `force_sensor.debug_sin_model` config), for parity with the human
  script's metadata.
- `tests/test_recorder.py` — 7 new tests (schema v3, `load_cell_wrench_tared`,
  the two sin-model fields, `None` defaults, `sin_model_theta_rad()`
  matching `rotation_magnitude`/`angle_diff_rpy` per axis). 226 tests total.
- `docs/data_recording_conventions.md` — schema recap bumped to v3, new §6
  subsection for the tared/sin-model fields (flagged experimental).
- `docs/gravity_comp_recording_plan.md` — marked IMPLEMENTED; §4 (the
  anomaly) got a resolution note (see Key findings below) and §6's
  checklist items marked done/explained.
- `CLAUDE.md` — new Current Status bullet, Session Roadmap entry, File
  Structure comments, Open Design Questions moved to Resolved, test count
  219→226.

### Decisions made by the user (binding, don't relitigate)

- Object-vs-target `angle_diff_rpy` (already in schema) was judged
  sufficient for "the change between them" — no new position-delta field
  was added.
- The `'r'`-to-start UI is `record_human_trial.py` only;
  `sdk_pose_tracking.py`'s robot recording still starts automatically at
  the top of Phase 2.
- Schema-field parity (the 3 new fields, no UI) WAS extended to
  `sdk_pose_tracking.py`/`run_pose_tracking_preview.py` — user explicitly
  wants robot and human pickles fully comparable.
- The `docs/gravity_comp_recording_plan.md` §4 anomaly (sin-model force
  reading backwards at "startup") was **NOT independently re-diagnosed**.
  Per the user: it worked correctly before, attributed to the reference
  needing a few frames to settle — addressed via
  `reference_settle_frames` rather than re-running `gravity_comp_debug.py`
  to distinguish the two hypotheses that doc had flagged. If the anomaly
  recurs with the settle in place, revisit hypothesis (b) (`solvePnP`
  instability for near-head-on tilt rotations) specifically — see that
  doc's §4 resolution note.

### Key findings

- **Pre-existing, unrelated environment issue found while test-running
  this session's changes — flagging for whoever next attempts robot-side
  recording (`sdk_pose_tracking.py`/`run_pose_tracking_preview.py`):**
  `pytest tests/test_sdk_transport.py` (and everything that shares its
  `robot.connect()` fixture: `test_grasp_demo_safety.py`,
  `test_pose_tracking_contact_loss.py`) fails on **this machine** with
  ```
  RuntimeError: input stream error
  ```
  raised from `g1_inspire_sdk/_ik/ik.py`'s `G1_29_ArmIK.load_cache()` (a
  `pickle.load()` on `g1_inspire_sdk/_ik/g1_29_model_cache.pkl`). Confirmed
  via `git stash` that this reproduces identically on the fully unmodified
  codebase — **nothing in this session's changes caused or touches it**.
  Root cause, evidenced not guessed: that cache file is a Pinocchio
  `robot.model`/`reduced_robot.model` pair, and Pinocchio's pickling of
  those routes through Boost's C++ serialization
  (`boost::archive`), whose *binary* archives are not portable across
  compilers/platforms — "input stream error" is literally
  `boost::archive`'s own error string for a binary stream it can't parse,
  not a generic "file corrupted" message. `git log` on the vendored SDK
  clone (`~/g1_inspire_sdk`) shows this exact file was committed as part
  of `019e942 Initial commit: g1_inspire_sdk` — i.e. it shipped as a
  pre-built asset with the SDK, most plausibly built on a Linux machine
  (the SDK's live-hardware tier — CycloneDDS, `unitree_sdk2py` — targets
  Linux, and this is very likely the actual robot's own compute
  environment), not generated on this Windows dev machine. This is
  consistent with the failure mode.
  - **Does NOT affect `record_human_trial.py`** (human trials) — that
    script is perception-only and never imports/instantiates
    `G1InspireRobot`/`G1Arm`/`G1_29_ArmIK` at all. Confirmed by reading its
    full import list; there is no code path there that could hit this.
  - **Would affect the robot recording path** — `robot.connect()`
    (`G1InspireRobot`) calls `G1Arm.connect()`, which unconditionally
    constructs `G1_29_ArmIK(...)` (even though our own code never calls
    the SDK's IK solve — we use `G1HandIK` per the Hard Rules; the SDK
    still builds that object as part of its own connect() plumbing). So
    this would block `robot.connect()` for BOTH `run_pose_tracking_preview.py`
    (dry-run) and any live SDK-6 bringup on this machine, before Phase 1
    even starts — not something this session's recorder/schema work
    touches, but worth fixing before attempting robot-side `--record`
    runs.
  - **Likely fix (not attempted — user asked to defer, human record path
    works fine as-is right now):** delete
    `~/g1_inspire_sdk/g1_inspire_sdk/_ik/g1_29_model_cache.pkl` on this
    machine. `G1_29_ArmIK.__init__` only takes the `load_cache()` branch
    `if os.path.exists(self.cache_path)`; otherwise it builds the model
    fresh from the bundled URDF and calls `save_cache()` itself
    (`_ik/ik.py:55-57,124-126`) — a fresh build should succeed since our
    own separate `G1HandIK` already builds successfully from a similar
    URDF on this same machine's Pinocchio install. This is outside our
    repo (`~/g1_inspire_sdk`); per the External SDK policy we `git pull`
    that clone rather than edit its source, but deleting a regenerable
    cache artifact (not source code) is a different kind of action — still
    worth confirming with the user before doing it, since it's a one-way
    delete of a committed file in that clone.
  - Not confirmed by a bare (non-pytest) `robot.connect()` repro on this
    machine — only confirmed via `pytest`, both scoped to
    `test_sdk_transport.py` alone and via `git stash` on the clean
    codebase. Nothing pytest-specific about a plain file-open-and-unpickle
    call, so a bare script repro is expected to fail identically, but
    wasn't directly executed this session (user asked to stop
    investigating and just document).
- Confirmed (via `--source mock --record --no-force-sensor --duration 2`
  under a `timeout` wrapper) that the new wait-for-`'r'` state actually
  blocks as designed: the process printed setup messages, reached "Press
  'r' to start recording," then hung until killed — and `data/runs/`
  didn't even exist afterward, confirming `recorder.save()` correctly
  never ran because recording never started.

### Out of scope / open

- The vendored-SDK IK-cache issue above — deferred to whenever robot-side
  `--record` runs are next attempted. Human-trial recording is unaffected
  and works today.
- Full interactive walkthrough of `record_human_trial.py`'s new `'r'`/`'q'`
  keypress flow with a real camera and real key presses — not done this
  session (only argparse/import/mock-headless-path smoke tests were run;
  see Key findings). Recommend a human-run smoke test with `--source
  realsense` and two real AprilTag markers before trusting a real trial's
  output.
- `sdk_pose_tracking.py`'s reference capture has no multi-frame settle
  (unlike `record_human_trial.py`'s new one) — deliberate, since there's no
  interactive gating point on the robot path to attach one to. If the
  sin-model anomaly shows up on a robot-side recording too, this is the
  first place to look.

### Things not modified

- `~/g1_inspire_sdk` — SDK not modified (including the stale IK cache
  file — flagged above, not deleted).
- `assets/g1_with_hands.urdf`, EE frame defs, shoulder_roll bounds, IK cost
  weights — unchanged.
- `scripts/gravity_comp_debug.py` — untouched; kept its own historical
  copy of the axis/theta logic rather than being refactored onto the new
  shared helper.

## Plotting tool follow-ups

### What was done

User asked to plot the pickle files being produced by the newly-completed
recording pipeline (both real trials: `human_run_20260822_143538_...` and
`human_run_20260822_144944_...`, both `--source realsense`). New
`scripts/plot_recorded_run.py`: loads a `.pkl` (default: newest under
`data/runs/` by mtime), saves 6 PNGs per figure group (`positions.png`,
`angle_diff_rpy.png`, `angular_velocity.png`, `load_cell_wrench.png`,
`sin_model.png`, `reprojection_error.png`), plus `theta.png` only when at
least one sample has non-`None` theta (robot-only field — correctly
skipped, with a printed note, for both human trials). A shared `_series()`
helper extracts a None-safe `(N,)` array (NaN where a field was `None` that
sample) so every plot degrades gracefully instead of crashing on a
partially-`None` field. `--pickle PATH`, `--output-dir DIR`, `--show`.

**Dependency note:** `matplotlib` was NOT actually installed in the
`g1_inspire_sdk` env, despite a stale claim in this same followups file's
SDK-1 entry that it was present as a transitive dependency (apparently no
longer true, or was never quite accurate). Added `matplotlib` to
`requirements.txt` and installed it directly into the env
(`pip install matplotlib`, user confirmed first) — pulled in `contourpy`,
`cycler`, `fonttools`, `kiwisolver`, `pyparsing`, `python-dateutil`, `six`
as transitive deps.

### Findings from the actual plotted data

Both real trials showed the same pattern, which directly motivated the next
session's fix (see below): brief, isolated spikes in `angle_diff_rpy`,
`object_angular_velocity`/`target_angular_velocity` (up to 40-50 rad/s —
physically implausible for a human), `load_cell_wrench_grav_comp`, and
`sin_model_theta_rad`/`sin_model_force_corrected` — even during periods the
torso wasn't being moved. `reprojection_error.png` stayed under ~1.1px
throughout both trials, including during the spikes — so reprojection error
does NOT reliably flag these bad reads. `load_cell_wrench.png` showed clean
raw/tared/grav_comp separation otherwise (working as designed). `sin_model.png`
showed the sin-model force visibly diverging from tared force (even going
negative) at the exact same instants as the spikes — a concrete real-world
instance of the still-unresolved anomaly `docs/gravity_comp_recording_plan.md`
§4 flagged.

### Out of scope / open

- No fix was implemented this session — purely a visualization/diagnostic
  tool. The spike pattern it surfaced was fixed in the very next session
  (see "Marker-pose noise filtering follow-ups" below).
- `record_human_trial.py`/`sdk_pose_tracking.py`/`recorder.py` — untouched.

### Things not modified

- All recording-pipeline code — this was a read-only analysis tool added
  on top of already-recorded pickles.

## Marker-pose noise filtering follow-ups

### What was done

Fixed the spike pattern found by the plotting tool above. Root cause
(confirmed via code read): every consumer of the current marker
rotation — `RunRecorder.record()`'s pose/angle_diff/angular-velocity
fields, `GravityCompensator.compensate()`'s `R_torso_current`, and
`sin_model_theta_rad()` — read straight from the tracker's per-frame
output with zero filtering, so a single bad `solvePnP` frame propagated
into all of them simultaneously.

**Files changed:**
- New `g1_spine_stabilization/perception/pose_filter.py` — `PoseNoiseFilter`:
  median-of-N then EMA on the (3,) translation; median-of-N then EMA on a
  quaternion for rotation, with hemisphere-continuity correction (flip sign
  against the previous FILTERED quaternion before appending to history, so
  `q`/`-q` — the same physical rotation — aren't treated as a spurious
  jump), re-normalized before converting back to a matrix. Same
  median-window/EMA recipe as `trajectory/sources.py`'s `NoiseFilteredSource`,
  generalized from `(box_center, theta)` to a full SE3 pose.
- `configs/default.yaml` — new `perception.pose_filter` block (`enabled`,
  `median_window: 5`, `ema_alpha: 0.3` — same defaults as
  `trajectory.noise_filter`, same problem class).
- `scripts/record_human_trial.py` — `_DirectMultiMarkerTracker` gains a
  `pose_filter_cfg` param; filters `det.pose_in_optical` (one
  `PoseNoiseFilter` instance per marker id) before deriving `pose_pelvis`
  via `compose_to_pelvis` and constructing the `MarkerObservation`.
  `reprojection_error`/`timestamp` stay tied to the raw per-frame
  detection. `--source mock` unaffected (synthetic, no filter applied).
  **No changes needed** to the main recording loop, wrench/sin-model
  computation, or the wait-for-`'r'`/reference-settle logic — they already
  consume `tracker.get_latest(...)`, which transparently returns filtered
  poses now.
- `g1_spine_stabilization/controllers/sdk_pose_tracking.py` — new
  `_filtered_marker_obs()` helper (constructs a new `MarkerObservation`
  with filtered `pose_in_optical`/`pose_in_pelvis`, or returns the input
  unchanged if there's nothing to filter); two `PoseNoiseFilter` instances
  created in Phase 2 (`_obj_pose_filter`/`_tgt_pose_filter`); `obj_obs`/
  `target_obs` filtered immediately after being read from `source`, before
  `R_torso`/`_sin_model_R_ref`/`recorder.record()` use them.
  `optical_to_d435_from_config`/`d435_to_pelvis_from_pinocchio(ik=ik)`
  computed once near the top of `run_pose_tracking_sdk` (reuses the
  already-loaded `ik`, no Pinocchio re-instantiation cost).
- `docs/data_recording_conventions.md` — new §7 explaining the filter, the
  frame-invariance rationale for filtering only in optical frame, and that
  `object_pose`/`target_pose` (and derived fields) are no longer strictly
  raw per-frame detections; updated §1's field descriptions accordingly.
- `tests/test_pose_filter.py` — 7 new hardware-free tests (median rejects
  a position/rotation outlier, quaternion hemisphere continuity across a
  sign flip, EMA reduces jitter variance, output is always a valid
  rotation matrix, `median_window=1`/`ema_alpha=1.0` is identity
  passthrough, constructor validation). 233 tests total.
- `CLAUDE.md` — Current Status, Session Roadmap, File Structure, test
  count updated; also backfilled a missing `CLAUDE.md` entry for the
  previous (plotting tool) session, which had been skipped.

**Key design point (verified against actual code, not assumed):**
`GravityCompensator.compensate()`'s own docstring states the relative
rotation `R_cur.T @ R_ref` is invariant to which fixed frame it's expressed
in (optical vs. pelvis give the same answer) — but that invariance only
holds because `compose_to_pelvis` is a single fixed linear
left-multiplication. Median-of-N is a nonlinear, order-statistics
operation and is NOT equivariant under an arbitrary rotation (only EMA
is) — so filtering translation/rotation independently in both optical AND
pelvis frame would silently give two different, inconsistent answers.
Filtering `pose_in_optical` exactly once, then re-deriving `pose_in_pelvis`
from the filtered value via the existing `compose_to_pelvis`, is what lets
one change clean up every consumer (recorded pose fields, F/T gravity-comp,
sin-model theta) consistently — no changes needed to `recorder.py` at all.

**Confirmed NOT double-filtering the robot's live arm-control path:**
`MarkerSource.step()`'s `last_obj_obs`/`last_tgt_obs` are set directly from
the tracker's raw detections, NOT from the `(box_center, theta)` value that
`NoiseFilteredSource`/`RateLimitedSource` wrap — so `obj_obs`/`target_obs`
were already an independent, unfiltered signal path before this fix, and
this fix only touches that path (used for recording), never `box_center`/
`theta` (used for the actual IK/arm-motion target).

### User's confirmed decisions (binding)

1. Apply to both `record_human_trial.py` and `sdk_pose_tracking.py` (parity).
2. Smooth the recorded pose/`angle_diff_rpy`/angular-velocity fields too,
   not just the F/T/sin-model inputs — meaning `object_pose`/`target_pose`
   are no longer strictly raw per-frame detections once this ships (no
   dual raw+filtered fields were requested).
3. Median-of-N then EMA (not a rate-of-change-rejection/hold approach).

### Out of scope / open

- Could not fully exercise the robot-path wiring end-to-end
  (`run_pose_tracking_preview.py --source mock --record`) — it hit the
  SAME pre-existing vendored-SDK IK-cache issue documented in the
  "Gravity-comp recording promotion follow-ups" entry above
  (`RuntimeError: input stream error` in `G1_29_ArmIK.load_cache()`,
  before Phase 2 / this fix's code ever runs). Confirmed the code up to
  that point (rate limiter, recorder construction, ForceTorque connect)
  runs cleanly; `sdk_pose_tracking.py` imports without error and
  `run_pose_tracking_sdk`'s `_filtered_marker_obs` helper is present.
  Full verification of this path is blocked on the same cache fix noted
  previously — still deferred, human's call on timing.
- Did verify `record_human_trial.py --source realsense --dry-run --record`
  reaches the wait-for-`'r'` state cleanly with the new
  `_DirectMultiMarkerTracker(pose_filter_cfg=...)` construction (synthetic
  frames, so the actual `.filter()` call inside `step()`'s marker-found
  branch wasn't exercised live — covered directly by the `test_pose_filter.py`
  unit tests instead).
- Retuning `median_window`/`ema_alpha` against the two real trials' actual
  noise characteristics was not attempted — kept the same starting
  defaults as `trajectory.noise_filter`. A live re-recording (per the
  Verification section in the approved plan) is needed to confirm the fix
  actually eliminates the spikes in practice.

### Things not modified

- `data_recording/recorder.py`, `sensors/force_torque.py` — untouched (the
  fix works entirely by cleaning the input before either ever sees it).
- `trajectory/sources.py` (`NoiseFilteredSource`/`RateLimitedSource`) — untouched;
  the robot's live arm-control filtering stack is unrelated to and
  unaffected by this fix.
- `~/g1_inspire_sdk` — SDK not modified (including the still-stale IK
  cache file).
- `assets/g1_with_hands.urdf`, EE frame defs, shoulder_roll bounds, IK cost
  weights — unchanged.

## Gravity-comp method swap follow-ups

### What was investigated

User noticed, plotting a real trial (torso rolled and brought back), that
`load_cell_wrench_grav_comp` read a *higher* force magnitude than
`load_cell_wrench_tared` during the roll — backwards from expectation,
since the orientation-aware correction should track closer to zero than
the orientation-blind tare, not further from it. Investigated
quantitatively against the actual recorded pickle (not just re-reading
code):

- Confirmed the anomaly persists in "clean" (non-glitch, plausible
  angular-velocity) samples, not just noise-contaminated ones:
  `corr(theta, |grav_comp|-|tared|) ≈ 0.87` across a whole clean trial.
- Re-derived `GravityCompensator.compensate()`'s formula from rigid-body
  kinematics first principles, twice independently — both landed exactly
  on the code's existing formula (`M @ (R_cur.T @ R_ref) @ M.T @ F0`),
  suggesting the rotation order is likely correct under this codebase's
  established SE3 conventions.
- Directly tested all 4 possible rotation-order/transpose combinations
  against real data, with `M` left at identity: RMS force error ranged
  6.09N-10.09N depending on order, vs. 6.79N for plain `tared` and **7.92N
  for the current code's exact combination — worse than doing nothing**.
  Two of the four alternate orders beat plain `tared`; two (including the
  current one) didn't.
- Least-squares-fit the best possible `marker_to_sensor_rpy` (`M`) against
  the same data for each of the 4 order variants: **every variant reaches
  statistically indistinguishable best-case residual (~5.6-5.7N) at some
  non-identity `M`**, and multiple random restarts per variant converge to
  wildly different rotation vectors (93°-283° apart) with the same
  residual. Conclusion: this trial's largely single-axis (roll) motion
  cannot separate "the rotation order in the code is wrong" from "the
  mounting offset (`M`) is uncalibrated" — both remain genuinely
  confounded in this dataset; a multi-axis calibration trial would be
  needed to actually distinguish them.
- `sin_model_force_corrected` (the experimental magnitude-only curve fit)
  visually tracked closer to zero on the same trial. Explained why this
  isn't strong evidence the underlying 3D physics is right: it's a
  single-parameter (`reference_force_n=7.5N`, fit from one 90°-rotation
  data point) curve fit directly against `tared`'s own magnitude-vs-theta
  trend, so subtracting it from that same trend is close to guaranteed to
  look flat, regardless of whether the model generalizes to a different
  axis/mounting/range. Also pointed out (from the user's own pasted graph)
  that `sin_model_force_corrected` swings to a large negative excursion
  (~-3N) during the exact same brief camera-glitch spike visible in
  `theta` — a real compensator shouldn't react that violently to one bad
  frame, which is itself evidence against "sin-model is working correctly
  as intended," even though it happens to track the smooth part of the
  trial's trend well.

### User's decision (explicit, final, after this investigation)

Use the sin-model correction as the primary method behind the recorded
`load_cell_wrench_grav_comp` field, overriding the technical reservations
raised above (magnitude-only, fit-to-symptom-not-cause, fragile to noise,
loses torque/direction information). Implemented as directed.

### What was done

- `g1_spine_stabilization/data_recording/recorder.py` — new
  `sin_model_wrench(tared_wrench, sin_model_force_corrected)`: builds a
  `(6,)` wrench from `tared`'s own force *direction* rescaled to
  `sin_model_force_corrected`'s magnitude (sign-preserving — a negative
  correction flips direction rather than clamping), with torque copied
  from `tared` unchanged (the sin-model has no torque information).
- `scripts/record_human_trial.py` and
  `controllers/sdk_pose_tracking.py` — both now build
  `load_cell_wrench_grav_comp` via `sin_model_wrench()` instead of
  forwarding `wrench_source.read_all()`'s 3rd return value (the rotation-
  matrix result). `read_all()` itself, `GravityCompensator`, and
  `scripts/gravity_comp_debug.py` are **unchanged** — the rotation-matrix
  method is still computed (discarded at the two recording call sites,
  still used by the debug tool and available for future recalibration).
- `metadata["sin_model_config"]["status"]` changed from
  `"experimental_unvalidated"` to `"primary_grav_comp_method"` in both
  `record_human_trial.py` and `run_pose_tracking_preview.py`.
- Docs updated for the new field semantics and to flag that
  `norm(load_cell_wrench_grav_comp[:3])` now reports
  `abs(sin_model_force_corrected)`, losing sign-flip information: `configs/
  default.yaml` (`gravity_comp`/`debug_sin_model` comments),
  `sensors/force_torque.py`'s `sin_model_gravity_residual()` docstring,
  `docs/data_recording_conventions.md` §6, `docs/gravity_comp_recording_
  plan.md` (new policy-change banner), `CLAUDE.md`.
- `tests/test_recorder.py` — 5 new tests for `sin_model_wrench()`
  (magnitude matches, direction preserved/flipped by sign, torque
  passthrough, no divide-by-zero at near-zero tared force). 238 tests
  total.
- Verified against real data without a new hardware trial: the two
  existing post-pose-filter pickles (`human_run_20260822_152723_...`,
  `human_run_20260822_153047_...`) already contain `load_cell_wrench_
  tared`/`sin_model_force_corrected` (recorded before this code change),
  so reconstructing `sin_model_wrench(sample.load_cell_wrench_tared,
  sample.sin_model_force_corrected)` per-sample and comparing against the
  old stored `load_cell_wrench_grav_comp` confirmed the new method comes
  out smaller (closer to zero) than both `tared` and the old rotation-
  matrix method on both trials.

### Out of scope / open

- The underlying calibration question (order vs. `M` vs. both) from the
  investigation is **not resolved** — this session changed which value is
  *recorded*, not the rotation-matrix method's own correctness. A proper
  fix would still need a multi-axis calibration trial if that method is
  ever revisited.
- Existing pickles recorded before this change (`143538`, `144944`,
  `152723`, `153047`) still have the OLD rotation-matrix `load_cell_
  wrench_grav_comp` baked in — not retroactively rewritten (their
  `load_cell_wrench_tared`/`sin_model_force_corrected` fields are
  unaffected and can be used to reconstruct what the new value would have
  been, as done for verification above, if ever needed).
- No new hardware trial was recorded this session to see the new field
  live end-to-end (`plot_recorded_run.py` against a freshly-recorded
  pickle) — verification was via reconstruction from already-recorded
  fields on two existing trials instead.

### Things not modified

- `sensors/force_torque.py`'s `GravityCompensator` class itself (`compensate()`/
  `tare()`/`set_reference()`) — untouched; still fully functional, just no
  longer wired into the recorded schema field.
- `scripts/gravity_comp_debug.py` — untouched; still shows all three
  (tared/rotation-matrix grav_comp/sin_model_force) side by side live.
- `~/g1_inspire_sdk`, `assets/g1_with_hands.urdf`, EE frame defs,
  shoulder_roll bounds, IK cost weights — unchanged.

