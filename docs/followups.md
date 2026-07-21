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

