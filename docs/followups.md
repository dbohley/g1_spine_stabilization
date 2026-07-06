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

