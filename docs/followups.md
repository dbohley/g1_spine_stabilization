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
