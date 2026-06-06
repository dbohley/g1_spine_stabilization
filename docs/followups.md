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
