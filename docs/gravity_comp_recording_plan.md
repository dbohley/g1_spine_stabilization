# Gravity-Comp Recording Plan

> **STATUS: IMPLEMENTED.** §5's spec has been carried out in
> `scripts/record_human_trial.py`, `controllers/sdk_pose_tracking.py`, and
> `scripts/run_pose_tracking_preview.py` (schema v2→v3: `load_cell_wrench_
> tared`, `sin_model_theta_rad`, `sin_model_force_corrected`). **§4's
> anomaly was NOT independently re-diagnosed** — see the resolution note
> appended to the end of §4 below before relying on this doc's "hypothesis
> (a)/(b), not yet distinguished" framing as still open.
>
> **POLICY CHANGE (later session): the sin-model is now the PRIMARY method
> behind `load_cell_wrench_grav_comp`, not an experimental side-note.** A
> real-trial investigation (rolling the torso and back) found the
> rotation-matrix `GravityCompensator.compensate()` method this doc
> describes below measured *worse* than no correction at all (7.92N RMS
> force error vs. 6.79N RMS for plain `tared`, on clean/non-glitch
> samples) — most likely due to an uncalibrated `force_sensor.gravity_comp.
> marker_to_sensor_rpy` (still the identity placeholder) and/or a rotation-
> order question that a single-axis trial couldn't cleanly separate from
> that calibration issue (multiple order/transpose variants, and multiple
> very different fitted `marker_to_sensor_rpy` values, all achieved
> statistically indistinguishable best-case residuals — this trial's motion
> structurally cannot determine which). By contrast, the sin-model method
> tracked visibly closer to zero. Per explicit project decision, `data_
> recording/recorder.py`'s new `sin_model_wrench()` (tared's force
> direction, rescaled to the sin-model-corrected magnitude; torque
> unchanged from tared) now populates the recorded `load_cell_wrench_
> grav_comp` field in both real recording scripts. The rotation-matrix
> method itself is untouched and still computed (via `read_all()`'s 3rd
> return value) but no longer recorded — it remains available for `scripts/
> gravity_comp_debug.py`'s live comparison and any future recalibration
> work. This does not resolve the open calibration question the rest of
> this doc describes — it changes which value is *recorded*, not the
> calibration state of the rotation-matrix method itself. See
> `docs/data_recording_conventions.md` §6 for the current field semantics.

**Purpose of this doc:** `scripts/gravity_comp_debug.py` was built as a
hands-on diagnostic to validate F/T sensor gravity compensation before
trusting real recorded trial data. That debugging arc produced a working
tare, an experimental sin(theta) alternative model, a real bug fix (wrong
rotation axis), a real bug find (invisible marker dropout), and one
**still-unresolved anomaly**. This doc consolidates all of it into a single,
actionable spec so a **new Claude session** (no memory of the debugging
conversation) can promote these findings from "debug script only" into the
real `--record` pipeline (`scripts/record_human_trial.py` and the robot's
recording path). Hand this doc directly to that session — every claim below
is grounded in a specific file:line, and the implementation spec (§5) is
concrete enough to execute without re-deriving the design. Read `docs/
data_recording_conventions.md` first for the schema/convention background
this doc builds on.

**Read §4 before doing anything else.** There is an open, unresolved
correctness anomaly in the experimental model this doc is about to promote
to production. Do not skip it.

---

## 1. Current state

**Schema today (`g1_spine_stabilization/data_recording/recorder.py`,
`SCHEMA_VERSION = 2`, line 231):** `MarkerSample` (line 150) has
`load_cell_wrench_raw`/`load_cell_wrench_grav_comp` (lines 189-190) — that's
it. `RunRecorder.record()` (line 268) takes matching `load_cell_wrench_raw`/
`load_cell_wrench_grav_comp` kwargs (lines 276-277) and stores them verbatim
(lines 346-350).

**Real recording call sites still use the OLD 2-tuple `read()`, not the
newer `read_all()`:**
- `scripts/record_human_trial.py:220-223` — `capture_reference(R_torso)`
  once, then `wrench_raw, wrench_comp = wrench_source.read(R_torso)` every
  iteration.
- `g1_spine_stabilization/controllers/sdk_pose_tracking.py:471-474` — same
  pattern, robot-side Phase 2.

**What already exists in `sensors/force_torque.py`, tested, but NOT wired
into either of the above:**
- `GravityCompensator.tare(raw_wrench)` (line 262) — plain, orientation-blind
  `raw - [F0, T0]` subtraction. Always available once a reference is
  captured, regardless of `gravity_comp.enabled`.
- `ForceTorqueSource.capture_reference()` (line 329) — already fixed to
  always capture a baseline (used to no-op when `gravity_comp.enabled=false`;
  that flag now only gates the orientation-aware reprojection).
- `ForceTorqueSource.read_all(R_torso)` (line 366) — returns
  `(raw, tared, grav_comp)` as a 3-tuple. **Currently used only by
  `scripts/gravity_comp_debug.py`.**
- `sin_model_gravity_residual(theta_rad, reference_force_n)` (line 80) —
  experimental alternative: `residual = reference_force_n * sin(theta_rad)`.
  Not part of `GravityCompensator`. Not used by any production recording
  path. Explicitly marked experimental in its own docstring.

**`configs/default.yaml`'s `force_sensor.debug_sin_model` block** (line 188):
```yaml
debug_sin_model:
  reference_force_n: 7.5   # N -- tared force magnitude observed at a 90-degree torso rotation
  axis: magnitude           # roll | pitch | yaw | magnitude
```
The `debug_` prefix reflects its debug-only origin (see §5's naming
decision).

**`rotation_magnitude(R_reference, R_moving)`** (`data_recording/
recorder.py`, alongside `angle_diff_rpy`/`angular_velocity_rotvec`) — total
rotation angle since reference, axis-agnostic
(`Rotation.from_matrix(R_reference.T @ R_moving).magnitude()`). This is
`debug_sin_model.axis`'s current default (`"magnitude"`).

---

## 2. How `scripts/gravity_comp_debug.py` currently uses all of this

For reference when porting the pattern (this file is NOT part of the
package, so it can't be imported — reimplement the same calls):

```python
wrench_source.capture_reference(last_R_optical)   # once, on first detection
...
_, wrench_tared, wrench_comp = wrench_source.read_all(last_R_optical)
tared_f_total = float(np.linalg.norm(wrench_tared[:3]))   # force-only magnitude
if sin_model_axis == "magnitude":
    theta_rad = rotation_magnitude(R_ref_pelvis, R_cur_pelvis)
else:
    theta_rad = orient_delta_rad[_RPY_AXIS_INDEX[sin_model_axis]]   # roll/pitch/yaw component
residual = sin_model_gravity_residual(theta_rad, sin_model_ref_force_n)
sin_model_force = tared_f_total - residual
```

Note the sin model currently only produces a **scalar force-magnitude
correction** — it does not touch torque, and it does not produce a full
6-vector like `wrench_raw`/`wrench_tared`/`wrench_grav_comp` do. Keep this in
mind for the schema design in §5 — it's not a drop-in replacement for a
`(6,)` wrench field.

---

## 3. Findings from the debugging arc

### 3.1 Tare was gated behind orientation-tracking (fixed, already in `sensors/force_torque.py`)

`ForceTorqueSource.capture_reference()` used to be a no-op whenever
`force_sensor.gravity_comp.enabled=false`, meaning no tare was ever captured
in that mode. Fixed: capture always runs; the config flag now only decides
whether `read()`/`read_all()` additionally attempt orientation-aware
reprojection (`GravityCompensator.compensate()`) on top of the tare.

### 3.2 Wrong rotation axis, root-caused and fixed

`angle_diff_rpy` (`data_recording/recorder.py`) decomposes rotation into
roll/pitch/yaw expressed in the **reference marker's own printed local
axes** — an arbitrary direction fixed by how the physical AprilTag happens to
be mounted, not necessarily the real-world axis being rotated about. The
sin-model's `axis` config defaulted to `pitch` (Euler index 1) as a guess.
Live testing showed `sin_model_force` barely differing from `wrench_tared`
even under a large, visible torso rotation — `pitch` was reading ~0 while the
real physical rotation was happening on a different axis. The user
confirmed via `gravity_comp_debug.py`'s video overlay (drawn axes on the
real marker) that the true rotation axis is `roll` (X, index 0), not `pitch`.

**Fix:** `rotation_magnitude(R_reference, R_moving)` — the torso's *total*
rotation angle since reference, correct regardless of which axis the
physical rotation aligns with. Now `debug_sin_model.axis`'s default
(`"magnitude"`), with `roll`/`pitch`/`yaw` still available for inspecting one
specific axis.

`tests/test_recorder.py` has a test demonstrating the exact failure mode: a
pure 90° roll rotation reads `pitch ≈ 0` via `angle_diff_rpy` while
`rotation_magnitude` correctly reports 90°.

### 3.3 Marker dropout was invisible (fixed in the debug script ONLY)

`gravity_comp_debug.py` originally called `MarkerTracker.step()` (discarding
the return value) then separately called `MarkerTracker.get_latest(torso_id)`
— but `get_latest()` (`perception/tracker.py:61-62`) reads from `self.
_latest`, a dict that's set once a marker is first seen (line 58) and
**never cleared**. So a `MarkerObservation` could never be `None` again after
the first sighting, even if the marker left the frame entirely for the rest
of the run — a mid-run dropout was completely invisible, silently reusing a
frozen pose forever with no indication.

**Fix, applied only in `gravity_comp_debug.py`:** stopped using
`MarkerTracker` for the live loop; reads `RealSenseSource`/`AprilTagDetector`
directly per frame instead (raw per-frame detections, no sticky cache).
Confirmed with the user: gravity comp should **keep running** through a
brief dropout using the last-known pose (compensation shouldn't stop over a
transient occlusion), but it must be **clearly visible** when that's
happening — every printed tick now shows `torso tracking: FRESH` or
`torso tracking: STALE (last seen N.Ns ago) -- using cached pose`.

**`scripts/record_human_trial.py` (lines 213-215) and `controllers/
sdk_pose_tracking.py` (Phase 2, same pattern) have the IDENTICAL latent
bug** — both markers, same `tracker.step()` + `get_latest()` call shape.
This was deliberately **not** fixed in those two files (scope decision made
mid-debugging-arc). See §5's companion-fix recommendation.

### 3.4 Open, UNRESOLVED anomaly — see §4

---

## 4. Decision point: an unresolved correctness anomaly (read this before implementing §5)

A real test run with the current `axis: magnitude` default produced:
- **"Startup"** (right after "Reference captured" prints, torso presumed at
  rest): `sin_model_force` ≈ **-7**.
- **After rotating ~90°:** `sin_model_force` ≈ **0**.

This is backwards from expectation — it should read ≈0 at **both** points.
`wrench_tared` itself behaves correctly (0 at rest, ~7 after rotating,
matching the single data point `reference_force_n=7.5` was fit from).

**Why this can't be a bug in the sin-model math itself:**
`GravityCompensator.tare()` takes no rotation argument at all — `wrench_
tared` depends *only* on the real F/T sensor's raw reading, never on the
camera-derived rotation. Its correct 0→~7 behavior means the sensor's own
account of "did the orientation really change" is trustworthy. And `theta`
(any axis, including `magnitude`) is mathematically guaranteed to equal
exactly 0 at the reference instant (`rotation_magnitude(R_ref, R_ref) = 0`
identically — this is a hard mathematical identity, not something that can
be off due to calibration). So `sin_model_force` reading -7 right at
"startup" means the **camera-derived `theta` was wrong** at the moment
"startup" was observed.

**Two competing hypotheses, not yet distinguished:**
(a) Real or apparent motion occurred between the "Reference captured" print
    and the instant the "startup" reading was taken (a timing/workflow
    issue, not a code bug).
(b) `solvePnP`'s pose estimate for a small, planar AprilTag viewed close to
    head-on is known to be less stable for the tilt/out-of-plane rotation
    component than for more oblique viewing angles — a transient noisy
    detection could produce a large apparent `theta` even though nothing
    physically changed.

**A diagnostic was added specifically to resolve this** — `gravity_comp_
debug.py` now prints, every tick:
```
theta_used(<axis>)=+68.4 deg   reproj_err=1.8px
```
showing exactly what `theta_rad` fed `sin_model_gravity_residual`, plus the
torso detection's `reprojection_error` (a leading indicator of an unstable
pose estimate). **This has not yet been re-run/read to confirm which
hypothesis is correct.**

**Recommendation:** re-run `gravity_comp_debug.py` and read the `theta_used`/
`reproj_err` line at the moment the anomaly reproduces, BEFORE (or as the
very first step of) implementing §5. Committing a demonstrably-inconsistent
experimental model into real recorded trial data, under a field name that
looks as authoritative as `load_cell_wrench_raw`/`_grav_comp`, risks
polluting a real dataset with a value known to be wrong at least some of the
time. If §5 proceeds anyway without resolving this, the recorded sin-model
field(s) MUST be clearly flagged experimental/unvalidated in both the
pickle's `metadata` and in `docs/data_recording_conventions.md` — not
presented as equivalent in trustworthiness to the raw/tared/grav_comp
wrenches.

**Resolution actually applied (read this, not just the recommendation
above):** this anomaly was **not independently re-diagnosed** — hypotheses
(a) and (b) above were never distinguished by re-running `gravity_comp_
debug.py` and reading `theta_used`/`reproj_err` as recommended. Instead,
per explicit project direction, it's addressed by requiring the camera/
marker reading to settle over a few consecutive fresh frames before locking
the reference at all (`data_recording.reference_settle_frames`, default 5,
in `configs/default.yaml` — gated behind `scripts/record_human_trial.py`'s
new `'r'`-to-start video window; there was no interactive gating point to
attach an equivalent settle to in `sdk_pose_tracking.py`, so the robot path's
reference capture is still a single instant, unchanged in this respect).
This is a plausible fix for hypothesis (a) (a transient reading right at
the reference instant) but does **not** rule out hypothesis (b) (systematic
`solvePnP` instability for near-head-on tilt rotations) — if the anomaly
recurs even with the settle in place, revisit hypothesis (b) specifically.
The sin-model fields are recorded flagged `"status": "experimental_
unvalidated"` in the pickle's `metadata["sin_model_config"]`, per this
section's fallback recommendation, since the anomaly was not confirmed
resolved by direct evidence.

---

## 5. Implementation spec

### 5.1 Schema change (`g1_spine_stabilization/data_recording/recorder.py`)

- Bump `SCHEMA_VERSION` 2 → 3.
- Add to `MarkerSample`:
  - `load_cell_wrench_tared: Optional[np.ndarray] = None` — `(6,)`, from
    `ForceTorqueSource.read_all()`'s second element (`GravityCompensator.
    tare()`). Same `None`-when-unavailable convention as the existing two
    wrench fields.
  - `sin_model_theta_rad: Optional[float] = None` — the actual `theta_rad`
    used this sample. Store this **per sample** (not just in run metadata)
    because it varies every sample and is essential for reproducing/
    auditing the (currently unvalidated) sin-model output later.
  - `sin_model_force_corrected: Optional[float] = None` — the scalar
    corrected force magnitude (`tared_F_total - sin_model_gravity_residual(
    theta_rad, reference_force_n)`). Scalar, not a 6-vector — see §2.
- `RunRecorder.record()`: add matching optional kwargs
  (`load_cell_wrench_tared`, `sin_model_theta_rad`,
  `sin_model_force_corrected`), following the exact existing pattern for
  `load_cell_wrench_raw`/`_grav_comp` (copy-on-store, `None` passthrough).
- Run-level `metadata` (constant per run, like the existing `perception`
  snapshot): add `sin_model_config: {"reference_force_n": ..., "axis": ...,
  "status": "experimental_unvalidated"}` — a fixed-per-run snapshot of what
  produced `sin_model_force_corrected`, plus an explicit status flag per the
  §4 recommendation.

### 5.2 Pipeline wiring

In both `scripts/record_human_trial.py` and `controllers/sdk_pose_tracking.
py` (Phase 2):
- Replace `wrench_source.read(R_torso)` with `wrench_source.read_all(
  R_torso)`, unpacking `(wrench_raw, wrench_tared, wrench_comp)`.
- Compute `theta_rad`/`sin_model_force_corrected` exactly as shown in §2
  (reusing `rotation_magnitude`/`sin_model_gravity_residual`; read
  `axis`/`reference_force_n` from the same `force_sensor.debug_sin_model`
  config block — see §5.4 for the naming decision). Both scripts already
  compute a target/torso rotation (`R_torso` — see `_obs_rotation()` in
  `record_human_trial.py` and its equivalent in `sdk_pose_tracking.py`) and
  a pelvis-frame torso pose reference; reuse those rather than
  re-deriving them.
- Pass the three new values into `recorder.record(...)`.

### 5.3 Companion fix (recommended, not mandatory — separate from the tare/sin-model ask)

Port `gravity_comp_debug.py`'s per-frame-freshness pattern (§3.3) into both
real recording loops: stop relying on `MarkerTracker.get_latest()`'s sticky
cache; track freshness explicitly per marker per frame. Consider also
recording a per-sample freshness flag (e.g. `object_obs_stale: bool`/
`target_obs_stale: bool`) so a downstream analysis can filter out or flag
samples built on a stale pose — directly relevant to the quality of the new
tare/sin-model fields, since a stale torso pose would silently corrupt
`sin_model_theta_rad` exactly the way it already silently corrupts
`angle_diff_rpy` today. If this is deferred, say so explicitly in the
session's summary — don't let it silently drop.

### 5.4 Naming decision to make (not resolved here)

`force_sensor.debug_sin_model` was named for its debug-only origin. Once
used by production recording, decide: rename the config block (e.g. drop the
`debug_` prefix) for clarity, or keep the name to minimize diff/avoid
breaking any in-flight config edits. Either is defensible — just be
consistent between the config key and how `metadata.sin_model_config` refers
to it.

### 5.5 Docs/tests to update alongside the code (not done in this doc)

- `docs/data_recording_conventions.md` — §1 schema recap (add the 3 new
  fields, bump the version table), extend §6 (F/T sensor fields) with the
  new tare/sin-model fields and their frames/units, prominently note the
  experimental/unvalidated status per §4.
- `tests/test_recorder.py` — coverage for the new fields (presence, `None`
  handling, schema version).
- `CLAUDE.md` — Current Status, File Structure, Session Roadmap, and the
  `docs/data_recording_conventions.md` cross-reference.

---

## 6. Summary checklist for the new session

- [x] ~~Re-run `gravity_comp_debug.py`, read `theta_used`/`reproj_err` at the
      "startup" anomaly point, and resolve/document what's actually
      happening (§4) — do this first.~~ **Not done as recommended** — per
      project direction, addressed instead via a multi-frame reference
      settle (`data_recording.reference_settle_frames`) rather than
      re-diagnosed. See the resolution note at the end of §4.
- [x] Bump `SCHEMA_VERSION` to 3; add `load_cell_wrench_tared`,
      `sin_model_theta_rad`, `sin_model_force_corrected` to `MarkerSample`;
      matching `RunRecorder.record()` kwargs; `sin_model_config` in
      run-level `metadata` (§5.1).
- [x] Wire `read_all()` + the sin-model computation into
      `record_human_trial.py` and `sdk_pose_tracking.py` Phase 2 (§5.2).
      Also wired into `run_pose_tracking_preview.py`'s `RunRecorder`
      construction (metadata only).
- [x] Marker-dropout companion fix (§5.3) — applied in
      `record_human_trial.py` via the new `_DirectMultiMarkerTracker`
      (direct per-frame reads + `is_fresh()`/`seconds_since_fresh()`,
      replacing `MarkerTracker`). **Deliberately left unfixed** in
      `sdk_pose_tracking.py`/`trajectory/sources.py` — out of scope for this
      task (no interactive UI there to build the fix around).
- [x] `debug_sin_model` config naming (§5.4) — kept as-is (no rename), to
      avoid pure churn now that 3 call sites read it.
- [x] Updated `docs/data_recording_conventions.md`, `tests/test_recorder.py`,
      `CLAUDE.md` (§5.5).
