# Data Recording Conventions

Self-contained reference for interpreting `.pkl` files produced by `--record` on `scripts/run_pose_tracking_preview.py` and by `scripts/record_human_trial.py`. Read this before analyzing recorded data or adding a new field to the schema.

---

## 1. Schema recap

Both scripts import the same `RunRecorder`/`MarkerSample` from `g1_spine_stabilization/data_recording/recorder.py`, so their output is structurally identical.

**Top-level pickle contents:**

```python
{
    "schema_version": 3,
    "origin": "robot" | "human",
    "source_label": "sine" | "mock" | "marker" | "realsense",   # robot: trajectory source; human: camera source
    "tracking_mode": "orient-only" | "6dof" | None,
    "object_marker_id": int,
    "target_marker_id": int,
    "metadata": {
        "perception": {...},          # snapshot of configs/default.yaml's perception: section
        "sin_model_config": {          # see §6 -- feeds load_cell_wrench_grav_comp
            "reference_force_n": float,
            "axis": "roll" | "pitch" | "yaw" | "magnitude",
            "status": "primary_grav_comp_method",
        },
    },
    "samples": [MarkerSample, ...],
}
```

**Per-sample fields (`MarkerSample`):** `t` (elapsed seconds since recording started), `wall_time` (absolute `time.time()` epoch — use this, not `t`, to correlate two different recordings against the real clock), `object_pose`/`target_pose` (4x4 SE3, pelvis frame — **median-of-N + EMA filtered**, see §7; `None` if that marker wasn't seen this sample), `object_obs_timestamp`/`target_obs_timestamp`, `object_reprojection_error`/`target_reprojection_error` (still the **raw**, unfiltered per-frame detection's own solvePnP reprojection error — a data-quality signal, not something to smooth), `angle_diff_rpy` (see §3; also filtered, since it's derived from `object_pose`/`target_pose`), `object_angular_velocity`/`target_angular_velocity` (see §4; also filtered, since they're derived from consecutive filtered `pose_in_optical` readings), `theta` (robot-only, always `None` from the human script), `load_cell_wrench_raw`/`load_cell_wrench_grav_comp`/`load_cell_wrench_tared` (ATI F/T sensor wrench — see §6; also benefit from the same pose filtering, since the F/T gravity-comp reprojection uses the same filtered marker rotation), `sin_model_theta_rad`/`sin_model_force_corrected` (feeds `load_cell_wrench_grav_comp` — see §6, also filtered for the same reason).

**Schema versions:** v1 had a single reserved `load_cell_wrench` (always `None`). v2 split it into `load_cell_wrench_raw` + `load_cell_wrench_grav_comp` when the F/T sensor was wired up. v3 (current) adds `load_cell_wrench_tared` plus `sin_model_theta_rad`/`sin_model_force_corrected` (initially experimental, since promoted to the primary basis for `load_cell_wrench_grav_comp`) — see §6.

## 2. Filename convention

`<origin>_run_<timestamp>_<source_label>[_<tracking_mode>].pkl`, e.g. `robot_run_20260720_152249_marker_6dof.pkl` or `human_run_20260720_152249_realsense_orient-only.pkl`. The `origin` prefix (`robot_run_`/`human_run_`) makes the two distinguishable at a glance in `data/runs/` without opening a file; `origin` is also stored as its own field for programmatic checks. `source_label` for a human trial is the *camera* source (`realsense`/`mock`), not a fixed `"human"` — avoids a redundant `human_run_..._human_...` name and tells you whether a file came from a real camera trial or a hardware-free mock smoke test.

## 3. `angle_diff_rpy` — reference-frame convention (read this carefully)

`angle_diff_rpy(R_reference, R_moving)` (`data_recording/recorder.py`) computes `Rotation.from_matrix(R_reference.T @ R_moving).as_euler('xyz')` — **roll/pitch/yaw decomposed in `R_reference`'s own local axes**, not a symmetric difference between the two markers. `RunRecorder.record()` always calls this as `angle_diff_rpy(target_pose, object_pose)` — **`target` is the reference, `object` is the moving segment** — regardless of which higher-level use case is recording:

| Use case | `object_marker_id` (moving) | `target_marker_id` (reference) | What `angle_diff_rpy` means |
|---|---|---|---|
| Robot box-tracking | box (held, actively driven by the arms) | external tracking goal | box's rotation relative to the tracking goal, in the goal's own frame |
| Human neck trial | **head** | **torso** | head's rotation relative to torso, decomposed in the **torso's** own axes |

This is the standard biomechanics convention for describing a distal/moving segment (head) relative to a proximal/stable reference segment (torso) — not an arbitrary choice, and not automatic: `target` plays the "reference" role in both conventions above *by definition* of what target/object mean in this codebase (target = the thing being tracked toward; object = the thing actively moving), which is what lets one function serve both use cases correctly without any anatomy-specific special-casing in `RunRecorder`.

**This is not just a sign flip.** `angle_diff_rpy(A, B)` and `angle_diff_rpy(B, A)` are not simply negatives of each other for non-trivial rotations — `R.T`'s Euler decomposition differs from `-euler(R)` elementwise except to first order for small angles, because the underlying intrinsic-XYZ sequence is order-dependent (roll is applied first, then pitch about the *already-rolled* Y axis, then yaw about the *twice-rotated* Z axis). Which marker is "reference" is a real, consequential modeling choice, not cosmetic.

**This is a different computation from `MarkerSource._theta_from_target_rotation`** (`trajectory/sources.py`), which is used only to drive `compute_hand_targets`/the live IK during Phase 2. That function computes a single-axis rotation of the target relative to *its own starting pose at the beginning of the run* (`R_delta = target_start_rot.T @ R_current`, then `atan2` of one matrix entry — a roll-about-X angle only). `angle_diff_rpy` is a full 3-axis instantaneous comparison of *object vs. target right now*, with no notion of a "starting pose." Don't read one as validating or contradicting the other — they answer different questions and are recorded as different fields (`theta` vs. `angle_diff_rpy`).

## 4. Two different frames, on purpose: `angle_diff_rpy` vs. angular velocity

The full transform chain is `pose_in_pelvis = T_d435_to_pelvis @ T_optical_to_d435 @ pose_in_optical`. For the robot, `T_d435_to_pelvis` is a real, physically accurate constant (the camera genuinely is mounted at the URDF's `d435_link`, head/waist genuinely are locked at zero). For `scripts/record_human_trial.py`, this same constant is reused regardless of where the human-trial camera is actually placed — it does **not** reflect the human rig's true camera pose; it's FK for a G1 that isn't there. Even `T_optical_to_d435` was calibrated against this specific robot's physical camera bracket, not a generic device-intrinsic constant.

**`angle_diff_rpy` needs this composition, and is exactly invariant to it.** Let `C = T_d435_to_pelvis @ T_optical_to_d435` (the fixed rotation applied to every marker reading in a frame). Then:

```
R_object_pelvis.T @ R_target_pelvis = (C·R_obj_opt).T @ (C·R_tgt_opt) = R_obj_opt.T @ R_tgt_opt      (C cancels, C.T@C = I)
```

So `angle_diff_rpy`'s value doesn't depend on `C` at all — camera position, mounting, and calibration error in `T_d435_to_pelvis`/`T_optical_to_d435` don't affect it, as long as both markers are read from the same camera frame (always true here). This is what makes it safe for the biomechanics/clinical mapping use in §3 despite `T_d435_to_pelvis` being fictional for a human trial.

**`object_angular_velocity`/`target_angular_velocity` don't need `C` at all, so they don't carry it.** Their purpose is different from `angle_diff_rpy`: cross-trial *normalization* — how fast was the torso/head actually moving in this trial, to compare against a trial where it moved more or less. That's a magnitude-only need, and magnitude is exactly invariant to *any* fixed rotation (rotations preserve vector norm) — `|ω_pelvis| = |C · ω_optical| = |ω_optical|`, always, regardless of what `C` is. Since carrying `C` through would only ever change *axis attribution* (never the magnitude these fields exist for), and `C` is partly fictional for a human trial anyway, these two fields are computed directly from each marker's own **camera optical-frame** readings (`MarkerObservation.pose_in_optical`) — no `T_optical_to_d435`/`T_d435_to_pelvis` composition, not even the pelvis frame `object_pose`/`target_pose` use.

**Consequence: the two rotation-derived quantities in this schema live in genuinely different frames, deliberately:**

| Field | Frame | Purpose |
|---|---|---|
| `angle_diff_rpy` | Target's own local frame (torso, in a neck trial) | Axis-level clinical/biomechanics interpretation (§3) |
| `object_angular_velocity` / `target_angular_velocity` | Camera optical frame (OpenCV: X right, Y down, Z forward) | Magnitude-only cross-trial normalization |

Don't cross-reference their x/y/z components against each other or assume either is anatomically meaningful axis-by-axis for the angular-velocity fields — use `np.linalg.norm(object_angular_velocity)` / `np.linalg.norm(target_angular_velocity)` for normalization, not the raw vector components. `MarkerObservation.pose_in_optical` is `None` for synthetic observations that don't provide it (rare — real detections via `MarkerTracker.step()` always populate it, and the `--source mock` trackers in both scripts populate a synthetic stand-in specifically so `--record` exercises this path in hardware-free smoke tests); when `None`, the corresponding angular-velocity field is `None` for that sample rather than silently falling back to a different frame.

## 5. Marker local-axis convention (how to read "roll/pitch/yaw" physically)

`AprilTagDetector`'s `solvePnP` object points (`perception/detector.py`) fix each marker's local axes independent of camera position or calibration:

```python
# top-left, top-right, bottom-right, bottom-left (Z out of marker, X right, Y up)
[-half,  half, 0.0]   # top-left
[ half,  half, 0.0]   # top-right
[ half, -half, 0.0]   # bottom-right
[-half, -half, 0.0]   # bottom-left
```

**+X = rightward along the tag's canonical-upright top edge, +Y = upward (bottom edge toward top edge), +Z = out of the printed face toward the viewer** — all relative to however that specific AprilTag ID's dictionary pattern reads as "upright" (not always obvious by eye, since the bit pattern doesn't visually indicate orientation the way a letter would).

To physically confirm which real-world direction each axis points on your mounted markers: run `python scripts/visualize_markers.py --source realsense` — it overlays `cv2.drawFrameAxes` (red=X, green=Y, blue=Z) on every live detection. Since it draws from `pose_in_optical` (before any pelvis-frame composition), what you see is the marker's true local axes in the real world regardless of camera calibration. **Check the `target` marker specifically** (torso, in the neck-trial convention) — per §3, that's the frame `angle_diff_rpy`'s roll/pitch/yaw are expressed in, not the object/head marker's frame.

## 6. Force/torque sensor fields (`load_cell_wrench_raw` / `_grav_comp` / `_tared`, and the sin-model fields)

The ATI Axia80-M20 6-axis F/T sensor (neck-mounted) is read every sample during a `--record` run (both scripts, and `sdk_pose_tracking.py`'s Phase 2) via `g1_spine_stabilization/sensors/force_torque.py`'s `ForceTorqueSource.read_all()`. Three `(6,)` fields, `[Fx,Fy,Fz,Mx,My,Mz]` in N / N·m, **all in the SENSOR frame**:

- **`load_cell_wrench_raw`** — the sensor's counts scaled by the config `cpf`/`cpt`. This is everything transmitted through the sensor: the weight of the payload mounted below it **plus** any external force (the neck device pulling). `None` when the sensor is disabled or unreachable (recording degrades gracefully — see `docs/force_sensor_setup.md`).
- **`load_cell_wrench_tared`** — plain DC-offset + startup-gravity subtraction (`GravityCompensator.tare()`), **orientation-blind**: `raw - [F0, T0]`, no re-projection as the torso rotates. Always populated once the startup reference exists, independent of the `gravity_comp.enabled` config flag below.
- **`load_cell_wrench_grav_comp`** — **built from the sin-model correction** (`data_recording/recorder.py`'s `sin_model_wrench()`), promoted to the primary method after the original rotation-matrix method (below) measured worse than no correction at all against real trial data:
  - **Force channels:** `load_cell_wrench_tared`'s own force *direction*, rescaled to the magnitude of `sin_model_force_corrected` (see the EXPERIMENTAL subsection below for what that value is) — sign-preserving, so a negative `sin_model_force_corrected` flips the direction rather than clamping to zero.
  - **Torque channels:** copied from `load_cell_wrench_tared` unchanged — the sin-model has no torque information at all (same as the original rotation-matrix method, which also only ever tared torque regardless of orientation).
  - `None` whenever `load_cell_wrench_tared` or `sin_model_force_corrected` is `None` (no reference captured yet, or the target/torso marker isn't tracked this sample).
  - **Caveat for anyone plotting this:** `scripts/plot_recorded_run.py`'s `load_cell_wrench.png` takes `norm()` of this field, which reports `abs(sin_model_force_corrected)` — this *loses* the sign flips that are still visible as a signed value in `sin_model.png`'s own `sin_model_force_corrected` trace. Read `sin_model_force_corrected` directly (not `load_cell_wrench_grav_comp`'s magnitude) if the sign matters.
  - **The original rotation-matrix method still exists** (`sensors/force_torque.py`'s `GravityCompensator.compensate()`) and is still computed by `ForceTorqueSource.read_all()`'s 3rd return value, but as of this change **that value is no longer recorded** into `load_cell_wrench_grav_comp` by either real recording script — it remains available for `scripts/gravity_comp_debug.py`'s live comparison printout and any future recalibration work. See `docs/gravity_comp_recording_plan.md` for the investigation that motivated this change (the rotation-matrix method's `force_sensor.gravity_comp.marker_to_sensor_rpy` is very likely uncalibrated, and/or there's a rotation-order question a single-axis trial couldn't cleanly separate from that).

This is **not** the same as the `gravity_comp:` section elsewhere in the config — that's Pinocchio joint-torque feedforward for the arm motors, unrelated to F/T sensor gravity cancellation.

To fill in the real `cpf`/`cpt` (placeholder `1000000`) and get the sensor reachable on this machine, see `docs/force_sensor_setup.md`.

### `sin_model_theta_rad` / `sin_model_force_corrected` — now the basis for `load_cell_wrench_grav_comp`

See `metadata["sin_model_config"]["status"] == "primary_grav_comp_method"`, and `docs/gravity_comp_recording_plan.md` for the investigation that led to this (the original rotation-matrix method measured worse than no correction at all on real trial data):

- **`sin_model_theta_rad`** — per-sample, the theta (radians) fed into `sensors/force_torque.py`'s `sin_model_gravity_residual()`: either `rotation_magnitude(R_ref, R_cur)` (axis-agnostic total rotation, the default) or one Euler component of `angle_diff_rpy(R_ref, R_cur)`, per `force_sensor.debug_sin_model.axis`. `R_ref` is the target/torso marker's rotation at the moment the reference was locked (see below), `R_cur` its rotation this sample.
- **`sin_model_force_corrected`** — `norm(load_cell_wrench_tared[:3]) - reference_force_n * sin(sin_model_theta_rad)`. A single scalar force-magnitude correction, fit from one observed data point (`force_sensor.debug_sin_model.reference_force_n`, default 7.5N at a 90° torso rotation) — see `sensors/force_torque.py`'s `sin_model_gravity_residual()` docstring.

Both are computed identically by `scripts/record_human_trial.py` and `controllers/sdk_pose_tracking.py`'s Phase 2 via the shared `data_recording/recorder.py`'s `sin_model_theta_rad()` helper. In `record_human_trial.py`, `R_ref` is locked only after a multi-frame "reference settle" (`data_recording.reference_settle_frames`, gated behind the `'r'`-to-start video window) — see that script's module docstring. `sdk_pose_tracking.py` has no such interactive gate; its reference is captured at the same single instant as `load_cell_wrench_grav_comp`'s reference (top of Phase 2).

## 7. Marker pose noise filtering (`perception.pose_filter`) — why `object_pose`/`target_pose` are no longer strictly "raw"

Real trials showed brief, isolated spikes in `angle_diff_rpy`, the angular-velocity fields, `load_cell_wrench_grav_comp`, and the sin-model fields — all occurring even when the torso wasn't actually moving (implied rotation rates of 40-50 rad/s, physically implausible for a human). Root cause: an occasional single-frame `solvePnP` misread (AprilTag pose instability), propagating unfiltered into every consumer of the current marker rotation.

**Fix:** `g1_spine_stabilization/perception/pose_filter.py`'s `PoseNoiseFilter` — the same median-of-N-then-EMA recipe as `trajectory/sources.py`'s `NoiseFilteredSource` (built for the identical problem class on the robot's live IK target), generalized here to a full SE3 pose: median-of-N then EMA on the (3,) translation, and median-of-N then EMA on a hemisphere-continuity-corrected quaternion (so `q` and `-q` — the same physical rotation — aren't treated as a spurious jump) for the rotation, re-normalized before converting back to a matrix.

**Where it's applied, and why this single change fixes everything above at once:** the filter runs on each marker's raw `pose_in_optical` — the camera-native frame every `MarkerObservation` carries — exactly once, per marker, per frame; `pose_in_pelvis` is then re-derived from the *filtered* optical pose via the existing `compose_to_pelvis()`. This is required, not a style choice: `GravityCompensator.compensate()`'s own docstring notes the relative rotation `R_cur.T @ R_ref` is invariant to a fixed frame change (optical vs. pelvis give the same answer) — but that invariance depends on the frame transform being applied via a single linear composition. Median is *not* linear/equivariant under an arbitrary rotation, so filtering translation/rotation independently in *both* frames would silently produce two inconsistent answers. Filtering once, upstream of the frame composition, is what lets `object_pose`/`target_pose` (and therefore `angle_diff_rpy`, the angular-velocity fields), the F/T gravity-comp reprojection, and the sin-model theta all end up consistently clean from one code change, with no filtering logic duplicated in `recorder.py` or either script's wrench computation.

**Consequence for this schema:** `object_pose`/`target_pose` are median+EMA-filtered, not untouched raw per-frame detections, whenever `perception.pose_filter.enabled` is true (default). The true unfiltered signal is not separately recoverable from a pickle — no raw+filtered dual fields exist. `object_reprojection_error`/`target_reprojection_error` remain tied to the actual raw per-frame detection and are the closest remaining per-sample data-quality signal, though real trials show these spikes do *not* reliably show up as elevated reprojection error either (stayed under ~1.1px throughout a trial with several large spikes) — so a low reprojection error does not guarantee an unfiltered/unspiked reading pre-fix.

`--source mock`/`--source sine` are synthetic/noiseless by construction and do not use this filter (matching `NoiseFilteredSource`'s existing "marker-only" precedent). The robot's live arm-control target (`box_center`/`theta`, driving `compute_hand_targets`/IK) is a completely separate signal path (already smoothed independently by `trajectory.rate_limit`/`trajectory.noise_filter`) and is **not** affected by this filter — this is a recording-only fix.
