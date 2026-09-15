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
    "metadata": {"perception": {...}},   # snapshot of configs/default.yaml's perception: section
    "samples": [MarkerSample, ...],
}
```

**Per-sample fields (`MarkerSample`):** `t` (elapsed seconds since recording started), `wall_time` (absolute `time.time()` epoch — use this, not `t`, to correlate two different recordings against the real clock), `object_pose`/`target_pose` (4x4 SE3, pelvis frame, raw per-frame marker detections — `None` if that marker wasn't seen this sample), `object_obs_timestamp`/`target_obs_timestamp`, `object_reprojection_error`/`target_reprojection_error`, `angle_diff_rpy` (see §3), `object_angular_velocity`/`target_angular_velocity` (see §4), `theta` (robot-only, always `None` from the human script; **also `None` on the robot side whenever the tracking source's rotation this step was a full (3,3) matrix rather than a scalar** — see the `tracked_rotvec` entry below and `docs/six_dof_tracking_fix.md`), `tracked_rotvec` (robot-only, `(3,)` rad pelvis frame — the rotvec of whatever rotation was fed to `compute_hand_targets` this step, populated for BOTH the scalar and matrix representations, so it's the field to read regardless of which one a given run used; `theta` is kept only for the scalar case, for backward compatibility), `load_cell_wrench_raw`/`load_cell_wrench_grav_comp` (ATI F/T sensor wrench — see §6).

**Schema versions:** v1 had a single reserved `load_cell_wrench` (always `None`). v2 (current) splits it into `load_cell_wrench_raw` + `load_cell_wrench_grav_comp` when the F/T sensor was wired up — see §6.

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

**This is a different computation from `MarkerSource._rotation_from_target`** (`trajectory/sources.py`), which is used only to drive `compute_hand_targets`/the live IK during Phase 2. That function computes the target's *full* rotation relative to *its own starting pose at the beginning of the run*, in the pelvis/world frame (`R_current @ target_start_rot.T`). `angle_diff_rpy` is a full 3-axis instantaneous comparison of *object vs. target right now*, with no notion of a "starting pose." Don't read one as validating or contradicting the other — they answer different questions and are recorded as different fields (`theta`/`tracked_rotvec` vs. `angle_diff_rpy`). (Before the fix in `docs/six_dof_tracking_fix.md`, `_rotation_from_target`'s predecessor computed only a single-axis roll-about-X scalar via `atan2` of one matrix entry — a confirmed bug that silently discarded the target's pitch/yaw, and used the wrong, body-frame form of the delta besides. Neither issue ever affected `angle_diff_rpy`, which was always a separate, full-rotation computation.)

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

## 6. Force/torque sensor fields (`load_cell_wrench_raw` / `_grav_comp`)

The ATI Axia80-M20 6-axis F/T sensor (neck-mounted) is read every sample during a `--record` run (both scripts) via `g1_spine_stabilization/sensors/force_torque.py`. Two `(6,)` fields, `[Fx,Fy,Fz,Mx,My,Mz]` in N / N·m, **both in the SENSOR frame**:

- **`load_cell_wrench_raw`** — the sensor's counts scaled by the config `cpf`/`cpt`. This is everything transmitted through the sensor: the weight of the payload mounted below it **plus** any external force (the neck device pulling). `None` when the sensor is disabled or unreachable (recording degrades gracefully — see `docs/force_sensor_setup.md`).
- **`load_cell_wrench_grav_comp`** — gravity-cancelled, isolating the external interaction from the payload weight:
  - **Force channels: orientation-aware.** At the start of the run a ~1 s at-rest baseline (`force_sensor.baseline_frames`) captures the gravity force `F0` in the sensor frame (have the neck device NOT pulling then — startup pose ≈ pure gravity). As the torso tilts, the gravity force is re-projected using the **target (torso) marker's** rotation: `F_grav = M·(R_cur.T R_ref)·M.T·F0`, where `M` is the fixed torso-marker→sensor rotation (`force_sensor.gravity_comp.marker_to_sensor_rpy`, default identity = marker mounted axis-aligned with the sensor). The relative rotation `R_cur.T R_ref` is frame-invariant, so it doesn't matter whether the marker rotation is taken in optical or pelvis frame.
  - **Torque channels: tared** (`T_raw - T0`). Exact at the reference pose; fully orientation-aware torque would need the payload's center-of-mass offset `r` (not measured — a future upgrade).
  - `None` until the startup reference is captured, and it falls back to a plain tare on all six channels for any sample where the target/torso marker isn't tracked.

This is **not** the same as the `gravity_comp:` section elsewhere in the config — that's Pinocchio joint-torque feedforward for the arm motors, unrelated to F/T sensor gravity cancellation.

To fill in the real `cpf`/`cpt` (placeholder `1000000`) and get the sensor reachable on this machine, see `docs/force_sensor_setup.md`.
