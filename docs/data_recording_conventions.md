# Data Recording Conventions

Self-contained reference for interpreting `.pkl` files produced by `--record` on `scripts/run_pose_tracking_preview.py` and by `scripts/record_human_trial.py`. Read this before analyzing recorded data or adding a new field to the schema.

---

## 1. Schema recap

Both scripts import the same `RunRecorder`/`MarkerSample` from `g1_spine_stabilization/data_recording/recorder.py`, so their output is structurally identical.

**Top-level pickle contents:**

```python
{
    "schema_version": 1,
    "origin": "robot" | "human",
    "source_label": "sine" | "mock" | "marker" | "realsense",   # robot: trajectory source; human: camera source
    "tracking_mode": "orient-only" | "6dof" | None,
    "object_marker_id": int,
    "target_marker_id": int,
    "metadata": {"perception": {...}},   # snapshot of configs/default.yaml's perception: section
    "samples": [MarkerSample, ...],
}
```

**Per-sample fields (`MarkerSample`):** `t` (elapsed seconds since recording started), `wall_time` (absolute `time.time()` epoch — use this, not `t`, to correlate two different recordings against the real clock), `object_pose`/`target_pose` (4x4 SE3, pelvis frame, raw per-frame marker detections — `None` if that marker wasn't seen this sample), `object_obs_timestamp`/`target_obs_timestamp`, `object_reprojection_error`/`target_reprojection_error`, `angle_diff_rpy` (see §3), `object_angular_velocity`/`target_angular_velocity` (see §4), `theta` (robot-only, always `None` from the human script), `load_cell_wrench` (reserved for the not-yet-built neck load cell, always `None` today).

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

## 4. What's invariant to camera placement/calibration, and what isn't

The full transform chain is `pose_in_pelvis = T_d435_to_pelvis @ T_optical_to_d435 @ pose_in_optical`. For the robot, `T_d435_to_pelvis` is a real, physically accurate constant (the camera genuinely is mounted at the URDF's `d435_link`, head/waist genuinely are locked at zero). For `scripts/record_human_trial.py`, this same constant is reused regardless of where the human-trial camera is actually placed — it does **not** reflect the human rig's true camera pose.

This matters less than it sounds, because of how the two per-marker rotations combine. Let `C = T_d435_to_pelvis @ T_optical_to_d435` (the full fixed rotation applied to every marker reading in a given frame). Then:

```
R_object_pelvis.T @ R_target_pelvis = (C·R_obj_opt).T @ (C·R_tgt_opt) = R_obj_opt.T @ R_tgt_opt      (C cancels, C.T@C = I)
```

So **`angle_diff_rpy` is exactly invariant to `C`** — camera position, mounting, and calibration error in `T_d435_to_pelvis`/`T_optical_to_d435` don't affect it at all, as long as both markers are read from the same camera frame (always true here). By the same conjugation argument, **angular-velocity *magnitude* (rad/s) is also invariant** — rotation vectors preserve norm under conjugation by a rotation matrix.

What is **not** invariant: absolute `object_pose`/`target_pose` positions, and which XYZ **axis** an angular velocity is attributed to (`object_angular_velocity`/`target_angular_velocity` are expressed in the ambient pelvis frame — ω_pelvis = C · ω_optical — a rotation of the true direction by whatever `C` actually is). If you need absolute positions or axis-labeled angular velocity to be physically meaningful for the human rig, the camera needs to be physically mounted where the robot's head camera would be, or the pipeline needs a real measured calibration for wherever it ends up — neither is currently implemented.

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
