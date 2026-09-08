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

**SDK-2 — COMPLETE** (perception re-platform): `RealSenseSource` now wraps `g1_inspire_sdk.camera.RealSenseCamera`. 6 new hardware-free tests; 35 tests pass total. `T_optical_to_d435` hand-calibrated (superseded later — see the "SDK-6 marker-tracking fix" and "Frame conventions" entries below for the current calibrated value in `configs/default.yaml`).

**SDK-3+4 — COMPLETE** (IK transport + preview): `sdk_pose_tracking.py` drives `G1InspireRobot` via `arm.set_joint_targets(q14)`; `SineSource` + `MarkerSource` in `trajectory/sources.py`; `run_pose_tracking_preview.py` (3 source modes: `sine|mock|marker`, 2 tracking modes: `orient-only|6dof`) is the dry-run preview platform and the pre-flight gate before every live bringup. **51 tests pass.**

**SDK-5 — COMPLETE** (grip force baseline + grasp-demo jerk fix): `GraspController` in `grip_force.py`; `scripts/grasp_demo.py` for bimanual hand-only validation. Phase 0 jerk fixed: interpolation now starts from `ik.fk(current_q)` rather than hardcoded zero-config FK positions — mirrors the Phase R pattern. **51 tests pass.**

**SDK-5.5 — COMPLETE.** Gravity compensation for Inspire hands: runtime Pinocchio inertia correction in `G1HandIK.__init__` (1.0 kg measured hand, URDF was 0.14 kg); `sol_tauff` now routed to `robot.arm.set_joint_targets(sol_q, sol_tauff)` via the SDK tau feedforward channel. Gated by `gravity_comp.enabled` config flag. 5 new hardware-free tests (56 total). SDK-6 gate is now clear.

**SDK-6-safety — COMPLETE.** Guaranteed safe shutdown phase-out. Root cause: `grasp_demo.py`'s `main()` had no `except` clause at all, so a Ctrl-C/exception during the grasp/hold phases skipped hand-release and arm-return-to-default entirely, falling straight from an actively-loaded grasp pose into the SDK's fixed one-shot damping command (`G1Arm.shutdown()`: `kp=0, kd=8, tau=0`, unmodifiable — vendored SDK). Fix: grasp phase logic moved to `controllers/grasp_sequence.py` (`run_grasp_phases`), restructured so hand-release + arm-return-to-default always run, even after Ctrl-C/exceptions, and a brief settle pause (`shutdown.settle_time` in config, default 0.4s) holds the final pose immediately before `robot.shutdown()` — the SDK's 250 Hz publisher thread already re-publishes the last `set_joint_targets()` call for free, so the settle is a plain `time.sleep()`. `sdk_pose_tracking.py`'s Phase 1 got the same `KeyboardInterrupt` treatment Phase 2 already had, plus the same settle pause at the end of Phase 3. `run_pose_tracking_preview.py` gained `--dry-run`/`--no-dry-run`/`--net` flags (mirroring `grasp_demo.py`'s existing convention) so pose-tracking with `mock`/`marker` sources can now actually be run live — this replaces the not-yet-built `run_pose_tracking_sdk.py` named in earlier roadmap drafts. 6 new hardware-free tests (62 total).

**SDK-6 jerk/smoothness fixes — COMPLETE.** First live hardware test (`run_pose_tracking_preview.py --source mock --tracking 6dof --no-dry-run`) surfaced two bugs. (1) Startup jerk: `sdk_pose_tracking.py`'s Phase 1 interpolated from the hardcoded `L_EE_DEFAULT`/`R_EE_DEFAULT` constants instead of `ik.fk(current_q)` — same bug class already fixed in `grasp_sequence.py`'s Phase 0 during SDK-5, just never ported to this file; now fixed identically, mirroring Phase 3's existing pattern in the same file. (2) Jerky tracking: measured `ik.solve_ik()` at ~100ms/call, meaning the real Phase-2 loop runs at ~6-10Hz, not the assumed 50Hz — since `SineSource`/mock sample a real-time sine, this inflated per-step target deltas 5-9x. New `RateLimitedSource` (`trajectory/sources.py`) slew-rate-limits `(box_center, theta)` against the actual measured dt before it reaches the IK, resolving the long-standing "Target filtering" Open Design Question. Sine default frequency lowered `0.3`→`0.1` Hz. 10 new hardware-free tests (72 total). Speeding up the IK solve itself is deliberately deferred — see "Known timing floor" in the IK Solver section.

**SDK-6 pre-bringup hardening — COMPLETE.** Deeper analysis of jerk/jitter beyond the previous fixes, plus the mock-to-real-marker transition. Six items: (0) reverted `INSPIRE_HAND_MASS` from an accidentally-left `1.6` back to the documented/measured `1.0` kg (gravity-comp torque had been ~60% too strong system-wide); (1) fixed `WeightedMovingFilter`'s reversed recency weighting (it weighted the *oldest* sample heaviest, doubling intended lag — verified against the vendored SDK's own reference implementation using the identical weights); (2) documented (not fixed) that `ik.solve_ik()` holds the Python GIL for ~100% of its duration, stalling the SDK's own 250Hz publisher thread, which additionally uses a fixed assumed tick period rather than measured elapsed time — lives inside the vendored SDK, cannot be modified; (3) added `ipopt_exhaustion_count` diagnostics for `solve_ik()`'s `max_iter` fallback path; (4) added an `ExponentialMovingAverageFilter` smoothing `sol_tauff` before handoff (`gravity_comp.tauff_filter` config), since the SDK applies feedforward torque as an unfiltered step every publisher tick; (5) added `NoiseFilteredSource` (median-of-N + EMA, `trajectory.noise_filter` config) — `RateLimitedSource` only clamps deltas *larger* than its threshold, so real AprilTag sensor noise (small, sub-threshold) would pass through it unfiltered; applied only to `--source marker`, since sine/mock are noiseless by construction. 24 new hardware-free tests (96 total).

**SDK-6 marker-tracking fix — COMPLETE.** A live test with real AprilTag markers using `--tracking 6dof` showed the hands positioning themselves around the target marker instead of the object marker. Root cause (confirmed via direct code read, not a marker-ID mixup): `MarkerSource`'s 6dof mode set `box_center` to the target marker's raw absolute position every step, instead of the object's own starting position shifted by however far the target had moved from *its own* starting position. Wrong whenever object and target are physically in different locations (the realistic setup — object in the robot's hands, target held separately). Fixed: both tracking modes now track the target's *displacement* from its own start (position delta for 6dof, rotation delta for both modes), composed onto the object's actual starting position — see `trajectory/sources.py`'s `MarkerSource`. Also added a Rerun diagnostic (`targets/L_hand`/`targets/R_hand`) showing the actual computed IK hand-target, distinct from the raw marker and the robot's rendered mesh, and diagnostic prints of the marker pose at intermediate transform stages to help investigate a separately-reported ~10cm marker-height discrepancy. Along the way, ruled out a wrist-vs-palm IK targeting bug (confirmed `solve_ik()`'s cost function targets `L_ee_link`/`R_ee_link`, not the wrist) and corrected several stale doc claims (see Open Design Questions and File Structure). 3 tests rewritten in `tests/test_marker_source.py` to cover the delta semantics.

**`d435_link_to_optical` — REVERTED to the original standard-convention default, see `docs/followups.md`.** The ~10cm marker-height discrepancy above WAS root-caused: with the old `xyz: [0,0,0]` / `rpy: [-1.5708, 0.0, -1.5708]`, measured error against a physically-measured marker was 10.06cm position / 9.19° rotation. Several single-point calibrations followed via `scripts/calibrate_camera_extrinsics.py`, but none was ever verified against a genuine held-out, non-collinear point (the multi-point procedure the calibration script's own `--verify` mode calls for). After a separate `marker_size` bug (0.04 vs. the correct 0.045) was found and fixed, the freshly-recomputed correction landed close to this same original default — evidence the camera mount itself was never far off, and that the original 10cm/9° error was mostly that bug, not a real mounting error. Given that, and given no fit was ever properly verified, the config has been reverted to `xyz: [0.0, 0.0, 0.0]`, `rpy: [-1.5708, 0.0, -1.5708]`. This does not touch `T_d435_to_pelvis` (Pinocchio FK against the URDF, never calibrated). If markers look badly misplaced in Rerun, a real multi-point-verified calibration (3-4 non-collinear points, at least one held out for `--verify`) is the bar for replacing this default again — see `docs/followups.md` for the procedure and full history.

**Data recording — COMPLETE.** Added the ability to record trial data from both the robot's live tracking runs and a human performing the same physical task, into pickle files sharing one schema so the two can be compared. New `g1_spine_stabilization/data_recording/recorder.py`: `RunRecorder` accumulates per-sample `MarkerSample`s (both markers' poses, a per-axis angle-diff, finite-difference angular velocities, time, plus a `load_cell_wrench` field reserved for the future neck-mounted load cell — always `None` until that sensor exists) and pickles them via an idempotent `save()`. `run_pose_tracking_sdk`'s Phase 2 (`sdk_pose_tracking.py`) gained an optional `recorder` parameter: `record()` runs once per iteration, and `save()` runs in a `finally` guaranteed regardless of how Phase 2 exits (normal timeout or `KeyboardInterrupt`) — additive only, Phase 2's existing try/except is untouched. `run_pose_tracking_preview.py` gained a `--record [PATH]` flag (works with any `--source`; `--source sine` records time/theta only, no real markers). New standalone `scripts/record_human_trial.py` — perception-only (no robot/IK), reuses the same camera/marker stack and `RunRecorder` import as the robot path, so its output is directly comparable. New `data_recording:` config section; `data/runs/` gitignored. Auto-generated filenames carry an explicit `robot_run_...`/`human_run_...` prefix so origin is visible at a glance; the human script's `source_label` is the camera source (`realsense`/`mock`), not a fixed `"human"`, to avoid a redundant name. **`angle_diff_rpy(R_reference, R_moving)` decomposes `R_reference.T @ R_moving` — `RunRecorder.record()` always passes `target` as the reference and `object` as moving** (biomechanics convention: for the neck-trial mapping `target_marker_id`=torso, `object_marker_id`=head, so recorded roll/pitch/yaw describe head rotation in the torso's own axes, matching standard neck-kinematics reporting — this is NOT a symmetric difference and swapping the arguments is not just a sign flip for non-trivial rotations). **`object_angular_velocity`/`target_angular_velocity` are computed from each marker's own camera OPTICAL-frame readings (no pelvis/mount composition at all)**, since their purpose is magnitude-only cross-trial normalization (how much the torso/head was actually moving) rather than axis-level interpretation — `MarkerObservation` gained a `pose_in_optical` field (populated by `MarkerTracker.step()`) to carry this. The two rotation-derived fields deliberately live in different frames for different purposes. Full convention reference: `docs/data_recording_conventions.md`. 24 new hardware-free tests.

**Force/torque sensor — COMPLETE.** The ATI Axia80-M20 6-axis F/T sensor (Ethernet, RDT-over-UDP port 49152) is now read into every `--record` run — robot and human — populating the pickle's two new wrench fields. New `g1_spine_stabilization/sensors/force_torque.py`: `ForceTorqueSensor` (background-daemon-thread UDP reader, lock-guarded latest wrench, mirrors `PalmForceReader`'s pattern; degrades to `None` with a printed diagnostic on connect timeout so recording never crashes), `GravityCompensator` (orientation-aware FORCE cancellation + tared TORQUE), `ForceTorqueSource` (glue both scripts share). `MarkerSample`'s reserved `load_cell_wrench` was split into `load_cell_wrench_raw` + `load_cell_wrench_grav_comp` (both `(6,)` `[Fx,Fy,Fz,Mx,My,Mz]` N/N·m, sensor frame); `SCHEMA_VERSION` bumped 1→2; `record()` gained two matching kwargs. Both record loops capture a ~1 s at-rest gravity reference at the top of the tracking phase (device not pulling → startup ≈ pure gravity) using the target/torso marker's rotation, then read raw+comp each iteration; gravity-comp math `F_grav = M·(R_cur.T R_ref)·M.T·F0` (M = marker→sensor rotation, config, default identity), torque tared. New `force_sensor:` config section (cpf/cpt are PLACEHOLDER — read the real values from the sensor web UI). `scripts/sensor_test.py` refactored to use the shared reader. `docs/force_sensor_setup.md` covers making the sensor reachable on this machine (needs a NIC static-IP assignment via sudo/admin — code + doc ready, network change is the user's step) and reading cpf/cpt. NOT the same as the arm `gravity_comp` (Pinocchio joint torques — unrelated). 15 new hardware-free tests. **207 tests pass total.**

**Live-marker misalignment fixes — IN PROGRESS.** A live retest of `--source marker --no-dry-run` surfaced real bugs beyond the earlier calibration/doc fixes: (1) `preview.set_camera_intrinsics()` was being passed our own `intrinsics_native` dict shape (`fx/fy/cx/cy/resolution`) instead of the SDK's expected shape (`width/height/fx/fy/ppx/ppy`), crashing with `KeyError: 'width'` — fixed by translating the dict at the call site in `run_pose_tracking_preview.py`, no vendored-SDK edit. (2) Bigger finding: `d435_to_pelvis_from_pinocchio()` (`perception/frames.py`) always assumed the robot's three waist joints sit at exactly zero, never checked against the robot's real state — a humanoid's balance controller can plausibly nudge the waist by a few degrees to counterbalance a loaded, outstretched arm, and empirically a mere 5° of unaccounted waist_pitch shifts `T_d435_to_pelvis` by 37mm at the camera mount alone (more once projected out to an actual marker), enough to explain the "tens of cm" live error. Fixed: new `waist_q_full()` helper (`frames.py`) + `MarkerTracker.set_T_d435_to_pelvis()` (`tracker.py`) let `run_pose_tracking_preview.py` read the robot's actual live waist angles post-connect (`robot.arm.low_state.motor_state[12/13/14].q`) and correct the transform in place, printing the measured angles and the delta from the zero-waist assumption. Also flagged (not fixed): neither this repo nor the vendored SDK ever calls Unitree's `MotionSwitcherClient.ReleaseMode()` before sending low-level commands, so whether our position commands are fully authoritative on this hardware vs. arbitrated by the robot's own onboard controller is unverified — a separate, higher-stakes question for later. Full writeup, root-cause reasoning, and the proper multi-point recalibration procedure (3-4 non-collinear measured points, not the original 2 collinear ones): `docs/followups.md`. 2 new hardware-free tests (209 total). (3) `perception.marker_size` was also found genuinely wrong (physically measured, corrected `0.04` → `0.045` in `configs/default.yaml`) — a wrong marker size scales every recovered marker distance, and re-running `calibrate_camera_extrinsics.py` with the corrected size showed the old "tentative" `d435_link_to_optical` (previously verified 6mm/4.05° at one distance) diverging badly (6.83cm/12.65°) at a different, non-collinear point — evidence the old calibration was mostly absorbing the marker_size bug rather than measuring a real mounting error. A further single-point fit followed a marker-repositioning event (`xyz: [-0.0500, -0.0919, 0.0154]`, `rpy: [-1.7103, 0.0446, -1.4039]`), but — like every fit before it — was never verified against a genuinely held-out point. **Superseded:** rather than continue confirming/re-fitting single unverified points, `d435_link_to_optical` has been reverted to the original standard-convention default (`xyz: [0,0,0]`, `rpy: [-1.5708, 0.0, -1.5708]`) — see the entry above and `docs/followups.md` for the full reasoning and the multi-point procedure needed to justify a future recalibration.

**RRT approach violent-motion fix — COMPLETE.** A live run of the RRT*-planned pre-grasp approach (Phase 1a/`sdk_pose_tracking.py`, Phase 0a/`grasp_sequence.py`) was smooth for about a second, then suddenly fast and violent near the end of the path — reproduced physically on hardware, not a Rerun rendering artifact (a prior session's dismissal of it as "just Rerun" was never actually verified, since no code anywhere logged per-tick joint commands). Root-caused to two independent, compounding bugs in `execute_joint_path` (`controllers/joint_path_execution.py`), the sole function turning RRT waypoints into `set_joint_targets()` calls: (1) per-segment time was allocated purely proportional to joint-space arc length, then discretized via `n_steps = max(1, int(seg_time/control_dt))` with no floor — a segment receiving a thin proportional time slice (e.g. the RRT goal-connect edge, after `shortcut_path` left earlier segments consuming most of the budget) could collapse to `n_steps=1`: one command at `q_from`, the very next command one `control_dt` later already at `q_to`, a near-instant full-amplitude jump with no filtering to catch it (unlike every `solve_ik()`-driven phase, which routes through `set_ee_targets`'s low-pass filter — `execute_joint_path` calls `set_joint_targets` directly). (2) Confirmed the dangerous motion still occurred even WITH `max_path_speed_rad_s`/`max_hand_speed_m_s` actively configured: those caps derived `total_time` from `path_length / speed`, which only bounds each segment's AVERAGE per-tick rate — but smoothstep's peak slope, at a segment's own midpoint, is 1.5x that average, so the caps silently allowed a true peak 50% above the configured value. Fixed both: added `max_step_rad` (new `collision_avoidance.max_step_rad: 0.05` config, default non-null) which floors every segment's `n_steps` against its own physical length regardless of time-budget slicing; scaled both existing caps' required-time derivation by the same smoothstep peak-slope factor (`_SMOOTHSTEP_PEAK_SLOPE = 1.5`) so the configured rad/s or m/s value now bounds the true peak, not the average. Also added optional per-tick CSV diagnostic logging (`log_path` param on `execute_joint_path`, `--log-joint-path PATH` on `run_pose_tracking_preview.py`/`grasp_demo.py`) — timestamp, segment index, step index, segment length/time/n_steps, per-tick delta norm, full `q_cmd` — deliberately independent of Rerun, so a future incident can be traced to the exact waypoint/tick without relying on the visualizer. 4 new hardware-free tests in `tests/test_joint_path_execution.py` (two proving the peak-vs-average cap bug and its fix, two proving the `max_step_rad` floor and its no-op-when-unset default). 264 tests pass total.

**RRT approach violent-motion fix, round 2 (Phase 1b/0b close) — COMPLETE.** The above fix covers only Phase 1a/0a (the RRT*-planned gross reach, via `execute_joint_path`). A follow-up dry-run retest with `max_hand_speed_m_s` lowered to 0.01 m/s **still showed the same violent snap at the very end** — a second, independent, previously-uninvestigated bug, not a residual issue in the already-fixed `execute_joint_path` (re-verified clean: its smoothstep loop covers `s == n_steps` exactly, no snap-to-final shortcut exists). Root cause: Phase 1b (`sdk_pose_tracking.py`, "closing straight in along the inter-palm axis," immediately after Phase 1a) and its mirror Phase 0b (`grasp_sequence.py`) are a *separate* smoothstep+IK loop whose step count (`steps_close`) was derived **purely from a fixed wall-clock `grasp_approach.close_time`** (default 1.5s), **never reading `collision_avoidance.max_hand_speed_m_s`/`max_path_speed_rad_s`/`max_step_rad` at all**. Quantified: with defaults (`pregrasp_offset=0.08` m, `close_time=1.5` s), true peak Cartesian speed (smoothstep's 1.5x-average peak, same factor as the round-1 fix) is a fixed `1.5*0.08/1.5 = 0.08 m/s` regardless of any configured cap — 8x over a 0.01 m/s target, landing the hands on the object at unsafe speed right at the end of the approach even with Phase 1a/0a's own reach already correctly capped and slow. This exactly matches "smooth reach, then a fast snap right at the very end." Fixed: new shared helper `close_phase_steps(close_time, pregrasp_offset, control_dt, max_hand_speed_m_s=None)` in `controllers/joint_path_execution.py` (reuses `_SMOOTHSTEP_PEAK_SLOPE`) floors `steps_close` the same way `max_step_rad` floors `execute_joint_path`'s segments, just Cartesian-native (no `ik.fk()` needed since `pregrasp_offset` IS the exact per-palm travel distance via `compute_hand_targets`'s `squeeze` semantics). Both Phase 1b and Phase 0b now call this helper instead of computing `steps_close` inline. 4 new tests for `close_phase_steps` (including a regression test reproducing the confirmed incident's ~8x-overshoot math), plus 3 existing `test_sdk_transport.py` tests updated (their hardcoded `phase1_calls` `KeyboardInterrupt`-injection arithmetic had to switch from the old fixed-`close_time` formula to `close_phase_steps` itself, since the real Phase 1b call count changed once the fix was applied against the repo's actual configured `max_hand_speed_m_s`). 268 tests pass total. **Not yet verified live** — human-only per Hard Rules; a live retest confirming both Phase 1a *and* Phase 1b now stay under the configured speed cap end-to-end is the outstanding step.

**Comprehensive dry-run instrumentation — COMPLETE.** After the round-2 fix, a live-hardware retest *still* showed the same violent motion, this time reported specifically between Phase 1a and Phase 1b. Rather than propose a third guessed mechanism, built real instrumentation to capture ground truth: new `diagnostics/` package — `run_log.py` (`RUN_LOG`, a single process-wide, thread-safe, wall-clock-anchored JSONL logger; no-op until `RUN_LOG.start(path)` is called) and `sdk_state_tap.py` (`start_sdk_state_tap`, a read-only daemon-thread poll of the vendored SDK's internal dry-run arm state — `G1Arm._q_target`/`_cmd_q_state`/`_sim_q`/`_arm_velocity_limit`, ~4ms resolution, never modifies the SDK). Wired into Phase 1a (`execute_joint_path`'s existing per-tick loop), Phase 1b, Phase 3, and a new Phase-1a→1b transition record comparing `execute_joint_path`'s handoff `q` against a fresh `robot.get_state()` read at the same instant. New `--diagnostic-log PATH` flag on `run_pose_tracking_preview.py`. 8 new hardware-free tests (`test_run_log.py`, `test_sdk_state_tap.py`). A `--source mock --tracking 6dof` dry run with this instrumentation completed all phases (Phase 1a/1b/3 tick data + phase-transition record all captured) but did NOT reproduce a jump at the Phase 1a→1b boundary in the data (`gap_norm=0.0`, no clipper-rate spike there) — it did surface a real spike (~28 rad/s) elsewhere, in the *unlogged* Phase 1.5→Phase 2 gap (`RateLimitedSource`'s `dt`-since-last-`step()` calculation is not clamped, so a 2s silent gap during hand-close inflates its allowed per-step delta far past what it's meant to permit — documented here as a separate, real, not-yet-fixed finding for a future session, out of scope for the live 1a→1b report). Since the reported incident is specifically live-hardware and specifically at the 1a→1b seam (which the dry-run sim cannot fully reproduce, since it has no real actuator lag), moved to a direct code-level fix instead — see next entry.

**Cartesian arrival gate before Phase 1b/0b — COMPLETE.** Direct code investigation (prompted by the live 1a→1b report) found `execute_joint_path`'s existing `arrival_tolerance_rad` check — meant to confirm Phase 1a's RRT-planned reach actually completed before Phase 1b begins — has two real gaps: (1) it's joint-space-only (`norm(actual_q - target_q)`), which doesn't guarantee Cartesian closeness (the same joint-space delta produces different Cartesian displacement depending on which joints it's concentrated in); (2) it's a poll-with-timeout (2s default), not a hard gate — on timeout it warns and proceeds anyway, so Phase 1b can begin warm-started from a `current_q` that's still meaningfully off from the intended final pose, particularly plausible on real hardware where actuator lag is real (unlike the dry-run sim). Also confirmed via direct read: `execute_joint_path` has no distance/Cartesian gate between individual waypoints during playback either — the segment loop advances by a precomputed tick count only, never checking real position against the current target mid-path. Added, per explicit user request: a second, independent Cartesian position+orientation arrival check (`cartesian_arrival_tolerance_m`/`cartesian_arrival_tolerance_rad`, both new `collision_avoidance` config keys, `null` by default) that runs in the SAME poll loop and shares the same `arrival_timeout_s` budget as the existing joint-space check (not a separate sequential wait) — Phase 1b begins only once every configured check (joint-space, Cartesian position, Cartesian rotation) is satisfied or the shared timeout elapses, and the timeout warning now names exactly which check(s) failed. Position error via `ik.fk(actual_q)` vs. `ik.fk(waypoints[-1])` (worst of L/R hand); rotation error via `pin.log3(R_actual @ R_goal.T)` (same convention `ik_solver.py`'s own IK cost function already uses). Wired through both `sdk_pose_tracking.py` Phase 1a and `grasp_sequence.py` Phase 0a call sites. 5 new hardware-free tests. 281 tests pass total. **Not yet verified live** — human-only per Hard Rules; `cartesian_arrival_tolerance_m`/`_rad` are UNMEASURED placeholders (`null`) and need real starting values (e.g. ~0.01m/~0.05rad) set before a live retest can actually exercise this gate. `arrival_timeout_s` also raised `2.0`→`5.0` (real hardware settling after a multi-waypoint RRT* reach can plausibly take longer than the dry-run sim suggests) — shared by both the joint-space and Cartesian checks.

**`RateLimitedSource` gap-inflation fix — COMPLETE.** Closes the separate finding from the comprehensive-instrumentation session (documented above, previously deferred as out of scope for the live 1a→1b report): `RateLimitedSource` (`trajectory/sources.py`) computed its allowed per-step delta as `max_theta_rate * dt`/`max_pos_rate * dt`, where `dt` is wall-clock time since the *last* `.step()` call — uncapped. This codebase's controllers only start calling `source.step()` once Phase 2 (tracking) begins; Phase 1a (RRT reach) + Phase 1b (straight-in close) + Phase 1.5 (hand-close) combine to several real seconds with zero `step()` calls. A 2-second gap alone lets the limiter's own math permit up to 120° of theta swing in a single subsequent step (vs. the sine's entire ±30° amplitude) — the rate limiter provides essentially no protection on the first tick after any such gap, exactly backwards from its purpose. Fixed: new `max_dt` constructor param (default 0.5s; config `trajectory.rate_limit.max_dt_s`) caps the `dt` used in the rate-limit math itself, so a long real gap is treated as if only `max_dt` of time passed — preserves the limiter's real per-tick constraint regardless of how long the caller went without calling `step()`, with no effect on normal Phase 2 operation (real inter-step gaps, even the documented ~90-180ms IK solve cost, stay well under the default ceiling). 2 new regression tests (one reproducing the exact gap-inflation math, one confirming normal fast-polling behavior is unaffected); one existing test's `dt` value adjusted since it exceeded the new default ceiling. 283 tests pass total.

**IK smoothing-filter staleness + goal_q local-minimum fix (round 6) — COMPLETE.** None of rounds 1-5 held on the latest live-hardware retest, so rather than guess again, reproduced the user's exact reported scenario in dry-run via `run_pose_tracking_preview.py --source mock --dry-run --test-start-left/--test-start-right <extreme pose>` with `--diagnostic-log`/`--log-joint-path`. This captured a real jump: Phase 1a→1b's handoff `q` is byte-identical (`gap_norm=0.0`), but Phase 1b's first `solve_ik()` tick alone produced a 0.60 rad joint-space delta, decaying over the next few ticks. Root-caused (by isolating `G1HandIK` outside the full harness) to TWO independent, compounding bugs: (1) `G1HandIK.smooth_filter` (the 4-tap output-smoothing `WeightedMovingFilter`) is only fed inside `solve_ik()`, but Phase 1a's RRT-planned reach (`execute_joint_path`) moves the arm via `set_joint_targets()` directly and never calls `solve_ik()` — so the filter's queue sits frozen at whatever it held before Phase 1a started, and Phase 1b's first `solve_ik()` call blends a correct fresh solution against 3 stale, pre-Phase-1a samples. Fixed with `G1HandIK.reset_smoothing(q)` (`ik_solver.py`) — reseeds the filter's queue and `init_data` from an explicit joint config, discarding history — called immediately after `execute_joint_path` returns, in both `sdk_pose_tracking.py` Phase 1a and `grasp_sequence.py` Phase 0a. (2) Independently, the single `solve_ik()` call that computes `goal_q` (the RRT planner's target) can converge "successfully" (IPOPT reports success, no `ipopt_exhaustion_count` hit) at a solution still several cm off the real Cartesian target when warm-started from a pose far away in joint space — a local-minimum/warm-start-quality effect, confirmed by showing a second re-solve from that same output converges much further (9cm → 3cm raw error → 1cm after a second pass, in the reproduced scenario). The RRT planner then faithfully drives the arm to that wrong `goal_q`. Fixed with `G1HandIK.solve_ik_refined(L_target, R_target, current_q, n_refine=3)` — re-solves 2-3 times, re-warm-starting from each solve's own output — used only for the one-shot `goal_q` seed call (not per-tick tracking calls, where the extra cost isn't worth it and `smooth_cost`'s "previous timestep" meaning doesn't apply the same way). **Measured on the exact repro: peak Phase-1b first-tick jump dropped from 0.60 rad to 0.16 rad (~4x), with a smooth decay instead of a sharp discontinuity.** Not fully zero — a residual ~0.16 rad delta remains on Phase 1b's first tick, above Phase 1a's own `max_step_rad=0.05` cap (which doesn't apply to Phase 1b — Phase 1b currently has no per-tick joint-space cap of its own, only `close_phase_steps`'s Cartesian speed floor). 4 new hardware-free tests (`test_kinematics.py`: `test_reset_smoothing_seeds_filter_and_init_data`, `test_reset_smoothing_clears_stale_queue_and_prevents_jump`, `test_solve_ik_refined_converges_closer_than_a_single_solve`; `test_sdk_transport.py`: `test_phase1a_resets_ik_smoothing_after_execute_joint_path`, a spy-based integration test confirming the reset is wired at the real Phase 1a→1b call site with the value `execute_joint_path` actually returned). 287 tests pass total. **Not yet verified live** — human-only per Hard Rules; this is the first fix across all six rounds that reproduces the exact failure mode in isolation, outside the RRT planner and dry-run harness entirely, from first principles, but the residual 0.16 rad delta means a live retest may still show a smaller, non-zero snap worth watching for.

**RRT head-obstacle sphere: goal visualization + failure-mode reference doc — COMPLETE.** Separate from motion-smoothness work above, a deep-dive into the RRT*'s head-obstacle sphere itself: the Cartesian pregrasp-standoff goal (`L_pre`/`R_pre` — what `ik.solve_ik_refined` and the RRT planner actually solve/plan toward) was never logged to Rerun during Phase 1a/0a-1b/0b, so there was no way to visually compare "where is the goal" against the red/orange sphere. Added `log_hand_goal_targets(L_target, R_target)` (`controllers/joint_path_execution.py`) — logs `targets/L_hand`/`targets/R_hand` (cyan/magenta arrows, same entity names/colors Phase 2's `_log_marker_info` already uses) — called once in both `sdk_pose_tracking.py` Phase 1a and `grasp_sequence.py` Phase 0a, right after `L_pre`/`R_pre` are computed, alongside the existing `log_head_sphere` call. New `docs/rrt_head_obstacle.md`: documents the red (`obstacles/head_true`, unpadded, checked via `is_valid_contact` for the goal config + final goal-connect edge) vs. orange (`obstacles/head_padded`, `+head_margin_m`, checked via `is_valid_transit` for the start config + everything else) sphere split and why the goal must never be checked against the padded shape; all three `RRTPlanningError` subclasses (`StartInCollisionError`, `GoalInCollisionError`, `RRTBudgetExhaustedError`) with their exact trigger conditions and printed message formats, and which config knobs are most likely responsible for each; and a step-by-step tuning procedure for sizing/placing the sphere against a real measured head using the new visualization. Flags that `head_radius_m` is currently `0.01` (1cm) — still the documented PLACEHOLDER, far smaller than a real head — as a plain, separate finding from the anchor-mismatch investigation in `docs/followups.md`. No code behavior changes beyond the new Rerun logging calls; no new tests (side-effecting visualization only, 287 tests unaffected).

**Stacked head-obstacle spheres (hand-under-head fix) — COMPLETE.** Live testing found a real gap in the single-sphere RRT collision model: in some configurations RRT* correctly finds the single head sphere collision-free by routing a hand UNDER it — safe against that one sphere, but unsafe in reality, since there's more physical head/neck and the table surface below the sphere the single-sphere model doesn't represent at all. Fixed by stacking additional spheres below the primary one, extending the modeled obstacle volume further along the same "into the object" depth axis the primary sphere is already offset along. `kinematics/collision.py`: new `build_head_obstacle_stack(center, radius, margin_m, stack_count, stack_spacing_m, axis)` (returns `[primary] + stack_count` more `HeadObstacle`s, same radius, spaced along `axis`; `stack_count=0` returns exactly `[primary]`, today's behavior unchanged); `CollisionChecker.__init__` generalized from a single `HeadObstacle` to `list[HeadObstacle]` — every obstacle gets its own uniquely-named padded/true geometry pair, all feeding the SAME `_padded_pair_ids`/`_true_pair_ids` lists `is_valid_transit`/`is_valid_contact`/`first_colliding_geometry` already consume, so those methods needed no changes at all; `first_colliding_geometry`'s obstacle-membership check generalized from a 2-tuple to a `_obstacle_geom_ids` set. `rrt_planner.py`'s `plan_collision_free_approach` gained an optional `axis` param and reads new `collision_avoidance.head_stack_count`/`head_stack_spacing_m` config (both default `0`/off). Both controllers pass `axis=-marker_R[:, 2]` when a real marker orientation is available (`sdk_pose_tracking.py`) or `axis=[0,0,-1]` (world -Z / gravity-down) when it isn't (`grasp_sequence.py`'s `grasp_demo.py` has no camera at all). `log_head_sphere` (`joint_path_execution.py`) extended to accept a list of centers (in addition to its original single-center usage) so the whole stack renders together as one multi-sphere Rerun entity. `docs/rrt_head_obstacle.md` updated with a new "Stacked spheres" section (also fixed a stale `head_radius_m` value reference). 8 new hardware-free tests: `test_head_obstacle.py` (5, covering `build_head_obstacle_stack`'s no-op-at-zero default, center placement, axis normalization, default-axis fallback, and sphere-overlap-when-spacing-tight); `test_rrt_planner.py` (2, the actual hand-under-head regression — a config that clears the PRIMARY sphere but intrudes on a STACKED one is still correctly reported as a collision by both `is_valid_contact` and `first_colliding_geometry`); existing `CollisionChecker(ik, head)` call sites in `test_rrt_planner.py` updated to `CollisionChecker(ik, [head])` for the new list-based signature. 297 tests pass total. **Not yet verified live** — human-only per Hard Rules; `head_stack_count`/`head_stack_spacing_m` are unmeasured placeholders (both default `0`/off) and need real values (spacing `< 2*head_radius_m` so consecutive spheres overlap) before a live retest can exercise the fix against the original hand-under-head scenario.

**Floor/table plane obstacle for RRT\* — COMPLETE.** The user identified a simpler, more direct complement to the stacked-sphere fix above: a hard Z-axis limit (a half-space "wall") that RRT* treats as solid below a configured pelvis-frame height, independent of how the head sphere/stack happens to be sized or placed. Verified directly before implementing: `hppfcl.Halfspace(n, d)` is a genuine `CollisionGeometry` subclass — same base class as `hppfcl.Sphere` — and slots into the existing `HeadObstacle` dataclass (`placement`/`padded_shape`/`true_shape`) with **zero changes to `CollisionChecker`**, confirmed end-to-end through the real `pin.GeometryObject`/`CollisionPair`/`pin.computeCollisions` pipeline (not just the bare `hppfcl.collide()` function) and against a real low arm pose. New `kinematics/collision.py`: `build_floor_obstacle(z_min, margin_m, normal)` — unlike the head sphere (where the goal is *meant* to sit close to the true surface), there's no legitimate reason for either RRT transit or the final goal to be below a real floor/table, so `padded_shape` sits `margin_m` *higher* than `true_shape` (same "padded = more conservative" direction as `head_margin_m`, applied to both checks since there's no goal exception here). `HeadObstacle`'s docstring updated to note it's a generic obstacle container (kept the name — no rename, per explicit user choice, to avoid touching every existing reference from the sphere-stacking session). `rrt_planner.py`'s `plan_collision_free_approach` reads new `collision_avoidance.floor_z_m`/`floor_margin_m` config directly (both default `null`/`0.0`/off) and appends the floor to the same obstacle list the head sphere/stack already builds — **zero call-site changes needed in either controller**, since (unlike the head sphere) the floor is a fixed world/pelvis-frame constant, not derived from a tracked marker. New `log_floor_plane` (`joint_path_execution.py`) renders the floor as a wide flat box near the tracked object/workspace (same red/orange true/padded convention as `log_head_sphere`), wired into both controllers alongside the existing sphere-stack visualization. `docs/rrt_head_obstacle.md` gained a new "Floor/table plane" section covering the half-space model, the both-tiers rationale, and the tuning trap (must sit below every reachable working height or it immediately raises `StartInCollisionError`/`GoalInCollisionError` — verified directly with both a safe value, dry-run completes cleanly, and a deliberately-too-high value, correctly aborts with a real arm link named). 12 new hardware-free tests: `test_head_obstacle.py` (7, covering `build_floor_obstacle`'s shape types, true/padded `d` values, margin-zero equality, default/custom normal, and trivial placement); `test_rrt_planner.py` (5, a real low-pose collision check mirroring the manual verification, the named-link diagnostic, both failure modes via the actual config-driven `plan_collision_free_approach` path, and a no-op-when-absent regression). 309 tests pass total. **Not yet verified live** — human-only per Hard Rules; `floor_z_m` is `null`/disabled by default and needs a real measured value (below every working height) before a live retest can exercise it.

**RealSense FOV widened (640x480 → 1280x720) — COMPLETE.** The user noticed `cheese` (a generic Linux UVC viewer) showed noticeably more of the scene through the D435 than our own scripts did — not enough room to comfortably frame both AprilTag markers. Root-caused, not assumed: `RealSenseSource.__init__` (`perception/frame_sources.py`) hardcoded `width=640, height=480, fps=30` as Python default arguments, and all 5 real call sites (`visualize_markers.py`, `calibrate_camera_extrinsics.py`, `record_human_trial.py`, and two sites in `run_pose_tracking_preview.py`) constructed it with no override — confirmed via exhaustive grep that this is the *only* place a resolution/FOV value is set anywhere in this repo or the vendored SDK (no crop/resize/ROI code exists at all). The D435's color sensor is native 16:9; requesting a 4:3 mode like 640×480 uses a narrower center-crop of the sensor rather than a full-width downsample — a known RealSense characteristic, and almost certainly why `cheese` (which opens a default 16:9 UVC mode) showed more. Confirmed safe to change before touching anything: intrinsics are queried *live* from the actual negotiated stream profile every time (never hardcoded), and `AprilTagDetector`/`CameraIntrinsics` take intrinsics and the frame as call-time parameters with zero resolution assumptions baked in — so a resolution change requires no compensating change anywhere else. The vendored SDK's `RealSenseCamera` requests one shared `width`/`height`/`fps` for *both* depth and color in a single `enable_stream` pair, capping the realistic choice at the highest resolution both streams support at once: **1280×720 @ 30fps** (confirmed against the real D435i: negotiates cleanly on both streams, intrinsics scale sanely — fx/fy ≈908 vs. the old ≈615 at 640-wide — camera is on a USB3 SuperSpeed 5000M port per `lsusb -t`, no bandwidth concern). Fixed: added `width`/`height`/`fps` as new, actually-read keys under `configs/default.yaml`'s `perception.camera` block (its pre-existing `intrinsics`/`distortion`/`resolution` sub-keys were already dead/unused placeholder code for a not-currently-wired-up fallback path — confirmed via grep, left untouched); all 5 call sites now read `perception.get("camera", {}).get("width"/"height"/"fps", <old default>)` and pass it through explicitly. `RealSenseSource.__init__`'s own defaults are unchanged (640/480/30), preserving today's behavior for any caller that doesn't pass config-driven values. 1 new hardware-free test (`test_non_default_resolution_flows_through_to_frame_and_intrinsics`) confirming a non-default resolution flows through dry-run frame shape + live intrinsics correctly. 310 tests pass total. **Live-verified** (camera-only, permitted under Hard Rules without a human): a direct non-dry-run connection at the new resolution against the real D435i succeeded on the first attempt — frame shape, live intrinsics, and resolution all matched the configured 1280×720 exactly.

**Infrared-stream marker detection (wider FOV) — COMPLETE.** Follow-up to the FOV fix above: the `cheese` screenshot the user compared against turned out to not be the color stream at all — it showed the D435i's active-stereo IR projector dot pattern, which the color sensor's built-in IR-cut filter always blocks, meaning the user had been looking at a raw infrared sensor, not color. Confirmed against Intel's own spec page: the RGB sensor's FOV is 69°×42°, while the depth/IR stereo module's FOV is a genuinely wider 87°×58° — a fixed optical/hardware difference between two physically different sensors, not something any resolution change can close. Investigated before writing code: the vendored `g1_inspire_sdk`'s `RealSenseCamera` has zero infrared/emitter support and no extension point (`connect()`/`capture()` hardcode exactly depth+color; depth and infrared share the same physical "Stereo Module" sensor, so a second independent pipeline can't claim infrared alongside the vendored pipeline's depth stream either) — so the new mode in `RealSenseSource` (`perception/frame_sources.py`) bypasses the vendored camera entirely for this mode and talks to `pyrealsense2` directly (same lazy-imported dependency it already uses one layer down), touching zero vendored-SDK files. `AprilTagDetector.detect()` already branches on `frame.ndim` and needed no changes at all for a single-channel IR frame; `MarkerTracker`/`types.py` are fully generic and also needed no changes. **Caught before it could cause a silent bug:** `pyrealsense2`'s `rs.extrinsics.rotation` is column-major (confirmed directly against both the installed package's own `help()` output and the vendored librealsense C header) — a naive row-major reshape would have silently produced the *transpose* of the real IR-to-color rotation, a valid-looking but wrong matrix. Fixed with an explicit `order='F'` reshape in a new `_extrinsics_to_se3()` helper, with a dedicated unit test using a deliberately asymmetric matrix specifically to make row-vs-column-major reshape errors unmissable. New `RealSenseSource` constructor params `stream="color"|"infrared"` and `emitter_enabled=False` (infrared only — disables the D435's dot projector, which otherwise overlays every IR frame and can degrade AprilTag corner contrast); a new `ir_to_color_extrinsic` property exposes the camera's own **live-queried factory extrinsic** (IR-left → color), queried once at connect time — meaning the existing `perception.d435_link_to_optical` value (calibrated for the color optical frame) needs no second copy or new manual calibration for IR mode; callers just compose `T_opt_to_d435 @ frame_source.ir_to_color_extrinsic` when it's not `None`. New `configs/default.yaml` keys `perception.camera.stream` (default `"color"`, unchanged behavior) and `.emitter_enabled` (default `false`); same 5 call sites as the FOV task wired through, plus a small cosmetic fix in `visualize_markers.py` (re-inflate a single-channel IR frame to BGR before drawing overlays, so axis/text annotations stay colored instead of silently degrading to grayscale). 7 new hardware-free tests: `test_realsense_source_sdk.py` (6 — infrared dry-run construction/read/intrinsics, `ir_to_color_extrinsic` is `None` in color mode and a valid orthogonal SE3 in infrared mode, unknown-stream-mode error, the column-major reshape regression test) and `test_detector.py` (1 — a genuine single-channel `(H, W)` synthetic frame, the shape the existing always-3-channel test fixture never actually exercised). 317 tests pass total. **Live-verified** (camera-only, permitted under Hard Rules without a human), and this is the strongest verification of any perception change so far: (1) a direct probe confirmed infrared+color negotiate together on the real D435i, the emitter disables successfully, and the reshaped rotation is genuinely orthogonal (`R@R.T≈I` to 1e-8, det≈1.0) with a translation (14.7mm, dominantly one axis) matching the D435i's known RGB-to-IR-left baseline; (2) **the critical cross-check** — the same real marker (id=0) detected via both the color and infrared paths in the same run agreed to **3.0mm / 1.51°** in the pelvis frame, well inside this project's existing ~5-15mm documented error budget, directly validating the new composition math including the column-major fix; (3) in that same scan, the color stream saw only marker 0 while the **infrared stream saw both marker 0 and marker 2** — a live, concrete demonstration of the wider FOV actually solving the "fit both markers in frame" problem this whole investigation started from. Worth being explicit about scope: the cross-check validates the new IR path is *consistent with* the existing, already-in-production color path, not that either is independently ground-truth perfect (`d435_link_to_optical` itself was never fully multi-point verified — see `docs/followups.md` — so this is the same bar the color path already lives with today). **Reversible by design**: every change is opt-in behind `perception.camera.stream`, defaulting to `"color"` — today's exact behavior; reverting is a one-line config edit touching zero code.

**No-target-marker hold-until-space + configurable/safety-clamped palm-force target — COMPLETE.** Two related asks: (1) when `--source marker` finds only the object marker (no target/torso marker), `sdk_pose_tracking.py` used to grasp then release immediately, giving no time to probe grip strength in the real world; (2) confirm the existing palm-press force control (close until a target force reads on the palms, `controllers/palm_force.py`) actually works, with an easy per-run override and a guarantee against commanding an unsafe force. Fixed/added together, since they're the same real-world grip-testing scenario: the no-target-marker path now **holds the grasp with the grip loop (palm-press or finger-force) still running live** — so palm-press keeps converging on/holding its target force and the contact-loss watchdog stays armed — until a human presses **SPACE** or Ctrl-C, instead of falling straight through to the guaranteed-release block. Implemented as a new non-blocking `SpacebarWatcher` (`utils/keyboard.py`, cbreak-mode terminal read via `select`/`termios`, degrades safely — no crash/hang — when stdin isn't a real TTY) rather than a blocking `input()` gate, since blocking would freeze the live press/watchdog loop for the whole hold. The per-tick grip-regulation code (palm-press depth/force-command update, contact-loss watchdog, `compute_hand_targets`+`solve_ik`, jacobian-mode press torque, `set_joint_targets`, finger-force step) was extracted out of Phase 2's loop into a shared closure `_grip_and_hold_tick` (`sdk_pose_tracking.py`) so both Phase 2 and the new hold phase run the identical safety-critical code, not two copies that could drift apart. `run_pose_tracking_sdk` gained an optional `release_watcher` param (`None` lazily constructs the real terminal watcher; tests inject a fake one, avoiding any TTY dependency). Separately, `run_pose_tracking_preview.py` gained `--target-force N`, overriding `palm_press.force_target` for that run, clamped into the configured `[force_min, force_max]` safety bounds via new `clamp_target_force()` (`controllers/palm_force.py`) — a value above `force_max` (the existing hard crush-guard ceiling) is clamped with a printed warning rather than attempted, and the effective/requested/bounds values are always printed so the operator sees the safety context every run; no effect (with a warning) if `palm_press.enabled` is false. `docs/palm_force_control.md` gained a section on both. 12 new hardware-free tests (`test_keyboard_release.py`: 4; `test_palm_press_controller.py`: 4 for `clamp_target_force`; `test_sdk_transport.py`: 2 new + 1 existing test updated to inject a fake `release_watcher`, since the new default watcher would otherwise loop forever waiting for a real keypress under pytest). 331 tests pass total. **Not yet verified live** — human-only per Hard Rules; this holds a real grasp indefinitely and actively presses toward a commanded force, so a live retest (confirm SPACE releases cleanly, confirm the palm-press loop actually converges toward `--target-force` on real hardware) is the outstanding step.

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
- **Force/torque sensor — COMPLETE.** ATI Axia80-M20 6-axis F/T sensor (RDT-over-UDP) wired into both `--record` paths. New `g1_spine_stabilization/sensors/force_torque.py` (`ForceTorqueSensor` background UDP reader, `GravityCompensator`, `ForceTorqueSource`); `MarkerSample.load_cell_wrench` split into `load_cell_wrench_raw` + `load_cell_wrench_grav_comp` (schema v2). Gravity cancellation: orientation-aware force (startup gravity reading re-projected by the torso marker), tared torque. New `force_sensor:` config; `docs/force_sensor_setup.md` for this-machine networking + cpf/cpt calibration. Degrades gracefully (None wrenches) if the sensor is unreachable. 15 new tests (207 total).
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
│   ├── sensors/                     # added F/T sensor session
│   │   ├── __init__.py
│   │   └── force_torque.py          # ForceTorqueSensor (RDT/UDP reader), GravityCompensator, ForceTorqueSource; ATI Axia80 F/T sensor for both record paths
│   ├── sim/
│   │   ├── __init__.py
│   │   ├── coppelia_utils.py        # joint handles, read/write state, box helpers
│   │   └── scene_setup.py           # import_g1_urdf()
│   └── utils/
│       ├── __init__.py
│       ├── transforms.py            # SE3 math (make_se3, slerp, smoothstep, Rx)
│       ├── config.py                # load_config()
│       └── keyboard.py              # added no-target-marker hold session; SpacebarWatcher (non-blocking space-to-release)
│
├── scripts/
│   ├── run_pose_tracking_sim.py     # main entry point (CoppeliaSim)
│   ├── run_pose_tracking_preview.py # added SDK-3/4; --source sine|marker|mock, --tracking orient-only|6dof; added SDK-6-safety: --dry-run/--no-dry-run/--net; added Data recording session: --record [PATH]; added no-target-marker hold session: --target-force N
│   ├── grasp_demo.py                # added SDK-5; bimanual hand open/close validation (--dry-run default); thin wrapper around grasp_sequence.run_grasp_phases (SDK-6-safety)
│   ├── setup_scene.py               # one-time URDF import
│   ├── verify_ik.py                 # FK + IK self-test (no sim needed)
│   ├── visualize_markers.py         # added in Session 2; --source realsense|mock, --dry-run flag
│   ├── record_human_trial.py        # added Data recording session; perception-only human-trial recorder, --source realsense|mock, --record [PATH]
│   └── sensor_test.py               # standalone ATI F/T sensor smoke test (uses sensors/force_torque.py)
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
│   ├── test_recorder.py             # added Data recording session; hardware-free tests for RunRecorder/angle_diff_rpy/angular_velocity_rotvec/default_run_path/wrench fields
│   ├── test_force_torque.py         # added F/T sensor session; 13 tests, GravityCompensator math + UDP-loopback ForceTorqueSensor
│   └── test_keyboard_release.py     # added no-target-marker hold session; 4 tests, SpacebarWatcher non-TTY degradation + pure space-detection helper
│
└── docs/
    ├── sdk_analysis.md              # SDK discovery notes (Session 0)
    ├── sdk_setup.md                 # per-machine env ledger (written in SDK-2 preflight)
    ├── ik_weight_analysis.md        # IK weight/payload brainstorm doc (SDK-5; paste into Claude chat)
    ├── gravity_comp_reference.md    # gravity-comp pipeline reference (SDK-5.5)
    ├── data_recording_conventions.md # added Data recording session; pickle schema (v2), angle_diff_rpy reference-frame convention, angular-velocity + F/T-wrench frames, marker axis convention
    ├── force_sensor_setup.md        # added F/T sensor session; this-machine networking fix + cpf/cpt calibration for the ATI Axia80
    ├── rrt_head_obstacle.md         # sphere geometry (red/true vs orange/padded), the 3 RRTPlanningError failure modes with exact trigger/message, tuning procedure for a real head
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

The transform chain is `pose_in_pelvis = T_d435_to_pelvis @ T_optical_to_d435 @ pose_in_optical`. `T_d435_to_pelvis` comes from Pinocchio FK at the locked head/waist joint configuration. `T_optical_to_d435` is a fixed rotation in config — reverted to the original standard-convention default (`xyz: [0.0, 0.0, 0.0]`, `rpy: [-1.5708, 0.0, -1.5708]` in `configs/default.yaml`) after repeated single-point fits via `scripts/calibrate_camera_extrinsics.py` were never verified against a held-out point — see "SDK-6 marker-tracking fix" above and `docs/followups.md`.

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
`T_d435_to_pelvis` is computed once from Pinocchio FK at the locked joint configuration. `T_optical_to_d435` is a fixed rotation from config — reverted to the original standard-convention default: `xyz: [0.0, 0.0, 0.0]`, `rpy: [-1.5708, 0.0, -1.5708]` in `configs/default.yaml`, after repeated single-point calibration fits were never verified against a held-out point (see `docs/followups.md`).

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
- **`T_optical_to_d435` calibration**: attempted and reverted. The original placeholder (`xyz: [0,0,0]`, `rpy: [-1.5708, 0.0, -1.5708]`) had a measured 10.06cm/9.19° error, which motivated several single-point calibration fits via `scripts/calibrate_camera_extrinsics.py`. None was ever verified against a genuinely held-out, non-collinear point, and one fit was later found to be substantially absorbing a since-fixed `marker_size` bug rather than measuring a real mounting error — once that bug was fixed, the freshly-recomputed correction landed close to the original placeholder. Current value (as of this revert): back to `xyz: [0.0, 0.0, 0.0]`, `rpy: [-1.5708, 0.0, -1.5708]` in `configs/default.yaml`. (A prior session tried changing this to `[3.14159, 0.0, 1.5708]`; that was wrong and it was reverted — unrelated to this later revert.) A real multi-point (3-4 non-collinear points, at least one held out for `--verify`) calibration is the bar for replacing this default again — see `docs/followups.md`.
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
2. **Unit tests:** `pytest tests/ -v` — 207 tests pass
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
    Each must complete and leave a `.pkl` file under `data/runs/` that unpickles with the expected schema keys (`schema_version`, `origin`, `source_label`, `tracking_mode`, `object_marker_id`, `target_marker_id`, `metadata`, `samples`). With the F/T sensor absent/disabled, `load_cell_wrench_raw`/`_grav_comp` are `None`; use `--no-force-sensor` to skip the connect attempt.
11. **(F/T sensor, once reachable — see `docs/force_sensor_setup.md`)** `python scripts/sensor_test.py` — passive network read, safe to run directly; prints live scaled wrench once the NIC has an IP on the sensor subnet and cpf/cpt are set. A recorded run then yields non-`None` `load_cell_wrench_*`, with `_grav_comp` ≈ 0 at the startup reference pose.

## When in Doubt

- Read this file first
- Check the relevant module's docstring at the top of the file
- Look at `docs/followups.md` for notes from previous sessions
- Verify with Pinocchio FK before changing anything in `trajectory/`
- Run `python scripts/verify_ik.py` — it's the standalone IK self-test
- For CoppeliaSim issues, the diagnostic pattern is: get the object handle, get its position/orientation/matrix, print all values
- For any "should I use code from the reference repo?" question — the answer is no. See Reference Codebase Policy.
