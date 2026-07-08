# G1 Spine Stabilization — Full Codebase + Technical Review

**Context.** Read-only diagnostic review requested by the user: (1) a prioritized code-health report, (2) a technical-foundations writeup of the perception & control stack suitable as the backbone of an academic paper. No code was modified. Method: read every source file, config, doc, and test-file structure directly; verified the test suite in the `g1_inspire_sdk` env — **112 tests pass** (59s). The repo is on branch `perception-replatform` with one uncommitted change (`configs/default.yaml`).

**Architecture verdict up front:** the layered design (frame source → detector → tracker → trajectory source wrappers → NLP-IK → SDK transport) is genuinely clean, boundaries match CLAUDE.md's description, and the safety discipline (dry-run defaults, confirm gates, guaranteed-release phases, interrupt propagation) is unusually good for research code. The dominant problems are **documentation drift** (CLAUDE.md is four sessions stale) and **a small number of assumed-dt / interrupt-semantics gaps of exactly the bug classes this project has already fixed twice elsewhere**.

> **Note on this document (2026-07-08 revision).** Section 2 was substantially expanded into a standalone technical reference: every load-bearing equation now carries a derivation of *why it takes that form*, an assumptions/failure-mode register, and a **Concepts & further reading** box that names the underlying field, the canonical textbook/paper, and search terms for your own deep dives. A consolidated concept index + reading list is added as Section 3. All LaTeX math was sanitized to ASCII (a stray Unicode combining-tilde inside a `$$` block was breaking the KaTeX render). The math uses `$…$`/`$$…$$` and renders in GitHub, Obsidian, Typora, and VS Code with a KaTeX/Markdown-Math extension.

---

# Section 1 — Codebase Review

Prioritized, most consequential first. "P" numbers are for reference in follow-up discussion.

## P1. CLAUDE.md is four sessions stale and internally contradictory (doc drift, highest leverage)

CLAUDE.md instructs every session to "read it fully at the start" — so wrong facts here propagate into every future session. It has already caused this once (the stale calibration value corrected in the marker-tracking-fix session). Current drift:

- **Test count:** headline says "51 tests pass"; the latest status entry says 96; actual is **112 passing** (verified this session).
- **Missing sessions:** the Current Status / Session Roadmap stop at "SDK-6 marker-tracking fix". `docs/followups.md` documents four newer completed sessions: **Ctrl-C interrupt-swallowing fix**, **light grip during pose tracking** (Phase 1.5 / guaranteed-release in `sdk_pose_tracking.py`), **d435_link_to_optical calibration** (resolving the ~10 cm discrepancy), and **box_half_height correction**.
- **File structure missing:** [calibration.py](g1_spine_stabilization/perception/calibration.py), [calibrate_camera_extrinsics.py](scripts/calibrate_camera_extrinsics.py), [test_calibration.py](tests/test_calibration.py), [test_ik_interrupt_handling.py](tests/test_ik_interrupt_handling.py).
- **Stale calibration claim (again):** CLAUDE.md states `T_optical_to_d435` is "hand-calibrated: `rpy: [-1.5708, 0.0, -1.5708]`" in ~4 places and that the ~10 cm marker-height discrepancy is "still unresolved". The config now holds the solved correction `xyz: [-0.0860, -0.0002, -0.0384]`, `rpy: [-1.4118, 0.0076, -1.5896]` ([default.yaml:77-79](configs/default.yaml#L77-L79)), verified to 6 mm against a second marker. This is the second time this exact doc field has drifted.
- CLAUDE.md still says grip exists only in `grasp_demo.py` ("Session 4 will replace this with actual finger control") — grip is now wired into Phase 2 of tracking.

**Recommendation:** one doc-sync pass of CLAUDE.md (status, roadmap, file table, calibration value, test count) before the next hardware session.

## P2. Uncommitted `configs/default.yaml` mixes tentative calibration with grip-tuning experiments

The working-tree diff changes three unrelated things at once: box size `[0.16]³ → [0.10, 0.11, 0.2]`, grip force weakened (`tau_baseline` 0.25→0.2, `delta_q_max` 1.0→0.2, `grasp_pose` `[0.9…]→[0.3…]`), and leaves the old grip block **commented out** at [default.yaml:117-124](configs/default.yaml#L117-L124). The calibrated extrinsics (P1) are also only in this file, marked TENTATIVE. Risks: (a) hardware-validated values live only in an uncommitted file; (b) experiments get silently baked in; (c) tests read this live file (`load_config()`), so tuning can silently change test behavior (current grip assertions are robust to it, but that's luck, not design). **Recommendation:** commit the calibration with its rationale, decide the grip values deliberately, delete the commented block.

## P3. Ctrl-C during Phase 1.5 (hand close) falls through into Phase 2 tracking

[sdk_pose_tracking.py:286-287](g1_spine_stabilization/g1_spine_stabilization/controllers/sdk_pose_tracking.py#L286-L287): the `except KeyboardInterrupt` around the hand-close prints "exiting hand-close early" and then execution **continues into Phase 2** — in marker mode, an infinite tracking loop. An operator Ctrl-C'ing during hand close almost certainly intends to abort toward the safe return, not to begin tracking with partially-closed hands (and since `arm()` may not have been called, the grip `step()` fallback keeps commanding `close_pose`, so the hands finish closing during tracking). Contrast: a Phase-1 interrupt correctly skips Phase 2, and `grasp_sequence.py` skips to release from any phase. This is the one remaining interrupt-path inconsistency after the SDK-6-safety and Ctrl-C-swallowing sessions. **Safety-adjacent; fix before live bringup.**

## P4. Marker-staleness timers use assumed `control_dt`, not wall clock — warnings fire 5–9× late

[sources.py:244](g1_spine_stabilization/g1_spine_stabilization/trajectory/sources.py#L244) and [sources.py:261](g1_spine_stabilization/g1_spine_stabilization/trajectory/sources.py#L261): `stale_s = stale_frames * control_dt` with `control_dt=0.02`, but the measured loop period is ~100–180 ms (documented "Known timing floor"). So the "target not seen for 0.5 s" warning actually fires after ~2.5–5 s of wall time, and `object_lost` (nominally 2 s) after ~10–18 s. This is precisely the assumed-dt-vs-measured-dt bug class already fixed in `RateLimitedSource` and documented (unfixable) in the SDK's `_motor_dt` — but this instance is in-repo and fixable (timestamp the last detection with `time.monotonic()`). **Safety-adjacent: these timers are the box-may-have-fallen monitors.**

## P5. `object_lost` only prints; the "stop here" policy is a TODO in disguise

[sdk_pose_tracking.py:303-305](g1_spine_stabilization/g1_spine_stabilization/controllers/sdk_pose_tracking.py#L303-L305): `"WARNING: object marker lost — box may have fallen! (SDK-6: stop here)"` — the comment names the intended behavior (stop) but the code only warns once and keeps tracking on last-known values indefinitely. Combined with P4 (the trigger itself is 5–9× late), the failure response to a dropped object is currently: nothing. Decide and implement the policy (hold pose / initiate Phase 3) before live marker runs.

## P6. `MarkerSource.startup()` captures its reference poses from a single unfiltered frame

[sources.py:157-161](g1_spine_stabilization/g1_spine_stabilization/trajectory/sources.py#L157-L161) (object) and [sources.py:180-181](g1_spine_stabilization/g1_spine_stabilization/trajectory/sources.py#L180-L181) (target): `_object_start_pos`, `_target_start_pos`, `_target_start_rot` come from the **first frame** in which each marker is detected. The noise-filter wrapper doesn't apply here (its `startup()` just resets state), so single-frame solvePnP noise — including an IPPE ambiguity flip — becomes a **permanent bias** on `box_center` and a permanent θ offset for the whole session. The calibration script already solved this pattern (averages 5 s of frames, [calibrate_camera_extrinsics.py:56-78](scripts/calibrate_camera_extrinsics.py#L56-L78)); reuse it: average N frames in Phases A/B. Since delta tracking makes the whole session relative to these references, this is the single highest-variance term in the error budget that is cheap to remove.

## P7. SDK control loop feeds back commanded state, never measured state (undocumented design decision)

[sdk_pose_tracking.py:257](g1_spine_stabilization/g1_spine_stabilization/controllers/sdk_pose_tracking.py#L257), [:311](g1_spine_stabilization/g1_spine_stabilization/controllers/sdk_pose_tracking.py#L311), [:386](g1_spine_stabilization/g1_spine_stabilization/controllers/sdk_pose_tracking.py#L386) and [grasp_sequence.py:173](g1_spine_stabilization/g1_spine_stabilization/controllers/grasp_sequence.py#L173), [:293](g1_spine_stabilization/g1_spine_stabilization/controllers/grasp_sequence.py#L293): after the initial `_get_q(robot)`, every iteration sets `current_q = sol_q` — the loop is open-loop w.r.t. the real arm (warm start, smoothness cost, and Phase-3 FK all use the last *commanded, filtered* q). The CoppeliaSim path reads measured state every step ([pose_tracking.py:169](g1_spine_stabilization/g1_spine_stabilization/controllers/pose_tracking.py#L169)). This is defensible (measured q is noisy; the SDK velocity-clips so commanded ≈ achieved at these speeds; avoids feedback of tracking error into the smooth cost) — but it is **nowhere written down**, and it means (a) the controller cannot notice commanded/actual divergence under load, and (b) Phase 3 interpolates from where the arm was told to be, not where it is — the same class of discontinuity as the twice-fixed startup jerk, bounded by PD tracking error. Document the decision, or periodically re-seed from measured q with a divergence check.

## P8. Three near-copies of the three-phase interpolation loop — and it has already bitten

`pose_tracking.py` (Phases 1–3), `sdk_pose_tracking.py` (Phases 1–3), `grasp_sequence.py` (Phases 0/R) each hand-roll the same smoothstep-interpolate → solve_ik → command loop with slightly different conventions. The cost is documented history: the FK-start fix was applied in `grasp_demo` (SDK-5) but "simply never ported" to `sdk_pose_tracking.py` until a live robot jerked (SDK-6 followups). Same story risk now for P3's interrupt semantics. A single `interpolate_ee_to(robot, ik, targets, cfg)` helper in `controllers/` would collapse ~6 loops into one audited implementation. Related duplication: `ARM_DEFAULT_Q` defined identically in [pose_tracking.py:35](g1_spine_stabilization/g1_spine_stabilization/controllers/pose_tracking.py#L35) and [sdk_pose_tracking.py:66](g1_spine_stabilization/g1_spine_stabilization/controllers/sdk_pose_tracking.py#L66) (and re-pasted in 4 test files); `_load_config` in pose_tracking.py duplicates `utils/config.load_config`; the 6-line CameraIntrinsics-from-native block appears 4× across scripts.

## P9. `grasp_sequence.py` violates the lazy-import hard rule

[grasp_sequence.py:32-34](g1_spine_stabilization/g1_spine_stabilization/controllers/grasp_sequence.py#L32-L34): `import rerun as rr` and `from g1_inspire_sdk import HAND_SOFT_LOWER_LIMITS` at module top. CLAUDE.md's convention (and `sdk_pose_tracking.py`'s own guarded `try: import rerun`) is that hardware/SDK deps are lazy so the package imports on machines without them. Consequence: this module (and its tests) hard-require `rerun-sdk` + the SDK; `requirements.txt` declares neither. Either guard the imports like its sibling module or add the deps to the documented stack.

## P10. Silent, inconsistent state-read fallbacks in `_get_q`

[sdk_pose_tracking.py:72-81](g1_spine_stabilization/g1_spine_stabilization/controllers/sdk_pose_tracking.py#L72-L81) falls back to `ARM_DEFAULT_Q`; [grasp_sequence.py:55-64](g1_spine_stabilization/g1_spine_stabilization/controllers/grasp_sequence.py#L55-L64) falls back to `np.zeros(14)`; both via bare `except Exception: pass` with no log. On hardware, a transient DDS state-read failure at Phase-1 start silently seeds the interpolation from a constant pose — re-creating the interpolate-from-wrong-start jerk that two sessions were spent fixing, and doing it silently. Minimum fix: print a loud warning on fallback; better: retry, then abort (a robot whose state can't be read shouldn't start a motion phase). Also unify the fallback constant.

## P11. Latent crash: `(ctrl_l or ctrl_r).close_pose` with both None

[grasp_sequence.py:134](g1_spine_stabilization/g1_spine_stabilization/controllers/grasp_sequence.py#L134): `run_grasp_phases` accepts `ctrl_l=None, ctrl_r=None` (both are keyword args) but dereferences `(ctrl_l or ctrl_r).close_pose` unconditionally → `AttributeError` before any motion. `grasp_demo.py`'s `--side` choices guarantee at least one today, so it's latent — but the failure would occur *before* the try block's safe-return machinery is relevant, and `grasp_demo.py` also ignores `grip_force.enabled` (constructs controllers unconditionally), so enabling that flag path would hit this. Add an early guard/assertion.

## P12. Dead code inventory

| Item | Location | Notes |
|---|---|---|
| `VideoFileSource`, `ImageFileSource` | [frame_sources.py:9-43](g1_spine_stabilization/g1_spine_stabilization/perception/frame_sources.py#L9-L43) | Never used anywhere, not even tests. Keep only if replay-from-video is still planned. |
| `detach_box` | [coppelia_utils.py:87-90](g1_spine_stabilization/g1_spine_stabilization/sim/coppelia_utils.py#L87-L90) | Never called; was "kept for Session 4", which became SDK-5 and doesn't use it. |
| `current_dq` parameter of `solve_ik` | [ik_solver.py:455-456](g1_spine_stabilization/g1_spine_stabilization/kinematics/ik_solver.py#L455-L456) | `v = current_dq * 0.0` — always zero; every caller passes `np.zeros(14)`. Misleading API surface (implies velocity affects the solve). Drop or document. |
| `MarkerTracker.get_latest` / mock equivalents | [tracker.py:60-61](g1_spine_stabilization/g1_spine_stabilization/perception/tracker.py#L60-L61) | Only tests call it; `MarkerSource` uses `step()` exclusively (docstring still advertises `get_latest` as part of the contract). |
| `target_stale_seconds` property | [sources.py:281-284](g1_spine_stabilization/g1_spine_stabilization/trajectory/sources.py#L281-L284) | Test-only consumer. |
| `GraspController.step(tau_des=…)` | [grip_force.py:79](g1_spine_stabilization/g1_spine_stabilization/controllers/grip_force.py#L79) | Deliberate unused extension point (documented) — fine, listing for completeness. |
| Commented-out `grip_force` block | [default.yaml:117-124](configs/default.yaml#L117-L124) | Uncommitted; delete when P2 is resolved. |
| Stray blank-line edit artifact | [visualize_markers.py:72-75](scripts/visualize_markers.py#L72-L75) | Cosmetic. |
| `_run_detect` never closes the camera | [run_pose_tracking_preview.py:108-146](scripts/run_pose_tracking_preview.py#L108-L146) | Resource leak on the `--detect` path (process exits right after, so benign, but inconsistent with every other camera user). |

## P13. Direction-naming inversion on the calibrated transform

The YAML key is `perception.d435_link_to_optical`; the function that consumes it is `optical_to_d435_from_config` ([frames.py:12](g1_spine_stabilization/g1_spine_stabilization/perception/frames.py#L12)); the value is used as `T_d435←optical` (it maps optical-frame poses into the d435 mount frame: `T_d435_to_pelvis @ T_optical_to_d435 @ pose_in_optical`). So the *function* name matches the math and the *config key* reads backwards. Given this exact transform's history (a session applied a wrong value; the 10 cm bug; "TENTATIVE" flag), the inconsistent naming is a live hazard for the next person who edits the YAML. Rename the key or put the direction convention in a comment at the key.

## P14. Config defaults duplicated in code at every call site

`settle_time=0.4`, `tauff_filter` defaults, rate-limit and noise-filter defaults are each re-specified via `.get(..., default)` in 2–4 places (e.g. [sdk_pose_tracking.py:397](g1_spine_stabilization/g1_spine_stabilization/controllers/sdk_pose_tracking.py#L397), [grasp_sequence.py:305](g1_spine_stabilization/g1_spine_stabilization/controllers/grasp_sequence.py#L305), [run_pose_tracking_preview.py:306-322](scripts/run_pose_tracking_preview.py#L306-L322)). Changing a YAML default doesn't change the code fallbacks, and vice versa. A tiny schema/defaults layer in `utils/config.py` would remove the class. Also one factual comment error: [default.yaml:17](configs/default.yaml#L17) claims the 60 °/s rate limit is ">6×" the sine's 18.8 °/s peak rate — it's 3.2×.

## P15. Packaging only works as an editable install; README is stale

`setup.py` has no `package_data`, and both `load_config` and the URDF path resolve via `__file__/../..` — a non-editable `pip install .` produces a package that can't find `configs/` or `assets/`. Fine for research use, but worth one README line ("editable install only"). Separately, [README.md](README.md) still describes the project as CoppeliaSim-only with "ArUco-based perception … planned" — three eras out of date (perception, SDK transport, gravity comp, grip all exist). `requirements.txt` omits `rerun-sdk` (see P9).

## P16. Test coverage: strong overall, three gaps worth knowing

112 tests, all hardware-free, well-targeted at past regressions (jerk fix, filter recency, interrupt handling, safe-release). Gaps: (a) the **filter composition chain** `RateLimitedSource(NoiseFilteredSource(MarkerSource))` is asserted only by construction order in the preview script — no test exercises the composed stack; (b) `GraspController` has no dedicated unit tests (integration-only via `test_sdk_transport`; acknowledged in followups); (c) the CoppeliaSim controller is untested (accepted — needs a sim). Also note tests consume the live `configs/default.yaml` (see P2 coupling).

## Security review (defensive framing)

No meaningful software attack surface: no servers, no secrets, `yaml.safe_load` throughout, no `eval`/`pickle`/subprocess-from-input. The real "security" surface here is **robot actuation safety**, which is handled well: dry-run defaults with explicit `--no-dry-run`, interactive confirmation gates before live connect ([run_pose_tracking_preview.py:197-204](scripts/run_pose_tracking_preview.py#L197-L204)), guaranteed release/return phases, interrupt propagation through the solver. Residual items are P3/P4/P5 above. One nit: `--net` defaults to `eth0`, so a live run that forgets `--net` targets the default interface — the confirm gate mitigates; consider making `--net` required when `--no-dry-run` is passed.

## Things that would bite later (brittle assumptions, no action needed yet)

- **θ wrap-around:** median and EMA on θ ∈ (−π, π] are not circular-safe; a target rotating through ±π produces filter transients. Safe at the current ±30° amplitude; document the limit.
- **θ axis assumption:** `_theta_from_target_rotation` extracts the roll of the delta rotation *in the target's starting frame* and applies it as pelvis-frame Rx. Correct only while the target's starting X axis ≈ pelvis X (markers placed near-identity). A target started 90° yawed would rotate the box about the wrong axis. (Full statement in §2.3 below.)
- **Effective filter bandwidth floats with loop rate:** all filters are per-call; their time constants in seconds scale with the ~100–180 ms (variable) solve time. Retuning IPOPT later will silently retune every filter.
- **Locked head/waist:** `T_d435_to_pelvis` computed once; any future head motion invalidates every marker pose (documented in CLAUDE.md, restated here because the constant is captured at script start).
- **GIL contention** stalling the SDK's 250 Hz publisher during each solve — documented, deferred, unfixable in-repo; it caps how far the current single-process architecture can go.
- **IK solve cost (~100 ms)** is the system's rate limit; everything downstream (P4 timers, filter tuning, rate limits) is shaped around it. The deferred solve-speed work is the highest-leverage future performance item.

---

# Section 2 — Technical Foundations

This section is written to be read twice: once linearly as an explanation of the whole pipeline, and once as a reference where each equation is justified from first principles. Every subsection ends with a **Concepts & further reading** box that names the field the method comes from, the canonical text or paper, and the exact search terms to use. Citations are to standard, well-known references; treat page-level claims as pointers to verify, not gospel.

**How the mathematical objects are written.** $R \in SO(3)$ is a $3\times 3$ rotation matrix; $T \in SE(3)$ is a $4\times 4$ homogeneous transform $T = \begin{bmatrix} R & t \\ 0 & 1\end{bmatrix}$; $q \in \mathbb{R}^{14}$ is the stacked joint vector of both 7-DOF arms; $e_i$ is the $i$-th standard basis vector. "Pelvis frame" is the world/inertial frame for everything (the robot base is bolted in place). A superscript/subscript pair $T^{A}_{B}$ denotes "the pose of frame $B$ expressed in frame $A$," equivalently the map that takes a point's $B$-coordinates to its $A$-coordinates.

### 2.0 System overview: what kind of system this is

The pipeline is a **classical (model-based, non-learned) perception-to-control cascade**. Nothing here is a neural network and nothing is trained; every block is an interpretable geometric or signal-processing operation whose parameters were set by hand or by a one-shot calibration. Per control step:

```
RGB frame ─► AprilTag corner detection ─► planar PnP per marker            [perception/detector.py]
          ─► SE(3) frame composition into the pelvis frame                 [perception/frames.py, tracker.py]
          ─► displacement reference (box center c_t, roll theta_t)         [trajectory/sources.py: MarkerSource]
          ─► median-of-N + EMA noise rejection                            [NoiseFilteredSource]
          ─► slew-rate limiting vs measured wall-clock dt                  [RateLimitedSource]
          ─► antipodal grasp map (c_t, theta_t) -> (T_L*, T_R*)            [trajectory/targets.py]
          ─► whole-arm IK as a 14-DOF nonlinear program (Pinocchio+IPOPT)  [kinematics/ik_solver.py]
          ─► weighted moving average on the joint solution q               [WeightedMovingFilter]
          ─► gravity feedforward tau_ff = g(q) via RNEA, then EMA          [ik_solver.py]
          ─► joint targets to a 250 Hz PD motor loop: tau = tau_ff+Kp e-Kd qdot   [vendored SDK]
```

A supervisory **three-phase state machine** (interpolate-to-grasp → track → interpolate-home) wraps the loop, with a force-feedback grip controller on the fingers during tracking.

The two design commitments that shape everything else: (1) there is **no probabilistic state estimator** (no Kalman/particle filter) — state estimation is replaced by a deterministic cascade of robust filters (§2.5), a defensible choice given a cooperative fiducial target and no motion model for a human hand; and (2) the controller is **kinematic, not dynamic** — it solves for *where* the joints should be (inverse kinematics) and lets a low-level PD loop plus a gravity feedforward term handle *how much torque* to apply (§2.6–2.7). It is a position-tracking controller with quasi-static torque compensation, not a torque/impedance controller.

> **Concepts & further reading (system architecture).**
> - **Sense-plan-act / perception-action loop**; **model-based vs. learning-based control**; **cascaded (modular) autonomy stacks**. Search: "modular robotics autonomy stack," "classical vs end-to-end robot control."
> - Overview texts spanning this whole stack: **Siciliano & Khatib (eds.), *Springer Handbook of Robotics*, 2nd ed.**; **Corke, *Robotics, Vision and Control*** (a single book covering the camera model, kinematics, and control used below — the closest one-volume match to this codebase); **Lynch & Park, *Modern Robotics*** (kinematics, dynamics, control; free PDF + videos).

---

### 2.1 Camera model and fiducial pose recovery (PnP)

**What the code computes.** For each detected AprilTag, recover its 6-DOF pose $(R, t)$ in the camera **optical frame** from the four detected corner pixels and the known physical marker size.

**The projection model.** A pinhole camera maps a 3-D point $X = (X_1,X_2,X_3)^\top$ (in the marker frame) to a pixel $(u,v)$ via a rigid transform then a projective division:

$$ s \begin{bmatrix} u \\ v \\ 1 \end{bmatrix} = K\,(R\,X + t), \qquad K = \begin{bmatrix} f_x & 0 & c_x \\ 0 & f_y & c_y \\ 0 & 0 & 1 \end{bmatrix}. $$

Here $K$ is the **intrinsic matrix** ($f_x,f_y$ focal lengths in pixels, $(c_x,c_y)$ the principal point), $(R,t) \in SE(3)$ is the marker's pose in the optical frame (the **extrinsics** for this observation), and $s>0$ is the unknown projective depth eliminated by the division $u = f_x (RX+t)_1/(RX+t)_3 + c_x$, etc. The optical frame convention is **X right, Y down, Z into the scene** (OpenCV / REP-103). Intrinsics are read from the RealSense at runtime ([frame_sources.py:59-66](g1_spine_stabilization/g1_spine_stabilization/perception/frame_sources.py#L59-L66)); the YAML `fx=fy=615, cx=320, cy=240` are placeholders only.

**Lens distortion.** Real lenses bend rays; before the linear model applies, normalized coordinates are warped by a **radial + tangential (Brown–Conrady / "plumb-bob") distortion** polynomial parameterized by $(k_1,k_2,p_1,p_2,k_3)$. The code passes the camera's coefficients straight into `cv2.solvePnP`/`cv2.projectPoints` ([detector.py:70-77,86](g1_spine_stabilization/g1_spine_stabilization/perception/detector.py#L70-L77)); residual mis-modeling is absorbed later into the extrinsic calibration (§2.4).

**Pose recovery = Perspective-n-Point (PnP).** The marker's four corners are known 3-D points in the marker frame, coplanar with $Z=0$ ([detector.py:50-56](g1_spine_stabilization/g1_spine_stabilization/perception/detector.py#L50-L56)):

$$ X_i \in \Bigl\{ (\pm \tfrac{s}{2}, \pm \tfrac{s}{2}, 0) \Bigr\}, \quad i=1..4, \qquad s = \text{marker side length (0.04 m)}. $$

Given the four image corners $u_i$, PnP finds the pose minimizing the **reprojection error** (sum of squared pixel residuals):

$$ (R^*, t^*) = \arg\min_{R \in SO(3),\, t \in \mathbb{R}^3} \ \sum_{i=1}^{4} \bigl\lVert u_i - \pi\!\left(K(R X_i + t)\right) \bigr\rVert^2, $$

where $\pi(\cdot)$ is the perspective division above. The code calls `cv2.solvePnP(..., flags=SOLVEPNP_IPPE_SQUARE)` ([detector.py:74-77](g1_spine_stabilization/g1_spine_stabilization/perception/detector.py#L74-L77)) — the **IPPE (Infinitesimal Plane-based Pose Estimation)** algorithm specialized to a square planar target. Because all four points are coplanar, the point-to-image relationship is a **homography** $H$ ($u_i \simeq H X_i$ in marker-plane coordinates); IPPE decomposes $H$ into $(R,t)$ analytically (no iterative optimization), which is why it is fast and deterministic. The pose is then converted from the axis-angle vector `rvec` to a matrix by **Rodrigues' formula** ([detector.py:80](g1_spine_stabilization/g1_spine_stabilization/perception/detector.py#L80)):

$$ R = I + \sin\phi\,[\hat a]_\times + (1-\cos\phi)\,[\hat a]_\times^2, \qquad \text{rvec} = \phi\,\hat a, $$

with $\phi$ the rotation angle, $\hat a$ the unit axis, and $[\hat a]_\times$ the skew-symmetric cross-product matrix. The mean per-corner reprojection error is kept as a quality signal ([detector.py:86-87](g1_spine_stabilization/g1_spine_stabilization/perception/detector.py#L86-L87)).

**Why this form / why these choices.**
- *Reprojection error, not 3-D error*, is minimized because the measurement noise lives in pixels (corner localization), so least-squares in the image is the (approximate) maximum-likelihood estimate under isotropic Gaussian pixel noise.
- *IPPE_SQUARE over the generic iterative PnP*: for a planar square the general PnP is overkill and the planar case has a clean closed form; IPPE is designed exactly for this and returns the better of the two planar solutions ranked by reprojection error.
- *AprilTag 36h11*: the "36h11" family encodes a 36-bit payload with a minimum Hamming distance of 11 between valid codes, giving strong false-positive rejection and bit-error correction — the reason this marker family is the robotics default for pose from a single tag.

**Assumptions and failure modes (state these to a reviewer).**
1. **Marker side length is exact.** $t$ scales linearly with the assumed $s$; a print-scale error is a pure depth-scale error, indistinguishable at a single distance from a translational mounting offset (this exact confound motivates the two-distance calibration check in §2.4).
2. **Planar two-fold ambiguity.** A planar target admits two poses with nearly equal reprojection error when viewed near-frontally; single-frame pose "flips" are expected physics, not a bug. This is the concrete justification for the **median** stage of the filter (§2.5) — a linear filter would blend a flipped pose in.
3. **Corner localization noise** dominates at long range / small pixel footprint; error in $t$ grows roughly with depth² for a fixed pixel error.
4. Distortion coefficients are trusted from the camera; any residual is folded into §2.4.

> **Concepts & further reading (computer vision / geometry).**
> - **Pinhole camera model, intrinsics/extrinsics, homogeneous & projective coordinates:** Hartley & Zisserman, *Multiple View Geometry in Computer Vision* (the standard reference); Szeliski, *Computer Vision: Algorithms and Applications* (free online); Ma, Soatto, Košecká & Sastry, *An Invitation to 3-D Vision*.
> - **Lens distortion (Brown–Conrady / plumb-bob), camera calibration (Zhang's method):** search "Zhang camera calibration 2000," "radial tangential distortion model," OpenCV `calibrateCamera`.
> - **PnP problem:** search "Perspective-n-Point," "P3P," "EPnP," "planar pose estimation." IPPE specifically: **Collins & Bartoli, "Infinitesimal Plane-Based Pose Estimation," IJCV 2014.** Homography decomposition: search "homography pose decomposition Malis Vargas."
> - **Robust fitting for outliers:** RANSAC (Fischler & Bolles 1981) — not used here (only 4 points) but the standard tool when there are many.
> - **Rodrigues' formula / axis-angle:** any of the robotics texts in §2.6; also the "micro Lie theory" paper in §2.6's box.
> - **Fiducial markers:** Olson, "AprilTag," ICRA 2011; Wang & Olson, "AprilTag 2," IROS 2016; Garrido-Jurado et al., "ArUco," Pattern Recognition 2014. Search "lexicode Hamming distance fiducial," "36h11."

---

### 2.2 Rigid-body frame composition (the transform chain)

**What the code computes.** Re-express each marker's optical-frame pose in the pelvis frame by composing three rigid transforms ([frames.py:51-57](g1_spine_stabilization/g1_spine_stabilization/perception/frames.py#L51-L57), staged explicitly in [tracker.py:46-49](g1_spine_stabilization/g1_spine_stabilization/perception/tracker.py#L46-L49)):

$$ T^{pelvis}_{marker} = T^{pelvis}_{d435} \cdot T^{d435}_{opt} \cdot T^{opt}_{marker}. $$

**Why this form.** Rigid transforms compose by matrix multiplication, and they compose **right-to-left** along a chain of frames: read the product as "a point known in the marker frame → optical frame → d435 mount frame → pelvis frame." This is the group operation of $SE(3)$; associativity is what lets the code precompute the constant left factor $T^{pelvis}_{d435}\,T^{d435}_{opt}$ once and reuse it every frame. Each factor:

- **$T^{pelvis}_{d435}$** — the camera mount pose from **forward kinematics** of the URDF at the locked joint configuration (head and waist joints held at zero), computed once via Pinocchio ([frames.py:24-48](g1_spine_stabilization/g1_spine_stabilization/perception/frames.py#L24-L48)). Forward kinematics chains the fixed and revolute joint transforms from the pelvis root out to the `d435_link` frame.
- **$T^{d435}_{opt}$** — the fixed mount→optical extrinsic, stored as a translation `xyz` plus a fixed-axis roll-pitch-yaw triple `rpy`, assembled with the **URDF/ROS convention** $R = R_z(\text{yaw})\,R_y(\text{pitch})\,R_x(\text{roll})$ ([frames.py:12-21](g1_spine_stabilization/g1_spine_stabilization/perception/frames.py#L12-L21)). Nominally this is the canonical ROS body→optical rotation $rpy = (-\pi/2, 0, -\pi/2)$; the calibrated value (§2.4) additionally absorbs the real mounting error.
- **$T^{opt}_{marker}$** — the per-frame PnP output from §2.1.

**Euler-angle convention — the part that bites people.** "rpy" here is **extrinsic** (fixed-axis): the three elemental rotations are about the *fixed* pelvis-frame axes, applied roll then pitch then yaw, which composes as $R_z R_y R_x$ (yaw outermost). The code builds it with `scipy Rotation.from_euler("ZYX", [yaw, pitch, roll])` — note "ZYX" with the angles reversed is scipy's way of spelling the same fixed-axis $R_z R_y R_x$. Getting this convention wrong is a classic silent bug; §2.4's `se3_to_xyz_rpy` is its exact inverse so a round-trip is consistent.

**Assumptions and failure modes.**
1. **Head/waist locked ⇒ constant camera extrinsic.** $T^{pelvis}_{d435}$ is computed once and cached. If the head ever moves, this factor must move *inside* the per-frame loop or every marker pose is wrong. (Critical Assumption 2 in CLAUDE.md.)
2. **Pelvis is the inertial frame** (robot bolted down). If the base moved, this whole frame would need a floating-base estimate.
3. Euler singularities ("gimbal lock") are irrelevant here because rpy is only a storage format for a fixed calibrated rotation, never integrated.

> **Concepts & further reading (rigid-body kinematics).**
> - **$SE(3)$/$SO(3)$, homogeneous transforms, transform composition, frame trees:** Lynch & Park, *Modern Robotics*, ch. 3; Craig, *Introduction to Robotics*, ch. 2; Murray, Li & Sastry, *A Mathematical Introduction to Robotic Manipulation*, ch. 2. Search "homogeneous transformation matrix," "coordinate frame composition robotics."
> - **Forward kinematics / kinematic trees / URDF:** Lynch & Park ch. 4; the ROS `tf2` library docs (the industrial version of exactly this transform chain); Pinocchio docs (**Carpentier et al., "The Pinocchio C++ library," SII 2019**).
> - **Euler angle conventions (intrinsic vs extrinsic, RPY, ZYX), gimbal lock:** search "Euler angle conventions robotics," "fixed-axis vs Euler-axis rotation," scipy `Rotation.from_euler` docs. ROS frame conventions: **REP-103** (units/axes) and **REP-105** (frames).

---

### 2.3 Reference generation: displacement (relative) tracking

**What the code computes.** Convert two marker streams into the controller's reference: a box center $c_t \in \mathbb{R}^3$ and a scalar roll angle $\theta_t$ about the pelvis X-axis. The key idea is that **both are displacements from a start pose, not absolute poses** ([sources.py:129-136](g1_spine_stabilization/g1_spine_stabilization/trajectory/sources.py#L129-L136), [:236-238](g1_spine_stabilization/g1_spine_stabilization/trajectory/sources.py#L236-L238)).

**Startup references** (captured once):

$$ c_{obj,0} = p_{mk}(0) - \tfrac{h}{2}\,R_{mk}(0)\,e_3, \qquad \bigl(p_{tgt}(0),\, R_{tgt}(0)\bigr). $$

$p_{mk},R_{mk}$ are the object marker's measured pelvis-frame position/rotation; $h$ is the box height; $e_3=(0,0,1)^\top$. The subtraction moves from the marker (mounted on the box's **top** face) down to the box's geometric center, **along the marker's own measured out-of-plane axis** $R_{mk}(0)\,e_3$ rather than assuming the box sits level — so it stays correct for a tilted box ([sources.py:157-161](g1_spine_stabilization/g1_spine_stabilization/trajectory/sources.py#L157-L161)).

**Per-step reference:**

$$ R_\Delta(t) = R_{tgt}(0)^\top\,R_{tgt}(t), \qquad \theta_t = \operatorname{atan2}\bigl([R_\Delta]_{32},\ [R_\Delta]_{33}\bigr), $$
$$ c_t = c_{obj,0} + \bigl(p_{tgt}(t) - p_{tgt}(0)\bigr) \quad\text{(6dof mode; orient-only holds } c_t = c_{obj,0}\text{).} $$

$R_\Delta(t)$ is the target's rotation **relative to its own start orientation**; $\theta_t$ is the roll (X-component) extracted from it; $c_t$ is the object's start center shifted by the target's **translational displacement**.

**Why this form (three distinct "why"s a reviewer will ask).**
1. **Why relative, not absolute?** Absolute servoing was the original code and was a real bug: it drove the hands to wherever the target physically was, even though the object is held in the hands and the target is held by a person somewhere else. Tracking the *displacement* decouples the two objects' physical locations. It also **cancels common-mode calibration bias**: any constant extrinsic error that corrupts both $p_{tgt}(t)$ and $p_{tgt}(0)$ subtracts out of $p_{tgt}(t)-p_{tgt}(0)$ (the same principle as differential/ratiometric measurement in instrumentation).
2. **Why $R_\Delta = R_{tgt}(0)^\top R_{tgt}(t)$ and not $R_{tgt}(t) R_{tgt}(0)^\top$?** Left-multiplying by the inverse-start gives the rotation **expressed in the start body frame** (a *body-frame* / intrinsic delta); the other order gives it in the world frame. The code wants "how far has the target turned from where it began, in its own frame," so the body-frame delta is correct. This is a **change of basis for rotations** (a similarity transform / the adjoint action): the same physical rotation looks like $R$ in one frame and $C R C^\top$ in a frame related by $C$.
3. **Why `atan2` on the (3,2),(3,3) entries?** For a pure rotation about X, $R_x(\theta) = \begin{bmatrix}1&0&0\\0&\cos\theta&-\sin\theta\\0&\sin\theta&\cos\theta\end{bmatrix}$, so $[R]_{32}=\sin\theta$, $[R]_{33}=\cos\theta$, and $\operatorname{atan2}(\sin\theta,\cos\theta)=\theta$ recovers the signed angle on the full circle $(-\pi,\pi]$ without the quadrant loss of $\arcsin$/$\arccos$. This is exactly the roll component of an XYZ Euler decomposition.

**Assumptions and where they break.**
1. **Single-axis (pelvis-X) rotation.** $\theta$ is extracted as the roll of $R_\Delta$ *in the target's starting body frame*, then re-applied downstream as $R_x(\theta)$ in the *pelvis frame* ([targets.py:56](g1_spine_stabilization/g1_spine_stabilization/trajectory/targets.py#L56)). These two "X axes" coincide only when $R_{tgt}(0) \approx I$ (marker placed axis-aligned to the pelvis at start). The physical protocol enforces "place markers near identity." A target *started* significantly yawed would command rotation about the wrong axis — the precise statement is that the commanded axis is $R_{tgt}(0)\,\hat x$ mapped through the roll extraction, which reduces to $\hat x$ only at identity. This is the strongest hidden assumption in the whole reference path.
2. **$\theta$ is treated as a real number by the downstream filters (§2.5),** which are not circular. Operation must stay away from the $\pm\pi$ wrap; safe at the current $\pm 30^\circ$ amplitude.
3. **Single-frame start capture** (finding P6): $c_{obj,0}$, $p_{tgt}(0)$, $R_{tgt}(0)$ come from one frame, so all subsequent tracking inherits that one sample's noise as a permanent bias.
4. **Loss of detection ⇒ zero-order hold** on $(c,\theta)$ with staleness warnings ([sources.py:242-267](g1_spine_stabilization/g1_spine_stabilization/trajectory/sources.py#L242-L267)); see P4 on the timer units.

> **Concepts & further reading (relative pose tracking).**
> - **Relative vs. absolute pose control, incremental/differential tracking, teleoperation "clutching":** search "relative pose teleoperation clutch," "position-position teleoperation mapping," Springer Handbook of Robotics ch. on telerobotics.
> - **Change of basis for rotations / conjugation / adjoint on $SO(3)$; body-frame vs space-frame twists:** Lynch & Park ch. 3.2–3.3 (space vs body form); Murray-Li-Sastry ch. 2. Search "adjoint representation SO(3)," "similarity transform rotation change of frame."
> - **Recovering angles from rotation matrices, `atan2` for angle extraction, Euler decomposition:** search "rotation matrix to Euler angles," "atan2 numerical robustness."
> - **Common-mode rejection / differential measurement** (the bias-cancellation argument): an instrumentation concept — search "common-mode rejection ratio," "ratiometric measurement."

---

### 2.4 Extrinsic calibration (single-marker closed form)

**What the code computes.** Given one marker physically placed at a **known** pelvis-frame pose $T^{pelvis}_{true}$ (orientation assumed identity) and the averaged raw optical-frame reading $T^{opt}_{mk}$, solve for the mount→optical extrinsic that reconciles them ([calibration.py:19-30](g1_spine_stabilization/g1_spine_stabilization/perception/calibration.py#L19-L30)):

$$ \widehat{T}^{d435}_{opt} = \bigl(T^{pelvis}_{d435}\bigr)^{-1}\ T^{pelvis}_{true}\ \bigl(T^{opt}_{mk}\bigr)^{-1}. $$

**Why this form.** Start from the composition of §2.2 and demand that the pipeline reproduce the known truth: $T^{pelvis}_{true} = T^{pelvis}_{d435}\,T^{d435}_{opt}\,T^{opt}_{mk}$. Left-multiply by $(T^{pelvis}_{d435})^{-1}$ and right-multiply by $(T^{opt}_{mk})^{-1}$ to isolate the unknown $T^{d435}_{opt}$ — the equation above is that algebra, exact. This is an **exactly-determined solve, not least squares**: one $SE(3)$ observation (6 DOF) pins down one $SE(3)$ unknown (6 DOF). $T^{pelvis}_{d435}$ (the URDF FK mount pose) is *trusted and not solved for*, so any real CAD/mounting error is deliberately pushed into the config-tunable $T^{d435}_{opt}$ slot rather than into the (change-controlled) URDF.

**Where the noise goes.** Because the solve is exact, single-frame noise would pass straight into the estimate. The script instead **averages ~5 s of frames** before solving ([calibrate_camera_extrinsics.py:56-78](scripts/calibrate_camera_extrinsics.py#L56-L78)): arithmetic mean on translation (variance shrinks like $1/N$ by the law of large numbers) and a **rotation average** on orientation via `scipy Rotation.mean()`, which minimizes the sum of squared chordal (Frobenius) distances — the $SO(3)$ analogue of a mean, distinct from naively averaging matrix entries (which would leave $SO(3)$).

**The observability trap (why one marker is not enough).** At a single distance, a **marker-size scale error** and a **translational mounting offset** produce identical residuals — they are unidentifiable from one observation. The `--verify` mode checks the *current* config against a **second marker at a different depth** without re-fitting ([calibrate_camera_extrinsics.py:138-153](scripts/calibrate_camera_extrinsics.py#L138-L153)); a residual that stays small (it dropped $10.06\text{ cm} \to 6\text{ mm}$ and did *not* grow with distance) rules out the scale hypothesis and confirms the fix generalizes. This is a **parameter-identifiability / experiment-design** argument: you need excitation at multiple baselines to separate the two error modes.

**The general problem this is a special case of.** Full **hand–eye calibration** solves $AX = XB$ for the camera-to-mount transform $X$ from many (motion $A$, camera-observed motion $B$) pairs, with no externally measured ground truth. This code sidesteps that by using an externally *measured* marker pose, reducing it to one linear $SE(3)$ solve. If you later cannot measure ground truth by hand, the $AX=XB$ machinery is the principled upgrade.

**Assumptions and observed cost.**
1. **Marker orientation is identity.** Any placement tilt enters the rotation estimate one-for-one — consistent with the ~4° rotation residual that remained after calibration (the user's own "can't place it perfectly" caveat).
2. **URDF FK mount pose is exact.** Real mount error is absorbed into $\widehat T^{d435}_{opt}$, which is exactly what you want *if* you never intend to trust the URDF's camera origin.
3. **Value is still TENTATIVE** in the config pending final review (§P2).

> **Concepts & further reading (calibration & estimation).**
> - **Camera extrinsic calibration, PnP-based calibration:** Hartley & Zisserman; OpenCV `solvePnP` calibration workflows.
> - **Hand–eye calibration ($AX=XB$):** search "hand-eye calibration Tsai-Lenz," "Park & Martin hand-eye," "Daniilidis dual quaternion hand-eye." This is the must-read if the ground-truth-measurement shortcut ever fails.
> - **Rotation averaging (chordal L2 vs geodesic/Karcher mean, quaternion averaging):** **Hartley, Trumpf, Dai & Li, "Rotation Averaging," IJCV 2013** (the definitive survey); search "Karcher mean SO(3)," "Markley quaternion averaging."
> - **Exactly-determined vs least-squares estimation; parameter identifiability / observability / persistency of excitation; degenerate configurations:** any estimation text; search "parameter identifiability robotics calibration," "observability Gramian," "scale-offset ambiguity monocular."
> - **Law of large numbers / $1/\sqrt N$ noise averaging:** any probability text; the reason averaging frames works.

---

### 2.5 The filtering stack (this system's "state estimator")

There is no Kalman filter. The estimator is a deliberately ordered cascade of four interpretable filters, each targeting one empirically-identified disturbance. The ordering rule, stated in the code comments, is **clean the raw signal first, then bound its rate, then smooth the solver output** — i.e. `MarkerSource → NoiseFilteredSource → RateLimitedSource → IK → WeightedMovingFilter`.

**(a) Median-of-N — outlier rejection.** Per degree of freedom over the last $N=5$ samples ([sources.py:432-436](g1_spine_stabilization/g1_spine_stabilization/trajectory/sources.py#L432-L436)):

$$ \tilde x_t = \operatorname{median}(x_{t-4}, x_{t-3}, \ldots, x_t). $$

*Why:* the median is an **order statistic** with a **breakdown point** of $\lfloor (N-1)/2 \rfloor = 2$ samples — it completely ignores up to two arbitrary outliers in the window, so it *rejects* (not merely attenuates) the single-frame IPPE pose flips of §2.1. A linear filter has breakdown point 0: one bad sample corrupts its output. Cost: up to $(N-1)/2$ samples of latency on a genuine step, and it is nonlinear (no transfer function).

**(b) Exponential moving average (EMA) — jitter smoothing.** ([sources.py:438-442](g1_spine_stabilization/g1_spine_stabilization/trajectory/sources.py#L438-L442); same primitive at [ik_solver.py:57-79](g1_spine_stabilization/g1_spine_stabilization/kinematics/ik_solver.py#L57-L79)):

$$ y_t = \alpha\,\tilde x_t + (1-\alpha)\,y_{t-1}, \qquad \alpha = 0.3. $$

*Why:* this is a **first-order IIR low-pass filter** (the discrete-time analogue of an RC circuit). Its transfer function is $H(z) = \alpha / (1 - (1-\alpha)z^{-1})$, unity DC gain ($H(1)=1$, so it does not bias a constant), pole at $1-\alpha$, effective time constant $\tau \approx -1/\ln(1-\alpha) \approx 2.8$ samples, and steady-state group delay $(1-\alpha)/\alpha \approx 2.3$ samples at $\alpha=0.3$. It smooths the *continuous small jitter* of solvePnP on a stationary marker — noise that sits **below** the slew limiter's threshold and so passes through (c) untouched. Applied to `--source marker` only, because sine/mock are noiseless and, unlike (c), this stage is **not** a no-op on a clean signal (it would add pure lag).

**(c) Slew-rate limiter — jump bounding against *measured* dt.** ([sources.py:343-371](g1_spine_stabilization/g1_spine_stabilization/trajectory/sources.py#L343-L371)):

$$ \Delta\theta \leftarrow \operatorname{sign}(\Delta\theta)\,\min\!\bigl(|\Delta\theta|,\ \omega_{max}\,\Delta t_{meas}\bigr), \qquad
\Delta c \leftarrow \Delta c\,\min\!\Bigl(1,\ \frac{v_{max}\,\Delta t_{meas}}{\lVert \Delta c \rVert}\Bigr), $$

with $\omega_{max}=60^\circ/\text{s}$, $v_{max}=0.3\text{ m/s}$, and — the load-bearing detail — $\Delta t_{meas}$ measured by **wall clock**, not the nominal `control_dt`. *Why measured dt:* the loop actually runs at ~6–10 Hz (§2.8), so a real-time reference sampled across a 100–180 ms gap produces per-step deltas 5–9× larger than a 50 Hz loop would; clamping to $(\text{max rate})\times(\text{actual elapsed})$ converts those into bounded-velocity motion regardless of loop jitter. It is a **nonlinear saturation element**: identity below threshold (hence the need for (b)), a hard velocity cap above it. Thresholds are set *above* the reference's own peak angular rate ($A\omega = 0.524 \cdot 2\pi \cdot 0.1 \approx 0.33\text{ rad/s} = 18.8^\circ/\text{s}$) so the intended signal is never clipped — only loop-induced jumps are.

**(d) Weighted moving average (WMA) on the IK output.** ([ik_solver.py:41-50](g1_spine_stabilization/g1_spine_stabilization/kinematics/ik_solver.py#L41-L50)):

$$ \bar q_t = \sum_{i=0}^{3} w_i\,q_{t-i}, \qquad w = (0.4, 0.3, 0.2, 0.1), \quad \textstyle\sum_i w_i = 1. $$

*Why:* a **4-tap FIR low-pass** on the joint-space solution, newest sample weighted heaviest. Its group delay is $\sum_i i\,w_i = 1.0$ sample. (The reversed-weight bug fixed in pre-bringup hardening had paired $w_0=0.4$ with the *oldest* sample, doubling the delay to 2.0 samples ≈ 250–300 ms at the real loop rate.) FIR (not IIR) here because a moving average has exactly linear phase (constant group delay), and it acts on the solver *output* $q$, orthogonal to (a)–(c) which act on the *input* $(c,\theta)$ — so the stages never double-filter the same signal.

**(e) EMA on the gravity feedforward $\tau_{ff}$** (§2.7): a fifth single-pole filter smooths the torque command because the vendored transport applies it as an unfiltered zero-order hold every 250 Hz tick against a soft wrist damping $k_d=1$.

**Why NOT a Kalman filter (worth a paragraph in a paper).** A Kalman/EKF is optimal for a *linear-Gaussian* system with a known process model and Gaussian measurement noise. Here (i) the "process" is a human hand — there is no credible motion model to predict; (ii) the dominant errors are **non-Gaussian outliers** (IPPE flips, dropouts), which a Kalman filter handles poorly and a median filter handles by construction; (iii) the fiducial gives a direct pose measurement, so there is little to gain from a model-based predictor. A robust deterministic cascade is the defensible engineering choice. The trade-off to acknowledge: no principled uncertainty estimate, and filter tuning is heuristic.

**Shared caveat.** All stages are **per-call** (per-sample), so their time constants in *seconds* are multiples of the actual, variable ~100–180 ms solve period (§2.8). The design accepts sample-index semantics rather than resampling to a fixed rate — clean and simple, but it means any future change to solver speed silently retunes every filter's real-world bandwidth.

> **Concepts & further reading (signal processing & estimation).**
> - **FIR vs IIR filters, low-pass filtering, transfer functions, z-transform, group/phase delay, cutoff frequency:** Oppenheim & Schafer, *Discrete-Time Signal Processing*; Smith, *The Scientist and Engineer's Guide to DSP* (free online). Search "first-order IIR low-pass EMA equivalence," "moving average linear phase FIR."
> - **Exponential smoothing:** search "exponential moving average time constant," "single-pole recursive filter," "alpha-beta filter" (the deterministic cousin of a steady-state Kalman filter — a good bridge concept).
> - **Median filter, order statistics, robust statistics, breakdown point:** Huber, *Robust Statistics*; search "median filter impulse noise," "breakdown point estimator."
> - **Slew-rate / rate limiting, saturation nonlinearity, describing functions, jerk limiting:** search "rate limiter control nonlinearity," "describing function saturation."
> - **The Bayesian alternatives to compare against:** Kalman (1960); search "Kalman filter tutorial," "EKF vs UKF," "particle filter," "complementary filter"; Thrun, Burgard & Fox, *Probabilistic Robotics* (the standard reference for why you would or wouldn't use each).
> - **Sampling / aliasing under variable rate:** search "Nyquist–Shannon sampling theorem," "aliasing non-uniform sampling."

---

### 2.6 Grasp geometry and the whole-arm inverse kinematics

#### 2.6.1 Antipodal rigid-transport grasp map

**What the code computes.** Map the object reference $(c,\theta)$ and box half-width $w$ to two hand $SE(3)$ targets ([targets.py:44-63](g1_spine_stabilization/g1_spine_stabilization/trajectory/targets.py#L44-L63)):

$$ p_{L/R} = c + R_x(\theta)\,(0,\ \pm w,\ 0)^\top, \qquad R_{L/R} = R_x(\theta)\,R^{base}_{L/R}, $$

where $R_x(\theta)$ is the elemental X-rotation and $R^{base}_{L/R}$ are the "palms facing each other" orientations measured from FK at zero configuration ([targets.py:31-41](g1_spine_stabilization/g1_spine_stabilization/trajectory/targets.py#L31-L41)).

**Why this form.** The box is modeled as a **rigid body grasped at two antipodal points**, one per palm, offset $\pm w$ along the object's Y-axis. Applying the *same* $R_x(\theta)$ to both the offset vectors and the palm orientations makes each hand frame **rigidly attached to the object frame**: the relative transform $T_L^{-1}T_R$ is constant for all $\theta$. That constancy is exactly the condition for the two hands to move the box as one rigid body without internal squeezing/spreading — the kinematic substitute for a real force-closure grasp (which the sim/position controller does not model). Geometrically the two contact points ride a circle of radius $w$ about $c$ in the plane normal to the rotation axis.

**Assumptions.** No slip; symmetric grasp; rotation axis passes through $c$ along pelvis-X (Critical Assumption 6); box width known from config; contacts are rigid (no compliance). The right hand "follows" only because IK holds both targets simultaneously (§2.6.2) — there is no explicit force coordination.

> **Concepts & further reading (grasping / closed chains).**
> - **Grasp mechanics: form closure, force closure, antipodal grasps, the grasp matrix $G$, rigid-body grasp constraint:** Murray, Li & Sastry ch. 5 (grasping); Prattichizzo & Trucco chapter in the *Springer Handbook of Robotics* ("Grasping"). Search "antipodal grasp," "force closure grasp matrix," "bimanual manipulation closed kinematic chain."
> - **Parallel/closed kinematic chains** (a two-arms-one-object system is a closed loop): Lynch & Park ch. 7; search "closed-chain kinematics parallel manipulator constraint."

#### 2.6.2 Whole-arm IK as a nonlinear program

**What the code computes.** Both 7-DOF arms are solved **simultaneously** as one 14-variable nonlinear program, re-solved (warm-started) every control step ([ik_solver.py:316-378](g1_spine_stabilization/g1_spine_stabilization/kinematics/ik_solver.py#L316-L378)):

$$ \min_{q \in \mathbb{R}^{14}}\ 50\,\lVert e_p(q)\rVert^2 + 5\,\lVert e_R(q)\rVert^2 + 0.02\,\lVert q\rVert^2 + 0.1\,\lVert q - q_{prev}\rVert^2 $$
$$ \text{s.t.}\quad q_{lb} \le q \le q_{ub}, \qquad q_{L,\text{roll}} \ge 0.15, \qquad q_{R,\text{roll}} \le -0.15, $$

with the stacked task errors over both end-effectors:

$$ e_p(q) = \begin{bmatrix} p_L(q) - p_L^* \\ p_R(q) - p_R^* \end{bmatrix} \in \mathbb{R}^6, \qquad
e_R(q) = \begin{bmatrix} \log\!\bigl(R_L(q)\,R_L^{*\top}\bigr)^\vee \\ \log\!\bigl(R_R(q)\,R_R^{*\top}\bigr)^\vee \end{bmatrix} \in \mathbb{R}^6. $$

**The pieces, and why each is there.**

- **Forward kinematics $p(q),R(q)$** come from a Pinocchio model *reduced* to the 14 arm joints (legs, waist, and all fingers locked at zero), compiled through Pinocchio's **CasADi** bindings so IPOPT gets exact first/second derivatives by **automatic differentiation** rather than finite differences ([ik_solver.py:281-311](g1_spine_stabilization/g1_spine_stabilization/kinematics/ik_solver.py#L281-L311)). The end-effector frames are `L_ee_link`/`R_ee_link` — palm-surface frames baked into the URDF, deliberately *not* the wrist joints.

- **Orientation error via the $SO(3)$ logarithm.** $\log(\cdot)^\vee$ maps a rotation matrix to its axis-angle vector in $\mathbb{R}^3$ (the inverse of Rodrigues in §2.1): for rotation angle $\phi=\arccos((\operatorname{tr}R-1)/2)$, $\log(R)^\vee = \frac{\phi}{2\sin\phi}\,(R-R^\top)^\vee$. Then $\lVert \log(R(q)R^{*\top})^\vee \rVert$ is the **geodesic distance on $SO(3)$** — the length of the shortest rotation carrying $R(q)$ to $R^*$. *Why this and not Euler-angle differences:* Euler differences are chart-dependent, discontinuous, and singular at gimbal lock; the geodesic metric is intrinsic, smooth away from $\phi=\pi$ (never approached here), and is the "right" Riemannian distance on the rotation manifold. This is the single most important mathematical modeling choice in the solver and the thing to defend most carefully to a reviewer.

- **Position weight 50 vs orientation weight 5** encodes a fixed **weighted-sum scalarization** of a two-objective problem (hit the position, hit the orientation): position is prioritized 10:1. This is a **Pareto** trade-off frozen at one point; the alternative is hierarchical/task-priority IK (§below).

- **Regularization $0.02\lVert q\rVert^2$** biases toward the zero posture. It resolves the arm's **kinematic redundancy** (7 joints for a 6-DOF task ⇒ a 1-D nullspace of equally-good solutions per arm) by picking the least-norm-ish one, and it conditions the Hessian. *Why it creates the documented ~1.8 mm floor:* at the optimum the total gradient vanishes,
$$ 100\,J_p^\top e_p + 10\,J_R^\top e_R + 0.04\,q + 0.2\,(q-q_{prev}) = 0, $$
so even when the task is perfectly reachable ($e_p=e_R=0$ feasible), the regularization terms $0.04q + 0.2(q-q_{prev})$ are generally nonzero and must be balanced by making $e_p,e_R$ slightly nonzero. The residual scales like $O(\lambda_{reg}/\lambda_{task})$ — you *cannot* drive it to zero without changing the weights, which is why the weights are change-controlled in CLAUDE.md and why the round-trip test tolerance is 5 mm. This is the classic **bias–variance / regularization trade-off** (Tikhonov / ridge regression) made concrete.

- **Smoothness $0.1\lVert q-q_{prev}\rVert^2$** is a discrete **velocity penalty** (Tikhonov damping toward the previous solution). Together with warm-starting from $q_{prev}$ it suppresses chattering between adjacent solves and biases toward temporal continuity. Note $q_{prev}$ is the previous *commanded, filtered* solution, not measured (finding P7).

- **Self-collision constraint = two tightened box bounds** on the shoulder-roll coordinates ($q_{L,\text{roll}}\ge 0.15$, $q_{R,\text{roll}}\le -0.15$). This is a deliberately crude, convex, zero-extra-cost stand-in for true collision-distance constraints: it only prevents the arms from collapsing inward into the torso. Arm–arm and arm–object collisions are unmodeled (Critical Assumption 7).

- **Solver: IPOPT (interior-point)** with `max_iter=30`, `tol=1e-4`, warm start ([ik_solver.py:365-378](g1_spine_stabilization/g1_spine_stabilization/kinematics/ik_solver.py#L365-L378)). On non-convergence the code commands the **last IPOPT iterate** (`opti.debug.value`) and increments a diagnostic counter ([ik_solver.py:436-449](g1_spine_stabilization/g1_spine_stabilization/kinematics/ik_solver.py#L436-L449)) — a real-time compromise: better to command a slightly-suboptimal pose than to stall the loop, with the WMA filter (§2.5d) and the SDK velocity clip bounding the damage. A CasADi-internal SIGINT (`NonIpopt_Exception_Thrown`) is specifically re-raised as a real `KeyboardInterrupt` so operator aborts are never swallowed as "solver failures."

**Deviations from textbook practice, and why (reviewer bait, answered).**
1. **Full NLP per step instead of differential (Jacobian) IK.** Standard real-time IK linearizes: $\dot q = J^\dagger \dot x$ (pseudoinverse) or damped least squares $\dot q = J^\top(JJ^\top+\lambda^2 I)^{-1}\dot x$. That is far faster but only valid for small steps and cannot enforce hard joint bounds exactly. The NLP is robust to large target jumps and enforces bounds as true constraints, at the cost of ~100 ms/solve — which becomes the system's control period (§2.8). **This is the single most consequential engineering trade in the stack.**
2. **Weighted-sum, not task-priority.** Position and orientation are combined in one scalar cost with fixed weights, rather than solved lexicographically (satisfy position exactly, then orientation in its nullspace). Simpler, but the trade-off is global and fixed.
3. **The 14-DOF program is block-separable.** Nothing in the cost or constraints couples the left and right joint variables — the bimanual coupling lives *entirely* in the target construction (§2.6.1). So the joint solve is mathematically equivalent to two independent 7-DOF solves; it is kept unified for implementation simplicity and to leave room for future coupling terms. Worth stating in a paper to preempt "why one solver?"

> **Concepts & further reading (IK & optimization).**
> - **Inverse kinematics (analytical vs numerical), Jacobian methods, redundancy resolution, nullspace projection, damped least squares:** Lynch & Park ch. 6; Siciliano et al. *Robotics: Modelling, Planning and Control* ch. 3; **Buss, "Introduction to Inverse Kinematics with Jacobian Transpose, Pseudoinverse and Damped Least Squares," 2004** (short, exactly the comparison to (1) above). Search "resolved-rate motion control," "task-priority inverse kinematics," "singularity-robust inverse."
> - **Lie groups $SO(3)/SE(3)$, matrix exp/log, hat/vee maps, geodesic distance, tangent space, Riemannian metric:** Lynch & Park ch. 3; Murray-Li-Sastry ch. 2; and the outstanding tutorial **Solà, Deray & Atchuthan, "A micro Lie theory for state estimation in robotics," 2018** (read this one first — it is written for exactly this use).
> - **Nonlinear programming, KKT conditions, interior-point methods:** Nocedal & Wright, *Numerical Optimization* (the standard); Boyd & Vandenberghe, *Convex Optimization* (free) for the convex background. IPOPT specifically: **Wächter & Biegler, Math. Programming 2006.**
> - **Automatic differentiation & the modeling layer:** **Andersson et al., "CasADi," Math. Prog. Computation 2019**; Pinocchio (Carpentier et al. 2019); search "forward vs reverse mode automatic differentiation."
> - **Regularization / Tikhonov / ridge, multi-objective (Pareto) scalarization:** search "Tikhonov regularization," "ridge regression bias variance," "weighted sum method multi-objective optimization," "lexicographic optimization."

---

### 2.7 Gravity compensation

**The motor model it feeds.** The vendored SDK runs a per-joint PD law with a torque feedforward channel:

$$ \tau = \tau_{ff} + k_p\,(q_{des} - q) - k_d\,\dot q. $$

Without feedforward ($\tau_{ff}=0$), gravity torque $\tau_g$ must be produced entirely by steady-state position error $\Delta q_{ss} = \tau_g/k_p$. The wrists have the softest gains ($k_p=20$, $k_d=1$), so ~1 N·m of uncompensated gravity ⇒ ~$0.05$ rad ≈ 3° of static droop per wrist joint. The feedforward channel exists to cancel this.

**What the feedforward is.** The full manipulator equation of motion is

$$ \tau = M(q)\,\ddot q + C(q,\dot q)\,\dot q + g(q), $$

with $M(q)$ the mass/inertia matrix (inertial term), $C(q,\dot q)\dot q$ the Coriolis/centrifugal term, and $g(q)$ the gravity term. Evaluating the **Recursive Newton–Euler Algorithm (RNEA)** — an $O(n)$ inverse-dynamics routine — at zero velocity and zero acceleration isolates gravity ([ik_solver.py:458-463](g1_spine_stabilization/g1_spine_stabilization/kinematics/ik_solver.py#L458-L463)):

$$ \tau_{ff} = \operatorname{RNEA}(q,\ \dot q{=}0,\ \ddot q{=}0) = g(q). $$

*Why this isolates gravity, and why it matters for the model correction:* $g(q) = (\partial P/\partial q)^\top$, the gradient of the gravitational potential energy $P(q)=\sum_i m_i\,g^\top r_{c_i}(q)$. It depends **only on link masses $m_i$ and center-of-mass positions $r_{c_i}$ — not on the rotational inertia tensors.** This fact is the justification for the approximate model correction below.

**The model correction (why it's needed and why it's a legitimate approximation).** The URDF models each Inspire hand at 0.141 kg; the real hand weighs **1.0 kg** (scale measurement). Rather than editing the change-controlled URDF, the code replaces the wrist-yaw body's **spatial inertia** at runtime, before the CasADi model is compiled ([ik_solver.py:128-147](g1_spine_stabilization/g1_spine_stabilization/kinematics/ik_solver.py#L128-L147), applied at [:387-405](g1_spine_stabilization/g1_spine_stabilization/kinematics/ik_solver.py#L387-L405)):

$$ \mathcal I^{comp}_{wrist} = \mathcal I^{bare}_{wrist} + X^{*}_{wrist \leftarrow hand}\bigl(\mathcal I_{hand}\bigr), $$

where $X^{*}$ is the $SE(3)$ **action on a spatial inertia** (Pinocchio `SE3.act(Inertia)`) — it applies both the frame rotation and the **parallel-axis (Steiner) theorem** translation in one operation, moving the hand's inertia from the hand-base frame into the wrist-yaw frame using the URDF attachment transform. The hand inertia is taken as the URDF hand tensor **scaled by the mass ratio** $1.0/0.14143 \approx 7.07$ at the URDF center of mass. **The tensor scaling is an approximation** (it scales the tensor's magnitude but not its spatial *distribution* — a real hand carries mass out at the fingers), *and it is exactly irrelevant here*, because $\tau_{ff}=g(q)$ uses only mass and CoM, never the tensor. The tensor would matter only for the $M(q)\ddot q$ and $C(q,\dot q)\dot q$ terms of full inverse-dynamics/computed-torque control — which at the operating speed (0.1 Hz sine, ±30°) are ~2% of gravity and are deliberately left to the PD loop.

$\tau_{ff}$ is then EMA-smoothed (§2.5e) before handoff, because the transport applies it as a step every 250 Hz tick against $k_d=1$.

**Assumptions and residuals.** Quasi-static motion (inertial/Coriolis negligible); rigid links; correct CoM. Known unmodeled residuals: the 0.2 kg box payload (~0.34 N·m/arm at the shoulder), wrist cabling mass not on the scale, and fingertip CoM offset — together ~1–1.5° of residual droop at current gains. The documented fix if droop appears is raising wrist $k_p$ (a control-gain change), *not* refining the inertia tensor.

**Why PD-plus-gravity is enough (stability).** For a manipulator, a PD law with gravity compensation is provably asymptotically stable to the setpoint (the classical **Takegaki–Arimoto** result) via a Lyapunov argument using the system's energy — you do not need full inverse-dynamics cancellation to hold or slowly track a pose, only the gravity term. That theorem is the formal license for this whole quasi-static approach.

> **Concepts & further reading (dynamics & control).**
> - **Manipulator equations of motion, $M/C/g$ decomposition, Lagrangian vs Newton–Euler:** Lynch & Park ch. 8; Siciliano et al. ch. 7; Murray-Li-Sastry ch. 4.
> - **Recursive Newton–Euler, spatial algebra / 6-D spatial vectors, articulated-body algorithm:** **Featherstone, *Rigid Body Dynamics Algorithms*, 2008** (the definitive text); Pinocchio implements these. Search "recursive Newton-Euler algorithm," "Featherstone spatial vector algebra."
> - **Spatial inertia, parallel-axis (Steiner) theorem, inertia frame transforms, adjoint action on inertia:** Featherstone; Lynch & Park ch. 8.2.
> - **Feedforward / computed-torque / inverse-dynamics control, gravity compensation, PD+gravity stability:** Siciliano et al. ch. 8; **Takegaki & Arimoto, "A new feedback method for dynamic control of manipulators," ASME JDSMC 1981**; search "computed torque control," "PD plus gravity compensation Lyapunov," "passivity-based control manipulator."

---

### 2.8 Timing architecture (what "50 Hz" really means)

The nominal `control_dt` is 0.02 s (50 Hz), but `solve_ik` costs ~100 ms/call (measured: $n=55$, mean 104 ms, max 162 ms) and the loop's `time.sleep(control_dt)` is **uncompensated** (it does not subtract the compute time already spent). So the true period is 120–180 ms, i.e. **~6–10 Hz**. Design consequences, all deliberate and documented rather than fixed:

- References are sampled in **real (wall-clock) time**, so a slow loop enlarges per-step deltas — absorbed by the *measured-dt* slew limiter (§2.5c), not by speeding the solver (deferred, because solver-speed tuning interacts with the accuracy floor of §2.6.2).
- **Phase interpolation is indexed by iteration count, not time.** The blend uses the **smoothstep** polynomial $s(\alpha) = 3\alpha^2 - 2\alpha^3$ with $\alpha = i/N$ ([transforms.py:26-29](g1_spine_stabilization/g1_spine_stabilization/utils/transforms.py#L26-L29)). Smoothstep is the unique cubic with $s(0)=0, s(1)=1, s'(0)=s'(1)=0$ — zero *velocity* at both endpoints, so start/stop are smooth. It is $C^1$ but **not** $C^2$: $s''(0)=6\ne 0$, a bounded acceleration step at segment boundaries (masked in practice by §2.5d and the SDK velocity clip; a jerk-limited or quintic profile would remove it). Because $\alpha$ is $i/N$, a slow loop just stretches the phase's *duration*; it does not add jerk.
- **Orientation interpolation is geodesic (SLERP).** $R(\alpha) = R_0 \exp\!\bigl(\alpha\,\log(R_0^\top R_1)\bigr)$ via `scipy Slerp` ([transforms.py:19-23](g1_spine_stabilization/g1_spine_stabilization/utils/transforms.py#L19-L23)) — constant-angular-velocity rotation along the shortest $SO(3)$ path, the rotational analogue of straight-line interpolation. Same Lie-group machinery as §2.6.2.
- **GIL contention:** the IPOPT solve holds the Python GIL for essentially its full duration, stalling the SDK's in-process 250 Hz publisher thread each solve (measured: a co-running thread drops to 2.2% throughput). Combined with the SDK's fixed assumed tick period, this is a documented, deferred, architectural limit of the single-process design.

> **Concepts & further reading (real-time & trajectories).**
> - **Real-time control loops, timing jitter, deadlines, rate monotonicity:** search "real-time control loop jitter," "control loop timing determinism."
> - **Motion profiles / trajectory blends: smoothstep (cubic Hermite), quintic polynomials, S-curves, jerk-limited trajectories, $C^0/C^1/C^2$ continuity:** Biagiotti & Melchiorri, *Trajectory Planning for Automatic Machines and Robots*; search "smoothstep function," "jerk limited trajectory generation," "quintic polynomial trajectory."
> - **SLERP / quaternion & rotation interpolation:** **Shoemake, "Animating rotation with quaternion curves," SIGGRAPH 1985**; search "SLERP spherical linear interpolation," "geodesic interpolation SO(3)."
> - **The GIL / concurrency:** search "Python GIL," "GIL-releasing C extension," "multiprocessing vs threading real-time."

---

### 2.9 Grip force control (interface level)

Finger closure wraps the SDK's `TransportGraspController`, one instance per hand ([grip_force.py](g1_spine_stabilization/g1_spine_stabilization/controllers/grip_force.py)). The sequence: lerp fingers to a configured **soft-close** pose, settle, snapshot the measured pose as the force-control baseline (`arm()`), then run force feedback each step toward a per-finger force setpoint $\tau^*$ (`tau_baseline`) with coupling gain $\kappa$ and closure authority saturated at $\Delta q_{max}$ beyond the grasp pose. Conceptually the law is an **integrating "close until the contact force target is met"** controller with a hard closure clamp:

$$ q_{cmd} = q_{grasp} + \Delta q, \qquad \Delta q \in [0,\ \Delta q_{max}], \qquad \Delta q \ \text{driven by}\ \kappa(\tau^* - f), $$

with $f$ the measured per-finger force. Before `arm()` is called it degrades to a static close-pose hold, so it is safe to call from any phase. The acceleration-coupled tightening extension $\tau_{des} = \tau^* + k_a\lVert a_{arm}\rVert$ is a documented, deliberately unused hook ([grip_force.py:10-13,79-101](g1_spine_stabilization/g1_spine_stabilization/controllers/grip_force.py#L10-L13)). In dry-run the force feedback is zeros, so only command plumbing is exercised — grip *behavior* is validated on hardware only.

> **Concepts & further reading (force/interaction control).**
> - **Force control, impedance vs admittance control, hybrid position/force control, grasp-force optimization:** Siciliano et al. ch. 9 (force control); Springer Handbook of Robotics, "Force Control" and "Contact Modeling" chapters. Search "impedance control Hogan," "hybrid position force control Raibert Craig," "internal grasp force optimization."

---

### 2.10 End-to-end error budget and assumption register

Approximate end-effector pose-error stack (from existing docs, annotated by this review). These add in a **root-sum-square** sense if independent, but several are systematic (they add linearly):

| Source | Magnitude | Status |
|---|---|---|
| Marker detection (PnP) noise, filtered | ~2–5 mm (informal; **never measured on this rig**) | Placeholder — config says so explicitly |
| Extrinsic calibration residual | ~6 mm position, ~4° rotation (measured at 0.5 m, second marker) | Measured; tentative pending final review |
| Startup-reference bias (single-frame capture) | up to one sample of raw PnP noise, **persistent** | Unbudgeted — finding P6 |
| IK regularization floor | ~1.8 mm | By design (weight trade-off, §2.6.2) |
| PD tracking error / droop residual | ~1–1.5° wrist-equivalent | Bounded by gravity comp + gains |

**Master assumption register** (each is load-bearing somewhere above): pelvis fixed (inertial base); head/waist locked (constant camera extrinsic, §2.2); markers placed near-identity orientation at startup (single-axis rotation validity, §2.3); single-axis pelvis-X target rotation with amplitude $\ll \pi$ (θ-extraction and filter linearity, §2.3/2.5); box dimensions known and constant (§2.6.1); rigid, symmetric, no-slip grasp (§2.6.1); quasi-static motion — gravity-only feedforward (§2.7); loop period set by the solver, filters tuned in samples not seconds (§2.5/2.8).

> **Concepts & further reading (error analysis).**
> - **Uncertainty/error propagation, covariance propagation, systematic vs random error, RSS combination:** Taylor, *An Introduction to Error Analysis*; search "first-order uncertainty propagation," "covariance propagation Jacobian," "error budget robotics." For pose uncertainty specifically: search "propagating uncertainty on SE(3)," Barfoot, *State Estimation for Robotics* (ch. on $SE(3)$ uncertainty).

---

# Section 3 — Consolidated concept index & reading list

Grouped for structured deep dives. Where I name a specific paper I am confident it exists; verify page-level claims yourself.

### 3.1 One-book-per-area starting points
- **Whole stack, single volume:** Corke, *Robotics, Vision and Control* (MATLAB/Python editions) — covers the camera model, kinematics, and control used here.
- **Kinematics/dynamics/control (free PDF + lectures):** Lynch & Park, *Modern Robotics: Mechanics, Planning, and Control*.
- **Computer vision / multi-view geometry:** Hartley & Zisserman, *Multiple View Geometry*; Szeliski, *Computer Vision* (free).
- **Dynamics algorithms:** Featherstone, *Rigid Body Dynamics Algorithms*.
- **Optimization:** Nocedal & Wright, *Numerical Optimization*; Boyd & Vandenberghe, *Convex Optimization* (free).
- **Probabilistic estimation (the road not taken):** Thrun, Burgard & Fox, *Probabilistic Robotics*; Barfoot, *State Estimation for Robotics*.
- **Reference handbook:** Siciliano & Khatib (eds.), *Springer Handbook of Robotics*.

### 3.2 Concept → where it appears here → search terms
| Concept | Used in | Search terms / key reference |
|---|---|---|
| Pinhole camera model, intrinsics/extrinsics | §2.1 | "pinhole camera model," Hartley & Zisserman |
| Lens distortion (Brown–Conrady) | §2.1 | "radial tangential distortion," "Zhang calibration" |
| Perspective-n-Point (PnP), planar PnP, IPPE | §2.1 | "Perspective-n-Point," Collins & Bartoli IJCV 2014 |
| Homography & its decomposition | §2.1 | "homography pose decomposition Malis" |
| Reprojection error / ML estimation in images | §2.1 | "reprojection error bundle adjustment" |
| Fiducial markers, Hamming/lexicode | §2.1 | "AprilTag Olson," "ArUco Garrido-Jurado," "36h11" |
| Rodrigues formula, axis-angle | §2.1, §2.6 | "Rodrigues rotation formula" |
| $SE(3)/SO(3)$, homogeneous transforms, frame trees | §2.2 | "homogeneous transformation," ROS tf2, REP-103/105 |
| Forward kinematics, URDF, Pinocchio | §2.2, §2.6 | "forward kinematics kinematic tree," Carpentier 2019 |
| Euler conventions, gimbal lock | §2.2 | "fixed-axis vs intrinsic Euler angles" |
| Relative/differential pose tracking, clutching | §2.3 | "relative pose teleoperation clutch" |
| Change of basis for rotations / adjoint | §2.3, §2.6 | "adjoint SO(3)," "similarity transform rotation" |
| Common-mode / differential bias cancellation | §2.3 | "common-mode rejection ratio" |
| Camera extrinsic & hand–eye calibration ($AX=XB$) | §2.4 | "hand-eye calibration Tsai-Lenz / Daniilidis" |
| Rotation averaging (chordal/Karcher mean) | §2.4 | Hartley et al. "Rotation Averaging" IJCV 2013 |
| Identifiability / observability / scale-offset ambiguity | §2.4 | "parameter identifiability calibration" |
| FIR/IIR filters, group delay, transfer function | §2.5 | Oppenheim & Schafer; Smith DSP guide (free) |
| EMA = first-order IIR low-pass, alpha-beta filter | §2.5 | "exponential moving average time constant" |
| Median filter, robust statistics, breakdown point | §2.5 | Huber *Robust Statistics* |
| Slew-rate limiting, saturation nonlinearity | §2.5 | "rate limiter," "describing function saturation" |
| Kalman/EKF/UKF/particle/complementary filters | §2.5 (contrast) | Thrun et al.; "Kalman filter tutorial" |
| Grasp mechanics: form/force closure, grasp matrix | §2.6.1 | Murray-Li-Sastry ch. 5; "antipodal grasp" |
| Closed/parallel kinematic chains | §2.6.1 | "closed-chain kinematics constraint" |
| Inverse kinematics: Jacobian, DLS, nullspace, task priority | §2.6.2 | Buss 2004; "damped least squares IK" |
| Lie theory: exp/log, hat/vee, geodesic distance | §2.6.2, §2.8 | **Solà et al. "micro Lie theory" 2018** |
| Nonlinear programming, KKT, interior-point (IPOPT) | §2.6.2 | Nocedal & Wright; Wächter & Biegler 2006 |
| Automatic differentiation, CasADi | §2.6.2 | Andersson et al. 2019; "reverse-mode AD" |
| Tikhonov/ridge regularization, bias–variance | §2.6.2 | "Tikhonov regularization," "ridge regression" |
| Multi-objective scalarization, Pareto, lexicographic | §2.6.2 | "weighted sum multi-objective," "lexicographic optimization" |
| Manipulator dynamics $M/C/g$, Lagrangian/Newton-Euler | §2.7 | Lynch & Park ch. 8; Siciliano ch. 7 |
| RNEA, spatial algebra, articulated-body | §2.7 | Featherstone *RBDA* |
| Parallel-axis (Steiner), spatial inertia transforms | §2.7 | Featherstone; "parallel axis theorem inertia" |
| Computed-torque / gravity comp / PD+gravity stability | §2.7 | Takegaki & Arimoto 1981; "computed torque control" |
| Passivity-based control | §2.7 | "passivity-based control manipulator" |
| Motion profiles: smoothstep, quintic, jerk-limited, S-curve | §2.8 | Biagiotti & Melchiorri; "jerk limited trajectory" |
| SLERP / quaternion interpolation | §2.8 | Shoemake SIGGRAPH 1985 |
| Real-time loops, jitter, GIL/concurrency | §2.8 | "real-time control jitter," "Python GIL" |
| Force/impedance/admittance/hybrid control | §2.9 | Siciliano ch. 9; "impedance control Hogan" |
| Error/uncertainty propagation, SE(3) uncertainty | §2.10 | Taylor *Error Analysis*; Barfoot *State Estimation* |

### 3.3 The five things to be able to defend cold (highest-value study targets)
1. **Why the geodesic $SO(3)$ log-metric for orientation error** and not Euler-angle differences (§2.6.2) — the micro-Lie-theory paper is the single best prep.
2. **Why $\tau_{ff} = \text{RNEA}(q,0,0) = g(q)$ depends only on mass/CoM, not the inertia tensor** — this is what makes the 7.07× tensor-scaling approximation legitimate (§2.7).
3. **Why the $0.02\lVert q\rVert^2$ regularizer forces a nonzero task residual** (the first-order-optimality gradient-balance argument) — the source of the ~1.8 mm floor (§2.6.2).
4. **Why displacement tracking, not absolute** — decoupling + common-mode bias cancellation, and the hidden near-identity-start assumption (§2.3).
5. **Why a robust filter cascade instead of a Kalman filter** — non-Gaussian outliers + no target motion model (§2.5).

---

## Suggested follow-ups (not part of this review's scope)

- Fix P3 (Phase-1.5 interrupt), P4 (wall-clock staleness), P6 (averaged startup references) before the next live marker session — all three are small, isolated changes.
- One CLAUDE.md/README sync pass (P1, P15) and a decision commit for the config (P2).
- The user's stated follow-up prompts (Methods-section draft, remediation plan for top-3, novelty flags, data-flow diagram) can all be built directly from Sections 2–3. For the novelty flags specifically, start from §3.3 plus: displacement-vs-absolute tracking, per-step NLP-IK vs. differential IK, the deterministic robust-filter cascade in place of a KF, and the single-marker identity-orientation calibration shortcut vs. full hand–eye.
