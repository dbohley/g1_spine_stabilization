# G1 Spine Stabilization — Full Codebase + Technical Review

**Context.** Read-only diagnostic review requested by the user: (1) a prioritized code-health report, (2) a technical-foundations writeup of the perception & control stack suitable as the backbone of an academic paper. No code was modified. Method: read every source file, config, doc, and test-file structure directly; verified the test suite in the `g1_inspire_sdk` env — **112 tests pass** (59s). The repo is on branch `perception-replatform` with one uncommitted change (`configs/default.yaml`).

**Architecture verdict up front:** the layered design (frame source → detector → tracker → trajectory source wrappers → NLP-IK → SDK transport) is genuinely clean, boundaries match CLAUDE.md's description, and the safety discipline (dry-run defaults, confirm gates, guaranteed-release phases, interrupt propagation) is unusually good for research code. The dominant problems are **documentation drift** (CLAUDE.md is four sessions stale) and **a small number of assumed-dt / interrupt-semantics gaps of exactly the bug classes this project has already fixed twice elsewhere**.

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

Written to stand behind in front of a reviewer: each equation stated with variables defined, why it exists, what it assumes, and where the implementation deviates from the textbook form. File references point at the exact code.

## 2.0 System overview

The system performs **bimanual rigid-body pose tracking**: a Unitree G1 humanoid holds a box with both hands and makes the box's pose track the *displacement* of a hand-held fiducial target. Per control step:

```
RGB frame ──► AprilTag corner detection ──► planar PnP (per marker)          [perception/detector.py]
          ──► SE(3) frame composition into the pelvis frame                  [perception/frames.py, tracker.py]
          ──► displacement-based reference (box_center c_t, roll θ_t)        [trajectory/sources.py: MarkerSource]
          ──► median-of-N + EMA noise rejection                              [NoiseFilteredSource]
          ──► slew-rate limiting against measured wall-clock dt              [RateLimitedSource]
          ──► antipodal grasp map: (c_t, θ_t) ↦ (T_L*, T_R*) ∈ SE(3)²        [trajectory/targets.py]
          ──► whole-arm IK as a 14-DOF NLP (CasADi/IPOPT on Pinocchio FK)    [kinematics/ik_solver.py]
          ──► output smoothing (weighted moving average on q)                [WeightedMovingFilter]
          ──► gravity feedforward τ_ff = g(q) via RNEA + EMA smoothing       [ik_solver.py]
          ──► joint targets to a 250 Hz PD motor loop: τ = τ_ff + Kp e − Kd q̇  [vendored SDK]
```

A supervisory three-phase sequence (interpolate-to-grasp → track → interpolate-home) wraps this loop, with a force-feedback grip controller on the fingers during the tracking phase. There is **no probabilistic state estimator and no dynamics-based trajectory optimization**; the design is deliberately a chain of interpretable geometric and filtering stages. The measured control period is ~100–180 ms (set by the IK solve), not the nominal 20 ms — several stages exist specifically to compensate for that (§2.8).

## 2.1 Camera model and fiducial pose recovery (PnP)

**Model.** Pinhole projection with intrinsic matrix K:

$$ s\,\begin{bmatrix}u\\v\\1\end{bmatrix} = K\,(R\,X + t),\qquad K=\begin{bmatrix}f_x&0&c_x\\0&f_y&c_y\\0&0&1\end{bmatrix} $$

where X ∈ ℝ³ is a point in the marker frame, (R, t) ∈ SE(3) is the marker's pose in the **camera optical frame** (X right, Y down, Z into scene), (u,v) are pixel coordinates, and s is the projective depth. Intrinsics are queried from the RealSense at runtime ([frame_sources.py:59-66](g1_spine_stabilization/g1_spine_stabilization/perception/frame_sources.py#L59-L66)); the YAML values are only placeholders.

**Pose recovery.** Each AprilTag (36h11, 6 cm -- `perception.marker_size`) gives 4 detected corner pixels u_i and 4 known coplanar object points X_i = (±s/2, ±s/2, 0) with Z out of the marker ([detector.py:50-56](g1_spine_stabilization/g1_spine_stabilization/perception/detector.py#L50-L56)). The pose solves the Perspective-n-Point problem

$$ (R^*,t^*) = \arg\min_{R,t}\ \sum_{i=1}^{4}\bigl\lVert u_i - \pi\!\left(K(RX_i+t)\right)\bigr\rVert^2 $$

implemented with `cv2.solvePnP(..., flags=SOLVEPNP_IPPE_SQUARE)` ([detector.py:74-77](g1_spine_stabilization/g1_spine_stabilization/perception/detector.py#L74-L77)) — the *Infinitesimal Plane-based Pose Estimation* method specialized to square planar targets, which is closed-form (no iterative refinement) and returns the reprojection-error-preferred member of the planar two-fold ambiguity pair. The mean corner reprojection error is retained per detection as a quality signal ([detector.py:86-87](g1_spine_stabilization/g1_spine_stabilization/perception/detector.py#L86-L87)).

**Assumptions and failure modes (state these to a reviewer):**
- *Marker side length is exact.* t scales linearly with the assumed marker size; a printed-size mismatch is a pure depth-scale error indistinguishable, at a single distance, from a mounting offset (this exact confound drove the two-marker verification protocol in §2.4).
- *Planar ambiguity.* Near-frontal viewing makes the two IPPE solutions nearly equal in reprojection error; single-frame pose "flips" are expected. This is the physical justification for the **median** stage of the noise filter (§2.5) — an EMA alone would partially incorporate a flipped pose.
- Distortion coefficients come from the camera; residual mis-calibration is folded into the extrinsic correction (§2.4).

## 2.2 Rigid-body frame composition

Every marker pose is re-expressed in the robot's **pelvis frame** (the global frame for all IK targets, robot fixed in space):

$$ T^{pelvis}_{marker} \;=\; T^{pelvis}_{d435}\; \cdot\; T^{d435}_{opt}\; \cdot\; T^{opt}_{marker} $$

([frames.py:51-57](g1_spine_stabilization/g1_spine_stabilization/perception/frames.py#L51-L57), computed in two explicit stages in [tracker.py:46-49](g1_spine_stabilization/g1_spine_stabilization/perception/tracker.py#L46-L49) so intermediate frames are inspectable via `get_diagnostic_pose`).

- **T^pelvis_d435** — the camera-mount pose from forward kinematics of the URDF at the locked joint configuration (head/waist at zero), computed once via Pinocchio ([frames.py:24-48](g1_spine_stabilization/g1_spine_stabilization/perception/frames.py#L24-L48)). *Assumption:* head/waist never move; if they do, this must move inside the per-frame loop.
- **T^d435_opt** — the fixed mount→optical extrinsic, stored as (xyz, rpy) with the URDF fixed-axis convention R = R_z(ψ)R_y(θ)R_x(φ) ([frames.py:12-21](g1_spine_stabilization/g1_spine_stabilization/perception/frames.py#L12-L21)). Nominally the canonical ROS mount↔optical rotation rpy = (−π/2, 0, −π/2); the *calibrated* value (§2.4) additionally absorbs the physical mounting error: xyz = (−0.0860, −0.0002, −0.0384) m, rpy = (−1.4118, 0.0076, −1.5896).

## 2.3 Reference generation: displacement (not absolute) tracking

`MarkerSource` converts two marker streams into the controller's scalar-plus-vector reference (c_t, θ_t): the box center and its roll about the pelvis X axis.

**Startup references** (captured once):
- Object marker at t=0 gives the box's true center, corrected for the marker being mounted on the box's *top* face:

$$ c_{obj,0} \;=\; p_{mk}(0)\; -\; \tfrac{h}{2}\, R_{mk}(0)\,e_3 $$

  where p_mk, R_mk are the object marker's measured pelvis-frame position/rotation, h the box height, and e_3 = (0,0,1)ᵀ — i.e., the offset follows the marker's own measured out-of-plane axis rather than assuming the box is level ([sources.py:157-161](g1_spine_stabilization/g1_spine_stabilization/trajectory/sources.py#L157-L161)).
- Target marker at t=0 gives reference pose (p_tgt(0), R_tgt(0)).

**Per-step reference** ([sources.py:129-136](g1_spine_stabilization/g1_spine_stabilization/trajectory/sources.py#L129-L136), [:236-238](g1_spine_stabilization/g1_spine_stabilization/trajectory/sources.py#L236-L238)):

$$ R_\Delta(t) = R_{tgt}(0)^{\!\top} R_{tgt}(t), \qquad \theta_t = \operatorname{atan2}\!\bigl([R_\Delta]_{32},\,[R_\Delta]_{33}\bigr) $$
$$ c_t = c_{obj,0} + \bigl(p_{tgt}(t) - p_{tgt}(0)\bigr) \quad\text{(6dof mode; orient-only holds } c_t = c_{obj,0}\text{)} $$

θ_t is the roll (X-Euler component) of the target's rotation *relative to its own start*; c_t is the object's start position shifted by the target's *displacement*. Absolute-pose servoing was the original implementation and was a real bug (hands flew to wherever the target physically was); displacement tracking decouples the two markers' physical locations and cancels any constant extrinsic-calibration bias common to both endpoints of the displacement.

**Assumptions:**
1. *Single-axis rotation about pelvis X.* θ is extracted as the roll of R_Δ expressed in the target's **starting body frame**, then re-applied as R_x(θ) in the **pelvis frame** ([targets.py:56](g1_spine_stabilization/g1_spine_stabilization/trajectory/targets.py#L56)). These agree only when R_tgt(0) ≈ I (marker axes aligned to pelvis axes at start). The physical protocol enforces this ("placed close to identity"); a target started significantly yawed would command rotation about the wrong axis. If reviewers push: the exact statement is that the commanded axis is R_tgt(0)·x̂ projected through the roll-pitch-yaw decomposition, which reduces to x̂ at identity.
2. *Reference quality.* Both start poses come from single frames (finding P6): all subsequent tracking is biased by that one sample's noise.
3. θ is a signed angle in (−π, π] from `atan2`; downstream filtering (§2.5) treats it as a real number, so operation must stay away from the ±π wrap. Current amplitude (±30°) is safe.
4. Loss of detection ⇒ zero-order hold on (c, θ) with staleness warnings ([sources.py:242-267](g1_spine_stabilization/g1_spine_stabilization/trajectory/sources.py#L242-L267)); see finding P4 on the timer units.

## 2.4 Extrinsic calibration (single-marker closed form)

Given one marker physically measured at a known pelvis pose T^pelvis_true (identity orientation assumed), and the averaged raw optical-frame reading T^opt_mk, the mount→optical extrinsic follows exactly from the chain in §2.2:

$$ \widehat{T}^{d435}_{opt} \;=\; \bigl(T^{pelvis}_{d435}\bigr)^{-1}\; T^{pelvis}_{true}\; \bigl(T^{opt}_{mk}\bigr)^{-1} $$

([calibration.py:19-30](g1_spine_stabilization/g1_spine_stabilization/perception/calibration.py#L19-L30)). One SE(3) observation exactly determines one SE(3) unknown, so this is an **exactly-determined solve, not least squares**; measurement noise is instead suppressed upstream by averaging ~5 s of frames — arithmetic mean on translation and the chordal-L2 rotation mean (`scipy Rotation.mean()`) on orientation ([calibrate_camera_extrinsics.py:56-78](scripts/calibrate_camera_extrinsics.py#L56-L78)).

**Assumptions and their observed cost:**
- *Marker orientation is identity.* Any physical placement tilt enters the rotation part of the estimate one-for-one — consistent with the observed 4° rotation residual after calibration.
- *T^pelvis_d435 (URDF FK) is trusted.* Any real CAD/mount error is deliberately absorbed into T^d435_opt (the config-tunable slot) rather than edited into the URDF.
- *Scale/offset confound.* At a single depth, a marker-size scale error and a mounting translation are indistinguishable. The protocol disambiguates with a **second marker at a different depth** (`--verify`, no fitting): the residual dropped 10.06 cm → 6 mm with no depth-dependent growth, ruling out the scale hypothesis and validating the transform.

## 2.5 The filtering stack (the system's "state estimator")

There is no Kalman filter; the estimator is a deliberate cascade of four interpretable stages, each targeting one empirically-identified disturbance. Order matters and is documented in code: *clean the raw signal first, then bound its rate, then smooth the solver output.*

**(a) Median-of-N (outlier rejection).** Per DOF over the last N=5 samples: x̃_t = median(x_{t−4},…,x_t) ([sources.py:432-436](g1_spine_stabilization/g1_spine_stabilization/trajectory/sources.py#L432-L436)). An order statistic with breakdown point ⌊(N−1)/2⌋ = 2 samples — it fully rejects the single-frame PnP ambiguity flips of §2.1, which a linear filter would only attenuate. Cost: ~(N−1)/2 samples of pure delay on genuine steps.

**(b) Exponential moving average (jitter smoothing).**

$$ y_t = \alpha x̃_t + (1-\alpha)\,y_{t-1},\qquad \alpha = 0.3 $$

([sources.py:438-442](g1_spine_stabilization/g1_spine_stabilization/trajectory/sources.py#L438-L442); same primitive as [ik_solver.py:57-79](g1_spine_stabilization/g1_spine_stabilization/kinematics/ik_solver.py#L57-L79)). A single-pole IIR low-pass, unity DC gain, time constant −1/ln(1−α) ≈ 2.8 samples, group delay (1−α)/α ≈ 2.3 samples. Targets the small continuous solvePnP jitter of a *stationary* marker — noise the rate limiter (c) structurally cannot touch because it lies below the clamp threshold. Applied to `--source marker` only: sine/mock are noiseless, and unlike (c) this stage is *not* a no-op on clean signals, so applying it there would add lag for nothing.

**(c) Slew-rate limiter (jump bounding, measured-dt).**

$$ \Delta\theta \leftarrow \operatorname{sign}(\Delta\theta)\min(|\Delta\theta|,\ \omega_{max}\Delta t_{meas}),\qquad
\Delta c \leftarrow \Delta c\,\min\!\Bigl(1,\ \tfrac{v_{max}\Delta t_{meas}}{\lVert\Delta c\rVert}\Bigr) $$

([sources.py:343-371](g1_spine_stabilization/g1_spine_stabilization/trajectory/sources.py#L343-L371)), with ω_max = 60 °/s, v_max = 0.3 m/s and — the load-bearing detail — **Δt measured by wall clock**, not the nominal control_dt. Because the loop actually runs at ~6–10 Hz (§2.8), a real-time reference sampled across a 100–180 ms gap produces per-step deltas 5–9× larger than a 50 Hz loop would see; the limiter converts those into bounded-velocity motion. It is a nonlinear element: identity for sub-threshold deltas (hence stage (b)), saturation for super-threshold ones. Thresholds are set above the reference's own peak rate (Aω = 18.8 °/s for the sine) so the intended signal is never clipped.

**(d) Weighted moving average on the IK output.**

$$ \bar q_t = \sum_{i=0}^{3} w_i\, q_{t-i},\qquad w = (0.4, 0.3, 0.2, 0.1),\ \textstyle\sum w_i = 1 $$

([ik_solver.py:41-50](g1_spine_stabilization/g1_spine_stabilization/kinematics/ik_solver.py#L41-L50)), a 4-tap FIR low-pass on the joint-space solution with group delay Σ i·w_i = 1.0 sample. (The reversed-weight bug fixed in pre-bringup hardening had paired w₀ with the *oldest* sample, giving delay 2.0 samples — ~250–300 ms at the real loop rate.) It operates on the solver *output*, orthogonal to (a)–(c) on the *input*, so the stages do not double-filter the same quantity.

A fifth EMA (α = 0.3) smooths the gravity feedforward τ_ff (§2.7) because the SDK's 250 Hz publisher applies torque as an unfiltered zero-order hold while it velocity-ramps position targets — an asymmetry in the vendored transport.

**Shared caveat (worth a sentence in a paper):** all stages are *per-call* discrete filters; their time constants in seconds are multiples of the actual (variable, ~100–180 ms) solve period. The design accepts sample-index semantics rather than resampling to a fixed rate; any future change to solver speed retunes every filter implicitly.

## 2.6 Grasp geometry and the bimanual IK

### 2.6.1 Antipodal rigid-transport grasp map

The box is modeled as a rigid body grasped symmetrically at two antipodal face centers. Given (c, θ) and box half-width w:

$$ p_{L/R} = c + R_x(\theta)\,(0,\ \pm w,\ 0)^{\!\top},\qquad
R_{L/R} = R_x(\theta)\, R_{L/R}^{base} $$

([targets.py:44-63](g1_spine_stabilization/g1_spine_stabilization/trajectory/targets.py#L44-L63)), where R_x(θ) is the elementary rotation about pelvis X and R^base_{L/R} are the palms-facing-each-other EE orientations measured from FK at zero configuration. Both hand frames are therefore **rigidly attached to the object frame** — the contact points transport with the body, which is what makes the (unmodeled-contact) grasp kinematically consistent: as long as both EE poses realize their targets, the relative hand-to-hand transform is constant and the box experiences pure rigid motion. *Assumptions:* no slip, symmetric contact, rotation axis through c along pelvis X (Critical Assumption 6), box width known from config.

### 2.6.2 Whole-arm IK as a nonlinear program

Both 7-DOF arms are solved simultaneously as one 14-variable NLP, re-solved from scratch (warm-started) every control step ([ik_solver.py:316-378](g1_spine_stabilization/g1_spine_stabilization/kinematics/ik_solver.py#L316-L378)):

$$
\min_{q\in\mathbb{R}^{14}}\;
50\,\lVert e_p(q)\rVert^2 \;+\; 5\,\lVert e_R(q)\rVert^2 \;+\; 0.02\,\lVert q\rVert^2 \;+\; 0.1\,\lVert q - q_{prev}\rVert^2
$$
$$
\text{s.t.}\quad q_{lb} \le q \le q_{ub},\qquad q_{L,roll} \ge 0.15,\qquad q_{R,roll} \le -0.15
$$

with stacked task errors over both end-effectors:

$$ e_p(q) = \begin{bmatrix} p_L(q) - p_L^* \\ p_R(q) - p_R^* \end{bmatrix} \in \mathbb{R}^6,
\qquad
e_R(q) = \begin{bmatrix} \log\!\bigl(R_L(q)\,R_L^{*\top}\bigr)^{\vee} \\ \log\!\bigl(R_R(q)\,R_R^{*\top}\bigr)^{\vee} \end{bmatrix} \in \mathbb{R}^6 $$

- **Forward kinematics** p(q), R(q) come from a Pinocchio model reduced to the 14 arm joints (legs/waist/fingers locked at zero), compiled through Pinocchio's CasADi bindings so IPOPT receives exact derivatives by automatic differentiation ([ik_solver.py:281-311](g1_spine_stabilization/g1_spine_stabilization/kinematics/ik_solver.py#L281-L311)). The EE frames are `L_ee_link`/`R_ee_link` — palm-surface frames defined in the URDF, not wrist joints.
- **Orientation metric.** log(·)^∨ is the SO(3) logarithm returning the axis–angle vector: for rotation angle φ = arccos((tr R − 1)/2), log(R)^∨ = φ(R − Rᵀ)^∨ / (2 sin φ). ‖log(R_ee R*ᵀ)^∨‖ is the geodesic distance on SO(3) — the standard chart-free orientation error, singular only at φ = π (never approached here).
- **Regularization 0.02‖q‖²** biases toward the zero posture, resolving redundancy and conditioning the problem. It is also why the solver *deliberately does not reach* the exact solution: at the optimum the gradients balance, 100·J_pᵀe_p ≈ −0.04·q, displacing the answer by O(λ_reg/λ_pos) — the documented **~1.8 mm round-trip floor** encoded in the 5 mm test tolerance. Tightening accuracy means retuning weights, which is why they're change-controlled.
- **Smoothness 0.1‖q − q_prev‖²** is a discrete velocity penalty (Tikhonov damping toward the previous solution) that, together with warm-starting from q_prev, suppresses solution chattering between adjacent solves. Note q_prev is the previous *commanded, filtered* solution (finding P7).
- **Self-collision avoidance** is reduced to two tightened box bounds on the shoulder-roll coordinates — a conservative, convex, constraint-count-free stand-in for true distance constraints. It prevents arm-torso collapse only; arm–arm and arm–object collisions are unmodeled (Critical Assumption 7).
- **Solver settings** ([ik_solver.py:365-378](g1_spine_stabilization/g1_spine_stabilization/kinematics/ik_solver.py#L365-L378)): IPOPT, max_iter 30, tol 1e-4, warm-started. On convergence failure the code falls back to the last IPOPT iterate (`opti.debug.value`) and increments a diagnostic counter — a real-time compromise: an unconverged iterate is commanded rather than stalling the loop, with the WMA filter (§2.5d) and the SDK's joint-velocity clip bounding the damage. A CasADi-internal SIGINT (`NonIpopt_Exception_Thrown`) is specifically re-raised as `KeyboardInterrupt` so operator aborts propagate ([ik_solver.py:436-449](g1_spine_stabilization/g1_spine_stabilization/kinematics/ik_solver.py#L436-L449)).

**Deviations from textbook practice, and why:**
1. *Full NLP per step instead of differential (Jacobian/damped-least-squares) IK.* Robust to large target jumps and enforces hard joint bounds exactly, at the cost of ~100 ms/solve — which becomes the system's control period (§2.8). This is the single most consequential engineering trade in the stack.
2. *Weighted-sum multi-objective instead of task-priority/hierarchical IK.* Position is weighted 10× rotation (50 vs 5); the trade-off is fixed and global rather than lexicographic.
3. *The NLP is block-separable.* Nothing in the cost or constraints couples the left and right arm variables — the bimanual coupling lives entirely in the *target construction* (§2.6.1). The joint 14-DOF solve is therefore mathematically equivalent to two 7-DOF solves; it is kept unified for implementation simplicity and future coupling terms. (Worth stating in a paper to preempt "why one solver?")

## 2.7 Gravity compensation

**Motor-side model** (vendored SDK, per joint): τ = τ_ff + k_p(q_des − q) − k_d q̇, with soft wrist gains (k_p = 20, k_d = 1). Without feedforward, gravity torque τ_g must be generated by steady-state position error Δq_ss = τ_g/k_p — ~3° of wrist droop per N·m — hence the feedforward channel.

**Feedforward** is the gravity term of the manipulator dynamics. From τ = M(q)q̈ + C(q, q̇)q̇ + g(q), evaluating the recursive Newton–Euler algorithm at zero velocity and zero acceleration isolates g(q):

$$ \tau_{ff} \;=\; \mathrm{RNEA}(q,\, 0,\, 0) \;=\; g(q) $$

([ik_solver.py:458-463](g1_spine_stabilization/g1_spine_stabilization/kinematics/ik_solver.py#L458-L463)). g(q) depends only on link masses and centers of mass — not on the rotational inertia tensors — which justifies the model correction below.

**Model correction.** The URDF models each Inspire hand at 0.141 kg; the real hand weighs 1.0 kg (scale measurement). Rather than editing the change-controlled URDF, the runtime replaces the wrist-yaw body's spatial inertia with a composite before the CasADi model is compiled:

$$ \mathcal{I}_{wrist}^{comp} \;=\; \mathcal{I}_{wrist}^{bare} \;+\; X^{*}_{wrist\leftarrow hand}\,\bigl(\mathcal{I}_{hand}\bigr) $$

([ik_solver.py:128-147](g1_spine_stabilization/g1_spine_stabilization/kinematics/ik_solver.py#L128-L147), applied at [:387-405](g1_spine_stabilization/g1_spine_stabilization/kinematics/ik_solver.py#L387-L405)), where X* is the SE(3) action on spatial inertias (Pinocchio `SE3.act(Inertia)` — frame rotation plus parallel-axis transport in one operation) with the URDF hand-attachment transform. The hand's inertia is the URDF tensor scaled by the mass ratio 1.0/0.14143 ≈ 7.07 at the URDF COM. **The tensor scaling is an approximation** (it scales magnitude, not spatial distribution — a hand's fingers carry mass far from the palm), but it is *exactly irrelevant for this use*: g(q) never touches the tensor, only mass and COM. It would matter only for full inverse dynamics (M q̈ + C q̇ terms), which are ~2% of gravity at the operating speeds (0.1 Hz, ±30°) and deliberately left to the PD loop. Known residuals: the 0.2 kg box payload is unmodeled (~0.34 N·m/arm at the shoulder), wrist cabling mass, and COM error — collectively ~1–1.5° droop at current gains.

τ_ff is then EMA-smoothed (α = 0.3, §2.5) before handoff, because the transport applies it as a step every 250 Hz tick against k_d = 1 wrist damping.

## 2.8 Timing architecture (what "50 Hz" actually means here)

Nominal control_dt is 0.02 s, but `solve_ik` costs ~100 ms/call (measured: n = 55, mean 104 ms, max 162 ms), and the loop's `time.sleep(control_dt)` is uncompensated — so the true period is 120–180 ms (~6–10 Hz). Design consequences, all deliberate and documented rather than fixed:

- References are sampled in **real time** (wall clock), so slow loops enlarge per-step deltas — absorbed by the measured-dt slew limiter (§2.5c), not by speeding the solver (deferred: solver settings interact with the accuracy floor of §2.6.2).
- Phase interpolations index smoothstep by **iteration count**, not time: s_i = 3(i/N)² − 2(i/N)³ with s′(0) = s′(1) = 0 ([transforms.py:26-29](g1_spine_stabilization/g1_spine_stabilization/utils/transforms.py#L26-L29)); a slow loop stretches the phase's duration without adding jerk. Orientation interpolates along the SO(3) geodesic, R(α) = R₀ exp(α log(R₀ᵀR₁)), via scipy Slerp ([transforms.py:19-23](g1_spine_stabilization/g1_spine_stabilization/utils/transforms.py#L19-L23)). (Smoothstep is C¹ but not C² — s″(0) = 6 ≠ 0, a bounded acceleration step at segment boundaries, masked in practice by §2.5d and the SDK velocity clip.)
- The IPOPT solve holds the Python GIL for its full duration, stalling the SDK's in-process 250 Hz publisher thread each solve (measured: background thread throughput drops to 2.2%); combined with the SDK's fixed assumed tick period, this is a documented, deferred architectural limit of the single-process design.

## 2.9 Grip force control (interface level)

Finger closure uses the SDK's `TransportGraspController` wrapped per-hand ([grip_force.py](g1_spine_stabilization/g1_spine_stabilization/controllers/grip_force.py)): after the fingers are lerped to a configured soft-close pose and settle, the measured pose is snapshotted (`arm()`), and each step runs force feedback toward a per-finger force setpoint τ* (`tau_baseline`, N) with coupling gain κ and closure authority saturated at Δq_max beyond the grasp pose — i.e., an integrating "close until the force target is met" law with hard closure limits, of the form q_cmd = q_grasp + Δq, Δq ∈ [0, Δq_max], Δq driven by κ(τ* − f). Before arming, `step()` degrades to a static close-pose hold, making it safe to call from any phase. The acceleration-coupled tightening extension (τ_des = τ* + k_a‖a_arm‖) is a documented, deliberately unused hook. In dry-run, force feedback is zeros, so only command plumbing is exercised — grip *behavior* is validated on hardware only.

## 2.10 End-to-end error budget and assumption register

Approximate EE pose error stack (per existing docs, with this review's annotations):

| Source | Magnitude | Status |
|---|---|---|
| Marker detection (PnP) noise, filtered | ~2–5 mm (informal; **never measured on this rig**) | Placeholder — the config says so explicitly |
| Extrinsic calibration residual | ~6 mm position, ~4° rotation (measured at 0.5 m, second marker) | Measured; tentative pending final review |
| Startup-reference bias (single-frame capture) | up to one sample of raw PnP noise, **persistent** | Unbudgeted — see finding P6 |
| IK regularization floor | ~1.8 mm | By design (weight trade-off) |
| PD tracking error / droop residual | ~1–1.5° wrist-equivalent | Bounded by gravity comp + gains |

Master assumption register (each is load-bearing somewhere above): robot pelvis fixed; head/waist locked (constant camera extrinsic); markers placed near-identity orientation at startup; single-axis (pelvis-X) target rotation, amplitude ≪ π; box dimensions known and constant; rigid symmetric no-slip grasp; quasi-static motion (gravity-only feedforward); loop period set by solver, filters tuned in samples not seconds.

---

## Suggested follow-ups (not part of this review's scope)

- Fix P3 (Phase-1.5 interrupt), P4 (wall-clock staleness), P6 (averaged startup references) before the next live marker session — all three are small, isolated changes.
- One CLAUDE.md/README sync pass (P1, P15) and a decision commit for the config (P2).
- The user's stated follow-up prompts (Methods-section draft, remediation plan for top-3, novelty flags, data-flow diagram) can all be built directly from Section 2.
