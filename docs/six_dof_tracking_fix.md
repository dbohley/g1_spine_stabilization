# True 6-DoF pose tracking — fix the single-axis rotation collapse

**Status: PLANNED, not implemented.** This document is the full implementation plan,
written up so it can be executed in a later session. No code, config, test, or other
doc has been changed yet. Investigation and all numeric verification below were done
against the repo as of branch `perception-replatform` (297 tests passing).

---

## Context

**The reported bug is real, and there is a second independent bug next to it.**

`MarkerSource._theta_from_target_rotation` (`g1_spine_stabilization/trajectory/sources.py:174-177`)
computes the *full* relative rotation of the target marker and then destroys it on the
very next line:

```python
R_delta = self._target_start_rot.T @ R_current
return float(np.arctan2(R_delta[2, 1], R_delta[2, 2]))   # <-- roll about X only
```

Downstream, `trajectory/targets.py:85` re-expands that scalar as `R = Rx(theta)` —
rotations constrained to the 1-parameter X subgroup. So `--tracking 6dof` delivers
**3 translation + 1 rotation = 4 DoF**, exactly as suspected.

Verified numerically in the `g1_inspire_sdk` env:

| target motion | commanded `theta` |
|---|---|
| pure yaw 0.5 rad | `0.0` — no hand response at all |
| pure pitch 2.0 rad + roll 0.2 | `-2.9416` = `0.2 − π` — a **180° flip** |

The `atan2(R[2,1], R[2,2])` form is the roll term of the standard
`R = Rz(yaw)·Ry(pitch)·Rx(roll)` decomposition. Pure yaw gives `R[2,1]=0, R[2,2]=1` →
`theta=0`, so the motion is ignored entirely. Pure pitch gives `theta=0` for `|p|<90°`
but **`theta=π` once pitch exceeds 90°** — a target tipped past vertical commands a
180° roll of both hands. It is also gimbal-degenerate at pitch = ±90°
(`atan2(0,0)=0`).

### Second bug (independent, found while designing the fix)

The code takes the **body-frame** delta `R_t0ᵀ @ R_t` (expressed in the target marker's
own starting axes) and then applies its roll as a **pelvis-frame** `Rx`. Those frames
are unrelated — a marker facing the camera is nowhere near pelvis-aligned.

`compute_hand_targets` builds the object orientation as `R @ G @ R_BASE`,
left-multiplying onto `grasp_frame_R` (the object marker's *absolute pelvis-frame*
orientation, captured once at `sdk_pose_tracking.py:495`). Left-multiplying an absolute
pelvis-frame orientation only type-checks if `R` is a pelvis-frame delta. So `R` must be
the **world/pelvis-frame** delta `R_t @ R_t0ᵀ`, since `R_t = ΔR_world @ R_t0`.

This is invisible today only because X-rotations commute
(`Rx(a)ᵀ @ Rx(b) == Rx(b) @ Rx(a)ᵀ` — verified), which is also why no existing test
catches it.

### Scope note

Both bugs are in code shared by `orient-only` *and* `6dof` — `_theta_from_target_rotation`
is called identically in both modes; `6dof` only ever changes the *position* branch.
So `orient-only` — the default, and the mode named in CLAUDE.md's live-bringup workflow
steps 8/9 — is really 0 translation + 1 rotation DoF.

**Decision taken (2026-09-10): both modes get full 3-DoF rotation**, and it ships
**live** with `enabled_axes: [true,true,true]` and a conservative `max_angle_deg: 20.0`
clamp, rather than shipping dormant behind an off-by-default flag.

**Outcome:** `orient-only` = genuine 3-DoF orientation tracking; `6dof` = genuine 6-DoF.

---

## Design

Widen the source contract from `theta: float` to **scalar-or-3×3**, normalized at three
public boundaries via a new `as_rotation()` helper. Every existing call site that passes
a float (the `theta=0.0` hardcodes in Phase 1a/1b, `grasp_sequence.py`,
`pose_tracking.py`, and ~12 test call sites) stays byte-identical.

Two constraints shaped this and **must not be lost during implementation**:

1. **The rotation must ride in `step()`'s return value, not a side-channel property.**
   Both wrappers proxy unknown attributes to the inner source via `__getattr__`
   (`sources.py:509`, `sources.py:582`) — a `tracked_rotation` property would be read
   *through* them, silently bypassing the noise filter and rate limiter. That is
   precisely the bug class behind the violent-motion incidents in `docs/followups.md`.

2. **The wrappers need a real scalar fast path.** The tempting "rotvec reduces exactly
   to the scalar arithmetic" shortcut is **false past |θ| = π**, and three existing
   tests use out-of-range stress values on purpose. Verified:

   ```
   rotvec(Rx(3.5))  = -2.783
   rotvec(Rx(5.0))  = -1.283
   rotvec(Rx(10.0)) = -2.566
   ```

   `tests/test_rate_limited_source.py:78` (`0.0 → 10.0`, expects `+0.1`) would become
   `-0.1` — a **sign flip**. `tests/test_rate_limited_source.py:49` (`theta=10.0`,
   expects `10.0`) and `tests/test_noise_filtered_source.py:59`
   (thetas `[1.0, 5.0, 2.0]`, expects median `2.0` → would give `1.0`) fail likewise.
   The exactness property only holds if the scalar branch is a genuinely separate code
   path running today's unwrapped arithmetic verbatim.

---

## Implementation

### 1. `g1_spine_stabilization/utils/transforms.py` — new helpers

Add below `Rx` (line 44). Chosen over `targets.py` because `Rx` already lives here and
`data_recording/recorder.py` needs these while its module docstring commits to
"numpy/scipy/pickle/stdlib only — no robot, no IK". `Rotation` is already imported at
line 8.

- `is_scalar_theta(x) -> bool` — python/numpy scalars and 0-d arrays.
- `as_rotation(x) -> (3,3)` — scalar/0-d → `Rx(float(x))`; `(3,3)` → `np.asarray(x, float)`;
  **`ValueError` naming the received shape otherwise**. A silently-accepted 4×4 SE3 would
  be nasty — several are in scope at these call sites.
- `rotation_angle(R) -> float` — `norm(Rotation.from_matrix(R).as_rotvec())`.
- `clamp_rotation(R, max_angle=None, enabled_axes=None)` — rotvec → zero masked
  components → scale to `max_angle` if over → back to matrix. Both `None` ⇒ exact
  identity passthrough.

  Docstring must note: axis masking by zeroing rotvec components is an exact projection
  for a single enabled axis and a well-behaved (norm-non-increasing) approximation for
  two — rotations do not decompose independently. Also note `[true,false,false]` is
  *not* bit-identical to today's `atan2` extraction; it is **better** (no π aliasing).

### 2. `trajectory/targets.py` — accept both types

- Line 85: `R = Rx(theta)` → `R = as_rotation(theta)`
- Line 155 (`press_axis`): `Rx(theta) @ G[:,1]` → `as_rotation(theta) @ G[:,1]`
- Keep the parameter **named `theta`** — it is positional at 12 call sites and keyword
  at none; renaming buys nothing and churns everything. Retitle in the docstrings
  (lines 53, 78, 142-153, and 173-174 which currently says "object roll about robot X"):
  *scalar → roll about pelvis X; 3×3 → full pelvis-frame rotation applied on the left*.
- Keep `Rx` imported — still used elsewhere.

No behavior change for any existing caller.

### 3. `trajectory/sources.py` — `MarkerSource`

- Line 162: `self._last_theta = 0.0` → `self._last_rotation = np.eye(3)`. Grep confirms
  no external reader, so the rename is safe.
- Replace `_theta_from_target_rotation` (174-177) with:

  ```python
  def _rotation_from_target(self, R_current):
      """Target's FULL rotation relative to its own start, in PELVIS frame."""
      return R_current @ self._target_start_rot.T
  ```

  The docstring must state **why** this is `R_t @ R_t0ᵀ` and not `R_t0ᵀ @ R_t` —
  `compute_hand_targets` left-multiplies it onto the pelvis-frame `grasp_frame_R` — and
  that the two coincide only for pure-X motion, which is why the old form went unnoticed.
- Update all four call sites: lines 254, 275, 292, 320.
- Line 352: `return self._box_center.copy(), self._last_rotation.copy()`.
- Rewrite the class docstring (53-68) and `step()`'s (301-310): both modes now track a
  full 3×3 pelvis-frame rotation, and `orient-only` is now full 3-DoF orientation.

### 4. `sources.py` — `RateLimitedSource` (lines 479-507)

Dispatch on `is_scalar_theta`:

- **Scalar branch: today's lines 481/486/494-498 verbatim** (unwrapped, `math.copysign`).
  Preserves `SineSource` and all of `tests/test_rate_limited_source.py`, including the
  deliberately out-of-range stress values.
- **Matrix branch:** `v = rotvec(R_lastᵀ @ R_target)`; if `‖v‖ > max_theta_rate*dt`,
  scale `v` to that norm; `R_last = R_last @ from_rotvec(v)`.

  Note: body- and world-form geodesics are the *identical* operation —
  `R_last @ expm(t·log(R_lastᵀ R_tgt)) == expm(t·log(R_tgt R_lastᵀ)) @ R_last`, since
  `R log(X) Rᵀ = log(R X Rᵀ)`. There is no correctness choice here, only readability.
- Store which branch is active alongside the state so a source that flips type mid-run
  **raises** rather than silently mixing state.
- First-step seed (484-488) and position clamping (500-505) unchanged.
- Document in the class docstring + `configs/default.yaml:21` that
  `max_theta_rate_deg_s` now bounds the **total** angular speed (rotvec norm) for matrix
  sources, not the X component — same number, generalized meaning, strictly tighter
  per-axis.

### 5. `sources.py` — `NoiseFilteredSource` (lines 558-580)

Same dispatch. **Scalar branch: lines 560/563-566/569-578 verbatim.** Matrix branch:

- **Median:** anchor on `hist[0]`; `v_i = rotvec(R_anchorᵀ @ R_i)` (all small after the
  rate limiter); elementwise `np.median`; recompose `R_anchor @ from_rotvec(median)`.
  Exact for in-range pure-X (`median(θᵢ−θ₀)+θ₀ == median(θᵢ)`) and removes the ±π wrap
  hazard from the median stage entirely. Still an approximation of a true geodesic L1
  median, but it preserves what actually matters — single-frame outlier rejection.
- **EMA:** geodesic — `R_ema = R_ema @ from_rotvec(α · rotvec(R_emaᵀ @ R_med))`.
  **Verified numerically** to yield exactly `Rx((1−α)a + αb)` for pure-X input, i.e.
  today's arithmetic, and it is wrap-safe at any absolute angle. Elementwise rotvec EMA
  is *not* — it does the wrong thing when two consecutive rotvecs sit near opposite
  sides of the ±π ball.
- Warmup passthrough (`len < median_window`) unchanged.

### 6. `sources.py` — new `RotationClampSource` (safety)

```python
RotationClampSource(inner, max_angle=None, enabled_axes=None)
```

Same duck-typed `startup()`/`step()`/`__getattr__` shape as the other two wrappers.
`step()` → `as_rotation` on the inner value → `clamp_rotation(...)` → **always returns a
3×3** (it sits outermost and `compute_hand_targets` accepts both, so there is no reason
to re-derive a scalar). Emit a one-time `[RotationClamp]` print the first time the clamp
actually engages — not per tick.

**Wire it outermost, after `RateLimitedSource`.** Three reasons:

1. It is a *position* bound, not a rate bound, so it must be the last word on the
   commanded value.
2. `RateLimitedSource`'s first step after `startup()` is unclamped **by design**
   (`sources.py:484-488`, pinned by `tests/test_rate_limited_source.py:170`). Phase
   1a/1b/1.5 take several real seconds during which the human can rotate the target
   arbitrarily; the first Phase-2 tick then passes that straight through. Under X-only
   that was bounded to ±π — under full SO(3) it is an arbitrary orientation. **The angle
   clamp is the only thing bounding it.**
3. Being a memoryless projection, it releases smoothly when the target returns inside
   the bound, with no windup.

Note this makes `--source sine` emit matrices downstream, which is fine for
`compute_hand_targets`/`press_axis` and is handled by steps 9 and 11.

### 7. `configs/default.yaml` — new block after `noise_filter` (ends line 58), before `box:`

```yaml
  # NEW: the tracked target rotation is now a full 3-DoF pelvis-frame
  # rotation (it was a single roll-about-X scalar, a confirmed bug -- see
  # docs/six_dof_tracking_fix.md). Applies to BOTH --tracking orient-only and 6dof.
  tracked_rotation:
    enabled: true
    # Per-axis [X, Y, Z] enable, applied as a rotvec-component mask in the
    # PELVIS frame. [true, false, false] reproduces the pre-fix single-axis
    # behavior (without its >90-degree pitch aliasing).
    enabled_axes: [true, true, true]
    # Hard ceiling on total tracked rotation angle (rotvec norm).
    # UNMEASURED: no reachability study of commanded yaw/pitch against the
    # shoulder-roll self-collision bounds (ik_solver.py's
    # L_SHOULDER_ROLL_MIN/R_SHOULDER_ROLL_MAX -- Hard Rules, do not touch) or
    # the wrist limits exists yet. Conservative starting point; raise only
    # after a dry-run sweep shows the IK tracks the commanded orientation
    # without ipopt_exhaustion_count climbing or the fk residual growing.
    max_angle_deg: 20.0
```

Matches the file's existing style: `enabled` flag, `_deg` unit suffix, and a long
comment marking unmeasured placeholders (mirrors lines 22-24 and 47-51).

### 8. `scripts/run_pose_tracking_preview.py`

1. **Wire the clamp** after the `RateLimitedSource` block (ends ~505). Read
   `cfg.get("trajectory",{}).get("tracked_rotation",{})`, gate on `enabled`, print the
   config like the two existing wrappers do (488-489, 504-505).
2. **Multi-axis mock.** `_AnimatedMockTracker.__init__` (line 74) gains
   `axes=(1.0, 0.0, 0.0)`. Replace lines 100-103's literal `[[1,0,0],[0,c,-s],[0,s,c]]`
   with a **phase-offset** rotvec:

   ```python
   w = 2*np.pi*freq*t
   rotvec = amplitude * np.array([ax*sin(w),
                                  ay*sin(w + 2*np.pi/3),
                                  az*sin(w + 4*np.pi/3)])
   tgt_pose[:3, :3] = Rotation.from_rotvec(rotvec).as_matrix()
   ```

   With `axes=(1,0,0)` this is exactly `Rx(amplitude·sin(w))` — **byte-identical
   default**, so today's mock runs are unchanged. The 120° phase offsets are
   load-bearing: a co-phased scaled rotvec keeps a *fixed* axis and would still not
   exercise non-commuting composition. Update the class docstring (64-72).
3. **New CLI flag** near `--tracking` (line 174):
   `--mock-rotation-axes X Y Z` (`type=float, nargs=3, default=[1.0, 0.0, 0.0]`).
   Help text: mock source only; per-axis amplitude weights for the synthetic target
   rotation in the pelvis frame; `1 0 0` is the historical pure-roll motion; `1 1 1`
   exercises pitch and yaw — **the only camera-free way to verify the 3-DoF fix**.
4. Fix stale docstrings: module line 17 ("theta tracks target marker X-rotation") and
   line 202's `--record` help ("only time/theta are recorded").
5. Mirror the `axes` param into `scripts/record_human_trial.py`'s own
   `_AnimatedMockTracker` (62-102) for parity — cosmetic, it computes no theta.

### 9. `controllers/sdk_pose_tracking.py`

1. **Line 206 will `TypeError`** (`math.degrees` on a 3×3) on the first Phase-2 log
   tick. Replace with full rpy + total angle, matching the format already used at
   214-215/224-225 (`_Rot` is already imported, used at line 231):

   ```python
   _R = as_rotation(rotation)
   rpy = np.degrees(_Rot.from_matrix(_R).as_euler('xyz'))
   ang = np.degrees(np.linalg.norm(_Rot.from_matrix(_R).as_rotvec()))
   ```

   Rename `_log_marker_info`'s `theta` param → `rotation` (line 189) — exactly one
   caller (line 922), so the rename is cheap and worth it for diagnostic clarity.
2. **Line 268:** `_R = Rx(theta)` → `as_rotation(rotation)`, so the Rerun
   `markers/applied_shift` triad actually shows pitch/yaw. Update docstring 199-203.
3. **`_grip_and_hold_tick` (537-586)** needs no logic change — `reader.update` (550),
   `compute_hand_targets` (570-572) and `compute_press_wrenches` (585) all accept either
   type after step 2. Update the closure's docstring and the `theta` mention at line 485.
   Line 993 (no-target hold, `0.0`) is unaffected.
4. **New reachability diagnostic.** Inside the existing ~1 Hz block at line 918 (not per
   tick — `fk` is not free), reuse the pattern already at lines 686-697:

   ```python
   L_act, R_act = ik.fk(current_q)
   rot_err = max(norm(pin.log3(L_act.rotation @ L_target[:3,:3].T)),
                 norm(pin.log3(R_act.rotation @ R_target[:3,:3].T)))
   ```

   Print it alongside `ik.ipopt_exhaustion_count`; warn above ~10° with wording naming
   the likely cause — commanded orientation may be unreachable under the shoulder-roll
   self-collision bounds or the wrist limits → lower
   `trajectory.tracked_rotation.max_angle_deg` or disable an axis.

   **This is the missing feedback loop.** `solve_ik` (`ik_solver.py:409-471`) currently
   returns `opti.debug.value(...)` on exhaustion with only a printed warning — a silent
   poor solution. This diagnostic converts that into something observable, and it is
   what makes a `max_angle_deg` sweep meaningful. `pin` is already imported.

### 10. `controllers/palm_force.py` — type hints only

Lines 268, 361, 395, 409: `theta: float` → untyped, documented as "scalar roll or 3×3
pelvis-frame rotation; forwarded verbatim to `press_axis`". Update docstrings at
362-364, 396-399, 410-412. **No logic change** — confirmed `theta` is used only at line
389 (`u = press_axis(theta, grasp_frame_R)`).

### 11. `data_recording/recorder.py`

- `MarkerSample` (line 160): keep `theta: Optional[float]`, add
  `tracked_rotvec: Optional[np.ndarray] = None` — `(3,)` rad, pelvis frame,
  `Rotation.as_rotvec()` of the rotation fed to `compute_hand_targets` this step.
- `record()` (line 255): make the `theta=` kwarg polymorphic.
  Scalar → `theta=float(x)`, `tracked_rotvec=[x,0,0]`.
  3×3 → **`theta=None`**, `tracked_rotvec=as_rotvec(x)`.
  `None` → both `None`.

  Rationale for `theta=None` on the matrix path rather than storing the rotvec's X
  component: populating `theta` from a 3-DoF rotation would silently resurrect exactly
  the lossy scalar this whole change exists to remove, and downstream analysis code
  could not tell the difference. This also keeps `tests/test_recorder.py:160-166`
  (`record(theta=0.3)` → `sample.theta == 0.3`) green.
- `SCHEMA_VERSION` (line 213): `2` → `3`, inline comment matching the existing style:
  `# v3: theta may be None for 3-DoF sources; tracked_rotvec added`.
- **Module docstring lines 36-42 are now factually wrong** — they assert
  `_theta_from_target_rotation` "computes a single-axis rotation". Rewrite: it now
  computes a full 3-DoF pelvis-frame delta relative to the target's own starting pose.
  The distinction from `angle_diff_rpy` (start-relative single rotation vs. instantaneous
  object-vs-target comparison, in different reference frames) still stands and still matters.
- Import the helpers from `utils/transforms.py` — preserves the
  "numpy/scipy/pickle/stdlib only" contract (`transforms.py` imports nothing else).

### 12. Docs

- **`CLAUDE.md`**
  - line ~341: "Orientations are `Rx(theta) @ R_BASE`" is now wrong.
  - the `sources.py` paragraph: add `RotationClampSource` alongside the other wrappers.
  - Open Design Questions, the "Orientation-only vs 6-DoF" resolved bullet (~412):
    currently says theta is the target's rotation relative to its own start without
    disclosing it was single-axis. Rewrite.
  - Add a new COMPLETE entry at the top of Current Status covering **both** bugs.
  - Standard Verification Workflow step 7: add the `--mock-rotation-axes 1 1 1` commands.
- **`docs/data_recording_conventions.md`**: line 26 field list (`tracked_rotvec`; `theta`
  now `None` for 3-DoF sources); line 47's paragraph describing
  `R_delta = target_start_rot.T @ R_current` then `atan2 of one matrix entry` is now
  **doubly** stale (both the collapse and the frame).
- **`docs/followups.md`**: new entry — both bugs, the frame-convention reasoning, the
  `--mock-rotation-axes` dry-run gate, the unmeasured `max_angle_deg` default, and an
  explicit follow-up to run a reachability sweep before raising it.

---

## Tests

### Must be rewritten (4 — they assert the scalar semantics being removed)

`tests/test_marker_source.py` lines 109/112, 334, 402/407, 434. Once `step()` returns a
3×3 these raise `ValueError: truth value of an array with more than one element is
ambiguous`. Rewrite against the returned matrix
(`np.testing.assert_allclose(R, Rx(expected_delta))`). Everything else in that file
(`box_half_height`, `box_center_offset`, `object_lost`, `on_observations`,
`target_stale_seconds`, `SineSource`) is untouched.

### Must stay green untouched — this is the regression signal for steps 4 and 5

`test_targets_squeeze.py`, `test_press_wrenches.py`, `test_trajectory.py`,
`test_rate_limited_source.py`, `test_noise_filtered_source.py`, `test_recorder.py`,
`test_pose_tracking_contact_loss.py`, `test_palm_force_reader.py`.
**If either wrapper file goes red, the scalar fast path is wrong.**

### New

- **`test_marker_source.py`**
  - Pure yaw of the target produces a commanded rotation with matching yaw — **the
    direct regression test for the reported bug; fails hard on `main`**.
  - Pure pitch, same.
  - Pitch beyond 90° does not alias to a π flip — pin the exact `-2.9416` failure mode.
  - **World- vs body-frame:** deliberately non-identity, non-commuting start rotation;
    assert the result `== R_t @ R_t0ᵀ` and explicitly `!= R_t0ᵀ @ R_t`. This is the
    bug-2 regression test and the one that would have caught the second bug.
  - Pure-X motion still returns `Rx(Δθ)` exactly — backward-compat pin.
  - Startup pre-seed and stale-hold return `np.eye(3)` / the held matrix, not `0.0`.
- **`test_transforms.py`** — `as_rotation` on scalar / 0-d / 3×3, `ValueError` on
  `(3,)` / `(4,4)` / `None`; `clamp_rotation` identity when both args `None`, angle
  clamp preserves axis direction, axis mask zeroes disabled components, masked+clamped
  never exceeds `max_angle`.
- **`test_rate_limited_source.py`** — matrix over-limit multi-axis jump clamped to
  exactly `max_theta_rate*dt` in rotvec norm with geodesic direction preserved;
  under-limit passes through bit-exact; **pure-X matrix signal == equivalent scalar
  signal** (pin the reduction claim as a test, not a comment); mixing scalar and matrix
  from the same inner source raises.
- **`test_noise_filtered_source.py`** — matrix single-frame multi-axis outlier rejected
  by the anchored median; matrix EMA reduces variance and converges to a constant input
  exactly; pure-X matrix matches the scalar path for in-range values.
- **`test_rotation_clamp_source.py`** (new) — passthrough under `max_angle`; clamped to
  exactly `max_angle` with axis preserved; `enabled_axes=[true,false,false]` kills
  pitch/yaw response; both args `None` ⇒ identity; `__getattr__` proxies to inner;
  **composed `RotationClampSource(RateLimitedSource(NoiseFilteredSource(MarkerSource)))`
  — clamp wins over the rate limiter's first-step passthrough of a huge rotation** (the
  step-6 safety property, deserves its own test).
- **`test_animated_mock_tracker.py`** (new) — `axes=(1,0,0)` reproduces the pre-change
  pure-`Rx` pose exactly; `axes=(1,1,1)` produces genuinely non-zero pitch and yaw with
  a *time-varying* rotation axis; **end-to-end hardware-free: multi-axis mock →
  `MarkerSource` → `compute_hand_targets` → assert both hand target orientations respond
  to yaw.** This is the test that makes the dry-run gate real — no camera, no SDK, no IK.
- **`test_recorder.py`** — `record(theta=0.3)` still gives `sample.theta == 0.3` **and**
  `tracked_rotvec ≈ [0.3,0,0]`; `record(theta=<3x3>)` gives `theta is None` and the
  correct rotvec; `SCHEMA_VERSION == 3` (line 183).

---

## Verification

Run from the repo root with the `g1_inspire_sdk` env
(`/home/moira/miniforge3/envs/g1_inspire_sdk/bin/python`):

1. `pytest tests/ -v` — 297 existing + ~26 new; the 4 rewritten marker-source tests are
   the only permitted churn.
2. `python scripts/verify_ik.py` — unchanged IK self-test.
3. **Backward-compat dry runs** (must be indistinguishable from today):
   ```
   python scripts/run_pose_tracking_preview.py --source sine
   python scripts/run_pose_tracking_preview.py --source mock --tracking orient-only
   python scripts/run_pose_tracking_preview.py --source mock --tracking 6dof
   ```
4. **The new gate** — the first camera-free run that exercises 3-DoF tracking:
   ```
   python scripts/run_pose_tracking_preview.py --source mock --tracking orient-only --mock-rotation-axes 1 1 1
   python scripts/run_pose_tracking_preview.py --source mock --tracking 6dof --mock-rotation-axes 1 1 1
   ```
   Watch in Rerun that `markers/applied_shift` and `targets/L_hand`/`targets/R_hand` now
   show pitch and yaw; watch in the console that the new IK-residual line stays small and
   `ipopt_exhaustion_count` does not climb. **Sweep `max_angle_deg` upward from 20° here
   to find the reachability ceiling before any live run.**
5. `python scripts/run_pose_tracking_preview.py --source mock --tracking 6dof --record`
   → unpickles with `schema_version == 3` and populated `tracked_rotvec`.

**Live steps stay human-run** per Hard Rules (`--source marker`, `--no-dry-run`). That
boundary matters more than usual here: this turns the *default* `orient-only` mode into
genuine 3-DoF orientation tracking for the first time, so step 4's multi-axis mock gate
exists specifically so a human is not the first thing to discover a reachability problem.

---

## Hard Rules compliance

Nothing in this plan touches `assets/g1_with_hands.urdf`, the EE frame definitions
(`L_ee_link`/`R_ee_link`), `L_SHOULDER_ROLL_MIN`/`R_SHOULDER_ROLL_MAX`, the IK cost
weights, or `ARM_JOINT_NAMES`. Step 9.4 only *reads* `ik.fk()` and
`ik.ipopt_exhaustion_count`. No new dependencies (numpy + scipy `Rotation`, both already
imported in `transforms.py`). No live-robot execution.
