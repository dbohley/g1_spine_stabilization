# Palm-Force Press Control

Status: **implemented** (hardware-free parts verified; live bringup is human-run).
This document is the reference for the palm-force grip architecture and the
paper-ready mathematical formulation of the press controller.

Code: `controllers/palm_force.py` (`PalmForceReader`, `PalmPressController`),
`trajectory/targets.py` (`compute_hand_targets` `squeeze` param),
`controllers/sdk_pose_tracking.py` (Phase 1.5/2 wiring), `configs/default.yaml`
(`palm_press` block), `scripts/read_palm_force.py` (readout + `--calibrate`).
Tests: `tests/test_palm_press_controller.py`, `tests/test_targets_squeeze.py`,
`tests/test_palm_force_reader.py`.

---

## 1. Architecture

The object is held **pressed between the two palms**, so the palm normal force
*is* the grip force. The control splits cleanly:

- **Fingers**: close to a fixed grasp pose (`grip_force.grasp_pose`) and **hold**.
  No force loop on the fingers — they only cage the object.
- **Palm force**: regulated by **pressing the two palms together**. An outer
  Cartesian loop reads the palm sensor and drives a scalar **press depth `s`**
  (meters) that shrinks the hand-to-hand target separation; the existing IK moves
  the wrist/elbow to close the gap and press.

```
palm taxels (14x8, per hand)
   │  reduce + calibrate  (PalmForceReader)
   ▼
F  (measured palm force, N)
   │  integral press law  (PalmPressController)
   ▼
s  (press depth, m)
   │  w = box_half_width − s
   ▼
compute_hand_targets(box_center, box_half_width, theta, squeeze=s)  →  p_L, p_R
   │  inner IK (G1HandIK, unchanged)
   ▼
q ∈ ℝ¹⁴  →  robot.arm.set_joint_targets(q, τ_ff)
```

This is a **cascade**: an outer scalar force loop wrapping the existing per-step
IK. The IK cost and weights are untouched (Hard Rules preserved) — the press
enters the IK *only* through the target positions.

### Approach (two-stage, `grasp_approach` config)

The same inter-palm axis (the `squeeze` knob) is reused to fix a grasp-approach
problem: a single straight-line interpolation to the grasp pose brings the palms
in **diagonally** and snags on the object edges. Phase 1 is therefore split:

- **Phase 1a — pre-grasp standoff:** interpolate to the grasp pose but held
  `pregrasp_offset` wider (`squeeze = −pregrasp_offset`), so the hands reach the
  correct position/orientation standing clear of the object.
- **Phase 1b — straight-in close:** ramp `squeeze` from `−pregrasp_offset` to `0`,
  translating both palms **purely along their facing normals** so the object
  seats flat between them.

One continuum on a single axis: **standoff `−offset` → grasp `0` → press `+s`**
(Phase 2). Config: `grasp_approach.pregrasp_offset` (m, `< box_half_width`),
`grasp_approach.close_time` (s). `pregrasp_offset: 0` disables the standoff.

---

## 2. Signal and calibration

The central palm tactile array is `palm_touch` (modbus register **4900**, 112
shorts → **14×8**), raw int16 counts, uncalibrated, with a per-cell rest offset.

Per-hand normal force estimate (baseline-subtracted, clipped, summed, scaled):

```
F = ( Σ_i clip(p_i − b_i, 0) ) / counts_per_newton        [Newtons]
```

- `p_i` = 112 taxel counts, `b_i` = per-cell baseline (no-contact mean, from a
  startup capture / `--baseline`).
- `counts_per_newton` = calibration scale. Obtain it with:
  ```
  python scripts/read_palm_force.py --no-dry-run --baseline --calibrate <N>
  ```
  press the palm with a known static load of `<N>` newtons (e.g. hang 0.5 kg on a
  level palm → 4.9 N); the tool prints `counts_per_newton = mean_total / N`.

The summed-counts→force model assumes the taxel sum is ~proportional to total
normal load; `counts_per_newton` absorbs the proportionality. It is approximate
(FSR taxels are nonlinear/hysteretic) — good for a setpoint/threshold, not for
precision force measurement.

**Two-hand combination:** `target_force() = min(F_L, F_R)` (default) so **both**
palms must reach the setpoint before the loop settles; `guard_force() =
max(F_L, F_R)` feeds the crush guard so the harder-pressed palm triggers retreat.

---

## 3. Mathematical Formulation (paper-ready)

### 3.1 Nominal grasp geometry

From `compute_hand_targets` (values are Pinocchio FK at zero config): with object
center `c ∈ ℝ³` (pelvis frame), nominal half-width `w₀ = box_half_width`, and
object roll `θ` about the robot X axis, `R(θ) = Rx(θ)`, the hand target positions
and orientations are

$$
p_L^0 = c + R(\theta)\,[0,\ w_0,\ 0]^\top,\qquad
p_R^0 = c + R(\theta)\,[0,\ -w_0,\ 0]^\top,
$$
$$
\mathcal R_L = R(\theta)\,R_L^{\text{base}},\qquad
\mathcal R_R = R(\theta)\,R_R^{\text{base}}.
$$

The palms face each other along the (rotated) inter-palm axis
$\hat u(\theta) = R(\theta)\,[0,1,0]^\top$, separated by $2w_0$.

### 3.2 Press parameterization

Introduce a scalar **press depth** $s \ge 0$ (meters). The effective half-width
is $w(s) = w_0 - s$, and the hand targets become

$$
p_L(s,\theta) = c + R(\theta)\,[0,\ w_0 - s,\ 0]^\top,\qquad
p_R(s,\theta) = c + R(\theta)\,[0,\ -(w_0 - s),\ 0]^\top .
$$

Both palms translate inward by $s$ along $\hat u$; the separation is
$2(w_0 - s)$. Orientations are unchanged. (Implementation clamps $w(s)\ge 0$.)

### 3.3 Force feedback

Per-hand force $F_L, F_R$ from §2. The scalar feedback for the loop is
$F = \min(F_L, F_R)$ and the guard is $F_g = \max(F_L, F_R)$.

### 3.4 Press control law (bounded scalar integral)

With target $F_{\text{des}} = \operatorname{clip}(F_{\text{target}},\,[F_{\min},
F_{\max}])$, gain $\kappa>0$ (m/N), and travel cap $s_{\max} < w_0$:

$$
u_k = \kappa\,(F_{\text{des}} - F_k), \qquad
u_k \leftarrow \min\!\big(u_k,\ \kappa\,(F_{\max} - F_{g,k})\big)\ \text{ if } F_{g,k} > F_{\max},
$$
$$
s_{k+1} = \operatorname{clip}_{[0,\ s_{\max}]}\big(s_k + u_k\big).
$$

The second line is a **release-only crush guard**: when the harder palm exceeds
$F_{\max}$ it forces a retreating (non-positive) increment regardless of the
target-side error — the same overshoot-only pattern as the SDK's
`HybridForcePositionController`. The form mirrors the SDK's
`TransportGraspController` (a clip-bounded accumulator, dt absorbed into $\kappa$),
but here the accumulated quantity is a **Cartesian press depth**, not finger
closure.

**Fixed point.** Unsaturated and below the crush limit, $s_{k+1}=s_k \iff
F_k = F_{\text{des}}$. The loop holds the palms at the desired normal force.

### 3.5 Composition with the inner IK

$p_L(s,\theta), p_R(s,\theta)$ and $\mathcal R_L, \mathcal R_R$ are the targets of
the existing per-step IK (`G1HandIK`), restated for reference:

$$
\min_{q\in\mathbb R^{14}}\ 50\lVert e_{\text{pos}}\rVert^2 + 5\lVert e_{\text{rot}}\rVert^2
+ 0.02\lVert q\rVert^2 + 0.1\lVert q - q_{\text{prev}}\rVert^2
$$
$$
\text{s.t. } q_{\text{lo}} \le q \le q_{\text{hi}},\quad
q_{L\text{-shoulder-roll}} \ge 0.15,\quad q_{R\text{-shoulder-roll}} \le -0.15,
$$

where $e_{\text{pos}}$ stacks $\text{FK}_L(q)-p_L(s,\theta)$ and
$\text{FK}_R(q)-p_R(s,\theta)$. The full system is the cascade

$$
F_k \xrightarrow{\ \text{press law}\ } s_k \xrightarrow{\ w(s)\ }
p_{L},p_{R} \xrightarrow{\ \text{IK QP}\ } q_k \xrightarrow{\ \text{robot}\ }
\text{plant} \xrightarrow{\ \text{taxels}\ } F_{k+1}.
$$

The IK objective and weights are **unchanged**; the press acts purely by moving
the Cartesian setpoints inward. The wrist/elbow take up the motion subject to the
shoulder-roll self-collision bounds.

### 3.6 Stability and assumptions

Let the quasi-static contact plant be $F = \Phi(s)$, monotone non-decreasing in
contact ($\Phi'(s) \ge 0$: pressing deeper cannot reduce force). Linearizing about
the fixed point $s^\*$ with stiffness $g = \Phi'(s^\*)$, the closed loop is
$s_{k+1} = s_k - \kappa g\,(s_k - s^\*)$, i.e. a first-order recursion with pole
$1-\kappa g$. It is asymptotically stable (and non-oscillatory) for

$$
0 < \kappa\,g < 2 \quad(\text{monotone convergence for } \kappa g \le 1).
$$

Choose $\kappa$ conservatively from the measured contact stiffness $g$ (N/m,
obtainable from the readout tool: $\Delta F / \Delta s$). Assumptions: (i)
quasi-static contact (the ~6–10 Hz loop is slow vs. hand dynamics); (ii)
taxel-sum ≈ total normal force; (iii) symmetric, roughly centered object (single
symmetric $s$; see §7 for the per-hand generalization).

---

## 4. Access path

- **Readout / calibration tool** (`scripts/read_palm_force.py`, robot not
  connected): **direct Modbus** read of register 4900. Safe only because nothing
  else owns the hand.
- **Integrated control** (`PalmForceReader`, robot connected): **DDS-subscribe**
  to `rt/inspire_hand/touch/{l,r}` (`inspire_sdkpy.inspire_dds.inspire_hand_touch`,
  field `palm_touch`, len-112 → 14×8). `InspireHand._reader_loop` already reads
  the tactile registers and **publishes** this topic every loop while connected —
  so the reader is a pure consumer and must **not** open a second Modbus master
  (which would contend with the SDK's reader). The DDS `ChannelFactory` is already
  initialized by `robot.connect()`, so `PalmForceReader` just makes subscribers.

`PalmForceReader` returns **0 N in dry-run** (no subscribers built), so the whole
preview/headless path runs unchanged with `s ≡ 0` (arm behaves exactly as the
pre-palm-press tracking loop).

---

## 5. Config (`configs/default.yaml` → `palm_press`)

```yaml
palm_press:
  enabled: true            # palm-press mode (fingers static + wrist press)
  counts_per_newton: 1.0   # from --calibrate; PLACEHOLDER (raw counts until set)
  force_min: 0.0           # hold floor (N)
  force_target: 0.0        # setpoint (N); 0.0 => NO press until tuned (safe default)
  force_max: 5.0           # crush guard (N): back off above this
  kappa: 0.0005            # integral gain (m/N per step)
  squeeze_max: 0.02        # max press depth s (m); MUST be < box_half_width (0.055)
  baseline_frames: 20      # no-contact frames averaged at Phase 1.5
  noise_threshold: 10      # counts above baseline to call a cell "active" (diagnostic)
  signal: min              # combine two palms for the target loop: min | mean
```

`force_target: 0.0` means the press never engages, so it is **safe to leave
`enabled: true` before calibration** — nothing presses until you set a target.
`counts_per_newton`, `force_target`, `force_max`, `kappa` are placeholders to be
grounded by calibration + the pull-test.

Interaction with `grip_force`: when `palm_press.enabled` is true the finger
force-feedback loop is bypassed (fingers hold `grip_force.grasp_pose` statically).
Arm-only behavior requires **both** `grip_force.enabled` and
`palm_press.enabled` false.

---

## 6. Integration (as-built, `sdk_pose_tracking.py`)

- **Phase 1 (1a + 1b):** approach to the pre-grasp standoff, then close straight
  in along the inter-palm axis (`grasp_approach` config; see §1 "Approach").
- **Phase 1.5:** close fingers to the grasp pose; if palm-press, capture the palm
  baseline (palms at nominal separation, `s=0`, not yet pressing). Else (finger
  mode) arm the finger controllers.
- **Phase 2 loop:** `F = reader.target_force()`, `F_g = reader.guard_force()`,
  `s = press_ctrl.step(F, F_g)`, then
  `compute_hand_targets(box_center, box_half_width, theta, squeeze=s)` → IK →
  `robot.arm.set_joint_targets`. Fingers stay at the grasp pose (SDK's 250 Hz
  publisher re-sends the last hand command for free). Per-second diagnostics print
  `s`, `F_target`, `F_guard` and log them to Rerun (`palm_press/*`).
- **Release / Phase 3:** unchanged (guaranteed finger release, arm return to
  default, settle).

---

## 7. Testing & tuning workflow

1. **Calibrate (human, live):**
   `read_palm_force.py --no-dry-run --baseline --calibrate <N>` → set
   `counts_per_newton`.
2. **Measure contact stiffness:** from the readout, note `ΔF/Δs` to pick a safe
   `kappa` (§3.6). Set `squeeze_max` (< 0.055 m).
3. **Set force targets by pull-test:** raise `force_target`, try to pull the
   object out of the hands; increase until it holds. Set `force_max` above that as
   the crush ceiling, `force_min` as the hold floor.
4. **Dry-run check (Claude/CI):**
   `run_pose_tracking_preview.py --source mock` with `palm_press.enabled: true` →
   loop runs, `s ≡ 0` (reader = 0 N), no arm discontinuity.
5. **Live bringup (human, E-stop in reach):** per Hard Rules, `grasp_demo.py`
   finger check first, then `run_pose_tracking_preview.py --no-dry-run --source
   marker`. Watch `F_guard` vs `force_max`.

**Future refinements:** per-hand press depths $s_L, s_R$ (generalize
`compute_hand_targets` to independent offsets) to balance an off-center object;
weight fingers; fuse palm + finger signals. Out of scope for v1.

---

## 8. Safety / Hard Rules

- Live steps (`--no-dry-run`, any actuation) are **human-run only**, robot
  mounted, E-stop in reach.
- No edits to the vendored `g1_inspire_sdk`; palm data comes via `inspire_sdkpy`
  DDS types (subscribe) / `pymodbus` (readout tool) — never by modifying
  `InspireHand`.
- Never open a second Modbus master while `G1InspireRobot` is connected.
- Bounds: `squeeze_max < box_half_width`, `force_max` crush guard,
  `force_target: 0.0` no-press default. Gravity comp (SDK-5.5) already satisfies
  the wrist-droop precondition for live grip work.
- IK cost/weights and shoulder-roll bounds unchanged (the press acts only through
  the Cartesian targets).
