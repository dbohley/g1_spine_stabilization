# Palm-Force Press Control

Status: **implemented** (hardware-free parts verified; live bringup is human-run).
This document is the reference for the palm-force grip architecture and the
paper-ready mathematical formulation of the press controller.

> New to this and want the intuition first? Read
> [`palm_force_control_explained.md`](palm_force_control_explained.md) — a
> from-first-principles walkthrough (spring analogy, a worked one-tick example with
> numbers, and the position-vs-Jacobian difficulty tiers). This doc is the terse spec.
>
> This doc describes the **position/stiffness** controller (`palm_press.mode: position`, the
> default). The alternative **Jacobian force** controller (`mode: jacobian`, "Road 1") is
> documented in [`palm_force_jacobian.md`](palm_force_jacobian.md).

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

### Palm tilt (thumb-first-contact mitigation)

A live test showed `s` saturate at `squeeze_max` while measured force stayed near zero for many
seconds — FK against the real URDF (at the live `grip_force.grasp_pose`) confirmed the thumb tip
sits 25mm (left) / 35mm (right) past `L_ee_link`/`R_ee_link` along the press axis, more than the
entire `squeeze_max` (20mm) budget, so the thumb plausibly contacts the object before the palm
sensor does — no collision checking runs during the live press loop to catch this.

`grasp_approach.palm_tilt_deg_left`/`palm_tilt_deg_right` (`[roll, pitch, yaw]` degrees, default
all-zero/no-op) apply a small **local/body-frame** rotation on top of every commanded palm
orientation (`trajectory/targets.py`'s `apply_palm_tilt`, composed as the rightmost factor so it
never disturbs `theta`/`squeeze`/`grasp_frame_R`), across Phase 1a/1b/2/hold and
`grasp_sequence.py`'s mirrored Phase 0a/0b. Axes are the hand's own: roll = local X (the
press/squeeze axis itself — unaffected by rolling about it), pitch = local Y (forward/reach axis,
same real-world sense both hands), yaw = local Z (up/down, **flips real-world sign between
hands** — see CLAUDE.md's Frame conventions). A real-world-symmetric tilt therefore needs
**opposite** signs between hands for roll/yaw but the **same** sign for pitch; the two hands are
independent config values on purpose rather than auto-mirrored, since that asymmetry is easy to
get backwards while tuning by feel. This is a live-tunable mitigation, not a predicted fix — try
small values and observe.

Complementary lever: `grip_force.grasp_pose`'s `thumb_flex` (currently `0.0` — thumb fully
straight/unflexed, i.e. maximally protruding) directly reduces thumb protrusion past the palm
plane, independent of wrist orientation — worth trying alongside or instead of the tilt.

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
- `counts_per_newton` = calibration scale. Obtain it **one hand at a time** with:
  ```
  python scripts/read_palm_force.py --no-dry-run --side left  --calibrate-kg 0.5
  python scripts/read_palm_force.py --no-dry-run --side right --calibrate-kg 0.5
  ```
  (`--calibrate <N>` takes the load in Newtons directly; `--calibrate-kg <m>`
  takes a mass and converts via g). Keep the palm untouched for the auto-captured
  baseline, then at the prompt **rest the known load flat on the central palm pad**
  (face-up, spread like the real object). The tool prints
  `counts_per_newton = mean_total / N` per side.

**Force, not pressure.** Each of the 112 cells reads local pressure, but the
**sum** `Σ p_i` aggregates to total normal **force** (no area division) — that is
what is calibrated against the known weight. The summed-counts→force
proportionality is clean only in the linear, unsaturated regime, so calibrate with
the load **spread across the pad like the grasped object** and **near the operating
force**; a concentrated point load can saturate cells and bias the scale. It is
approximate (FSR taxels are nonlinear/hysteretic) — good for a setpoint/threshold,
not precision force measurement. (Left/right gains differ; `counts_per_newton` in
config is a single scalar, so average or pick per your setup.)

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

## 3.7 Alternative force source: wrist wrench (`palm_press.force_source`)

Live testing found the palm tactile sensors themselves unreliable, and separately that
`signal: min` (the old default, §2) kept driving whichever hand read weaker toward the target
while the other ran up toward the crush-guard ceiling with no corresponding benefit. Two fixes:

**`signal` default is now `mean`** (still `min | mean`, both readers below): drives the PAIR's
average toward `force_target`, rather than forcing both hands individually to hit it (which is
unreachable if the two hands have any real asymmetry). `guard_force()` (the crush guard) is
**always max of the two hands regardless of `signal`** — an independent safety concept, unaffected.

**`palm_press.force_source: palm_sensor | wrist_wrench`** (default `palm_sensor`, i.e. today's
`PalmForceReader`). `wrist_wrench` is **not a hardware sensor** — confirmed by direct inspection
that no wrist F/T sensor exists on this robot or in the vendored SDK (`unitree_hg`'s
`LowState_`/`MotorState_` carry only a per-motor `tau_est` scalar, no wrench field; the SDK's own
"wrist force" terminology already means this same derived quantity). It's `WristWrenchForceReader`
(`controllers/palm_force.py`): `tau_ext = tau_measured - gravity_torque(q)`, then
`W_ext ≈ pinv(J(q)^T) @ tau_ext` per arm (`G1HandIK.ee_jacobians`/`gravity_torque`, the exact same
Jacobians §3.5 already uses the other direction), projected onto `press_axis(theta, grasp_frame_R)`
— the same axis `compute_press_wrenches` commands a press along (which itself gained an optional
`grasp_frame_R` param alongside this work: it previously hardcoded pelvis-Y even when
`compute_hand_targets` was already using the object's own measured axis, a stale-axis gap now fixed).

Sign convention — **two independent considerations**: (1) `compute_press_wrenches` defines `W` as
the wrench the palm exerts ON the object — left `-F·u`, right `+F·u`. By Newton's third law the
object's reaction ON the hand is the negative of that — `+F·u` on the left hand, `-F·u` on the
right — so a genuine symmetric press produces *opposite* raw signs between hands.
`WristWrenchForceReader` dots the left hand's raw wrench with `+u` and the right's with `-u`,
turning those opposite raw signs into a matching positive reading for both — mirroring
`PalmForceReader.force()`'s always-non-negative convention. This part was derived from the math
and is unit-tested, but a synthetic-Jacobian test can't validate (2): **whether `tau_est` itself
agrees in sign with Pinocchio's `gravity_torque(q)` on real hardware.** First live test showed it
doesn't — both hands read increasingly *negative* together as real squeeze/contact genuinely
increased (both moving the same way ruled out a per-hand (1)-type mixup, which would show one
hand flip relative to the other instead). Fixed by negating in `_estimate`
(`tau_ext = gravity_torque(q) - tau`, not the other order). See
`tests/test_palm_force_reader.py`'s sign-convention regression tests, and re-confirm live after
any hardware/SDK change that could touch how `tau_est` is reported.

**Caution, not hidden:** torque-derived force is inherently noisier and more model-dependent than
a dedicated sensor — gravity-model error, Jacobian conditioning near singularities, joint
friction/`tau_est` bias all feed into it. `force_max`/`touch_threshold`/`loss_floor` tuned against
the palm sensor's Newton values are **not guaranteed to mean the same real-world force here**, even
though both report "Newtons" — re-validate these live against the new source rather than assuming
equivalence.

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
  hold_mode: seat          # seat | continuous -- when the press STOPS regulating (§7.7)
  seat_tolerance: 0.5      # N: |F - force_target| band counting as converged; PLACEHOLDER
  seat_frames: 5           # consecutive in-band ticks to declare the press seated
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

## 7.5 No-target-marker hold + `--target-force` (real-world grip testing)

When `--source marker` finds only the **object** marker (no target/torso marker in
frame), `sdk_pose_tracking.py` still grasps normally, but instead of releasing
immediately it **holds the grasp with the grip loop still running live** —
palm-press keeps converging on/holding `force_target`, and the contact-loss
watchdog stays armed — until a human presses **SPACE** (or Ctrl-C) in the
terminal. This is deliberately non-blocking (`utils/keyboard.py`'s
`SpacebarWatcher`, cbreak mode, no Enter needed) rather than a blocking
`input()` gate, precisely so the press loop and safety watchdog don't go dead
for the whole hold. Periodic status lines print the live force/press reading.

`run_pose_tracking_preview.py --target-force N` overrides `palm_press.force_target`
for that one run, clamped into the configured `[force_min, force_max]` bounds
(`controllers/palm_force.py`'s `clamp_target_force`) — a value above `force_max`
is clamped to it with a printed warning rather than actually attempted. No
effect if `palm_press.enabled` is false (a warning prints instead). This is the
supported way to try a specific grip force live without hand-editing YAML:

```
python scripts/run_pose_tracking_preview.py --source marker --tracking orient-only \
    --target-force 3.0 --no-dry-run
```

## 7.6 Contact-loss watchdog: freezes the press, never the run

`PalmContactWatchdog` arms once force crosses `touch_threshold`, then confirms a
loss after `loss_frames` consecutive frames below `loss_floor`. On a confirmed
loss it **freezes the press** — position mode holds the last depth, jacobian mode
zeroes the commanded wrench — and the arm holds pose stiffly via the unchanged
`kp` servo.

**It does not stop tracking.** Phase 2 keeps running, the IK keeps solving, and
`set_joint_targets` is still called every tick; only the inward press stops
advancing. Ending the run is the operator's call (Ctrl-C).

Three knobs, all under `palm_press`:

| Key | Default | Effect |
|---|---|---|
| `watchdog_enabled` | `true` | `false` disables the guard **without** disabling the press loop (`palm_press.enabled: false` would kill both) |
| `watchdog_latch` | `false` | `false`: a confirmed loss clears as soon as force recovers above `loss_floor`. `true`: the original behavior — frozen until the process exits, since nothing calls `reset()` mid-run |
| `loss_frames` | `10` | Consecutive below-floor frames before a loss is confirmed (~1–1.7 s at the real 6–10 Hz loop rate) |

`watchdog_latch: false` and the raised `loss_frames` both exist because
`force_source` is currently `wrist_wrench` — a torque-derived estimate that is
noisier and more model-dependent than the palm tactile sensor these thresholds
were originally written for (see §3.7's CAUTION). A momentary dip below 0.2 N is
far more likely to be sensor noise than a dropped head, and under the old latched
guard that noise would freeze the press for the entire remaining session.

The `[SAFETY] palm contact lost` / `palm contact regained` messages are
edge-triggered, so the log shows the real loss/recovery history rather than one
stale line. If the guard still fires spuriously on hardware, raise `loss_frames`
further before reaching for `watchdog_enabled: false`.

## 7.7 Seat-and-freeze: the press stops regulating once the grip is established

**The press loop's job is to establish the grip, not to regulate force for the
whole run.** Under the original always-regulating behavior, `press_ctrl.step()`
ran on every Phase 2 tick, so the press depth `s` kept moving in response to
force error while the arm was simultaneously trying to track the target marker.

That is not a benign background loop. `s` is **not** a private side channel of
the force controller — §3.5 shows it feeding straight into
`compute_hand_targets(..., squeeze=s, ...)`, i.e. into the very same IK hand
targets the tracking controller is trying to hit. A force loop rewriting those
targets every tick fights the tracker on the press axis, and the force loop
wins. With `force_source: wrist_wrench` (§3.7) it is fighting the tracker using
*noise*, since that estimate is torque-derived rather than a direct measurement.

`palm_press.hold_mode` selects the behavior:

- **`seat` (default)** — run the press loop until the measured force converges,
  then hold that depth for the rest of the run. Tracking owns the hand targets
  from then on.
- **`continuous`** — the original always-regulating behavior, kept so the two
  can be A/B compared on hardware without a code revert.

**What "frozen" means.** Freezing `s` does *not* freeze the hands. `s` keeps
flowing through `compute_hand_targets` with the live `theta` / `box_center`, so
the grip geometry still rotates with the tracked object exactly as before. Only
the force-driven *adjustment* of the palm separation stops. This is a frozen
scalar, not a frozen joint configuration.

**The seat criterion** (`PressSeatDetector`, `controllers/palm_force.py`) is
`|F − force_target| <= seat_tolerance` for `seat_frames` **consecutive** ticks —
deliberately not "force first crossed the target". A single `wrist_wrench` noise
spike would otherwise seat the grip early at too shallow a depth, and there is no
un-seating. At the real ~6-10 Hz Phase 2 loop rate the default `seat_frames: 5`
is ~0.5-0.8 s of sustained in-band force, so a spike costs a counter reset rather
than a bad grip.

**No un-seating, by design.** Once seated the depth is held even if contact is
later lost. The watchdog (§7.6) still prints `[SAFETY]` loudly on every loss and
regain, but control of the hand targets is never handed back to the force loop
mid-tracking — the same "warn, don't halt" principle §7.6 follows. Only an
explicit `reset()` at hand release clears it, before a re-grasp.

One consequence worth stating plainly: the crush guard lives inside
`press_ctrl.step()`, so it stops running once seated. That is correct — a fixed
depth cannot crush further than it already has — and the watchdog's per-tick
`[SAFETY]` prints remain the live contact diagnostic.

In jacobian mode the analogous quantity is frozen: the commanded press force
`F_cmd` is held at its converged value rather than re-stepped, which is the
force-domain equivalent of a frozen depth.

The ~1 Hz status line shows `[seating…]` or `[SEATED — depth held]` so the
operator can see at a glance which regime the press is in.

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
