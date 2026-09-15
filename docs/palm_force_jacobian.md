# Palm-Force Control — Jacobian Mode (Road 1) — build, bring-up, debugging

Status: **implemented, hardware-free parts verified** (168 tests pass). Live bring-up is
human-run (Hard Rules). This is the operator's workup for the `palm_press.mode: jacobian`
force controller — what it is, how to run it, what to watch for, and the verifiable math.

For the position/stiffness controller (the default `mode: position`) and the intuition, see
[`palm_force_control.md`](palm_force_control.md) and
[`palm_force_control_explained.md`](palm_force_control_explained.md). This is the "Road 1"
the explained doc's §8 sketched, now built.

---

## 1. What it is, in one paragraph

In **position mode** the grip force is a *side effect* of aiming the palms slightly past the
object and letting the stiff joint servo (`kp`) push. In **jacobian mode** we instead hold the
palms at a fixed "just touching" **seat** and command the grip force **directly** as a joint
**torque feedforward**: a desired inward palm force `F_cmd` (N) is turned into a Cartesian
wrench `W`, mapped to joint torque by the end-effector Jacobian `τ_press = JᵀW`, and added on
top of the gravity-compensation torque before `set_joint_targets`. **`kp` is unchanged** — this
is the doc's "Road 1", not the low-`kp` impedance path (Road 2). The advantage over position
mode is responsiveness: the feedforward is applied at the SDK's 250 Hz servo rate rather than
built up by the ~6–10 Hz outer position loop. A light force-feedback **trim** corrects for
contact compliance so the *delivered* force matches the setpoint.

Safety net: the **contact-loss watchdog** (shared with position mode) zeros the wrench the
instant the measured palm force falls away — the robot never keeps squeezing a head that has
dropped or slipped out.

---

## 2. Per-step data flow (jacobian mode, Phase 2)

```
palm taxels (14x8 per hand)  ──►  PalmForceReader.target_force()  = F_meas  [N]   (min of 2 palms)
                                        │
PalmContactWatchdog.update(F_meas) ─────┤  (arm on touch; trip on loss -> zero the wrench)
                                        ▼
PalmForceController.step(F_meas):   F_des = ramp(0 -> force_target)
                                    trim += kappa_f*(F_des - F_meas)
                                    F_cmd  = clip(F_des + trim, 0, force_max)      [N]
                                        │  (0 if watchdog.press_frozen)
                                        ▼
compute_press_wrenches(theta, F_cmd)  ──►  W_L=[-F_cmd*u,0], W_R=[+F_cmd*u,0]   u=Rx(theta)*ŷ
                                        │
compute_hand_targets(c, w, theta, squeeze=seat)  ──►  L,R SE3   (seat is FIXED, not force-driven)
                                        ▼
ik.solve_ik(L,R)  ──►  sol_q, sol_tauff(gravity)
ik.ee_jacobians(sol_q)  ──►  J_L, J_R  (6x14, LOCAL_WORLD_ALIGNED / pelvis)
                                        ▼
tau_press = clip(J_Lᵀ W_L + J_Rᵀ W_R, ±tau_press_max)          [N·m, 14]
tau_ff    = sol_tauff + tau_press
robot.arm.set_joint_targets(sol_q, tau_ff)                     [q AND tau_ff, one call]
   └─ SDK 250 Hz: tau_motor = kp*(sol_q - q) + kd*(-dq) + tau_ff   (re-sent between IK solves)
```

Files: `ik_solver.py` (`ee_jacobians`), `trajectory/targets.py` (`compute_press_wrenches`,
`compute_hand_targets`), `controllers/palm_force.py` (`PalmForceController`,
`PalmForceReader`, `PalmContactWatchdog`), `controllers/sdk_pose_tracking.py` (Phase-2 wiring).

**Reference magnitudes** (measured at the seat grasp pose, θ=0):

| F_cmd | peak |τ_press| joint | vs. gravity τ_ff peak |
|------:|------:|-------|------:|
| 1 N | 0.33 N·m | shoulder_yaw | 4.7 N·m |
| 5 N | 1.65 N·m | shoulder_yaw | 4.7 N·m |
| 8 N | 2.63 N·m | shoulder_yaw | 4.7 N·m |

So the press torque is small next to gravity comp, and the default `tau_press_max: 8.0` N·m is
a *loose* safety cap (real press torque at max force is ~2.6 N·m) — it catches Jacobian
blow-ups / runaways, not normal operation. You can tighten it to ~4 N·m for a snugger guard.

---

## 3. Config reference (`configs/default.yaml` → `palm_press`)

| Key | Meaning | Touch when |
|---|---|---|
| `enabled` | palm-press grip on (fingers static, palms press) | leave `true` |
| `mode` | `position` (default, s-depth loop) or `jacobian` (this doc) | **flip to `jacobian` live, after L1** |
| `counts_per_newton` | palm-count → Newton calibration | **MUST calibrate first** (`read_palm_force.py --calibrate`) |
| `force_target` | `F_des` setpoint (N) | set by pull-test |
| `force_max` | hard clamp on `F_cmd` (N) | crush ceiling |
| `force_min` | position-mode hold floor (N) | position mode only |
| `kappa` | position-mode integral gain (m/N) | position mode only |
| `squeeze_max` | position-mode max depth (m) | position mode only |
| `kappa_f` | **jacobian** force-loop trim gain (N per step per N err) | tune at L2 (start small) |
| `seat_squeeze` | fixed "just touching" seat depth (m) | 0.0 if `box.size[1]` = real width |
| `f_des_ramp_time` | ramp `F_des` 0→target (s) — soft start | 1–3 s |
| `force_feedforward` | include the `F_des` feedforward term (fast) vs pure-integral | `true` |
| `tau_press_max` | per-joint saturation on `tau_press` (N·m) | 4–8 |
| `touch_threshold` | watchdog arms when min-palm ≥ this (N) | just above sensor noise |
| `loss_floor` | watchdog "contact lost" below this (N) | ≤ `touch_threshold` |
| `loss_frames` | consecutive below-floor frames to latch trip | 3 |
| `baseline_frames`, `noise_threshold`, `signal` | signal conditioning | as position mode |

`force_target` and `force_max` are **shared** by both modes; the watchdog keys are shared too.

---

## 4. Script arguments reference

Everything a live operator touches. **`--no-dry-run` = actuation = human-run only, E-stop in
reach** (Hard Rules). Default is always `--dry-run`.

### `scripts/read_palm_force.py` — calibrate + inspect the palm sensor (READ-ONLY hardware)
| Flag | Effect |
|---|---|
| `--side {both,left,right}` | which hand(s); calibrate **one at a time** |
| `--calibrate <N>` / `--calibrate-kg <kg>` | rest a known load, prints `counts_per_newton`, exits |
| `--baseline` (`--baseline-frames N`) | capture & subtract the no-contact rest offset |
| `--raw` | also print the 14×8 ASCII heatmap |
| `--noise-threshold <c>` | counts above baseline to call a cell "active" |
| `--rate <Hz>`, `--duration <s>` | read loop rate / run length |
| `--left-ip`, `--right-ip` | Modbus IPs |
| `--dry-run` / `--no-dry-run` | synthetic vs live hands (Modbus; read-only but connects) |

**Do this first.** Without `counts_per_newton`, every force number (targets, watchdog
thresholds) is in raw counts and meaningless.

### `scripts/grasp_demo.py` — finger open/close sanity (no arm force)
| Flag | Effect |
|---|---|
| `--side {both,left,right}` | which hand(s) to actuate |
| `--hold-time <s>` | seconds to hold the grasp |
| `--preview` | Rerun viewer with per-finger force |
| `--rate <Hz>` | hand interpolation rate |
| `--net <iface>` | DDS interface (ignored in dry-run) |
| `--dry-run` / `--no-dry-run` | simulate vs live |

Run `--no-dry-run` here before any pose-tracking live run — confirms hands actuate and the
guaranteed release/return works.

### `scripts/run_pose_tracking_preview.py` — the main controller (arms + palm press)
| Flag | Effect |
|---|---|
| `--source {sine,marker,mock}` | trajectory: sine wave / live AprilTag / animated mock |
| `--tracking {orient-only,6dof}` | marker/mock: rotation-only vs also translation |
| `--detect` | scan AprilTag IDs 5 s and exit (needs `--source marker`) |
| `--config <path>` | alternate YAML |
| `--net <iface>` | DDS interface (ignored in dry-run) |
| `--dry-run` / `--no-dry-run` | Rerun preview (no robot) vs **live** |

Jacobian mode is selected in the **config** (`palm_press.mode: jacobian`), not a CLI flag, so
it applies to whichever `--source` you run.

---

## 5. Live bring-up order (human-run)

Do these in order; do not skip. Each gate de-risks the next.

0. **Calibrate** `counts_per_newton` per hand (`read_palm_force.py --no-dry-run --calibrate-kg`).
   Set the watchdog `touch_threshold`/`loss_floor` in real Newtons.
1. **Fingers** — `grasp_demo.py --no-dry-run`; confirm actuation + guaranteed release.
2. **Dry preview** — `run_pose_tracking_preview.py --source mock` with `mode: jacobian`; confirms
   the wiring runs and Rerun shows sane arms. (Note: dry-run winds `F_cmd` to `force_max` — it
   has no sensor feedback; see Pitfalls. This step validates kinematics, not force.)
3. **L1 — sign check (tiny force, position net still under it).** Temporarily keep
   `mode: position` but add a *small* `tau_press` on top (or set `mode: jacobian`, `force_target`
   ≈ 0.5–1 N, `seat_squeeze` small-positive so position still holds). Confirm the palms push
   **together** (not apart, not sideways) and the magnitude is sane. If they push apart/sideways,
   stop — the wrench sign or Jacobian frame is wrong (see Pitfalls).
4. **L2 — Road 1.** `mode: jacobian`, `seat_squeeze ≈ 0`, `f_des_ramp_time` 1–3 s, small
   `force_max`, `kappa_f` small. Raise `force_target` by pull-test until the object holds; set
   `force_max` above it. Mid-hold, **remove the object** and confirm the `[SAFETY]` freeze fires
   and the arms do not clap. Watch the `Palm force[jacobian]` diagnostic: `F_cmd`, `F_meas`,
   `|tau_press|max`.

---

## 6. Pitfalls & debugging

| Symptom | Likely cause | What to check / do |
|---|---|---|
| Palms push **apart** or sideways | wrench sign or Jacobian frame wrong | Offline `test_press_wrenches`/`test_ee_jacobians` pass (mirror-symmetric τ), so suspect a live convention flip. Convention here: `W` = force the palm exerts **on the object**, inward; `τ=JᵀW`; `LOCAL_WORLD_ALIGNED`. This matches gravity-comp's proven sign channel. |
| `|tau_press|max` == `tau_press_max` at low `F_cmd` | Jacobian near-singular, or `F_cmd` ran away | Don't just raise the cap. Check arm pose isn't near a singularity; check `F_cmd` in the diagnostic isn't pinned at `force_max` (means `F_meas` reads ~0 → no contact / bad calibration). |
| Grip too weak / too strong vs. setpoint | contact attenuation `γ=k_e/(k_e+K_cart)`; `counts_per_newton` off | The trim corrects steady-state IF `kappa_f`>0 and `F_meas` is real. Verify calibration; increase `kappa_f` (watch stability, §8). |
| Palms don't touch at seat / drift inward | `seat_squeeze` vs real object width mismatch | `seat=0` assumes `box.size[1]` = real object width. Set `box.size[1]` to the real head width, or bump `seat_squeeze` so first contact registers on the sensor. |
| Wrist jitter / buzzing | `tau_press` is applied unfiltered (bypasses the tauff EMA, by design) | Lower `kappa_f`, raise `f_des_ramp_time`, ensure `RateLimitedSource` is on (θ steps). The gravity term is still EMA-smoothed; only the press is prompt. |
| Arm droops / press direction corrupted | gravity comp off or wrong `INSPIRE_HAND_MASS` | Keep `gravity_comp.enabled: true`; verify the arm holds pose with `force_target: 0` before pressing. |
| Force lags fast head motion | J computed per IK solve (~6–10 Hz); SDK holds `tau_ff` between solves | Expected — same rate floor as position mode. Move slowly, or this is where Road 2 would help (out of scope). |
| Dry-run shows `F_cmd` climbing to `force_max` | `PalmForceReader` returns 0 N in dry-run → trim winds up against `F_meas=0` | **Not a bug.** Dry-run has no sensor feedback and does not represent real force behavior. Use dry-run only to validate wiring, never to tune force. |
| Watchdog never trips / thresholds meaningless | `counts_per_newton` not calibrated | Calibrate first; thresholds are in Newtons only after calibration. |
| Contact-loss clap (should not happen) | watchdog disabled / thresholds wrong | The watchdog zeros the wrench on trip; verify `touch_threshold`/`loss_floor` are sane and the `[SAFETY]` line prints on object removal. |

---

## 7. What was deliberately NOT done (and why)

Kept out of scope (the "unnecessary complexity" — see
[`palm_force_control_explained.md`](palm_force_control_explained.md) §7): lowering/retuning the
SDK `arm_kps`/`arm_kds`, directional/hybrid Cartesian-impedance stiffness (Road 2 / Tier B/C),
and re-measuring hand inertials for a "safety-critical" gravity comp (§F). `kp` is untouched;
the seat-pin + trim make the feedforward accurate without softening the servo. Revisit Road 2
only if the force isn't crisp enough on *soft or fast-moving* contact — a head hold is neither.

---

## 8. Math (paper-ready, verifiable)

**8.1 Jacobian force relation (virtual work).** The EE Jacobian `J(q) ∈ ℝ^{6×14}` maps joint
velocity to the EE spatial twist, `v = J(q) q̇`, `v = [ẋ; ω]`, expressed in the
`LOCAL_WORLD_ALIGNED` frame (axes aligned to the pelvis/base, origin at the EE point
`L_ee_link`/`R_ee_link`). For a wrench `W = [f; m]` the end-effector exerts **on the object**,
expressed in the *same* frame, virtual work gives the joint torque:

```
δW = τᵀ δq  =  Wᵀ δx  =  Wᵀ J δq   ∀ δq   ⇒   τ = J(q)ᵀ W.
```

`W` and `J` **must share a frame** or the direction is wrong (this is the #1 live pitfall).
Verified offline by `test_ee_jacobians::test_jacobian_matches_central_difference`
(`FK(q+δ)−FK(q) ≈ J δ`).

**8.2 Inter-palm wrench.** With object roll `θ` about robot X, the inter-palm axis is
`u(θ) = Rx(θ) ŷ`, `ŷ = [0,1,0]`. The left palm sits at `+w·u` and presses toward center
(`−u`); the right at `−w·u` presses `+u`:

```
W_L = [ −F_cmd · u(θ);  0 ],      W_R = [ +F_cmd · u(θ);  0 ]      (pure force, zero moment)
τ_press = J_Lᵀ W_L + J_Rᵀ W_R.
```

Each arm's Jacobian is structurally zero in the other arm's 7 columns, so `τ_press` loads each
arm from its own wrench only (`test_single_arm_wrench_loads_only_its_arm`).

**8.3 Total feedforward (superposition).** `RNEA(model, q, v=0, a=0) = g(q)`, the pure gravity
generalized force, is independent of any external wrench, so

```
tau_ff = g(q) + τ_press,        tau_motor = kp(sol_q − q) + kd(−q̇) + tau_ff   (SDK, 250 Hz).
```

**8.4 Closed-loop force law (feedforward + trim).** `F_meas` = combined palm force
(min of two hands). With ramp `F_des,k = force_target · clip((t−t₀)/T_ramp, 0, 1)`:

```
trim_{k+1} = clip( trim_k + κ_f (F_des,k − F_meas,k),  −F_max, +F_max )
F_cmd,k    = clip( [ff]·F_des,k + trim_k,  0, F_max ),     [ff] ∈ {0,1} = force_feedforward
```

Fixed point (steady `F_des`, unsaturated): `trim` stops changing ⇔ `F_meas = F_des`. So the
**delivered** force equals the setpoint regardless of contact compliance.

**8.5 Contact transmission — why the seat is pinned and the trim exists.** Model contact as a
spring `k_e` in series with the arm's Cartesian stiffness `K_cart` (from `kp`). Quasi-static,
gravity handled, seat pinned (position error ≈ 0):

```
F_deliver = γ · F_cmd,      γ = k_e / (k_e + K_cart) ∈ (0, 1].
```

Rigid object → `γ→1` (commanded ≈ delivered). Compliant pad/object → `γ<1`; the trim raises
`F_cmd` by `1/γ` so `F_deliver = F_des`. Pinning the seat removes the *unknown-surface* position
force `K_cart·Δ` that plain feedforward would ride on top of (explained doc §7).

**8.6 Stability.** Linearize `F_meas = γ F_cmd`. The trim recursion has pole `1 − κ_f γ`;
asymptotically stable and non-oscillatory for `0 < κ_f γ ≤ 1` (bounded up to `< 2`). Pick
`κ_f < 1/γ̂` from a measured `ΔF_meas/ΔF_cmd`. (Mirror of the position loop's `0<κg<2` bound in
[`palm_force_control.md`](palm_force_control.md) §3.6.)

**8.7 Why `kp` is unchanged.** The press is delivered by `τ_press` through `γ`; with the seat
pinned the residual position force is ≈0, so we never soften `kp` to "let the feedforward
through." Softening `kp` would (a) worsen tracking — the SDK sends `q̇_des = 0`, so a moving
`q_des` is chased by `kp` alone (lag ≈ load/`kp`) — and (b) make gravity comp the *primary*
support. Road 1 avoids both.

---

### See also
- [`palm_force_control.md`](palm_force_control.md) — position/stiffness mode spec + math.
- [`palm_force_control_explained.md`](palm_force_control_explained.md) — intuition + the
  position-vs-Jacobian tiers this doc's Road 1 came from.
- Tests: `tests/test_ee_jacobians.py`, `tests/test_press_wrenches.py`,
  `tests/test_palm_force_controller.py`, `tests/test_palm_contact_watchdog.py`,
  `tests/test_pose_tracking_contact_loss.py`.
