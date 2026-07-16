# Palm-Force Control — Deep Dive (how it actually works)

This is the **from-first-principles explanation** of how the palm grip force is
controlled today, why it works, and how hard it would be to switch to a
Jacobian/torque approach. For the terse spec + config keys, see
[`palm_force_control.md`](palm_force_control.md); this doc is the "explain it to me
like I need to understand it" companion.

The single most important idea, up front:

> **We never command a force. We command a *position* — where the palms should be —
> and deliberately aim that position slightly *inside* the object. The arm can't get
> there (the object is in the way), so it pushes. That push *is* the grip force. To
> press harder, we aim a little deeper.**

Everything below is an unpacking of that sentence.

---

## 1. The mental model: the arm is a spring, the object is a wall

Think of each arm as a stiff spring. When you tell the arm "put your palm at position
X," the low-level motor controller behaves like a spring pulling the palm toward X: the
farther the palm is from X, the harder the motors push. (This is literally what a PD
position controller is — a programmable spring. More on the exact numbers in §4.)

Now put an object between the two palms and command each palm to a point that is
**1 cm past the object's surface**. The palm physically can't reach that commanded
point — the object blocks it. So the palm ends up resting *on* the object surface, but
the controller still "wants" it 1 cm deeper. That 1 cm of unreachable, leftover
distance is the spring's stretch, and the spring pushes on the object with a force
proportional to it. **That force is the grip force.**

- Aim 1 cm past the surface → some force.
- Aim 2 cm past the surface → about twice the force.
- Aim exactly *at* the surface → (ideally) zero force.

So "how hard do we grip?" becomes "how far past the surface do we aim?" — and that aim
distance is the one number our controller actually computes. We call it the **press
depth `s`**.

The whole system is just a loop that adjusts `s` until the measured force equals the
force we want.

---

## 2. The five layers, top to bottom

Force control here is a stack of five layers. Each one takes a simple input and
produces the input to the next. Reading top-to-bottom is "sense → decide," and the
physical world closes the loop at the bottom.

```
        ┌────────────────────────────────────────────────────────────┐
  L0    │ PALM SENSOR   14x8 taxels  ── sum & calibrate ──►  F  [N]    │  ~ as fast as we read
        └────────────────────────────────────────────────────────────┘
                                   │  measured force
        ┌────────────────────────────────────────────────────────────┐
  L1    │ PRESS LAW     s += kappa*(F_des - F)          ──►  s  [m]    │  once per outer tick
        └────────────────────────────────────────────────────────────┘
                                   │  press depth
        ┌────────────────────────────────────────────────────────────┐
  L2    │ GEOMETRY      w = w0 - s ; palms at ±w        ──►  xyz  [m]  │  once per outer tick
        └────────────────────────────────────────────────────────────┘
                                   │  where each palm should be
        ┌────────────────────────────────────────────────────────────┐
  L3    │ INVERSE KIN.  hand xyz  ─►  14 joint angles   ──►  q  [rad]  │  ~6-10 Hz (IK is slow)
        └────────────────────────────────────────────────────────────┘
                                   │  joint targets
        ┌────────────────────────────────────────────────────────────┐
  L4    │ MOTOR SERVO   tau = kp*(q_des - q) + kd*(-dq) + tau_ff       │  250 Hz (SDK thread)
        │               motors push arm toward q_des; object resists   │
        └────────────────────────────────────────────────────────────┘
                                   │  arm moves / presses
                              ┌─────────┐
                              │ OBJECT  │  resists → builds contact force → sensed at L0
                              └─────────┘
```

| Layer | Job | Input → Output (units) | Code |
|---|---|---|---|
| **L0 sensor** | Turn taxels into a force number | 14×8 counts → `F` (N) | `PalmForceReader`, `palm_force_newtons` — [palm_force.py](../g1_spine_stabilization/controllers/palm_force.py) |
| **L1 press law** | Decide how deep to aim | `F_des − F` (N) → `s` (m) | `PalmPressController.step` — [palm_force.py](../g1_spine_stabilization/controllers/palm_force.py) |
| **L2 geometry** | Turn press depth into palm positions | `s` (m) → two `xyz` (m) | `compute_hand_targets(squeeze=s)` — [targets.py](../g1_spine_stabilization/trajectory/targets.py) |
| **L3 IK** | Turn palm positions into joint angles | 2×`xyz` → `q` (14 rad) | `G1HandIK.solve_ik` — [ik_solver.py](../g1_spine_stabilization/kinematics/ik_solver.py) |
| **L4 servo** | Push motors toward `q`, feel the object | `q` (rad) → motor torque → motion | SDK `G1Arm._live_publisher_loop` — [g1_arm.py](../../g1_inspire_sdk/g1_inspire_sdk/g1_arm.py) |

The important thing: **force only appears at L4**, and it appears as a *side effect* of
the servo trying to reach a position it can't. Layers L1–L3 are pure kinematics — they
just compute geometry. Nobody in L1–L3 ever computes a force or a torque.

---

## 3. One control tick, with real numbers

Let's walk a single pass through the stack. Assume you've calibrated
`counts_per_newton ≈ 365` (a real value from a dry-run calibration) and set
`force_target = 5 N`. The object is already held between the palms and currently
sensed at **3 N**. Config: `kappa = 0.0005 m/N`, `squeeze_max = 0.02 m`,
`box_half_width w0 = 0.055 m`. Suppose the current press depth is `s = 6 mm`.

**L0 — read force.** The left palm's 112 baseline-subtracted counts sum to ≈ 1095.
`F_L = 1095 / 365 ≈ 3.0 N`. Same for the right. Combined target signal
`F = min(F_L, F_R) = 3.0 N`.

**L1 — press law.** Error `= F_des − F = 5 − 3 = 2 N`.
`Δs = kappa · error = 0.0005 × 2 = 0.001 m = 1 mm`.
New `s = 6 mm + 1 mm = 7 mm` (still under `squeeze_max = 20 mm`, so no clamp).

**L2 — geometry.** Effective half-width `w = w0 − s = 0.055 − 0.007 = 0.048 m`.
Each palm target moves from ±0.049 m to ±0.048 m about the object center — i.e. **each
palm is now aimed 1 mm further inward** (2 mm closer to each other).

**L3 — IK.** The left target becomes `p_L = [0.3549, +0.048, 0.0952]` (was `+0.049`),
right mirrored. `solve_ik` returns 14 joint angles that place the palms there — a shift
of a fraction of a degree to ~1° across the shoulder/elbow/wrist joints.

**L4 — servo.** The new `q_des` corresponds to palms 1 mm *deeper into the object* than
before. The object blocks that last millimeter, so the arm can't reach `q_des`; the
position error grows slightly, the motors push a bit harder, and the contact force
rises — say from 3.0 N to ≈ 3.5 N by the next read.

**Next tick:** error is now `5 − 3.5 = 1.5 N`, so `Δs = 0.75 mm`, force climbs to
≈ 3.9 N… and so on. Each tick closes ~25% of the remaining gap (see §4 for why), so it
homes in on 5 N over roughly 10–15 ticks (~1.5–2.5 s at the real loop rate). Once
`F = 5 N`, the error is 0, `Δs = 0`, and `s` stops growing — **the loop has found the
exact aim-depth that produces 5 N.** That is the whole controller.

---

## 4. Why aiming deeper produces a *specific* force (the spring math)

Layer L4 is where physics turns position into force. The SDK sends each motor this
command every 4 ms ([g1_arm.py](../../g1_inspire_sdk/g1_inspire_sdk/g1_arm.py) —
`_set_motor_command` / `_live_publisher_loop`):

```
tau_motor = kp * (q_des - q) + kd * (0 - dq) + tau_ff
```

with fixed per-joint gains (from the SDK's `g1.yaml`), per arm:

```
kp = [100, 100, 50, 50, 30, 30, 30]     # shoulder_pitch, shoulder_roll, shoulder_yaw,
kd = [  2,   2,  2,  2,  1,  1,  1]      # elbow, wrist_roll, wrist_pitch, wrist_yaw
tau_ff = sol_tauff  (gravity compensation only — see §5)
```

The `kp * (q_des - q)` term is the programmable spring: torque proportional to the
position error `Δq = q_des - q`. When the palm is pressed against the object and can't
reach `q_des`, `Δq` settles at a nonzero value and the joint holds a steady torque
`tau = kp * Δq`.

That joint torque becomes an end-effector force through the arm geometry. Roughly, if
one joint of stiffness `kp` (N·m/rad) dominates the inward motion and the palm sits a
lever arm `L ≈ 0.35 m` from that joint, then a Cartesian press depth `Δy` at the palm
corresponds to a joint deflection `Δq ≈ Δy / L`, producing a palm force:

```
F ≈ tau / L = kp * Δq / L = kp * (Δy / L) / L = (kp / L²) * Δy
```

Plugging in a shoulder-dominated `kp = 100` and `L = 0.35 m`:

```
F ≈ (100 / 0.35²) * Δy ≈ 800 * Δy      [N, with Δy in meters]
```

So the arm behaves like a **Cartesian spring of stiffness ≈ 800 N/m**. To hold ~5 N it
needs about `Δy ≈ 5 / 800 ≈ 6 mm` of "aim past the surface" — which is exactly the kind
of press depth `s` the loop converges to. This also tells you the **max** force: with
`squeeze_max = 20 mm`, the arm-spring alone tops out around `800 × 0.02 ≈ 16 N` (and in
practice the motor torque limit caps it sooner).

Two honest caveats on the numbers:
- `800 N/m` is a single-joint back-of-envelope. The true Cartesian stiffness is the
  full `K_cart = J⁻ᵀ · diag(kp) · J⁻¹` across all 14 joints and varies with arm pose. The
  *concept* — force ≈ stiffness × aim-depth — is exact; the constant is a stand-in.
- The force the outer loop actually sees per millimeter of `s` (the "contact stiffness"
  `ΔF/Δs`) is the arm-spring **in series with** the object's own squishiness. A rigid
  object → `ΔF/Δs ≈ 800 N/m` (arm-limited); a soft object → softer. In §3 I used
  ≈ 500 N/m, i.e. an ~800 N/m arm pressing a somewhat compliant object. This series
  stiffness is what sets the convergence speed:

```
gap-close ratio per tick = 1 - kappa * (ΔF/Δs)
   e.g. 1 - 0.0005 * 500 = 0.75   → 25% of the error removed each tick (as in §3)
```

For a stable, non-oscillating loop you want `0 < kappa * (ΔF/Δs) < 1`. That is the one
rule for choosing `kappa`: measure `ΔF/Δs` on the real object (press by hand, watch the
readout), then keep `kappa` below `1 / (ΔF/Δs)`.

---

## 5. The two loops, the rates, and the safety rails

There are two nested loops running at very different speeds:

- **Inner servo loop — 250 Hz (SDK thread).** Constantly drives the motors toward
  whatever `q_des` currently is. This is the "spring." We don't touch it; we only feed
  it targets.
- **Outer force loop — ~6–10 Hz (our Phase-2 loop).** Reads force, updates `s`, runs IK,
  sets the new `q_des`. It's slow because `solve_ik` (IPOPT) costs ~100 ms per call, so
  the loop runs far below the nominal 50 Hz. That's *fine* for a press loop — the object
  isn't going anywhere in 100 ms — and it's why `kappa` is defined **per step**, not per
  second.

`tau_ff` note: the servo also adds a feedforward torque `tau_ff`, but today that carries
**only gravity compensation** — `sol_tauff = pin.rnea(model, data, q, v=0, a=0)`
([ik_solver.py](../g1_spine_stabilization/kinematics/ik_solver.py) — RNEA with zero
velocity and acceleration is exactly the gravity-hold torque). It holds the arm's own
weight up; it contributes **nothing** to the press. This channel is the hook we'd reuse
for a Jacobian force (§6–7), but right now it is not carrying any grip force.

Safety rails, all in the `palm_press` config block:
- `squeeze_max` (0.02 m) — hard cap on how deep we can aim; bounds the max force
  regardless of sensor error. Must stay `< box_half_width` so the palms can never be
  commanded through the object's center.
- `force_max` (5 N) — crush guard: if the harder-pressed palm exceeds it, the loop is
  forced to *retreat* `s` even if the target-side reads low (`guard_force() =
  max(F_L, F_R)`).
- `force_target: 0.0` (default) — with a 0 N setpoint the loop never presses, so the
  feature is safe to leave enabled before you've calibrated and tuned.

Full grasp workflow this loop lives inside (all in
[sdk_pose_tracking.py](../g1_spine_stabilization/controllers/sdk_pose_tracking.py)):
**Phase 1a** reach a pre-grasp standoff (palms wider than the object) → **Phase 1b**
close straight in along the inter-palm axis → **Phase 1.5** close fingers to hold + grab
the no-contact palm baseline → **Phase 2** run this press loop while tracking the target.

---

## 6. Position control vs. Jacobian/torque control

What we do **(position / stiffness control):** compute *where* to put the palms, let the
fixed-stiffness servo turn "aim past the surface" into force. Force is an *emergent*
property of a position command.

The alternative **(Jacobian / explicit force control):** compute the force you want as a
Cartesian wrench `W`, convert it directly to joint torques with the arm Jacobian,
`tau_press = Jᵀ(q) · W`, and command *that torque* to the motors. Force is *commanded
directly*, not conjured from a position error.

The key realization that makes these two closer than they look: our servo is a fixed
`kp` spring, and a spring already maps deflection to force (`F = K_cart · Δy`). **So our
position controller already _is_ a force controller — a stiffness one.** Switching to
`Jᵀ W` mostly changes *where the force is decided* (explicitly, up front, vs. emerging
from the position error) rather than granting a brand-new capability — **until you also
lower `kp`.** The stiffness of the servo is the whole story; the next subsection makes
that precise.

### Does a feedforward force get "swallowed"? It depends on the contact stiffness

A natural objection: *during a press the arm is stalled against the object and isn't
moving, so surely the servo won't fight an added feedforward torque?* That intuition is
**right for a rigid contact** — and my earlier "it gets swallowed" wording was too
absolute. Here is the actual result.

Quasi-static, gravity already handled by `tau_ff`, one joint pressing an object modeled
as a contact spring of stiffness `k_e`. Balancing the motor torque against the object's
reaction gives the grip torque that actually reaches the object:

```
τ_grip = [ k_e / (k_e + kp) ] · ( kp·(q_des − q_contact) + τ_ff )
```

so the fraction of any feedforward that shows up as real contact force is:

```
∂τ_grip / ∂τ_ff  =  k_e / (k_e + kp)
```

- **Rigid object (`k_e ≫ kp`): fraction → 1.** The palm can't move (the object stops it),
  so the added torque transmits almost fully to force. **The "arm isn't moving" intuition
  is correct here — nothing is swallowed.**
- **Soft object (`k_e ≪ kp`): fraction → `k_e/kp` (small).** The arm *can* sink into a
  compliant object, so adding `τ_ff` makes it drift inward; its own position-error term
  `kp·(q_des − q)` drops by nearly what you added, and net force barely changes. That is
  the "swallowing" — and the mechanism is **the arm moving into the object, not the servo
  actively resisting.** (The damping term `kd·(−q̇)` is ~0 when the arm is stalled, so
  there is no active fight against a static torque.)

Real palm-pad-on-object contact is *partly* compliant, so in practice you land somewhere
between the two extremes (both `k_e` and `K_cart ≈ 800 N/m` are in the hundreds–thousands
N/m range) — a fraction well below 1 but not near 0.

**But transmitting is not the same as controlling.** Even in the rigid case where the
feedforward passes through, two things stop it from being *clean* force control — see the
Tier-A "catch" in §7 below (it adds on top of an unknown position force, and the outer
loop cancels it). Both are cured only by lowering `kp`.

*(Aside: the "arm deflects only `Δq = τ/kp`" picture is the **free-space** case — no
object. There, a feedforward torque just repositions the arm to a new equilibrium and
produces **zero** net force. That is a different situation from pressing on something,
where the `k_e/(k_e+kp)` result above applies.)*

---

## 7. How hard is it to switch? (three tiers)

The plumbing is *mostly already there*: Pinocchio can produce the EE Jacobian in one
call (`pin.computeFrameJacobian(model, data, q, L_ee_id)` — the reduced model and EE
frame IDs already live in `G1HandIK`), and the SDK's `set_joint_targets(q, tau_ff)`
already carries a feedforward torque (that's how gravity comp rides in). So adding
`Jᵀ W` to the torque is not the hard part. The hard part is the **fixed-stiffness servo**.

| Tier | What you'd do | Effort | The catch |
|---|---|---|---|
| **A. Feedforward assist** | Keep the position press exactly as-is; additionally compute `Jᵀ W_des` and **add it to `sol_tauff`** before `set_joint_targets`. | ~half a day; hardware-free testable | Not clean force control at high `kp` — but *not* because the servo "resists" it. Two reasons (worked below): it rides on top of an unknown position force, and the outer `s` loop cancels it. A useful *bias/assist* at best. |
| **B. Soft servo + torque force loop** | Lower the arm `kp` (SDK `arm_kps` — a config value, not SDK code) so the feedforward torque actually dominates near contact, and run the force loop on `W_des` instead of on `s`. | ~1–2 days + live tuning | Weakening the position spring means gravity comp must be near-perfect (nothing else holds the arm up), and it needs careful live tuning + safety gating. This is where you get genuinely clean force control. |
| **C. Pure Jacobian/impedance** | `kp ≈ 0`: command `tau = gravity(q) + Jᵀ W_des` only; the arm is torque-controlled with no position holding. | multi-day, safety-heavy | The arm goes "limp" except for gravity + press torque. Fights the SDK's velocity-clip and position-target assumptions; needs a stability analysis and strong E-stop discipline. Essentially re-architecting the arm control mode. |

### Why Tier A isn't clean force control (even when the feedforward transmits)

Take the rigid case where the feedforward *does* reach the contact (`k_e/(k_e+kp) ≈ 1`).
Two problems remain, and both are cured only by lowering `kp`:

1. **It rides on top of an *unknown* position force.** Pressing a rigid object, the arm
   stalls *at* the surface, so the total grip is `K_cart·Δ + τ_ff`, where `Δ` = (where you
   aimed) − (the surface). You command the aim, but you **don't know where the surface is**
   — that's the whole reason we have a force sensor. So if your surface estimate is off by
   just 2 mm, the position term contributes `800 N/m × 0.002 m = 1.6 N` of force you never
   asked for, on top of the feedforward's setpoint — and you can't correct it, because the
   surface is unknown. Lower `kp` 10× (`K_cart → 80 N/m`) and that same 2 mm error becomes
   `0.16 N` — now the feedforward dominates and the leftover position force is negligible.

2. **The outer `s` loop cancels it.** If we keep regulating force by position (the squeeze
   loop) and *also* add feedforward: the feedforward raises `F_measured`, the `s` loop sees
   it exceed `F_des` and **backs off `s`**, which drops the position force, until
   `F_measured = F_des` again. Same delivered force — only the *split* between position and
   feedforward changed. Two knobs (`s` and `τ_ff`) move one output (force), but only one
   loop closes on it, so the loop counteracts the other knob. (It can still speed up the
   *transient* — feedforward delivers force faster than the slow ~6–10 Hz `s` loop can ramp
   — which is the one genuine benefit, hence "assist.")

So to make feedforward actually *be* the force controller (Tier B) you must **(a)** stop
regulating force with `s` — let `s` just place the hands kinematically at the nominal
surface — **and (b)** lower `kp` so the leftover `K_cart·Δ` is negligible, then close the
force loop on `τ_ff` via `Jᵀ`. That's a control-architecture change, not a config tweak.

**Bottom line:** the current positional controller is a sensible fit for this arm — it's
a stiff position servo, and stiff position servos are natural stiffness force
controllers. **Tier A** (add `Jᵀ W` feedforward on top of the position press) is a cheap,
safe experiment but gives at most a transient assist — the outer loop owns the
steady-state force. **Tier B** (drop the position force loop, lower `kp`, close the force
loop on torque) is the real upgrade — worth it mainly for compliant or fast-moving
contact where regulating force through slow position updates isn't crisp enough. Tier C is
a research effort, not a tweak.

---

## 8. Implementation checklist — building the Jacobian force-control version

> **Built.** Road 1 was implemented as `palm_press.mode: jacobian` (kept `kp`, pinned seat +
> `JᵀW` feedforward + trim + contact-loss watchdog). See
> [`palm_force_jacobian.md`](palm_force_jacobian.md) for the as-built controller, per-step data
> flow, live bring-up order, debugging, and paper-ready math. The checklist below is the
> original design sketch it came from.

This is the concrete starting point if we decide to build it. Every item below names the
exact file + knob. **First read the next subsection** — it decides which of two roads you
take, and the recommended one keeps the tracking stiffness untouched.

### The kp-vs-tracking tension, and two roads

`kp` (the position *stiffness*; `kd` is the separate *damping* term) does **double duty** in
this system:
1. it makes the arm crisply **track** the object's rotation/translation trajectory, and
2. in today's design it also **generates the grip force** from position error.

These fight each other the moment you consider force control. And tracking's reliance on
`kp` is stronger than it looks: the SDK sends **no velocity feedforward** (`qd = 0` is
hardcoded in `g1_arm.py`'s publisher), so a moving `q_des` is chased by `kp` alone —
tracking lag ≈ load/`kp`. **Uniformly lowering `kp` directly degrades tracking.** So we do
*not* want a global `kp` cut.

The clean resolution is that these two jobs live on **different axes**: force is controlled
along **one** direction (the inter-palm press axis), while tracking happens in **all the
others** (the object's translation + rotation). The textbook answer is therefore
**hybrid / directional control** — stay stiff in the tracked directions, be compliant *only*
along the press axis. You never lower the global stiffness. On this arm (isotropic per-joint
`kp` only) that means synthesizing the stiffness yourself in the torque channel (Cartesian
impedance: `τ = Jᵀ K_cart (x_des − x)`, soft on the press axis, stiff elsewhere, with the SDK
`kp` set low so it doesn't add its own isotropic stiffness). That's the "proper" version, and
it's more work.

**So there are two roads:**
- **Road 1 (recommended first):** keep `kp`, get the press force from `Jᵀ W` feedforward +
  a light trim, with the contact position pinned (below). Preserves tracking; simplest.
- **Road 2 (the proper, harder path):** directional/hybrid stiffness — §E. Only worth it for
  crisp force on *soft or fast-moving* contact where Road 1's trim isn't good enough.

*(In §7's terms: Road 1 is the feedforward idea made to actually work — pinning contact and
the trim remove the two problems that made plain Tier-A feedforward a mere "bias"; Road 2 is
the low-`kp` / Tier-B direction, done directionally so it doesn't wreck tracking.)*

### Road 1 (recommended first): pin contact + feedforward + trim — keeps your tracking `kp`

The idea that makes this work without touching `kp`: **measure the surface instead of
guessing it.** Push in; the instant the palm force crosses a small threshold on each side,
snapshot that pose as the contact/seat and pin `q_des` there. Then press with feedforward
and correct with a light loop:
```
q_des   = contact pose (pinned at first-touch; NOT driven past the surface)
tau_ff  = sol_tauff + Jᵀ(F_cmd · û_inward)          # gravity + press feedforward
F_cmd  += k · (F_des − F_meas)                       # light force trim
```
- **Why pinning helps:** it removes the *unknown-surface offset* force (the 1.6 N-from-a-2 mm
  guess in §7) — you're now sitting at zero position error, so the servo adds no spurious
  offset.
- **What pinning does *not* remove:** pressing a compliant object still makes the arm
  penetrate slightly, which the stiff servo resists, so the *delivered* force is
  `F_cmd · K_env/(K_env + K_cart)` — attenuated, and object-dependent. The **trim** cancels
  that automatically (it just commands a bit more `F_cmd`; e.g. ~1.8× the net force when
  `K_cart ≈ K_env` — a few extra N·m, well within motor limits). Only a *truly rigid* object
  gives "commanded = delivered" open-loop.
- **Why it fits "keep `kp`":** it runs at your **current tracking stiffness** — no global
  `kp` cut — and the feedforward reacts at the 250 Hz servo rate, giving a *faster* force
  transient than today's 6–10 Hz squeeze loop.
- **Caveats:** costs some torque headroom; the pinned contact **goes stale during tracking**
  (as the hands follow a moving object the true contact pose moves — re-detect continuously,
  or use this for a static/slow hold); and the **contact-loss runaway** guard (§H) is still
  mandatory. It is *not* fully open-loop — the trim is what makes it accurate on real pads.

The lettered items below (A–I) are the building blocks for both roads; §E is the Road-2-only
step. §J gives the build order.

### A. Expose the end-effector Jacobians — `kinematics/ik_solver.py`
Add a method to `G1HandIK`, e.g. `ee_jacobians(q) -> (J_L, J_R)`. The one-call form does
the kinematics internally:
```python
J_L = pin.computeFrameJacobian(self.reduced.model, self.reduced.data, q,
                               self.L_ee_id, pin.LOCAL_WORLD_ALIGNED)   # 6 x 14
J_R = pin.computeFrameJacobian(self.reduced.model, self.reduced.data, q,
                               self.R_ee_id, pin.LOCAL_WORLD_ALIGNED)   # 6 x 14
```
- **Frame convention matters:** our press force is expressed in the **pelvis frame**, so use
  `LOCAL_WORLD_ALIGNED` (axes aligned to the base, origin at the EE). The wrench `W` must be
  in the *same* frame as the Jacobian, or the torque is wrong.
- Each `J` is `6 × 14`, but only that arm's **7 columns are nonzero** (the reduced model is
  bimanual). `τ_L = J_Lᵀ W_L` therefore only loads the left joints; summing L+R is clean.
- Purely kinematic — the gravity-comp inertia correction (§F) does not affect it.

### B. Desired-wrench generator (reuse the existing geometry)
From a commanded force magnitude `F_cmd` (N) build a **pure force, zero moment** wrench per
hand pointing inward along the (rotated) inter-palm axis — the same axis `squeeze` uses:
```
û_L = -R(θ)·ŷ   (left palm presses toward center)     W_L = [F_cmd·û_L, 0, 0, 0]
û_R = +R(θ)·ŷ   (right palm presses toward center)    W_R = [F_cmd·û_R, 0, 0, 0]
```
`R(θ)` and the `ŷ` convention already live in `compute_hand_targets` /
`R_L_BASE`/`R_R_BASE` ([targets.py](../g1_spine_stabilization/trajectory/targets.py)) —
factor that direction out rather than re-deriving it.

### C. New force-domain controller — `controllers/palm_force.py`
Add a controller whose **output is a force `F_cmd` (N)**, not a depth `s` (m):
```
F_cmd = clip( F_cmd + kappa_f·(F_des − F_meas), 0, F_max )     # + optional feedforward F_cmd≈F_des
```
Keep the existing `PalmPressController` (position/`s`) as the `mode: position` fallback; this
new one is `mode: jacobian`. Reuse `PalmForceReader` for `F_meas` (add force filtering — see §H).

### D. Torque composition + wiring — `controllers/sdk_pose_tracking.py` (Phase 2)
```
q_des   = compute_hand_targets(box_center, box_half_width, θ, squeeze=seat)   # "just touching", NOT force-driven
J_L,J_R = ik.ee_jacobians(q_meas)          # from measured q
τ_press = J_Lᵀ W_L + J_Rᵀ W_R              # 14-vec
tau_ff  = sol_tauff + τ_press               # gravity + press
robot.arm.set_joint_targets(q_des, tau_ff)
```
- The force now comes from `tau_ff`, so `q_des` is a fixed seat pose (position error → ~0
  once `kp` is soft), **not** the squeeze-driven target it is today.
- **Decoupling opportunity:** the seat-pose IK is near-constant, so it can be solved rarely
  while the `J`/wrench/`tau_ff` update runs every tick from measured `q` — the force loop no
  longer has to wait on the ~100 ms IK solve (a real responsiveness win vs. the position loop).

### E. (Road 2 only) Soften the servo — and why it fights tracking
This step is **only** for Road 2 (directional/hybrid stiffness). Skip it entirely on Road 1.

At high `kp` the press feedforward is attenuated (§7), so a torque-domain force loop that
does *not* use a trim needs low `kp` to deliver commanded ≈ actual force. **But lowering `kp`
uniformly degrades tracking** — with `qd = 0` (no velocity feedforward), a moving target is
chased by `kp` alone, so lag ≈ load/`kp`. That is exactly why Road 2 is "the proper, harder
path": to keep tracking crisp you must **not** cut `kp` globally but instead synthesize
*directional* stiffness — stiff in the tracked directions, soft only on the press axis — as a
Cartesian-impedance torque (`τ = Jᵀ K_cart (x_des − x)`) while the SDK `kp` is held low so it
contributes no isotropic stiffness of its own. That reimplements the position servo in our
own torque, which is the bulk of Road 2's cost.

Setting the SDK `kp/kds` (needed for Road 2's low-isotropic-stiffness base) is Hard-Rule-safe
and must be applied **before `start_streaming()`** (the publisher snapshots `kps/kds` once at
thread start — `g1_arm.py:_live_publisher_loop`):
- Mutate in place: `robot.arm.config.arm_kps = [...]; robot.arm.config.arm_kds = [...]` in
  `scripts/run_pose_tracking_preview.py` after constructing the robot; **or**
- Ship a repo-local yaml (copy of the SDK's `g1.yaml` with lowered `left/right_arm_kps`) and
  pass it via `G1InspireRobot(config_path=...)` — the constructor already supports this.

### F. Gravity comp becomes safety-critical — `kinematics/ik_solver.py`
Today the stiff position servo masks gravity-model error. With a soft servo, `sol_tauff`
(`INSPIRE_HAND_MASS`, `_apply_hand_inertia_correction`) is the **primary** thing holding the
arm up — droop scales directly with mass/inertia error. Re-verify and, if needed, re-measure
the hand inertials before softening `kp`. Also decide whether `τ_press` should **bypass** the
`tauff_filter` EMA (the press should respond promptly, unlike the slow gravity term).

### G. Config — `configs/default.yaml` `palm_press`
Add: `mode: position | jacobian`; force-loop gain `kappa_f` (N units, distinct from the
position `kappa`); wrench/`force_max` saturation; `seat_squeeze` (fixed touch depth); the
soft `arm_kps`/`arm_kds` overrides. Keep all existing position-mode keys for fallback.

### H. Safety & stability — the genuinely hard part
- **Contact-loss runaway (most important):** under force control, if the object is dropped
  or slips, the inward press wrench has nothing to react against → the hands accelerate
  **into each other**. Must detect a force drop / `object_lost` and immediately zero `W`
  (and/or fall back to stiff position hold). The position controller does not have this
  failure mode (a position target can't run away); the force controller does.
- **Self-saturate the wrench/torque:** the SDK does **not** clamp `tau_ff` in software
  (only the position target is velocity-limited), so cap `F_cmd` and the resulting `|τ|`
  ourselves.
- **Contact instability:** a force loop through a soft servo + laggy sensing can oscillate.
  Keep `kd` damping, ramp `F_des` from zero, start with a very small target, and filter
  `F_meas` before it becomes torque (noise maps straight to commanded torque).
- Human-run bringup, E-stop in reach — same Hard Rules as any live actuation.

### I. Tests (all hardware-free)
- Jacobian correctness: finite-difference `FK(q+δ)` vs `J·δ`; assert the off-arm columns
  are ~0.
- Wrench generator: inward direction and sign at several `θ`.
- `τ = JᵀW` composition: left wrench loads only left joints, etc.
- Force-domain controller: converges to `F_des` against a linear plant, saturates at
  `F_max`, never negative.
- Contact-loss guard: force-drop → wrench zeroed.

### J. Suggested build order
1. **Offline:** A (Jacobian) + B (wrench) + I (tests) — no hardware. Both roads need these.
2. **Live sanity check (tiny force, current `kp`):** add `τ_press` on top of the existing
   position loop; confirm the arm pushes the *right way* and the magnitude is sane. Validates
   the J/wrench signs and frames with a stiff servo still holding position as a safety net.
3. **Road 1 (recommended):** pin the contact pose, keep the feedforward, add the light force
   **trim** (C/D at current `kp`), and the contact-loss guard + saturation + ramping (H).
   Tracking `kp` is untouched. Tune `k` (trim gain), `F_max`. **Do not do §E.**
4. **Road 2 (only if Road 1's force isn't crisp enough on soft/fast contact):** implement
   directional/Cartesian-impedance stiffness with low SDK `kp` (E), tighten gravity comp (F),
   re-verify tracking, then tune live.
5. **Tune live** with E-stop in reach.

---

### See also
- [`palm_force_control.md`](palm_force_control.md) — spec, config keys, paper-ready math.
- Code: [palm_force.py](../g1_spine_stabilization/controllers/palm_force.py),
  [targets.py](../g1_spine_stabilization/trajectory/targets.py),
  [ik_solver.py](../g1_spine_stabilization/kinematics/ik_solver.py),
  [sdk_pose_tracking.py](../g1_spine_stabilization/controllers/sdk_pose_tracking.py).
