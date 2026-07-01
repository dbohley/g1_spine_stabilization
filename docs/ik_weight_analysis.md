# IK Weight & Payload Analysis — Brainstorm Doc

*Self-contained context for a clean planning session. Paste this into a Claude chat.*

---

## What we have

We are building a bimanual rigid-body pose tracking controller for a Unitree G1 humanoid robot with Inspire RH56 hands. The robot holds an object in both hands and matches its pose to a moving target marker.

**IK solver (`g1_spine_stabilization/kinematics/ik_solver.py`):**

- `G1HandIK` uses Pinocchio + CasADi/IPOPT on a **14-DOF reduced arm model** (7 DOF per arm; legs, waist, fingers locked out).
- Cost function (code constants — cannot change without explicit approval):
  ```
  50 · ||pos_err||²  (translation)
  +  5 · ||rot_err||²  (rotation)
  + 0.02 · ||q||²     (regularization toward zero)
  + 0.1 · ||q - q_prev||²  (smoothness)
  ```
- Output smoothing: `WeightedMovingFilter` with weights `[0.4, 0.3, 0.2, 0.1]` over last 4 solutions (applied post-solve).
- `solve_ik()` returns two things:
  - `sol_q` — the 14-DOF joint solution (USED: sent to `arm.set_joint_targets(q14)`)
  - `sol_tauff` — feedforward torque from `pin.rnea(model, data, q, v=0, a=0)` (COMPUTED but NEVER SENT anywhere)

**Hardware transport (`g1_inspire_sdk`):**

- `G1InspireRobot` runs a 250 Hz motor thread. We set targets at ~50 Hz via `robot.step(q_arm=sol_q)`.
- The SDK motor controller presumably has its own internal gravity compensation, but we have no visibility into it.
- `arm.set_joint_targets(q14)` sends position targets only. There is no exposed API for feedforward torque through the SDK.

---

## Problem 1: Hand mass in the Pinocchio reduced model

**What the URDF contains:**
- Our `assets/g1_with_hands.urdf` includes the full Inspire RH56 kinematic chains merged at the wrist joints (left attachment: `xyz="0.0415 0.003 0" rpy="0 0 1.5708"`, right: `xyz="0.0415 -0.003 0" rpy="3.14159 0 -1.5708"`).
- The URDF has link masses for all finger links.

**What the reduced model retains:**
- `G1HandIK` calls `pin.buildReducedModel(full_model, locked_joint_ids, q_locked)` to lock out legs, waist, and finger joints.
- **Critical question:** When Pinocchio locks a joint, the child link's mass/inertia is *merged into the parent body*, not discarded. So if the finger joints are locked, their link masses should be lumped into the wrist links in the reduced model.
- **However:** The SDK's own config (`~/g1_inspire_sdk/g1_inspire_sdk/_config/g1.yaml`) has:
  ```yaml
  hand_config:
    models:
      inspire:
        mass_kg: 0.0  # TODO: measure your Inspire RH56 (typical ~0.54 kg)
  ```
  This placeholder suggests the hand mass in the SDK's Pinocchio model is also effectively zero.

**Impact:**
- If finger/hand link masses are zero (or near-zero placeholders) in our URDF, `sol_tauff` understates wrist and elbow torques by up to ~5 N·m per arm (0.54 kg × 9.8 m/s² × ~1 m moment arm at elbow).
- For the current session-5 scope (position control only, no torque feedforward), this has **no effect on joint angle accuracy**. It only matters if/when we try to use `sol_tauff` for gravity compensation.

**Proposed verification (read-only, ~1 minute):**
```python
from g1_spine_stabilization.kinematics.ik_solver import G1HandIK
ik = G1HandIK()
for i, inertia in enumerate(ik.model.inertias):
    if inertia.mass > 0.01:
        print(f"joint {i} ({ik.model.names[i]}): mass = {inertia.mass:.4f} kg")
```
This immediately tells us what masses survived the model reduction.

---

## Problem 2: Box (payload) is completely unmodeled

**Current state:**
- Box mass: `0.2 kg` in `configs/default.yaml` (used only in CoppeliaSim physics; completely absent from IK).
- When the robot carries the 0.2 kg box bimanually, each wrist sees approximately:
  - Static load: `0.2 kg × 9.8 / 2 = 0.98 N` downward at the EE
  - At elbow (~0.4 m moment arm): `~0.4 N·m` extra torque
- The box attachment is symmetric, so both wrists are equally loaded.
- For a 0.2 kg box this is small but non-negligible compared to the arm/hand weight (~5 N·m scale).

**How `pin.rnea` handles external forces:**
Pinocchio's RNEA signature has an optional `fext` argument:
```python
pin.rnea(model, data, q, v, a, fext)
```
where `fext` is a `StdVec_Force` — a list of 6D forces (force + torque) applied at each joint frame.

To add a payload of mass `m` at the left EE:
1. Look up the left EE frame ID: `ik.model.getFrameId("L_ee_link")`
2. Compute the gravitational wrench at the EE in the local frame: `F = [0, 0, -m*g, 0, 0, 0]` (world Z down, expressed in EE frame after rotation)
3. Build `fext` with this wrench at the EE joint

Alternatively (simpler): treat the box as extra mass at the midpoint between the two hands (box center) and compute an analytical correction torque analytically without modifying the RNEA call.

---

## Problem 3: `sol_tauff` is computed but never sent

**Current code path:**
```python
# In ik_solver.py:
sol_q, sol_tauff = ik.solve_ik(L_target, R_target, current_q, current_dq)

# In sdk_pose_tracking.py:
robot.step(q_arm=sol_q)   # sol_tauff is silently discarded
```

**Why it's not used:**
1. `arm.set_joint_targets(q14)` only accepts joint positions, not torque.
2. The SDK's motor controller handles gravity compensation internally at 250 Hz.
3. The SDK does not expose a feedforward torque channel in `G1InspireRobot`.

**Could we use it?**  
Possibly via a lower-level API in `unitree_sdk2py` (DDS-level commands). This would bypass the SDK's safety abstractions and is out of scope for now.

**Practical impact of the current gap:**
- Position accuracy: **unaffected**. The motor controller holds the commanded position against gravity on its own.
- Motor saturation risk: if the combined arm + hand + box weight requires torques near the motor limits, the motor controller may not compensate fully under acceleration. We have no visibility into this margin.
- For `0.2 kg` box: at elbow the extra torque is ~0.4 N·m; G1 elbow max is likely ~50 N·m. Margin is fine.

---

## Proposed changes for the clean code session

Listed in recommended execution order:

### Step 1 (10 min, read-only): Verify hand mass in reduced model

Run the snippet from Problem 1 above. Expected outcome:
- If finger link masses are zero: confirms `sol_tauff` is underestimating; proceed to Step 2.
- If finger link masses are correctly merged: `sol_tauff` is already accounting for hand weight; skip Step 2.

### Step 2 (prereq before coding): Measure the Inspire RH56 hand

Use a scale to weigh one hand (with all finger segments attached but no cables). Record:
- `hand_mass_kg` (expected ~0.54 kg per SDK TODO comment)
- `hand_com_offset` from wrist: roughly [+0.045, 0, 0] in the hand's local X direction (SDK estimate)

Update:
- `docs/sdk_setup.md` — record the measured value
- `assets/g1_with_hands.urdf` — update hand base link mass (requires explicit approval per Hard Rules since this changes IK behavior)
- OR: keep URDF unchanged and add a `payload_kg` correction to `solve_ik()` that captures the unmodeled hand mass analytically

### Step 3 (30–60 min): Add `payload_kg` parameter to `solve_ik`

Extend the signature:
```python
def solve_ik(self, L_target, R_target, q_prev, dq_prev, payload_kg=0.0):
```

Inside, after the IPOPT solve, add the payload contribution to the RNEA call:
```python
fext = self._build_payload_fext(payload_kg, sol_q)
sol_tauff = pin.rnea(model, data, sol_q, np.zeros(14), np.zeros(14), fext)
```

This makes `sol_tauff` correct for verification/logging even if we can't send it.

### Step 4 (15 min): Log `sol_tauff` in the preview script

Add a Rerun channel for `sol_tauff` alongside joint positions. This lets us monitor torque headroom visually during dry-run without changing any control behavior. Useful diagnostic before live bringup.

### Step 5 (future, if motor sag is observed): IK cost weight change

If the robot visibly sags under load during live bringup, the options are:
1. Increase translation cost weight (currently 50) — makes IK fight harder for position accuracy; may worsen smoothness
2. Add an external wrench term to the IK cost itself (EE force from payload)
3. Accept a small position sag and tune the payload correction empirically

**This requires explicit approval (Hard Rule).** Do not change cost weights without it.

---

## Questions to answer before the clean session

1. **What does `pin.buildReducedModel` do with locked joint child masses?** Run the inertia check above. If masses are non-zero, hand is already in model; if zero, update the URDF.

2. **Does the SDK's motor controller expose a feedforward torque channel below `G1InspireRobot`?** Check `unitree_sdk2py` DDS command types. If yes, we could pass `sol_tauff` as a feedforward term; if no, it's moot.

3. **What is the actual hand mass?** Weigh the Inspire RH56 before any URDF edits.

4. **Is position sag visible in live Rerun preview?** Check the joint angle trajectory during Phase 2 in Rerun — if the arm drifts downward over time, the motor controller is not fully compensating.

5. **Is 0.2 kg box mass even worth modeling?** At 0.98 N per arm, this is at the level of AprilTag detection noise (~5–15 mm → ~1–3 N equivalent). For the current demo, probably fine to leave unmodeled. Only matters for precision tasks or heavier payloads.

---

## Summary table

| Gap | Current impact | Urgency |
|-----|---------------|---------|
| Hand mass possibly zero in Pinocchio | `sol_tauff` inaccurate (not used anyway) | Low — verify first |
| Box payload not in RNEA | `sol_tauff` inaccurate for 0.2 kg load | Low — 0.98 N per arm |
| `sol_tauff` never sent | No gravity feedforward to motors | Low — SDK motor ctrl compensates |
| IK cost has no force terms | Position accuracy unaffected under load | Low until load increases |
| `hand_mass_kg = 0.0` in SDK config | Admittance gravity subtraction biased | Not in our code path yet (no ArmAdmittance used) |
