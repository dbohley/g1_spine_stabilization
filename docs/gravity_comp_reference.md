# Gravity Compensation Reference

Self-contained reference for the SDK gravity compensation pipeline. Read this before adding any new script that drives the G1 arm via the SDK.

---

## 1. Why gravity comp is needed

The G1's SDK motor controller runs a pure PD loop:

```
τ_actual = τ_ff + kp · (q_des − q) + kd · (0 − dq)
```

Without feedforward (`τ_ff = 0`), the motor must use **position error** to generate gravity-resisting torque. The arm droops until the error is large enough to hold it up. The wrists are especially vulnerable because they have the softest gains:

| Joint group | kp | kd |
|---|---|---|
| Shoulder pitch/roll | 100 | 2 |
| Shoulder yaw, elbow | 50 | 2 |
| **Wrist roll/pitch/yaw** | **20** | **1** |

At kp=20, 1 N·m of uncompensated gravity → 0.05 rad ≈ 3° of droop per joint.

The URDF models each Inspire RH56 hand at 0.14 kg; real measured mass = **1.0 kg** (~7× under-modeled). Without correction, RNEA underestimates the gravity torques by a factor of ~3–4× at the wrist.

---

## 2. Full pipeline — end to end

```
configs/default.yaml
  └─ gravity_comp.enabled = true
          │
          ▼
G1HandIK.__init__()   [kinematics/ik_solver.py:123]
  ├─ pin.RobotWrapper.BuildFromURDF(g1_with_hands.urdf)
  ├─ buildReducedRobot()
  │    ├─ Locks legs, waist, finger joints → 14 DOF arm model
  │    └─ Locked finger link masses are MERGED into parent bodies
  │         (finger mass lumped into wrist_yaw body automatically)
  ├─ _apply_hand_inertia_correction()   [ik_solver.py:352]
  │    └─ for each side in [left, right]:
  │         getJointId("left_wrist_yaw_joint")
  │         reduced.model.inertias[wrist_yaw_id] = _make_composite_wrist_inertia(side)
  │              ← returns pin.Inertia for bare wrist + 1.0 kg hand, both in wrist_yaw frame
  │         result: wrist_yaw composite mass = 1.0846 kg  (was ~0.28 kg from URDF)
  │    └─ reduced.data = reduced.model.createData()   ← refresh so RNEA sees new inertias
  └─ CasADi graph compiled from corrected model   [ik_solver.py:247]
       (inertia correction MUST happen before this line)

Every control step:
  sol_q, sol_tauff = ik.solve_ik(L_target, R_target, current_q, np.zeros(14))
    ├─ IPOPT solves for sol_q (14-DOF joint angles)
    ├─ WeightedMovingFilter([0.4,0.3,0.2,0.1]) smooths sol_q
    └─ sol_tauff = pin.rnea(reduced.model, reduced.data, sol_q, v=0, a=0)
         └─ static gravity torques at sol_q using the CORRECTED inertias
            = ONLY the g(q) term from τ = M(q)·q̈ + C(q,q̇)·q̇ + g(q)
              (v=0 and a=0 zero out the inertial and Coriolis terms)

Caller sends both outputs to the arm:
  robot.arm.set_joint_targets(sol_q, sol_tauff if gravity_comp_enabled else None)
    └─ SDK publishes sol_tauff to motor_cmd.tau at 250 Hz via DDS
         └─ τ_actual = τ_ff + kp·(q_des−q) + kd·(0−dq)
              ← gravity mostly cancelled; PD handles residual tracking error
```

---

## 3. Where every constant comes from

All constants live in [kinematics/ik_solver.py:67–94](../g1_spine_stabilization/kinematics/ik_solver.py).

| Constant | Value | Source |
|---|---|---|
| `INSPIRE_HAND_MASS` | 1.0 kg | **Scale measurement** of real Inspire RH56 hand |
| `_WRIST_MASS` | 0.08457647 kg | URDF `left_wrist_yaw_link <mass>` |
| `_WRIST_COM_L` | `[0.02200, +0.00049, 0.00054]` | URDF `left_wrist_yaw_link <inertial><origin>` |
| `_WRIST_COM_R` | `[0.02200, −0.00049, 0.00054]` | Same, Y sign flipped for right |
| `_WRIST_INERTIA_L/R` | 3×3 matrix | URDF `left/right_wrist_yaw_link <inertia>` |
| `_HAND_BASE_MASS_URDF` | 0.14143 kg | URDF `L_hand_hand_base_link <mass>` — used **only** as scale reference |
| `_HAND_BASE_COM_L` | `[−0.0025264, −0.066047, −0.0019598]` | URDF `L_hand_hand_base_link <inertial><origin>` |
| `_HAND_BASE_COM_R` | `[−0.0025264, −0.066047, +0.0019598]` | Same, Z sign flipped for right |
| `_HAND_BASE_INERTIA_URDF` | 3×3 matrix | URDF `L_hand_hand_base_link <inertia>` |
| `_ATTACH_XYZ` | `[0.0415, 0.003, 0]` | URDF `L_hand_attach_joint <origin xyz>` |
| Attachment rotation (left) | Rz(+90°) | URDF `L_hand_attach_joint rpy=[0,0,π/2]` |
| Attachment rotation (right) | Rz(−90°)·Rx(180°) | URDF `R_hand_attach_joint rpy=[π,0,−π/2]` |
| `gravity_comp.enabled` | true | `configs/default.yaml` — kill switch |

### How `_make_composite_wrist_inertia` works

```
composite = I_wrist_bare  +  T_wrist_to_hand.act(I_hand_in_handbase)
```

1. `I_wrist_bare` — bare `wrist_yaw_link` inertia from URDF constants above
2. `I_hand_in_handbase` — hand inertia at **1.0 kg** (URDF tensor × scale factor 7.07), in `hand_base_link` frame
3. `T_wrist_to_hand` — SE3 transform from URDF attachment joint (Rz(90°) + [0.0415, 0.003, 0])
4. `SE3.act(Inertia)` — Pinocchio transforms the inertia from `hand_base_link` frame into `wrist_yaw_link` frame, applying both the rotation change and the parallel-axis theorem automatically

Result: composite wrist mass = **0.0846 + 1.0 = 1.0846 kg**

---

## 4. Comparison with endeffector-config.json

The file `~/Documents/endeffector-config (3).json` describes the same physical hand:

```json
{
  "mass": 1,
  "centerOfMass": [0, 0.02, 0.1],
  "transformation": [0.7071,-0.7071,0,0, 0.7071,0.7071,0,0, 0,0,1,0, 0,0,0.1034,1],
  "inertia": [0.001,0,0, 0,0.0025,0, 0,0,0.0017]
}
```

The `transformation` is a column-major 4×4 matrix (CoppeliaSim convention): rotation = Rz(−45°), translation = [0, 0, 0.1034].

| Property | JSON | Our constants | Why they differ |
|---|---|---|---|
| **Mass** | 1.0 kg | `INSPIRE_HAND_MASS = 1.0` | **Same** — both from the scale measurement |
| **COM** | `[0, 0.02, 0.1]` | `_HAND_BASE_COM_L = [−0.0025264, −0.066047, −0.0019598]` | **Different frames**: JSON in CoppeliaSim EE convention; ours in URDF `hand_base_link` frame |
| **Attachment rotation** | Rz(−45°) | Rz(+90°) from URDF | JSON relative to CoppeliaSim's object frame; URDF relative to `wrist_yaw_link` |
| **Attachment translation** | `[0, 0, 0.1034]` (along Z) | `[0.0415, 0.003, 0]` (along X) | Same frame mismatch |
| **Inertia Ixx** | 0.001 | 0.00087 | 1.2× — close |
| **Inertia Iyy** | 0.0025 | 0.00059 | **4.2× larger in JSON** |
| **Inertia Izz** | 0.0017 | 0.00054 | **3.1× larger in JSON** |

#### COM distance analysis

Despite different axis conventions, both models put the hand COM roughly the same distance from the wrist joint:

```
Our URDF COM transformed to wrist_yaw frame: [0.1075, 0.0005, −0.002] m  → 10.76 cm
JSON COM [0, 0.02, 0.1] (if in wrist frame):                             → 10.20 cm
```

**The JSON does NOT suggest the COM is farther away.** The 10 cm distance is consistent between models; only the frame axis convention differs.

#### Why the inertia Iyy/Izz is 3–4× larger in the JSON

The URDF `hand_base_link` shape represents only the palm. Finger links extend 10–15 cm further and contribute large Iyy/Izz via the parallel-axis theorem. When we scaled the URDF inertia tensor by 7.07× (mass ratio), we scaled its *magnitude* but not its *spatial distribution*. The JSON likely measured the assembled hand's actual principal moments, which capture the fingertip mass contribution.

**This matters only for dynamic (accelerating) motion — not for static gravity compensation.** See Section 5.

#### Why we don't use the JSON values directly

Our Pinocchio model is built entirely from the URDF. All joint frames, FK chains, and RNEA computations share URDF coordinate conventions. The JSON uses CoppeliaSim's EE frame convention — inserting those values directly would introduce a silent frame error. The URDF already gives us the COM and inertia in `hand_base_link` frame, which is what Pinocchio needs; we just correct the mass from 0.14143 → 1.0 kg.

---

## 5. What sol_tauff is — and what it can't do during motion

We call RNEA with zero velocity and zero acceleration:

```python
sol_tauff = pin.rnea(model, data, sol_q, v=0, a=0)
```

The full Newton-Euler equation is:

```
τ = M(q)·q̈   +   C(q,q̇)·q̇   +   g(q)
     ↑                ↑              ↑
  inertial         Coriolis       gravity only
  (v=0,a=0        (v=0 zeros     uses COM+mass
   zeros this)     this out)     NOT inertia tensor
```

**`sol_tauff` is purely `g(q)`** — the static gravity term. The inertia tensor is not used at all.

### What this means for motion (Phase 0, Phase 2, Phase R)

During active arm motion, the motors also experience:

| Term | At 0.3 Hz sine | Relative to gravity |
|---|---|---|
| Inertial (M·q̈) | ~0.02 N·m at wrist | ~2% of gravity — negligible |
| Coriolis (C·q̇) | < inertial | Even smaller |
| Gravity (g) | ~1.0 N·m at wrist | 100% — dominates |

At our operating speeds (0.3 Hz sine, 3s smoothstep interpolation), inertial and Coriolis forces are ~2% of gravity. The PD loop handles them. **Gravity-only feedforward is the right choice for this application.**

### The 3–4× inertia mismatch: what it means in practice

The JSON shows our Iyy/Izz are 3–4× too small. Since `sol_tauff` never uses the inertia tensor, this has **zero effect on current gravity compensation**. It would only matter if we computed full inverse dynamics (`M·q̈ + C·q̇ + g`) — which would cause overshoot/oscillation during fast arm acceleration. That is not in scope.

**If wrist lag appears during fast marker tracking:** The fix is increasing wrist `kd` (currently 1) in `~/g1_inspire_sdk/g1_inspire_sdk/_config/g1.yaml` to damp oscillation — not correcting the inertia tensor.

---

## 6. The 4-line contract every script must follow

Any script that uses `G1HandIK` with the SDK arm transport must do all four of these:

```python
# 1. Read the config flag
cfg = load_config()
gravity_comp_enabled = cfg.get("gravity_comp", {}).get("enabled", True)

# 2. Pass the flag to IK constructor
ik = G1HandIK(gravity_comp_enabled=gravity_comp_enabled)

# 3. Capture BOTH return values — never discard sol_tauff with _
sol_q, sol_tauff = ik.solve_ik(L_target, R_target, current_q, np.zeros(14))

# 4. Use robot.arm.set_joint_targets (NOT robot.step)
robot.arm.set_joint_targets(sol_q, sol_tauff if gravity_comp_enabled else None)
```

### Anti-patterns that silently break gravity comp

```python
# WRONG — discards gravity compensation torques
sol_q, _ = ik.solve_ik(L_target, R_target, current_q, np.zeros(14))

# WRONG — robot.step() has no tau parameter; internally calls set_joint_targets with no tau
robot.step(q_arm=sol_q)
```

`robot.step(q_arm=sol_q)` internally calls `self.arm.set_joint_targets(q_arm)` with no `tau_ff14` argument — tau stays 0 on every cycle.

---

## 7. Hold-phase caching pattern

During phases where the arm stays at a **fixed configuration** (e.g., hand open/close while arm is stationary), reuse the last computed `sol_tauff` rather than re-solving IK:

```python
# At end of interpolation (Phase 0):
grasp_arm_q = current_q.copy()
grasp_tauff = sol_tauff.copy()   # gravity comp at the grasp configuration

# Every hold-phase tick (Phases A, B, C, D):
robot.arm.set_joint_targets(grasp_arm_q, grasp_tauff if gravity_comp_enabled else None)
robot.hand.set_joint_targets(
    q_left=q_cmd_l,    # handles None per side safely
    q_right=q_cmd_r,
)
```

**Important:** `robot.hand.set_joint_targets` must be a **separate call** from the arm command. There is no combined arm+hand+tau API — `robot.step()` accepts `q_hand_left/right` but not tau.

---

## 8. Script-by-script status

| Script | tau_ff routed? | How |
|---|---|---|
| `controllers/sdk_pose_tracking.py` | Yes | All 3 phases use `robot.arm.set_joint_targets` |
| `scripts/grasp_demo.py` | Yes | Fixed SDK-5.5+; hold phases cache `grasp_tauff` |
| `scripts/run_pose_tracking_preview.py` | Yes | Delegates entirely to `sdk_pose_tracking.py` |
| `scripts/run_pose_tracking_sim.py` | N/A | CoppeliaSim path — no SDK arm transport |
| `scripts/verify_ik.py` | N/A | FK/IK self-test only, no robot connection |

---

## 9. Tuning knobs

| Knob | Location | What it does |
|---|---|---|
| `gravity_comp.enabled` | `configs/default.yaml` | Kill switch — set false to send zero tau |
| `INSPIRE_HAND_MASS` | `ik_solver.py:67` | If real hand mass differs from 1.0 kg (e.g., includes cables) |
| Wrist `kp` (currently 20) | `~/g1_inspire_sdk/.../g1.yaml` | Increasing to 40–50 reduces droop from any residual model error |
| Wrist `kd` (currently 1) | `~/g1_inspire_sdk/.../g1.yaml` | Increasing damps oscillation during fast marker tracking |

### Remaining under-compensation sources

Even with tau_ff correctly routed and 1.0 kg mass, small residuals can remain:

| Source | Estimated torque |
|---|---|
| Box/object payload (0.2 kg, bimanual) | ~0.34 N·m/arm at shoulder |
| Wrist cables not on scale during weighing (~150g) | ~0.29 N·m |
| Real COM slightly farther than URDF (fingertips) | +0.1–0.3 N·m at wrist |

At wrist kp=20, these residuals produce 0.7–1.5° of visible droop. Increasing wrist kp to 40 halves this without any model change.
