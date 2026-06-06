# G1 Inspire SDK — Source-Grounded Analysis (Session 0)

**Status:** read-only discovery. Nothing migrated, nothing installed, no repo
code touched except this file and `docs/followups.md`.

**Reference clone:** `~/reference/g1_inspire_sdk` (outside the package tree, so
it can never be accidentally imported or vendored). All citations below are
`path:line` into that clone unless prefixed with `ours:`, which means our repo
(`g1_spine_stabilization/...`).

**Lens for this SDK (vs. the `g1_data_collection` "patterns only" repo):** this
one is meant to become a *dependency we build on*. So the question per module is
not "what pattern do we copy" but "what do we call, what do we keep of our own,
what do we throw away." The mapping table near the bottom answers that per file.

**Version read:** `g1_inspire_sdk` 0.1.0 (`pyproject.toml:8`), author Daniel
Huang. Python ≥3.10 declared (`pyproject.toml:10`); the conda env targets 3.12
(`environment.yml`).

---

## Decisions for the human

Three forks in the road. Each has a recommended default, but **all three are
left open** — they need a human call, ideally before Session 3/4 work starts.

### Decision 1 — IK: keep our `G1HandIK`, or adopt the SDK's `G1_29_ArmIK`?

**Recommendation (open): KEEP `G1HandIK`. Feed the SDK
`arm.set_joint_targets(q14)`, never `arm.set_ee_targets(...)`.**

Evidence the SDK IK is *not* a drop-in for ours:

- **No shoulder-roll self-collision bounds.** The SDK IK adds only joint
  position-limit constraints (`_ik/ik.py:178-184`). Our `L_SHOULDER_ROLL_MIN =
  0.15` / `R_SHOULDER_ROLL_MAX = -0.15` (`ours: kinematics/ik_solver.py`) have
  no analogue. Adopting the SDK IK silently drops a CLAUDE.md Hard-Rule safety
  constraint.
- **Different rotation cost weight.** SDK cost is `50·trans + 1·rot + 0.02·reg
  + 0.1·smooth` (`_ik/ik.py:185-190`). Ours is `50·trans + 5·rot + 0.02·reg +
  0.1·smooth`. The SDK weights orientation 5× *less*. For bimanual rigid-body
  pose tracking (the whole point of this project), orientation fidelity is not
  something to silently weaken.
- **Different EE-target frame.** SDK adds `L_ee`/`R_ee` frames at a fixed
  `[0.05, 0, 0]` (5 cm +X) offset from `left/right_wrist_yaw_joint`
  (`_ik/ik.py:108-123`). Ours uses `L_ee_link`/`R_ee_link` baked into our URDF
  at the inner palm surface (`ours: kinematics/ik_solver.py:153-154`; offset
  `xyz="-0.0175 -0.1136 0"` per CLAUDE.md). The two conventions place the
  target at physically different points — `compute_hand_targets` is calibrated
  to *ours*.
- **Looser solver tolerances.** SDK: `max_iter=10, tol=1e-3,
  acceptable_tol=5e-3` (`_ik/ik.py`). Ours: `max_iter=30, tol=1e-4,
  acceptable_tol=5e-4`. SDK trades accuracy for speed (it has to keep up with a
  250 Hz publisher); we don't have that constraint when we set targets at our
  own cadence.

What makes "keep ours" cheap to wire: `G1Arm.set_joint_targets(q14)` is a clean
bypass that writes `_q_target` as `left[0:7] + right[7:14]`
(`g1_arm.py`, `set_joint_targets`). Our `solve_ik` returns `sol_q` in exactly
that reduced order (L7 then R7 — `ours: kinematics/ik_solver.py`), and the
SDK's per-side joint order (shoulder pitch/roll/yaw, elbow, wrist
roll/pitch/yaw — `_config/g1.yaml`) **matches our `ARM_JOINT_NAMES` ordering by
construction**. So the translation is the identity: solve with our IK, hand the
14-vector straight to `set_joint_targets`. No re-indexing.

When you'd reconsider: if real-robot bring-up shows our IK is too slow to feed
the arm at a useful rate, the SDK IK's speed-tuned weights become attractive —
but only after re-adding shoulder-roll bounds and re-checking the rot weight.

### Decision 2 — Simulation: retire CoppeliaSim, or keep it?

**Recommendation (open): KEEP CoppeliaSim as the sim of record for now.
RerunPreview is a visualizer, not a physics replacement. DO NOT resolve this
fork in code this session.**

Evidence:

- **RerunPreview is FK-only.** `preview.py` runs `forwardKinematics +
  updateFramePlacements + updateGeometryPlacements` and logs `Transform3D` per
  link. There is no contact, no dynamics, no integration step anywhere in the
  file. It draws where links *would* be given a joint vector.
- **CoppeliaSim does things Rerun structurally cannot:**
  - **Box creation** as a physical body (`ours:
    sim/coppelia_utils.py:create_box` → `createPrimitiveShape`, respondable,
    mass, inertia).
  - **Attach-to-hand grasp** (`ours: sim/coppelia_utils.py:attach_box_to_hand`
    → `setObjectParent`, made static while held).
  - **Real-time physics stepping** (`ours: controllers/pose_tracking.py` →
    `sim.step()` + `step_realtime`).
  - The whole three-phase controller's premise — object attached to the left
    hand and carried through a rotation — is a *physics* behavior, not a draw
    call.

So: Rerun and CoppeliaSim are not substitutes. Rerun could become a *live
preview overlay* alongside hardware (it logs camera RGB/depth/pointcloud and EE
target triads too), but it does not let us delete the simulator. Go/no-go on
ever retiring CoppeliaSim is a human decision and is explicitly **not made
here**.

### Decision 3 — Grip force (Session 4): adopt the SDK controllers, or build our own?

**Recommendation (open): START FROM `TransportGraspController`. Treat it as a
strong starting point, not a drop-in.**

Evidence it fits the Session 4 goal (hold an object, let grip self-tighten
under arm motion):

- `TransportGraspController(n_joints, tau_des=0.30, kappa=0.05,
  delta_q_max=1.5)` is **close-only**: `Δq = clip(Δq + κ·(τ_des − |τ|), 0,
  Δq_max)`, `q_cmd = q_grasp + Δq` (`_ik/force_controller.py`). It can tighten
  but never loosen past the grasp snapshot — exactly the safety property you
  want for "don't drop the object."
- "Grip self-tightens with arm acceleration" is achieved by the *caller*
  passing `tau_des = τ* + k_a·‖a‖` — the acceleration coupling is **not**
  inside the controller (`_ik/force_controller.py`, docstring). So we own the
  policy; the controller just integrates it.
- It needs a force signal. The Inspire reader exposes per-finger `FORCE_ACT`
  (`inspire_hand.py`, modbus reader; `get_state` → `{q, force}`). That is the
  `|τ|` proxy the controller consumes.

What's *not* a fit / what must be measured:

- `HybridForcePositionController` is **release-only** (PID drives toward opening,
  can never close past `q_ref` — `_ik/force_controller.py`). That's the opposite
  polarity from "hold tight," so it's the wrong primitive for the carry phase
  (though possibly right for a controlled set-down).
- **`hand_mass_kg = 0.0` is a placeholder** (`_config/g1.yaml`, with a `# TODO
  ~0.54kg` note; mirrored in `_ik/admittance.py` default `hand_mass_kg=0.0`).
  Gravity-comp / admittance force estimates are wrong until this is measured.
  **Action for Session 4: weigh the Inspire hand (+ any held-object mass model)
  and set `hand_mass_kg`.** Also confirm `hand_com_local=(0.045,0,0)`.

Build-our-own only makes sense if the close-only integral law turns out too
coarse; even then the math is ~20 lines and worth copying the *shape* of, not
the import.

---

## A — IK (the crux)

**Q: Does `G1_29_ArmIK` enforce our shoulder-roll bounds?**
No. Constraints are joint position limits only (`_ik/ik.py:178-184`). No
custom shoulder-roll floor/ceiling. (Ours adds `lower[1] ≥ 0.15`,
`upper[8] ≤ −0.15`.)

**Q: How do the cost weights compare?**

| term       | SDK (`_ik/ik.py:185-190`) | Ours |
|------------|---------------------------|------|
| translation| 50                        | 50   |
| rotation   | **1**                     | **5**|
| regularize | 0.02                      | 0.02 |
| smoothing  | 0.1                       | 0.1  |

Only the rotation weight differs (SDK is 5× weaker on orientation).

**Q: Solver settings?**
SDK `max_iter=10, tol=1e-3, acceptable_tol=5e-3` (`_ik/ik.py`); ours
`max_iter=30, tol=1e-4, acceptable_tol=5e-4, acceptable_iter=5`. Both use
`WeightedMovingFilter([0.4,0.3,0.2,0.1], 14)` (`_ik/ik.py:211`; ours identical).

**Q: Does its EE-target convention match `L_ee_link`/`R_ee_link`?**
No. SDK synthesizes `L_ee`/`R_ee` at `[0.05, 0, 0]` off each
`wrist_yaw_joint` (`_ik/ik.py:108-123`). Ours are URDF frames at the inner
palm. Different physical target points.

**Q: Can we keep `G1HandIK` and feed `arm.set_joint_targets(q14)` instead of
`set_ee_targets`?**
Yes, cleanly. `set_joint_targets` writes `_q_target` as left[0:7] + right[7:14]
(`g1_arm.py`). Our `solve_ik` already returns 14 values in that reduced order.

**Q: `ARM_JOINT_NAMES` → SDK [0:14] ordering translation?**
Identity. SDK per-side order is shoulder pitch/roll/yaw, elbow, wrist
roll/pitch/yaw (`_config/g1.yaml`), combined left-then-right
(`_config/config.py`, `arm_slice_for_side`: left=`slice(0,7)`,
right=`slice(7,14)`). That is exactly our reduced-model order
(`ours: sim/coppelia_utils.py:ARM_JOINT_NAMES`, left then right). No remap.

**Caveat (README discrepancy worth flagging):** the SDK README says rebuilding
the IK cache against the bundled `g1_body29_hand14.urdf` "will fail," but the
code actually *filters out* joint names not present in the URDF
(`_ik/ik.py:99-102`) rather than erroring. Documented mismatch; not a blocker
because we don't plan to rebuild its cache.

---

## B — Camera

**Q: Is `capture()["color"]` HxWx3 BGR, consumable by `AprilTagDetector`
unchanged?**
Yes. `camera.py:184-242` documents and returns `color` as `(H, W, 3)` uint8
**BGR** (the stream is configured `rs.format.bgr8`, `camera.py:128`). Our
detector converts BGR→GRAY when `ndim == 3` (`ours:
perception/detector.py:detect`). Clean swap: `frame = cam.capture();
detector.detect(frame["color"], intrinsics)`.

**Q: Intrinsics mapping (SDK → our `RealSenseSource.intrinsics_native`)?**
SDK `cam.intrinsics` is `{width, height, fx, fy, ppx, ppy, model, coeffs}`
(`camera.py:147-156`). Our consumer wants `{fx, fy, cx, cy, distortion,
resolution}` (`ours: perception/frame_sources.py:intrinsics_native`). Remap:

| our key      | SDK source                          |
|--------------|-------------------------------------|
| `fx`         | `intr["fx"]`                        |
| `fy`         | `intr["fy"]`                        |
| `cx`         | `intr["ppx"]`                       |
| `cy`         | `intr["ppy"]`                       |
| `distortion` | `intr["coeffs"]`                    |
| `resolution` | `(intr["width"], intr["height"])`  |

Our `CameraIntrinsics` dataclass (`ours: perception/detector.py`) takes
`fx, fy, cx, cy, distortion` and exposes `camera_matrix` — direct fit.

**Q: Depth for marker pose — future option?**
Yes, bonus, not needed now. `capture()["depth"]` is `(H, W)` uint16 in units of
`cam.depth_scale` metres (default 0.001 — `camera.py:84,184-190`), and with
`align_depth=True` (default, `camera.py:130`) depth is registered into the color
frame so `depth[v,u]` matches `color[v,u]`. That makes per-corner depth lookups
possible to refine/validate solvePnP later. Out of scope for Session 2/3; note
it and move on.

---

## C — Simulation (RerunPreview vs CoppeliaSim)

**Q: Confirm in source that RerunPreview does no contact physics.**
Confirmed. `preview.py` only does Pinocchio FK (`forwardKinematics`,
`updateFramePlacements`, `updateGeometryPlacements`) and Rerun logging
(`Transform3D` per link, EE target triads under `ee_target/{left,right}`, hand
scalars, camera RGB/depth/pointcloud, pinhole). No solver, no contact, no
integrator. `update_arm(q14)` splits left[0:7]/right[7:14]; mimic multipliers
are applied manually because Pinocchio drops `<mimic>` tags.

**Q: What does CoppeliaSim do that Rerun can't?**
- Create a physical box (`ours: sim/coppelia_utils.py:create_box`).
- Attach the box to the hand as a grasp (`ours:
  sim/coppelia_utils.py:attach_box_to_hand`).
- Step real-time physics (`ours: controllers/pose_tracking.py`, `sim.step()` +
  `step_realtime`).
- Carry the held object through Phase 2's rotation as a *consequence of
  dynamics*, not a redraw.

**Go/no-go:** evidence laid out, **decision deferred to the human** (Decision 2).
Rerun is a candidate live-preview overlay, not a simulator replacement.

---

## D — Hand / grip force (Session 4)

**Q: Map `TransportGraspController` and `HybridForcePositionController` to our
Session 4 goal — drop-in / starting point / mismatch?**

| controller (`_ik/force_controller.py`) | polarity | Session 4 fit |
|----------------------------------------|----------|---------------|
| `TransportGraspController(n_joints, tau_des=0.30, kappa=0.05, delta_q_max=1.5)` | close-only (`Δq=clip(Δq+κ(τ_des−|τ|),0,Δq_max)`, `q_cmd=q_g+Δq`) | **strong starting point** for the carry/hold phase |
| `HybridForcePositionController(n_joints, tau_des=0.30, kp=0.20, ki=1.0, kd=0.0, offset_max=1.5)` | release-only PID on `|τ|` (never closes past `q_ref`) | **mismatch** for holding; possibly right for controlled set-down |
| `AdaptiveForceController` | adapts `tau_des` from wrist disturbance | optional refinement |

`set_grasp(q_g)` snapshots the closed pose; acceleration self-tightening is
caller-supplied (`tau_des = τ* + k_a·‖a‖`), not internal.

**Q: What must we measure for `hand_mass_kg = 0.0`?**
Weigh the Inspire RH56 hand (SDK TODO estimates ~0.54 kg — `_config/g1.yaml`)
and set `hand_mass_kg`. Confirm/adjust `hand_com_local = (0.045, 0, 0)`
(`_ik/admittance.py`). Until then admittance/gravity-comp force estimates are
off. Force feedback comes from the modbus reader's per-finger `FORCE_ACT`
(`inspire_hand.py`, `get_state` → `{q, force}`).

**Hand command facts for Session 4:**
- Canonical finger order: `(index_mcp, middle_mcp, ring_mcp, little_mcp,
  thumb_rotate, thumb_flex)` (`inspire_hand.py`).
- Soft limits: open `[0,0,0,0,0.3,0.14]`, closed `[1.47,1.47,1.47,1.47,1.2,
  0.56]` (`inspire_hand.py`; matches README table).
- `set_joint_targets(q_left=None, q_right=None)` soft-clips to those limits.
- Convenience: `open()`, `close_fist()`.

---

## E — Lifecycle / threading

**Safe bring-up (from `robot.py`):**
1. `connect()` → arm, hand, then camera (`robot.py:119-124`). Builds
   `G1_29_ArmIK` inside `G1Arm.connect()` (cold start ~5–15 s).
2. `initialize()` → `arm.initialize()`, `hand.initialize()`, and in live mode
   `hand.wait_for_state()` (`robot.py:126-131`).
3. `start_streaming()` → **arm only**; the hand has no streaming thread
   (`robot.py:133-135`).
4. `shutdown()` → arm, hand, camera; arm sends a damping cmd and drops DDS
   (`robot.py:143-148`).

**Context-manager caveat:** `__enter__` calls **`connect()` only** — NOT
`initialize()` or `start_streaming()` (`robot.py:150-152`). `with
G1InspireRobot(...) as r:` gives you a connected-but-not-initialized robot; you
must call `initialize()` / `start_streaming()` yourself.

**Q: Does our loop still pace with `step_realtime`, or does the SDK own
timing?**
Split ownership:
- The **SDK owns motor timing**: `G1Arm` publishes at 250 Hz (`_motor_dt =
  1/250`, `g1_arm.py`) on its own thread, with a velocity ramp (`20.0 →
  30.0` over `gradual_ramp_seconds=5`; dry-run loop uses fixed 30).
- **We own control timing**: our job is to *set targets* (`set_joint_targets`
  or `set_ee_targets`) at our own cadence; the SDK interpolates/publishes
  underneath. The live examples set targets at ~60 Hz against the 250 Hz
  publisher (`examples/05_live_ee_control.py`).
- In **CoppeliaSim** we keep pacing with `step_realtime(sim, control_dt)`
  (`ours: controllers/pose_tracking.py`) — that path doesn't go through the SDK
  publisher at all.

So: keep `step_realtime` for the sim controller; for hardware, set targets in a
self-paced loop and let the SDK's 250 Hz thread handle motor I/O.

**Extra filtering caveat:** `set_ee_targets` applies an output LPF
(`alpha = dt/(arm_filter_tau+dt)`, `arm_filter_tau=0.30`) *on top of* the IK's
internal `WeightedMovingFilter` → **double-filtered** (`g1_arm.py`). Another
reason to prefer `set_joint_targets` with our own IK (Decision 1): one filter,
predictable latency.

---

## F — URDF / assets

**The conflict.** Three different URDFs are in play:

| URDF | used by |
|------|---------|
| `g1_body29_hand14.urdf` | SDK IK default (`_ik/ik.py`), cache `g1_29_model_cache.pkl` |
| `g1_body29_inspire_hand.urdf` (`DEFAULT_G1_INSPIRE_URDF`) | RerunPreview default (`preview.py`) |
| `assets/g1_with_hands.urdf` | **ours** — merged Inspire chains, `L_ee_link`/`R_ee_link`, tuned (CLAUDE.md Hard Rule: never modify) |

**Rebuild caveat:** README warns that deleting the pickle and rebuilding the IK
cache on the bundled URDF "will fail"; the code actually filters missing joints
(`_ik/ik.py:99-102`) — see the Section A discrepancy note. Either way we don't
rebuild it.

**Which URDF the preview loads:** `g1_body29_inspire_hand.urdf` by default
(`preview.py`, `DEFAULT_G1_INSPIRE_URDF`), overridable via
`RerunPreview(urdf_path=...)`.

**Does the URDF conflict matter if we keep our IK?**
Largely no, *for kinematics*. If we keep `G1HandIK` (Decision 1) and feed
`set_joint_targets(q14)`, the SDK IK's URDF is never loaded for solving — we
only use the SDK as a motor/hand/camera transport. The one place it surfaces:
**RerunPreview draws the robot from its own URDF**, so a preview overlay would
render the SDK's link geometry, not ours. Cosmetic divergence (e.g. hand mesh
detail), not a correctness problem, as long as joint *ordering* matches (it
does — Section A). If we ever pass our URDF to the preview
(`RerunPreview(urdf_path="assets/g1_with_hands.urdf")`), check that its mimic
handling and link names line up first.

---

## Module-by-module mapping (our file → SDK equivalent → verdict)

| ours | SDK equivalent | verdict |
|------|----------------|---------|
| `kinematics/ik_solver.py` (`G1HandIK`) | `_ik/ik.py` (`G1_29_ArmIK`) | **KEEP ours** — SDK lacks shoulder-roll bounds, weaker rot weight, different EE frame (Decision 1). Feed SDK `set_joint_targets`. |
| `sim/coppelia_utils.py`, `sim/scene_setup.py` | `preview.py` (RerunPreview, FK-only) | **KEEP ours** — no SDK physics substitute (Decision 2). Rerun is optional overlay only. |
| `controllers/pose_tracking.py` | `robot.py` `step()` + `g1_arm.py` | **LAYER** — sim path stays ours; a hardware path can drive `G1InspireRobot.step(q_arm=...)`. |
| `controllers/grip_force.py` (empty stub) | `_ik/force_controller.py` (`TransportGraspController`) | **DECIDE / start from SDK** (Decision 3). |
| `perception/frame_sources.py` (`RealSenseSource`) | `camera.py` (`RealSenseCamera`) | **REPLACE-able** — SDK camera is more complete (align, pointcloud, intrinsics). Remap intrinsics dict (Section B). Either keep ours or wrap SDK's. |
| `perception/detector.py` (`AprilTagDetector`) | — none | **KEEP ours** — SDK has no AprilTag detector. Consumes SDK BGR `color` unchanged. |
| `perception/tracker.py`, `perception/frames.py`, `perception/types.py` | — none | **KEEP ours** — SDK has no marker→pelvis frame layer. |
| `trajectory/targets.py`, `trajectory/sources.py` | — partial (`g1_arm.default_ee_targets`, examples' arc/lerp) | **KEEP ours** — our geometry is calibrated to our EE frames. |
| `utils/transforms.py`, `utils/config.py` | `_common/...`, `_config/config.py` | **KEEP ours** — SDK's `weighted_moving_filter.py` imports `matplotlib.pyplot` at module top (hard dep) — do not import it. |
| `assets/g1_with_hands.urdf` | `g1_body29_hand14.urdf` / `g1_body29_inspire_hand.urdf` | **KEEP ours** (Hard Rule). Cosmetic-only divergence in preview (Section F). |
| — (no hardware transport today) | `robot.py`, `g1_arm.py`, `inspire_hand.py`, `_dds.py`, `_common/command_helper.py` | **ADOPT** — this is the value of the SDK: safe DDS/modbus motor + hand transport. |

---

## Exact API signatures (everything we'd actually call)

**Top-level facade** (`robot.py`):
```python
G1InspireRobot(
    network_interface: str | None = None, *,
    arm_side: str = "both",          # "left" | "right" | "both"
    hand_side: str = "right",
    config_path: str | Path | None = None,
    left_ip: str = "192.168.11.210",
    right_ip: str = "192.168.123.210",
    camera: bool | dict | RealSenseCamera | None = None,
    dry_run: bool = False,
    preview: object | None = None,
)
r.connect(); r.initialize(); r.start_streaming()
r.step(*, q_arm=None, ee_targets=None, q_hand_left=None, q_hand_right=None)  # q_arm XOR ee_targets
r.capture_camera() -> dict | None
r.get_state() -> dict
r.shutdown()
# NOTE: __enter__ calls connect() ONLY (not initialize/start_streaming)
```

**Arm** (`g1_arm.py`) — the bypass we want:
```python
arm.set_joint_targets(q14: np.ndarray, tau_ff14: np.ndarray | None = None)  # left[0:7]+right[7:14]
arm.set_ee_targets(L_tf: 4x4, R_tf: 4x4)        # AVOID: runs SDK IK + extra LPF (double-filtered)
arm.get_ee_pose() -> (L_tf, R_tf)               # FK
arm.default_ee_targets() -> (L_tf, R_tf)        # home from yaml
arm.initialize(); arm.start_streaming(); arm.shutdown()
# NUM_ARM_JOINTS=14, _motor_dt=1/250
```

**Hand** (`inspire_hand.py`):
```python
hand.set_joint_targets(q_left=None, q_right=None)   # canonical order, soft-clipped
hand.open(); hand.close_fist()
hand.get_state(side) -> {"q": [...6], "force": [...6]}   # FORCE_ACT per finger
hand.wait_for_state()  # live only
# canonical order: (index_mcp, middle_mcp, ring_mcp, little_mcp, thumb_rotate, thumb_flex)
# SOFT open  = [0,0,0,0,0.3,0.14]; SOFT closed = [1.47,1.47,1.47,1.47,1.2,0.56]
```

**Camera** (`camera.py`):
```python
RealSenseCamera(*, width=640, height=480, fps=30, serial=None,
                align_depth=True, pointcloud=None, dry_run=False, preview=None)
cam.connect()
cam.capture() -> {
    "color": (H,W,3) uint8 BGR,
    "depth": (H,W) uint16 * depth_scale metres,
    "timestamp": float,
    "points": (N,3) float32, "colors": (N,3) uint8,   # empty unless pointcloud
}
cam.intrinsics  # {width,height,fx,fy,ppx,ppy,model,coeffs}
cam.depth_scale # default 0.001
cam.shutdown()
```

**IK (SDK, for reference only — we keep ours)** (`_ik/ik.py`):
```python
G1_29_ArmIK(urdf_path=None, mesh_dir=None, Unit_Test=False, Visualization=False)
.solve_ik(left_wrist, right_wrist, current_lr_arm_motor_q=None,
          current_lr_arm_motor_dq=None) -> (sol_q[14], sol_tauff[14])
.compute_current_ee(q) -> (L_tf, R_tf)
.compute_arm_dynamics(q, dq, tau_meas) -> (tau_rnea, F_ee_l, F_ee_r, F_ee_l_ext, F_ee_r_ext)
```

**Grip / force controllers** (`_ik/force_controller.py`):
```python
TransportGraspController(n_joints, tau_des=0.30, kappa=0.05, delta_q_max=1.5)
  .set_grasp(q_g); .step(tau, ...) -> q_cmd            # close-only
HybridForcePositionController(n_joints, tau_des=0.30, kp=0.20, ki=1.0, kd=0.0, offset_max=1.5)
  .step(tau, q_ref, q, dt) -> q_cmd                    # release-only
ArmAdmittance(arm_ik, *, side="right", hand_mass_kg=0.0,
              hand_com_local=(0.045,0,0), gain=0.005, deadband_n=2.0,
              qdot_max=0.3, damped_pinv_lambda=0.05, gravity=9.81)
  .step(q, dq, tau_meas) -> (qd_side[7], F_world[3], tau_rnea[14])
```

**Preview** (`preview.py`):
```python
RerunPreview(*, urdf_path=None, spawn=True,
             application_id="g1_inspire_preview", config_path=None)
# FK-only. update_arm(q14), camera/EE-target logging hooks. No physics.
```

**DDS / commands** (`_dds.py`, `_common/command_helper.py`):
```python
init_dds(network_interface)        # idempotent ChannelFactoryInitialize
create_damping_cmd(...)            # kd=8
create_zero_cmd(...); init_cmd_hg(...)
MotorMode.PR = 0; MotorMode.AB = 1
```

---

## Gotchas to carry into next session

- **`_common/weighted_moving_filter.py` imports `matplotlib.pyplot` at module
  top.** Importing the SDK's filter pulls matplotlib as a hard dependency. Use
  our own filter; don't import theirs.
- **Pinocchio is intentionally absent from `pyproject.toml`** (`pyproject.toml`
  comment block) — install via conda-forge only (PyPI `pin` lacks
  `pinocchio.casadi`). Same constraint we already live with.
- **`unitree_sdk2py` + `inspire_sdkpy` are off-PyPI** — install from checkout;
  `unitree_sdk2py` needs CycloneDDS (README).
- **README "rebuild will fail" vs. code that filters joints** — documented
  discrepancy (Section A / F). Don't trust the README over the source here.
- **`__enter__` ≠ full bring-up** — connects only; call `initialize()` /
  `start_streaming()` explicitly (Section E).
- **`set_ee_targets` double-filters** — prefer `set_joint_targets` (Section E).
- **`hand_mass_kg = 0.0` is a TODO** — measure before trusting force estimates
  (Section D).
