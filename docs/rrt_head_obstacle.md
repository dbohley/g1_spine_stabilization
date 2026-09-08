# RRT* head-obstacle: sphere geometry, failure modes, and tuning

Reference for the RRT*-planned pre-grasp approach's head/object obstacle
(`kinematics/collision.py`, `kinematics/rrt_planner.py`) — what the red and
orange spheres in Rerun mean, exactly when and why planning can fail, and
how to size/place the sphere against a real physical head without RRT
failing for reasons that aren't obvious from the code alone.

This doc covers the **sphere and the planner's pass/fail behavior only**.
It does not cover motion smoothness/jerk during a successful approach —
see `docs/followups.md`'s "IK smoothing-filter staleness + goal_q local-
minimum" entry and CLAUDE.md's Current Status for that separate history.

## Sphere geometry: red (true) vs. orange (padded)

`log_head_sphere()` (`controllers/joint_path_execution.py`) logs two
concentric spheres to Rerun, both centered at the same point
(`sphere_center`):

| Entity | Color | Radius | Rerun alpha |
|---|---|---|---|
| `obstacles/head_true` | red `[255, 80, 80]` | `head_radius_m` exactly | 90 (more opaque) |
| `obstacles/head_padded` | orange `[255, 200, 80]` | `head_radius_m + head_margin_m` | 40 (more transparent), only logged when `head_margin_m > 0` |

These map directly onto `kinematics/collision.py`'s `CollisionChecker`,
which builds both an unpadded (`true_shape`) and padded (`padded_shape`)
`hppfcl.Sphere` from the same `HeadObstacle` and checks different things
against each:

- **Red (true, unpadded)** — `is_valid_contact()`. Used for the **goal
  configuration** (`q_goal`, the pregrasp standoff the hands are trying to
  reach) and the **final goal-connect edge** (the last stretch of the RRT
  path, right before it touches `q_goal`).
- **Orange (padded)** — `is_valid_transit()`. Used for the **start
  configuration** (`q_start`, wherever the arm currently is) and **every
  other point along the planned path** (all RRT sampling, edge checks, and
  shortcutting).

**Why the split exists, not just what it is:** the pregrasp standoff is
*meant* to sit close to the head — that's the whole point of a "standoff"
(`pregrasp_offset` away from the object, not far from it). If the goal were
checked against the padded sphere, every legitimate standoff goal would
read as "in collision" and RRT could never succeed. `head_margin_m` exists
to buffer general transit motion (marker/calibration uncertainty, ~0.5-1.5cm
per the config's own comment) against getting too close to the head while
still *moving toward* it — it is not meant to buffer the final resting
goal itself.

### Sphere placement (`head_sphere_center_from_marker`)

```
depth  = head_anchor_distance_m if set else head_radius_m
center = marker_pos - depth * marker_R[:, 2] + marker_R @ head_center_offset_m
```

- The object marker sits *on top of* the physical head/object, so its raw
  detected position is shifted down along its own measured "out of
  surface" axis (`marker_R[:, 2]`, the marker's local Z) by `depth` to
  reach the obstacle's center.
- **Coupled-by-default footgun:** if `head_anchor_distance_m` is `null`
  (the default), `depth` silently equals `head_radius_m` — tuning the
  radius for collision-avoidance sizing *also* relocates the sphere's
  center along the marker's Z axis. Set `head_anchor_distance_m` explicitly
  (a real measured marker-to-centroid depth) to decouple "how big a margin
  do I want" from "where is the head's true center," so `head_radius_m` can
  be tuned purely for avoidance sizing afterward.
- `head_center_offset_m` is an additional `[x, y, z]` nudge in the
  marker's *own* rotation frame (same axis convention as the depth shift
  above — X/Y are the marker's in-plane axes, Z is its "out of surface"
  axis), added on top of the depth shift. Use this to fit the sphere to a
  head that isn't perfectly spherical or centered under the marker, without
  touching `head_radius_m`/`head_anchor_distance_m`.

**Current config state:** `head_radius_m` is still marked `PLACEHOLDER` in
its own config comment ("MEASURE THE REAL HEAD/OBJECT RADIUS AND SET THIS
DIRECTLY") — verify it against a real measured head radius (~9-10cm)
before relying on the sphere for real protection — see Tuning below.

### Stacked spheres (`head_stack_count`)

A single sphere only models the volume right around the tracked marker.
In some configurations, RRT* correctly finds that one sphere collision-free
by routing a hand **under** it — technically valid against the single-
sphere model, but unsafe in reality, since there is more physical head/neck
and (often) a table surface below the sphere that the single-sphere model
doesn't represent at all.

`kinematics/collision.py`'s `build_head_obstacle_stack(center, radius,
margin_m, stack_count, stack_spacing_m, axis)` extends the modeled obstacle
volume by adding `stack_count` additional spheres, each the same radius as
the primary, spaced `stack_spacing_m` apart along `axis` — the same
"into the object" depth direction the primary sphere is already offset
along (`-marker_R[:, 2]` when a real marker orientation is available;
world `-Z`/gravity-down as the fallback when it isn't, e.g.
`grasp_sequence.py`'s hand-only `grasp_demo.py`, which has no camera).

Config (`collision_avoidance:`):
```yaml
head_stack_count: 0        # 0 (default) = today's single-sphere behavior, unchanged
head_stack_spacing_m: 0.0  # center-to-center distance; should be < 2*head_radius_m
                            # so consecutive spheres overlap and leave no gap an
                            # RRT edge could slip through
```

`CollisionChecker` treats every obstacle in the stack identically — a
config only needs to intrude on ANY one of them (primary or stacked) to be
reported as a collision, so `is_valid_transit`/`is_valid_contact`/the
`GoalInCollisionError`/`StartInCollisionError` failure modes below all
apply the same way whether the offending sphere is the primary one or a
stacked one. `first_colliding_geometry`'s named-link diagnostic works
identically regardless of which sphere in the stack was hit.

`log_head_sphere` renders every sphere in the stack (same red/orange
true/padded convention as the single-sphere case) as one multi-sphere
`obstacles/head_true`/`obstacles/head_padded` Rerun entity, so the whole
stack is visible together in the viewer.

## Floor/table plane (`floor_z_m`)

Spheres (single or stacked) are still an approximation of the real
obstacle volume. `collision_avoidance.floor_z_m` adds a **hard Z-axis
limit** instead — a half-space "wall," solid on the side at or below a
configured pelvis-frame height, that RRT* (and the goal check) treat
exactly like any other obstacle. This is the simplest, most direct way to
guarantee the arm never goes below a physical surface (a table, or the
floor), independent of how well the head sphere/stack happens to be sized
or placed.

Implemented via `kinematics/collision.py`'s `build_floor_obstacle(z_min,
margin_m, normal)`, using `hppfcl.Halfspace` — a genuine
`CollisionGeometry`, the same base class as the sphere's `hppfcl.Sphere` —
reusing the existing `HeadObstacle` container (placement + padded/true
shape pair) with zero changes to `CollisionChecker`. The floor is appended
to the same obstacle list the head sphere/stack already builds, so it's
checked alongside them automatically whenever `floor_z_m` is configured;
no controller call-site changes are needed (unlike the head sphere, the
floor is a fixed world/pelvis-frame constant, not derived from a tracked
marker, so it's read directly from config inside
`plan_collision_free_approach`).

**Both tiers matter here, not just transit — this is the one place the
sphere's own goal-exception rationale does NOT apply.** For the head
sphere, the goal is *meant* to sit close to the true surface (the whole
point of a standoff), so the goal check uses the unpadded radius. For a
floor/table, there's no such exception: neither the transit path NOR the
final grasp goal should ever legitimately be below a real physical
surface. So `floor_z_m`'s `padded_shape` (transit) sits `floor_margin_m`
**higher** than `true_shape` (goal) — same "padded = more conservative"
direction as `head_margin_m`, just applied uniformly here since there's no
goal carve-out to make.

Config (`collision_avoidance:`):
```yaml
floor_z_m: null        # null (default) = disabled, today's behavior unchanged
floor_margin_m: 0.0     # transit floor sits this much higher than the exact
                        # floor_z_m used for the goal check
```

**Tuning trap, worth stating plainly:** `floor_z_m` must sit BELOW every Z
height the arm legitimately needs to reach — its resting pose, and every
grasp/standoff height — or it will immediately trigger a failure mode the
moment collision avoidance runs: `StartInCollisionError` if the resting
pose itself is already "below the floor" (checked against the *padded*
tier), or `GoalInCollisionError` if the standoff/grasp goal is (checked
against the *true* tier). Verified directly: with `floor_z_m` set well
above the robot's normal working height, `plan_collision_free_approach`'s
very first precondition check raised `StartInCollisionError` naming a real
arm link — the safety-abort path works exactly like the head sphere's, no
special-casing needed. Set it comfortably below the lowest Z the arm
actually needs during the approach/grasp, verify visually in Rerun
(`obstacles/floor_true`/`obstacles/floor_padded`, same red/orange
convention, rendered as a wide flat box near the tracked object/workspace)
before trusting it live.

## Failure modes

All three planning failures are subclasses of `RRTPlanningError`
(`kinematics/rrt_planner.py`), caught identically in both controllers
(`controllers/sdk_pose_tracking.py` Phase 1a, `controllers/
grasp_sequence.py` Phase 0a) as one hard safety abort:

```
[SDKController] SAFETY ABORT: <ExceptionClassName>('<message>') — not falling
back to straight-line motion near the head. Exiting Phase 1 (skipping Phase
2, returning to default).
```

The exception's own class name appears in the printed message (via
`{exc!r}`), so the three modes are already distinguishable in the console
output today. This is a deliberate hard abort, not a fallback — the module
docstring's own rationale: falling back to the old straight-line motion
would reintroduce exactly the collision hazard this planner exists to
remove, precisely when the head is inconveniently placed (i.e. most likely
to matter).

### `StartInCollisionError`

**Trigger:** `q_start` (the arm's actual current joint configuration, read
before any planning happens) already intrudes on the **padded** (orange)
sphere.

**Message:**
```
[RRTPlanner] q_start already intrudes on the padded head obstacle (link: <name>).
```
`<name>` is the actual colliding collision-geometry link name (e.g.
`L_hand_hand_base_link_0`, an elbow/forearm mesh, etc.) — found via
`CollisionChecker.first_colliding_geometry()`, a diagnostic-only re-check
run just for this error message (not on the hot planning-loop path).

**What it means:** the arm was already too close to the (padded) head
volume *before Phase 1a/0a even began planning* — nothing about the RRT
search itself is at fault here. Most likely causes: `head_margin_m` is
large relative to the resting/current pose's real clearance from the head,
or `sphere_center` is placed such that it overlaps wherever the arm
naturally starts (e.g. a wrong `head_center_offset_m`, or an `ARM_DEFAULT_Q`
resting pose that's genuinely close to the head for this robot's geometry).

### `GoalInCollisionError`

**Trigger:** `q_goal` (`goal_q`, computed by `ik.solve_ik_refined(L_pre,
R_pre, current_q)` — the pregrasp standoff target) intrudes on the **true**
(red, unpadded) sphere.

**Message:**
```
[RRTPlanner] q_goal intrudes on the true head obstacle (link: <name>).
```

**What it means:** the pregrasp standoff itself — where the hands are
trying to end up, `pregrasp_offset` away from the object on each side — is
geometrically inside the true (red) sphere. This does NOT mean "no path
exists" or "planner ran out of budget" — it means the *destination itself*
is invalid before any search starts. Most likely causes: `head_radius_m`
too large for the real object/head being grasped, `pregrasp_offset` too
small to clear a correctly-sized sphere, or `sphere_center`/`box_center`
placed such that the goal and the obstacle overlap even though they
shouldn't geometrically (an anchor/placement mismatch rather than a true
size problem — see `docs/followups.md`'s existing writeup on the
`box_center` vs. `sphere_center` depth difference for a documented real
case of this). **This is exactly what the new `targets/L_hand`/
`targets/R_hand` Rerun visualization (see below) is for** — it lets you see
`L_pre`/`R_pre` directly next to the red sphere and judge visually whether
the goal is really inside it, rather than reasoning about it from joint-
space error messages alone.

### `RRTBudgetExhaustedError`

**Trigger:** neither endpoint failed its own precondition check, but the
sampler ran for the full `max_iters`/`time_budget_s` budget without finding
any collision-free path connecting `q_start` to `q_goal`.

**Message:**
```
[RRTPlanner] No collision-free path found within max_iters=<N>/time_budget_s=<S>.
```

**What it means:** both the start and goal are individually valid, but
every route the planner sampled within its budget was blocked by the
padded sphere (or otherwise invalid). This is the least specific failure —
it can mean a genuinely difficult/tight gap around the sphere, but it can
also just be bad luck: this is a **stochastic, one-shot planner** (fixed
iteration budget, no retry), and the module's own docstring documents
~90-93% empirical success rate at reasonable iteration counts as expected,
*safe* behavior — not a bug. If this fires occasionally with otherwise
reasonable config values, that's consistent with normal stochastic
variance, not necessarily a sign something is misconfigured. If it fires
*consistently*, worth checking: `max_iters`/`time_budget_s` (config:
`collision_avoidance.max_iters`, default 3000; `time_budget_s`, default
2.0s — cheap to raise, since typical successful plans finish in ~0.05-0.1s),
`segment_bias`/`segment_sigma_rad` (whether the straight-line-biased
sampling can actually reach around the obstacle), or whether the sphere is
simply large enough relative to the arm's reachable workspace that few/no
valid detours exist at all.

## Visualizing the Cartesian goal against the sphere

`controllers/joint_path_execution.py`'s `log_hand_goal_targets(L_target,
R_target)` logs the actual SE3 targets Phase 1a/1b's IK/RRT are solving
toward — `targets/L_hand` (cyan) and `targets/R_hand` (magenta) — the same
entity names and colors Phase 2's tracking loop already uses for its own
per-step targets (`controllers/sdk_pose_tracking.py`'s `_log_marker_info`),
so the viewer sees continuous, consistent goal markers across all phases
instead of a gap during the RRT approach. Both controllers call this once,
right after computing `L_pre`/`R_pre` (the pregrasp standoff — the same
target `goal_q` is solved for), alongside the existing `log_head_sphere`
call.

With this, a dry-run preview shows, at the same time: the red/orange
sphere (the obstacle), the cyan/magenta arrows (the actual Cartesian goal
IK/RRT are aiming for), and (on any `RRTPlanningError` abort)
`log_hand_collision_boxes(ik, current_q)`/`log_hand_collision_boxes(ik,
goal_q)` — the real collision geometry at the start and goal configs. Together
these directly answer "is the goal really inside the sphere, or is
something about how the sphere/goal are anchored not lining up the way I
expect" — visually, without having to reason about it from raw numbers.

## Tuning procedure for a real physical head

1. Measure the real head/object: its radius (or half-width, if not
   spherical), and the physical distance from the marker's mounting point
   down to the head's true centroid (this is `head_anchor_distance_m`, NOT
   necessarily the same as the radius you want to use for avoidance).
2. Set `head_anchor_distance_m` to that measured depth (decoupling it from
   `head_radius_m`, per the coupled-by-default footgun above), and set
   `head_radius_m` to the measured (or slightly padded) real radius —
   replacing the current `0.01` placeholder.
3. Run a dry-run preview with Rerun open, e.g.:
   ```
   python scripts/run_pose_tracking_preview.py --source mock --tracking orient-only --dry-run
   ```
4. In Rerun, visually check: does the red sphere (`obstacles/head_true`)
   actually enclose the real head volume you're modeling? Do the cyan/
   magenta goal arrows (`targets/L_hand`/`targets/R_hand`) land clear of
   the red sphere, at roughly the standoff distance you expect
   (`pregrasp_offset` outside the object surface)?
5. If the goal arrows visually overlap the red sphere but you believe they
   shouldn't (geometrically), that's a placement/anchor issue, not a size
   issue — check `head_center_offset_m` and compare `sphere_center`'s
   computation against `box_center`'s (`trajectory/sources.py`) for a
   depth/anchor mismatch (see `docs/followups.md` for a documented real
   instance of this).
6. If planning fails, read the printed exception class name against the
   Failure Modes section above to know which of the three situations
   you're actually in before changing anything.
7. Iterate: adjust `head_radius_m`/`head_center_offset_m`/
   `head_anchor_distance_m`/`pregrasp_offset` as needed, re-run, repeat
   until the sphere visually matches the real head and the standoff goal
   sits just outside it.

See also: `docs/followups.md`'s "IK smoothing-filter staleness + goal_q
local-minimum" and earlier calibration entries for related, previously-
investigated anchor/placement issues in this same pipeline.
