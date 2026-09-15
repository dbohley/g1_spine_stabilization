"""
Head-obstacle collision checking for the pre-grasp approach planner.

Builds a static sphere obstacle representing the tracked head/object at some
center, and checks arm/hand joint configurations against it using the
GeometryModel Pinocchio already builds for free from the URDF's <collision>
tags (G1HandIK.reduced.collision_model) -- no URDF edit, no new dependency.
A sphere (rather than the earlier axis-aligned box) is orientation-invariant,
so it doesn't matter that we never rotate the obstacle to match the physical
head/object marker's actual measured tilt.

Two-tier padded/true split: the final grasp/standoff goal is meant to sit
right next to (or touching) the head, so it must never be checked against a
margin-padded sphere (every goal would then read as "in collision"). Padded
shape governs general transit; true (unpadded) shape governs the goal
config and the final short goal-connection edge.
"""

import copy
from dataclasses import dataclass

import numpy as np
import pinocchio as pin
import hppfcl


@dataclass
class HeadObstacle:
    """Generic static collision obstacle: a placement plus a padded/true
    CollisionGeometry shape pair. Despite the name (this module started
    sphere-only), the fields are shape-agnostic in practice --
    CollisionChecker only ever calls generic CollisionGeometry/Pinocchio
    APIs on padded_shape/true_shape, so this same dataclass is reused for
    both sphere obstacles (build_head_obstacle) and plane obstacles
    (build_floor_obstacle, an hppfcl.Halfspace instead of an hppfcl.Sphere)."""
    placement: pin.SE3
    padded_shape: "hppfcl.CollisionGeometry"
    true_shape: "hppfcl.CollisionGeometry"


def build_head_obstacle(center, radius: float, margin_m: float) -> HeadObstacle:
    """Static sphere obstacle at center (pelvis frame), given radius (meters).

    Rotation is irrelevant for a sphere -- placement carries translation only.
    Margin is added directly to the radius (unlike the old box's "+2*margin
    per full side length" convention: hppfcl.Sphere(r) takes the radius
    itself, so growing the padded radius by margin_m on every side is just
    r + margin_m, not r + 2*margin_m).
    """
    center = np.asarray(center, dtype=float)
    radius = float(radius)
    placement = pin.SE3(np.eye(3), center)
    padded_shape = hppfcl.Sphere(radius + margin_m)
    true_shape = hppfcl.Sphere(radius)
    return HeadObstacle(placement=placement, padded_shape=padded_shape, true_shape=true_shape)


def build_head_obstacle_stack(center, radius: float, margin_m: float,
                               stack_count: int = 0, stack_spacing_m: float = 0.0,
                               axis=None) -> "list[HeadObstacle]":
    """[primary] + stack_count additional HeadObstacles, each the same
    radius as the primary, spaced stack_spacing_m apart along axis (a unit
    or non-unit 3-vector -- normalized here so callers don't have to).

    Root cause this addresses: a single sphere only models the volume right
    around the tracked marker -- in some configurations RRT* correctly finds
    the single sphere collision-free by routing a hand UNDER it, which is
    unsafe in reality since there is more physical head/neck and the table
    surface below the sphere that the single-sphere model doesn't represent
    at all. Stacking extends the modeled obstacle volume further along the
    same "into the object" depth direction the primary sphere is already
    offset along (see head_sphere_center_from_marker), so a hand routing
    under the primary sphere runs into a stacked one instead.

    stack_count=0 (default) returns exactly [build_head_obstacle(center,
    radius, margin_m)] -- today's single-sphere behavior, unchanged. For
    the stacked spheres to leave no gap an RRT edge could slip through,
    stack_spacing_m should be less than 2*radius (so consecutive spheres
    overlap); this is the caller's responsibility to configure sensibly,
    not enforced here.
    """
    center = np.asarray(center, dtype=float)
    obstacles = [build_head_obstacle(center, radius, margin_m)]
    if stack_count <= 0:
        return obstacles
    axis = np.array([0.0, 0.0, -1.0]) if axis is None else np.asarray(axis, dtype=float)
    norm = np.linalg.norm(axis)
    if norm > 0:
        axis = axis / norm
    for i in range(1, stack_count + 1):
        stacked_center = center + i * stack_spacing_m * axis
        obstacles.append(build_head_obstacle(stacked_center, radius, margin_m))
    return obstacles


def build_floor_obstacle(z_min: float, margin_m: float = 0.0,
                          normal=(0.0, 0.0, 1.0)) -> HeadObstacle:
    """Static half-space obstacle (a 'floor' or 'table') -- everything on
    the solid side of the plane (by default, z <= z_min in pelvis frame) is
    treated as an obstacle. Reuses HeadObstacle's generic
    placement+padded/true-shape structure (despite the name, it's shape-
    agnostic -- CollisionChecker only ever calls generic
    CollisionGeometry/pin APIs on padded_shape/true_shape, verified working
    identically for hppfcl.Halfspace as for hppfcl.Sphere).

    Unlike the head sphere (where the goal is MEANT to sit close to the
    true surface), there's no legitimate reason for either RRT transit OR
    the final goal to be below a real physical floor/table -- so padded
    (transit) sits margin_m HIGHER than true (goal), same directional
    convention as head_margin_m (padded = more conservative), applied
    here to both checks since there's no goal exception to carve out.

    `normal`, default world/pelvis +Z: the direction pointing AWAY from
    the solid region (i.e. "up" off the floor). placement is trivial
    (identity, zero translation) -- the plane's location is carried
    entirely in each shape's own `d` parameter for simplicity, unlike the
    sphere builders which carry their center in `placement`.
    """
    n = np.asarray(normal, dtype=float)
    n = n / np.linalg.norm(n)
    placement = pin.SE3(np.eye(3), np.zeros(3))
    padded_shape = hppfcl.Halfspace(n, float(z_min) + float(margin_m))
    true_shape = hppfcl.Halfspace(n, float(z_min))
    return HeadObstacle(placement=placement, padded_shape=padded_shape, true_shape=true_shape)


def head_sphere_center_from_marker(marker_pos, marker_R, radius: float, offset=None, anchor_distance=None) -> np.ndarray:
    """Sphere center such that the sphere's TOP surface sits at the marker.

    Mirrors trajectory/sources.py's box_half_height convention: the object
    marker is mounted on top of the physical head/object, so its raw detected
    position is shifted down along its own measured "out of surface" axis
    (column 2 of its rotation matrix) by `anchor_distance` (default: the
    sphere's own radius) to recover the obstacle's true center.

    `anchor_distance`, if given, decouples the marker-to-center depth from
    `radius`: the marker sits a fixed physical distance above the head's true
    centroid regardless of how big a collision-avoidance radius is chosen, so
    conflating the two means tuning the radius also silently relocates the
    sphere. `None` (the default) reproduces the old coupled behavior
    (anchor depth == radius) exactly.

    `offset`, if given, is an additional [x, y, z] nudge expressed in the
    marker's own rotation frame (same axis convention as the shift above),
    rotated into world frame and added on top of it -- lets the center be
    fit to a non-perfectly-spherical/off-center head. `None` (the default)
    reproduces the plain behavior exactly.
    """
    marker_pos = np.asarray(marker_pos, dtype=float)
    marker_R = np.asarray(marker_R, dtype=float)
    depth = float(radius) if anchor_distance is None else float(anchor_distance)
    center = marker_pos - depth * marker_R[:, 2]
    if offset is not None:
        center = center + marker_R @ np.asarray(offset, dtype=float)
    return center


class CollisionChecker:
    """Checks a 14-DOF joint config against one or more HeadObstacles.

    Builds its own per-instance copy of the collision model/data (via
    copy.deepcopy) so repeated construction across planning calls never
    mutates G1HandIK's shared ik.reduced.collision_model, and never
    accumulates stale geometry/pairs from a previous call (box_center moves
    between grasp attempts).
    """

    def __init__(self, ik, obstacles: "list[HeadObstacle]"):
        self.model = ik.reduced.model
        self.data = self.model.createData()
        self.gmodel = copy.deepcopy(ik.reduced.collision_model)

        # Every obstacle (the primary sphere, plus any stacked ones -- see
        # build_head_obstacle_stack) gets its own uniquely-named padded/true
        # geometry pair, but ALL of them feed the SAME _padded_pair_ids/
        # _true_pair_ids lists below -- is_valid_transit/is_valid_contact
        # already just check "any collision among these pair ids," so a
        # single-obstacle vs. multi-obstacle stack needs no change to those
        # methods, only to how these lists get populated here.
        self._obstacle_geom_ids = set()
        self._padded_pair_ids = []
        self._true_pair_ids = []
        for obs_idx, head in enumerate(obstacles):
            padded_id = self.gmodel.addGeometryObject(
                pin.GeometryObject(f"head_padded_{obs_idx}", 0, head.placement, head.padded_shape)
            )
            true_id = self.gmodel.addGeometryObject(
                pin.GeometryObject(f"head_true_{obs_idx}", 0, head.placement, head.true_shape)
            )
            self._obstacle_geom_ids.add(padded_id)
            self._obstacle_geom_ids.add(true_id)

            # Only pair the head against geometry whose pose actually depends
            # on q (parentJoint > 0 -- the 14 arm/hand DOFs). Static leg/
            # waist/torso geoms (parentJoint == 0) are excluded: their pose
            # relative to the head never changes for a fixed box_center, and
            # including them would falsely report every sample as colliding
            # if the tracked head position happened to already sit near a
            # static torso shape -- a failure no arm motion could ever fix.
            for i, geom in enumerate(self.gmodel.geometryObjects):
                if i in self._obstacle_geom_ids:
                    continue
                if geom.parentJoint > 0:
                    self._padded_pair_ids.append(len(self.gmodel.collisionPairs))
                    self.gmodel.addCollisionPair(pin.CollisionPair(i, padded_id))
                    self._true_pair_ids.append(len(self.gmodel.collisionPairs))
                    self.gmodel.addCollisionPair(pin.CollisionPair(i, true_id))
        if not self._padded_pair_ids:
            raise RuntimeError(
                "[CollisionChecker] No arm/hand collision geometry found "
                "(parentJoint > 0) -- reduced collision model may be empty."
            )

        self.gdata = pin.GeometryData(self.gmodel)

    def _check(self, q, pair_ids) -> bool:
        """True if q is collision-free against exactly the given pair ids.

        Only the requested pairs are activated before checking, so a
        stop_at_first_collision=True result unambiguously refers to this
        geometry (padded or true head) -- never a pair belonging to the
        other one.
        """
        self.gdata.deactivateAllCollisionPairs()
        for k in pair_ids:
            self.gdata.activateCollisionPair(k)
        in_collision = pin.computeCollisions(
            self.model, self.data,
            self.gmodel, self.gdata, np.asarray(q, dtype=float),
            True,
        )
        return not in_collision

    def is_valid_transit(self, q) -> bool:
        return self._check(q, self._padded_pair_ids)

    def is_valid_contact(self, q) -> bool:
        return self._check(q, self._true_pair_ids)

    def first_colliding_geometry(self, q, pair_ids) -> "str | None":
        """Name of the first arm/hand geometry found colliding against the
        given obstacle-tier pairs (self._padded_pair_ids/_true_pair_ids), or
        None if q is collision-free against them. Diagnostic only -- used on
        the exception path (a failed start/goal precondition) to turn an
        opaque "GoalInCollisionError" into something that names the actual
        offending link (e.g. a hand's collision box, or an elbow/forearm
        mesh), not the hot planning loop, so this re-running
        pin.computeCollisions has no performance impact there."""
        self.gdata.deactivateAllCollisionPairs()
        for k in pair_ids:
            self.gdata.activateCollisionPair(k)
        pin.computeCollisions(
            self.model, self.data,
            self.gmodel, self.gdata, np.asarray(q, dtype=float),
            True,
        )
        for k in pair_ids:
            if self.gdata.collisionResults[k].isCollision():
                pair = self.gmodel.collisionPairs[k]
                geom_idx = pair.first if pair.second in self._obstacle_geom_ids else pair.second
                return self.gmodel.geometryObjects[geom_idx].name
        return None

    def first_colliding_pair(self, q, pair_ids) -> "tuple[str, str] | None":
        """(arm_geometry_name, obstacle_geometry_name) for the first colliding
        pair, or None if q is collision-free against pair_ids.

        Companion to first_colliding_geometry, which names only the ARM side.
        That is not enough to diagnose an abort: every obstacle -- the primary
        head sphere, each stacked sphere, AND the floor half-space -- is
        registered here as "head_padded_{i}"/"head_true_{i}", so a message
        built from the arm name alone cannot say WHICH obstacle was hit. The
        obstacle index maps to the order obstacles were passed to __init__
        (see rrt_planner.plan_collision_free_approach: spheres first, floor
        last when configured)."""
        self.gdata.deactivateAllCollisionPairs()
        for k in pair_ids:
            self.gdata.activateCollisionPair(k)
        pin.computeCollisions(
            self.model, self.data,
            self.gmodel, self.gdata, np.asarray(q, dtype=float),
            True,
        )
        for k in pair_ids:
            if self.gdata.collisionResults[k].isCollision():
                pair = self.gmodel.collisionPairs[k]
                if pair.second in self._obstacle_geom_ids:
                    arm_idx, obs_idx = pair.first, pair.second
                else:
                    arm_idx, obs_idx = pair.second, pair.first
                return (self.gmodel.geometryObjects[arm_idx].name,
                        self.gmodel.geometryObjects[obs_idx].name)
        return None
