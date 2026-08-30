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
    placement: pin.SE3
    padded_shape: "hppfcl.Sphere"
    true_shape: "hppfcl.Sphere"


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


def head_sphere_center_from_marker(marker_pos, marker_R, radius: float) -> np.ndarray:
    """Sphere center such that the sphere's TOP surface sits at the marker.

    Mirrors trajectory/sources.py's box_half_height convention: the object
    marker is mounted on top of the physical head/object, so its raw detected
    position is shifted down along its own measured "out of surface" axis
    (column 2 of its rotation matrix) by the sphere's radius to recover the
    obstacle's true center.
    """
    marker_pos = np.asarray(marker_pos, dtype=float)
    marker_R = np.asarray(marker_R, dtype=float)
    return marker_pos - float(radius) * marker_R[:, 2]


class CollisionChecker:
    """Checks a 14-DOF joint config against a HeadObstacle.

    Builds its own per-instance copy of the collision model/data (via
    copy.deepcopy) so repeated construction across planning calls never
    mutates G1HandIK's shared ik.reduced.collision_model, and never
    accumulates stale geometry/pairs from a previous call (box_center moves
    between grasp attempts).
    """

    def __init__(self, ik, head: HeadObstacle):
        self.model = ik.reduced.model
        self.data = self.model.createData()
        self.gmodel = copy.deepcopy(ik.reduced.collision_model)

        self._padded_id = self.gmodel.addGeometryObject(
            pin.GeometryObject("head_padded", 0, head.placement, head.padded_shape)
        )
        self._true_id = self.gmodel.addGeometryObject(
            pin.GeometryObject("head_true", 0, head.placement, head.true_shape)
        )

        # Only pair the head against geometry whose pose actually depends on
        # q (parentJoint > 0 -- the 14 arm/hand DOFs). Static leg/waist/torso
        # geoms (parentJoint == 0) are excluded: their pose relative to the
        # head never changes for a fixed box_center, and including them
        # would falsely report every sample as colliding if the tracked head
        # position happened to already sit near a static torso shape -- a
        # failure no arm motion could ever fix.
        self._padded_pair_ids = []
        self._true_pair_ids = []
        for i, geom in enumerate(self.gmodel.geometryObjects):
            if i in (self._padded_id, self._true_id):
                continue
            if geom.parentJoint > 0:
                self._padded_pair_ids.append(len(self.gmodel.collisionPairs))
                self.gmodel.addCollisionPair(pin.CollisionPair(i, self._padded_id))
                self._true_pair_ids.append(len(self.gmodel.collisionPairs))
                self.gmodel.addCollisionPair(pin.CollisionPair(i, self._true_id))
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
                geom_idx = pair.first if pair.second in (self._padded_id, self._true_id) else pair.second
                return self.gmodel.geometryObjects[geom_idx].name
        return None
