"""
Joint-space RRT* planner for the collision-avoiding pre-grasp approach.

Runs ONCE per approach (not a per-tick replanner) between a start and goal
joint configuration, using G1HandIK.solve_ik() only once (by the caller, to
get q_goal) -- all sampling/collision-checking here is cheap Pinocchio
forward-kinematics + collision queries, no IPOPT involved.

Practical simplifications from textbook RRT*, deliberate and confirmed:
  - Fixed rewire radius, not the asymptotically-shrinking-ball formula --
    with a one-shot ~1500-iteration budget in 14D we're nowhere near the
    regime where the asymptotic optimality guarantee matters; path quality
    is delivered by shortcut_path(), not RRT*'s own convergence proof.
  - No cost propagation to a rewired node's descendants -- acceptable for a
    bounded one-shot tree that isn't reused across many queries.
  - Search stops at the FIRST valid goal connection rather than continuing
    to refine for the rest of the budget.
  - Segment-biased sampling, not pure uniform-over-joint-limits. Measured
    directly against a real obstruction: pure uniform sampling in 14D failed
    to find ANY path within 5000 iterations (most samples differ from both
    q_start and q_goal in many irrelevant joints simultaneously, so nearest-
    neighbor extension mostly "fixes" those instead of making progress
    toward the goal -- a known failure mode of uniform RRT in higher
    dimensions). The identical scenario succeeds in ~150 iterations once
    most samples are drawn near the straight-line q_start->q_goal segment
    (with Gaussian spread, for the detour) instead. This is the right
    trade-off for THIS use case specifically: one localized obstacle near an
    otherwise-direct path, not general exploration of the full 14D space. A
    smaller fraction of samples still come from pure uniform sampling, so
    the planner isn't strictly confined to the segment's neighborhood if a
    wider detour is ever needed.
"""

import time
from typing import Callable, Optional

import numpy as np


class RRTPlanningError(Exception):
    """Base class for all planning failures. Callers should treat any of
    these as a hard safety abort -- never silently fall back to the old
    straight-line motion, since that would reintroduce exactly the hazard
    this planner exists to remove, precisely when the head is inconveniently
    placed (i.e. most likely to matter)."""


class StartInCollisionError(RRTPlanningError):
    """The arm's current configuration already intrudes on the padded head
    obstacle before any motion starts."""


class GoalInCollisionError(RRTPlanningError):
    """The IK-solved goal configuration intrudes on the true (unpadded) head
    obstacle -- e.g. pregrasp_offset too small for this head size."""


class RRTBudgetExhaustedError(RRTPlanningError):
    """No collision-free path to the goal was found within max_iters/
    time_budget_s."""


def _edge_collision_free(q1: np.ndarray, q2: np.ndarray,
                          is_valid_fn: Callable[[np.ndarray], bool],
                          resolution_rad: float,
                          include_endpoints: bool = True) -> bool:
    """Check a straight joint-space line q1->q2 at resolution_rad spacing."""
    dist = float(np.linalg.norm(q2 - q1))
    if dist == 0.0:
        return True
    n = max(1, int(np.ceil(dist / resolution_rad)))
    lo = 0 if include_endpoints else 1
    hi = n if include_endpoints else n - 1
    for i in range(lo, hi + 1):
        t = i / n
        if not is_valid_fn(q1 + t * (q2 - q1)):
            return False
    return True


class RRTStarPlanner:
    def __init__(self, lower: np.ndarray, upper: np.ndarray, checker, *,
                 step_size_rad: float = 0.3, goal_bias: float = 0.15,
                 segment_bias: float = 0.8, segment_sigma_rad: float = 0.2,
                 rewire_radius_rad: float = 0.6, max_iters: int = 3000,
                 time_budget_s: float = 2.0,
                 edge_check_resolution_rad: float = 0.05,
                 goal_connect_radius_rad: float = 0.4,
                 seed: Optional[int] = None):
        self.lower = np.asarray(lower, dtype=float)
        self.upper = np.asarray(upper, dtype=float)
        self.checker = checker
        self.step_size_rad = step_size_rad
        self.goal_bias = goal_bias
        self.segment_bias = segment_bias
        self.segment_sigma_rad = segment_sigma_rad
        self.rewire_radius_rad = rewire_radius_rad
        self.max_iters = max_iters
        self.time_budget_s = time_budget_s
        self.edge_check_resolution_rad = edge_check_resolution_rad
        self.goal_connect_radius_rad = goal_connect_radius_rad
        self.rng = np.random.default_rng(seed)

    def _sample(self, q_start: np.ndarray, q_goal: np.ndarray) -> np.ndarray:
        r = self.rng.random()
        if r < self.goal_bias:
            return q_goal.copy()
        if r < self.goal_bias + self.segment_bias:
            u = self.rng.random()
            base = (1 - u) * q_start + u * q_goal
            sample = base + self.rng.normal(0.0, self.segment_sigma_rad, size=base.shape)
            return np.clip(sample, self.lower, self.upper)
        return self.rng.uniform(self.lower, self.upper)

    def _steer(self, q_from: np.ndarray, q_to: np.ndarray) -> np.ndarray:
        delta = q_to - q_from
        dist = float(np.linalg.norm(delta))
        if dist <= self.step_size_rad:
            return q_to.copy()
        return q_from + delta / dist * self.step_size_rad

    def _transit_edge_valid(self, q1: np.ndarray, q2: np.ndarray) -> bool:
        return _edge_collision_free(
            q1, q2, self.checker.is_valid_transit,
            self.edge_check_resolution_rad,
        )

    def _culprit_suffix(self, q, pair_ids) -> str:
        """" (link: <arm geom> vs <obstacle geom>)" for the first colliding
        pair, or "" if nothing collides / the checker predates
        first_colliding_pair. Names BOTH sides deliberately: the obstacle list
        holds the head sphere, any stacked spheres, and the floor half-space,
        all registered as "head_padded_{i}"/"head_true_{i}", so reporting only
        the arm link (as this did originally) leaves the reader unable to tell
        which obstacle actually fired."""
        pair_fn = getattr(self.checker, "first_colliding_pair", None)
        if pair_fn is not None:
            culprit = pair_fn(q, pair_ids)
            if culprit:
                return f" (link: {culprit[0]} vs {culprit[1]})"
            return ""
        culprit = self.checker.first_colliding_geometry(q, pair_ids)
        return f" (link: {culprit})" if culprit else ""

    def plan(self, q_start: np.ndarray, q_goal: np.ndarray) -> np.ndarray:
        """Returns (N,14) waypoints; path[0]==q_start, path[-1]==q_goal exactly."""
        q_start = np.asarray(q_start, dtype=float)
        q_goal = np.asarray(q_goal, dtype=float)

        if not self.checker.is_valid_transit(q_start):
            raise StartInCollisionError(
                "[RRTPlanner] q_start already intrudes on a padded obstacle"
                f"{self._culprit_suffix(q_start, self.checker._padded_pair_ids)}."
            )
        if not self.checker.is_valid_contact(q_goal):
            raise GoalInCollisionError(
                "[RRTPlanner] q_goal intrudes on a true obstacle"
                f"{self._culprit_suffix(q_goal, self.checker._true_pair_ids)}."
            )

        # Fast path: if the direct line is already clear, skip sampling.
        if _edge_collision_free(q_start, q_goal, self.checker.is_valid_transit,
                                 self.edge_check_resolution_rad, include_endpoints=False):
            return np.array([q_start, q_goal])

        tree_q = [q_start]
        parent = [-1]
        cost = [0.0]
        goal_idx = None
        deadline = time.monotonic() + self.time_budget_s

        for _ in range(self.max_iters):
            if time.monotonic() > deadline:
                break

            q_rand = self._sample(q_start, q_goal)
            tree_arr = np.array(tree_q)
            dists_to_rand = np.linalg.norm(tree_arr - q_rand, axis=1)
            near_idx = int(np.argmin(dists_to_rand))
            q_new = self._steer(tree_q[near_idx], q_rand)

            if not self.checker.is_valid_transit(q_new):
                continue
            if not self._transit_edge_valid(tree_q[near_idx], q_new):
                continue

            dists_to_new = np.linalg.norm(tree_arr - q_new, axis=1)
            near_set = np.where(dists_to_new <= self.rewire_radius_rad)[0]

            best_parent = near_idx
            best_cost = cost[near_idx] + float(dists_to_new[near_idx])
            for j in near_set:
                j = int(j)
                if j == near_idx:
                    continue
                candidate_cost = cost[j] + float(dists_to_new[j])
                if candidate_cost < best_cost and self._transit_edge_valid(tree_q[j], q_new):
                    best_cost = candidate_cost
                    best_parent = j

            new_idx = len(tree_q)
            tree_q.append(q_new)
            parent.append(best_parent)
            cost.append(best_cost)

            for j in near_set:
                j = int(j)
                if j == best_parent:
                    continue
                candidate_cost = best_cost + float(dists_to_new[j])
                if candidate_cost < cost[j] and self._transit_edge_valid(q_new, tree_q[j]):
                    parent[j] = new_idx
                    cost[j] = candidate_cost

            if np.linalg.norm(q_new - q_goal) <= self.goal_connect_radius_rad:
                # The whole final edge -- not just the q_goal endpoint -- is
                # checked against is_valid_contact (true, unpadded box), not
                # is_valid_transit (padded). Using the padded check here was
                # a real bug: a standoff goal deliberately sits close to the
                # padded boundary by design (head_margin_m buffers general
                # transit, not the standoff's own intended clearance), so
                # checking the approach edge against the padded box could
                # make the goal permanently unreachable even though q_goal
                # itself has full clearance against the true head surface.
                if _edge_collision_free(q_new, q_goal, self.checker.is_valid_contact,
                                         self.edge_check_resolution_rad, include_endpoints=True):
                    goal_idx = len(tree_q)
                    tree_q.append(q_goal.copy())
                    parent.append(new_idx)
                    cost.append(best_cost + float(np.linalg.norm(q_new - q_goal)))
                    break

        if goal_idx is None:
            raise RRTBudgetExhaustedError(
                f"[RRTPlanner] No collision-free path found within "
                f"max_iters={self.max_iters}/time_budget_s={self.time_budget_s}."
            )

        path = []
        idx = goal_idx
        while idx != -1:
            path.append(tree_q[idx])
            idx = parent[idx]
        path.reverse()
        return np.array(path)


def shortcut_path(path: np.ndarray, checker,
                   edge_check_resolution_rad: float = 0.05,
                   max_attempts: int = 150,
                   seed: Optional[int] = None) -> np.ndarray:
    """Random-shortcut smoothing: repeatedly try connecting two non-adjacent
    waypoints directly, keep the shortcut if the interior of the new segment
    is collision-free. Endpoints are never re-validated (they're already
    known-valid waypoints from the RRT* path)."""
    rng = np.random.default_rng(seed)
    waypoints = [np.asarray(q, dtype=float) for q in path]

    for _ in range(max_attempts):
        if len(waypoints) <= 2:
            break
        i, j = sorted(rng.integers(0, len(waypoints), size=2))
        if j - i < 2:
            continue
        if _edge_collision_free(waypoints[i], waypoints[j], checker.is_valid_transit,
                                 edge_check_resolution_rad, include_endpoints=False):
            waypoints = waypoints[:i + 1] + waypoints[j:]

    return np.array(waypoints)


def plan_collision_free_approach(ik, q_start: np.ndarray, q_goal: np.ndarray,
                                  center: np.ndarray, radius: float,
                                  cfg: dict, axis=None) -> np.ndarray:
    """The one function controllers call. Builds a HeadObstacle stack (the
    primary sphere at `center`, plus any configured stacked spheres) +
    CollisionChecker from cfg['collision_avoidance'], runs RRTStarPlanner,
    shortcuts the result, and returns the final waypoint array. Raises
    RRTPlanningError subclasses on failure -- callers must not silently fall
    back to straight-line motion (see module docstring).

    `axis`, if given, is the direction (3-vector, need not be normalized)
    any configured stacked spheres extend along, past the primary sphere --
    see build_head_obstacle_stack. Only matters when
    collision_avoidance.head_stack_count > 0; ignored (and no stack built)
    otherwise, so passing None here is always safe for the default
    single-sphere behavior.
    """
    # Imported here (not at module top) to keep collision.py's hppfcl/coal
    # dependency lazy for any caller that only needs the planner's exceptions.
    from g1_spine_stabilization.kinematics.collision import (
        build_head_obstacle_stack, build_floor_obstacle, CollisionChecker,
    )

    ca_cfg = cfg.get("collision_avoidance", {})
    obstacles = build_head_obstacle_stack(
        center, radius, ca_cfg.get("head_margin_m", 0.0),
        stack_count=ca_cfg.get("head_stack_count", 0),
        stack_spacing_m=ca_cfg.get("head_stack_spacing_m", 0.0),
        axis=axis,
    )
    # Floor/table plane: a fixed pelvis-frame constant (unlike the head
    # sphere/stack, which depend on the tracked marker), so it's read
    # directly from config here rather than threaded through as a caller
    # argument -- applies automatically to every call site once configured.
    floor_z = ca_cfg.get("floor_z_m", None)
    if floor_z is not None:
        obstacles.append(build_floor_obstacle(floor_z, ca_cfg.get("floor_margin_m", 0.0)))
    checker = CollisionChecker(ik, obstacles)
    lower, upper = ik.effective_joint_limits()

    planner = RRTStarPlanner(
        lower, upper, checker,
        step_size_rad=ca_cfg.get("step_size_rad", 0.3),
        goal_bias=ca_cfg.get("goal_bias", 0.15),
        segment_bias=ca_cfg.get("segment_bias", 0.8),
        segment_sigma_rad=ca_cfg.get("segment_sigma_rad", 0.2),
        rewire_radius_rad=ca_cfg.get("rewire_radius_rad", 0.6),
        max_iters=ca_cfg.get("max_iters", 3000),
        time_budget_s=ca_cfg.get("time_budget_s", 2.0),
        edge_check_resolution_rad=ca_cfg.get("edge_check_resolution_rad", 0.05),
        goal_connect_radius_rad=ca_cfg.get("goal_connect_radius_rad", 0.4),
        seed=ca_cfg.get("seed", None),
    )
    raw_path = planner.plan(q_start, q_goal)
    return shortcut_path(
        raw_path, checker,
        edge_check_resolution_rad=ca_cfg.get("edge_check_resolution_rad", 0.05),
        max_attempts=ca_cfg.get("shortcut_attempts", 150),
        seed=ca_cfg.get("seed", None),
    )
