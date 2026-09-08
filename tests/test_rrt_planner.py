"""
Hardware-free tests for kinematics/rrt_planner.py.

Uses a real G1HandIK (module-scoped, ~10s cold start amortized across this
file's tests -- same convention as test_kinematics.py/test_gravity_comp.py)
for real collision geometry, but plans against small synthetic obstacles
placed at deliberately-chosen joint configurations rather than the
production box/head geometry, so these tests are fast and independent of
whatever configs/default.yaml's collision_avoidance tuning happens to be.
"""

import numpy as np
import pytest

from g1_spine_stabilization.kinematics.ik_solver import G1HandIK
from g1_spine_stabilization.kinematics.collision import (
    build_head_obstacle,
    build_floor_obstacle,
    CollisionChecker,
)
from g1_spine_stabilization.kinematics.rrt_planner import (
    plan_collision_free_approach,
    RRTStarPlanner,
    StartInCollisionError,
    GoalInCollisionError,
    RRTBudgetExhaustedError,
)

# Test-local RRT* tuning: fast, deterministic (explicit seed), decoupled from
# whatever configs/default.yaml's collision_avoidance section is tuned to.
_TEST_CA_CFG = {
    "collision_avoidance": {
        "seed": 7,
        "max_iters": 2000,
        "time_budget_s": 3.0,
        "step_size_rad": 0.3,
        "goal_bias": 0.15,
        "segment_bias": 0.8,
        "segment_sigma_rad": 0.2,
        "rewire_radius_rad": 0.6,
        "goal_connect_radius_rad": 0.4,
        "edge_check_resolution_rad": 0.05,
        "shortcut_attempts": 150,
        "head_margin_m": 0.0,
    }
}


@pytest.fixture(scope="module")
def ik():
    return G1HandIK(gravity_comp_enabled=True)


def _obstructed_start_goal(ik):
    """A start/goal pair (left arm only) whose straight-line joint
    interpolation is deliberately obstructed by a small sphere placed at the
    Cartesian midpoint of their FK positions. Both endpoints individually
    have clearance from that sphere."""
    q_start = np.zeros(14)
    q_start[0] = -0.3
    q_start[1] = 0.5
    q_start[3] = -0.3

    q_goal = np.zeros(14)
    q_goal[0] = 0.5
    q_goal[1] = 1.2
    q_goal[3] = -1.2

    L0, _ = ik.fk(q_start)
    Lg, _ = ik.fk(q_goal)
    sphere_center = (L0.translation + Lg.translation) / 2.0
    radius = 0.05
    return q_start, q_goal, sphere_center, radius


def test_straight_line_precondition_actually_clips_the_obstacle(ik):
    """Sanity check on the test fixture itself: if this fails, the scenario
    below wouldn't actually be testing collision avoidance at all."""
    q_start, q_goal, sphere_center, radius = _obstructed_start_goal(ik)
    head = build_head_obstacle(sphere_center, radius, margin_m=0.0)
    checker = CollisionChecker(ik, [head])

    clips = any(
        not checker.is_valid_transit((1 - t) * q_start + t * q_goal)
        for t in np.linspace(0, 1, 40)
    )
    assert clips, "test fixture is broken: straight line does not clip the obstacle"


def test_finds_path_around_deliberately_placed_obstacle(ik):
    q_start, q_goal, sphere_center, radius = _obstructed_start_goal(ik)

    path = plan_collision_free_approach(ik, q_start, q_goal, sphere_center, radius, _TEST_CA_CFG)

    np.testing.assert_allclose(path[0], q_start)
    np.testing.assert_allclose(path[-1], q_goal)

    head = build_head_obstacle(sphere_center, radius, margin_m=0.0)
    checker = CollisionChecker(ik, [head])
    n = len(path)
    for i in range(n - 1):
        # Last segment is checked against the true (unpadded) sphere -- see
        # rrt_planner.py's goal-connection design; margin is 0.0 here so
        # padded == true anyway, but keep the check semantically correct.
        check_fn = checker.is_valid_contact if i == n - 2 else checker.is_valid_transit
        for t in np.linspace(0, 1, 25):
            q = (1 - t) * path[i] + t * path[i + 1]
            assert check_fn(q), f"path segment {i} (t={t:.2f}) collides with the obstacle"


def test_no_obstacle_direct_short_path(ik):
    q_start, q_goal, _, radius = _obstructed_start_goal(ik)
    far_away_center = np.array([50.0, 50.0, 50.0])  # nowhere near the robot

    path = plan_collision_free_approach(ik, q_start, q_goal, far_away_center, radius, _TEST_CA_CFG)

    np.testing.assert_allclose(path[0], q_start)
    np.testing.assert_allclose(path[-1], q_goal)
    # Fast path in RRTStarPlanner.plan() should return exactly [start, goal]
    # when the direct line is already clear -- no detour needed.
    assert len(path) == 2

    path_len = np.linalg.norm(path[-1] - path[0])
    straight_len = np.linalg.norm(q_goal - q_start)
    assert path_len == pytest.approx(straight_len)


def test_goal_in_collision_raises_goal_error(ik):
    q_start, q_goal, _, _ = _obstructed_start_goal(ik)
    Lg, _ = ik.fk(q_goal)
    # Obstacle centered exactly on the goal's own hand position -- q_goal
    # itself must be in collision against the true (unpadded) sphere.
    sphere_center = Lg.translation.copy()
    radius = 0.10

    with pytest.raises(GoalInCollisionError):
        plan_collision_free_approach(ik, q_start, q_goal, sphere_center, radius, _TEST_CA_CFG)


def test_goal_in_collision_error_names_the_colliding_link(ik):
    """The diagnostic added alongside the box->sphere swap: an opaque
    GoalInCollisionError isn't enough to tell a human watching a live run
    WHICH part of the arm intruded (e.g. a hand's oversized collision box vs.
    an elbow) -- the message should name the actual geometry."""
    q_start, q_goal, _, _ = _obstructed_start_goal(ik)
    Lg, _ = ik.fk(q_goal)
    sphere_center = Lg.translation.copy()
    radius = 0.10

    with pytest.raises(GoalInCollisionError) as excinfo:
        plan_collision_free_approach(ik, q_start, q_goal, sphere_center, radius, _TEST_CA_CFG)
    assert "link:" in str(excinfo.value)


def test_start_in_collision_raises_start_error(ik):
    q_start, q_goal, _, _ = _obstructed_start_goal(ik)
    L0, _ = ik.fk(q_start)
    sphere_center = L0.translation.copy()
    radius = 0.10
    # Non-trivial margin so q_start (which sits right at the sphere surface by
    # construction) is pushed into the padded "transit" obstacle specifically.
    cfg = {**_TEST_CA_CFG, "collision_avoidance": {**_TEST_CA_CFG["collision_avoidance"], "head_margin_m": 0.05}}

    with pytest.raises(StartInCollisionError):
        plan_collision_free_approach(ik, q_start, q_goal, sphere_center, radius, cfg)


def _low_q(shoulder_pitch=1.0384615384615388):
    """A joint config with a genuinely low left-hand Z (~-0.105m), found by
    sweeping shoulder_pitch -- verified manually before this session's
    floor-obstacle implementation. Everything else stays at zero."""
    q = np.zeros(14)
    q[0] = shoulder_pitch
    return q


def test_floor_obstacle_catches_a_genuinely_low_hand(ik):
    """Direct CollisionChecker test (no RRT/config involved) mirroring the
    manual verification done before implementing build_floor_obstacle: a
    low hand pose is correctly flagged by both tiers, a normal/resting
    pose is not. floor_z=0.02/margin=0.02 (padded floor at 0.04) is chosen
    so the WHOLE zero-config arm/hand chain clears it -- the lowest zero-
    config arm/hand geometry (a finger segment) sits at z~0.065, comfortably
    above 0.04; a naive higher floor_z would instead catch a finger/elbow
    at zero config too, which isn't what this test is checking."""
    floor = build_floor_obstacle(0.02, margin_m=0.02)
    checker = CollisionChecker(ik, [floor])

    q_low = _low_q()
    assert not checker.is_valid_contact(q_low)
    assert not checker.is_valid_transit(q_low)

    q_normal = np.zeros(14)
    assert checker.is_valid_contact(q_normal)
    assert checker.is_valid_transit(q_normal)


def test_floor_obstacle_names_the_colliding_link(ik):
    floor = build_floor_obstacle(0.02, margin_m=0.0)
    checker = CollisionChecker(ik, [floor])
    culprit = checker.first_colliding_geometry(_low_q(), checker._true_pair_ids)
    assert culprit is not None


def test_plan_collision_free_approach_raises_goal_error_for_floor_via_config(ik):
    """The config-driven path (floor_z_m read inside plan_collision_free_approach
    itself, not a direct CollisionChecker construction) must also raise --
    confirms the actual wiring, not just build_floor_obstacle in isolation.
    Head sphere placed far away so only the floor plays a role here."""
    q_start = np.zeros(14)
    q_goal = _low_q()
    far_away_center = np.array([50.0, 50.0, 50.0])
    cfg = {
        **_TEST_CA_CFG,
        "collision_avoidance": {**_TEST_CA_CFG["collision_avoidance"], "floor_z_m": 0.05},
    }

    with pytest.raises(GoalInCollisionError):
        plan_collision_free_approach(ik, q_start, q_goal, far_away_center, 0.01, cfg)


def test_plan_collision_free_approach_raises_start_error_for_floor_via_config(ik):
    q_start = _low_q()
    q_goal = np.zeros(14)
    far_away_center = np.array([50.0, 50.0, 50.0])
    cfg = {
        **_TEST_CA_CFG,
        "collision_avoidance": {**_TEST_CA_CFG["collision_avoidance"], "floor_z_m": 0.05},
    }

    with pytest.raises(StartInCollisionError):
        plan_collision_free_approach(ik, q_start, q_goal, far_away_center, 0.01, cfg)


def test_plan_collision_free_approach_unaffected_when_floor_z_m_absent(ik):
    """floor_z_m absent (not just None-but-present) from config must not
    change behavior at all -- the .get(..., None) default path."""
    q_start, q_goal, sphere_center, radius = _obstructed_start_goal(ik)
    # _TEST_CA_CFG has no floor_z_m key at all -- this should behave
    # identically to test_finds_path_around_deliberately_placed_obstacle.
    path = plan_collision_free_approach(ik, q_start, q_goal, sphere_center, radius, _TEST_CA_CFG)
    np.testing.assert_allclose(path[0], q_start)
    np.testing.assert_allclose(path[-1], q_goal)


def _stack_hitting_only_the_second_sphere(ik, q_goal, radius=0.05):
    """Build a 2-obstacle stack (primary + 1 stacked) where the PRIMARY
    sphere is placed far away (no collision), and the STACKED sphere is
    placed exactly at q_goal's hand position (guaranteed collision) --
    i.e. simulate the actual hand-under-head scenario: a config that
    clears the primary sphere but intrudes on the extended volume a
    stacked sphere represents. Built directly from two build_head_obstacle
    calls (not build_head_obstacle_stack's own spacing math) so the two
    centers can be placed independently and unambiguously far apart,
    regardless of stack_spacing_m's own overlap-tuning concerns (covered
    separately by test_head_obstacle.py)."""
    Lg, _ = ik.fk(q_goal)
    far_away_primary_center = Lg.translation.copy() + np.array([5.0, 5.0, 5.0])
    primary = build_head_obstacle(far_away_primary_center, radius, margin_m=0.0)
    stacked = build_head_obstacle(Lg.translation.copy(), radius, margin_m=0.0)
    return [primary, stacked]


def test_stacked_sphere_catches_a_config_that_clears_the_primary_sphere(ik):
    """Regression test for the actual hand-under-head scenario: a config
    that has full clearance from the PRIMARY sphere but intrudes on a
    STACKED sphere below it must still be reported as a collision --
    is_valid_transit/is_valid_contact must check EVERY obstacle in the
    stack, not just the primary one."""
    q_goal = np.zeros(14)
    q_goal[0] = 0.5
    q_goal[1] = 1.2
    q_goal[3] = -1.2

    stack_obstacles = _stack_hitting_only_the_second_sphere(ik, q_goal)

    # Sanity: q_goal is comfortably clear of the PRIMARY sphere alone.
    checker_primary_only = CollisionChecker(ik, [stack_obstacles[0]])
    assert checker_primary_only.is_valid_contact(q_goal), (
        "test fixture is broken: q_goal should be clear of the primary sphere alone"
    )

    checker_stack = CollisionChecker(ik, stack_obstacles)
    assert not checker_stack.is_valid_contact(q_goal), (
        "q_goal collides with the STACKED sphere but is_valid_contact did not catch it -- "
        "CollisionChecker must check every obstacle in the stack, not just the primary"
    )


def test_first_colliding_geometry_names_a_link_against_a_stacked_sphere(ik):
    """first_colliding_geometry's obstacle-membership check must generalize
    correctly to multiple obstacles -- confirms it doesn't only recognize
    the primary sphere's geometry ids."""
    q_goal = np.zeros(14)
    q_goal[0] = 0.5
    q_goal[1] = 1.2
    q_goal[3] = -1.2

    stack_obstacles = _stack_hitting_only_the_second_sphere(ik, q_goal)
    checker = CollisionChecker(ik, stack_obstacles)
    culprit = checker.first_colliding_geometry(q_goal, checker._true_pair_ids)
    assert culprit is not None, "expected a named colliding link against the stacked sphere"


def test_budget_exhausted_raises_distinct_error_not_silent_fallback(ik):
    """A near-impossible budget (1 iteration) on a genuinely obstructed
    scenario must raise RRTBudgetExhaustedError, not silently return
    anything -- callers rely on this to trigger their safety-abort path."""
    q_start, q_goal, sphere_center, radius = _obstructed_start_goal(ik)
    lower, upper = ik.effective_joint_limits()
    head = build_head_obstacle(sphere_center, radius, margin_m=0.0)
    checker = CollisionChecker(ik, [head])
    planner = RRTStarPlanner(lower, upper, checker, max_iters=1, time_budget_s=100.0, seed=7)

    with pytest.raises(RRTBudgetExhaustedError):
        planner.plan(q_start, q_goal)
