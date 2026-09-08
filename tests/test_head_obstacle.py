"""
Pure geometry tests for kinematics/collision.py's build_head_obstacle.
No G1HandIK needed -- fast.
"""

import numpy as np
import pytest

import hppfcl

from g1_spine_stabilization.kinematics.collision import (
    build_head_obstacle,
    build_head_obstacle_stack,
    build_floor_obstacle,
    head_sphere_center_from_marker,
)


def test_true_shape_matches_radius_exactly():
    head = build_head_obstacle([0.36, 0.0, 0.10], 0.10, margin_m=0.0)
    assert head.true_shape.radius == pytest.approx(0.10)


def test_padded_shape_adds_margin_to_radius():
    head = build_head_obstacle([0.36, 0.0, 0.10], 0.10, margin_m=0.04)
    assert head.padded_shape.radius == pytest.approx(0.14)


def test_padded_equals_true_when_margin_zero():
    head = build_head_obstacle([0.36, 0.0, 0.10], 0.10, margin_m=0.0)
    assert head.padded_shape.radius == pytest.approx(head.true_shape.radius)


def test_placement_translation_matches_center():
    center = np.array([0.42, -0.05, 0.12])
    head = build_head_obstacle(center, 0.10, margin_m=0.0)
    np.testing.assert_allclose(head.placement.translation, center)


def test_placement_rotation_is_identity():
    head = build_head_obstacle([0.36, 0.0, 0.10], 0.10, margin_m=0.04)
    np.testing.assert_allclose(head.placement.rotation, np.eye(3), atol=1e-12)


def test_sphere_center_from_marker_shifts_down_marker_up_axis():
    # Marker with identity rotation -- column 2 (its "out of surface" axis) is world Z.
    marker_pos = np.array([0.4, 0.0, 0.3])
    marker_R = np.eye(3)
    center = head_sphere_center_from_marker(marker_pos, marker_R, radius=0.08)
    np.testing.assert_allclose(center, [0.4, 0.0, 0.22])


def test_sphere_center_from_marker_uses_markers_own_tilted_axis():
    # Marker rotated 90 deg about Y: its local Z (column 2) now points along world +X,
    # so the "shift down the marker's own surface normal" moves along X, not world Z.
    marker_pos = np.array([0.5, 0.0, 0.3])
    marker_R = np.array([
        [0.0, 0.0, 1.0],
        [0.0, 1.0, 0.0],
        [-1.0, 0.0, 0.0],
    ])
    center = head_sphere_center_from_marker(marker_pos, marker_R, radius=0.1)
    np.testing.assert_allclose(center, [0.4, 0.0, 0.3], atol=1e-12)


def test_sphere_center_offset_none_matches_no_offset():
    marker_pos = np.array([0.4, 0.0, 0.3])
    marker_R = np.eye(3)
    center_default = head_sphere_center_from_marker(marker_pos, marker_R, radius=0.08)
    center_explicit_none = head_sphere_center_from_marker(marker_pos, marker_R, radius=0.08, offset=None)
    np.testing.assert_allclose(center_default, center_explicit_none)


def test_sphere_center_offset_zero_matches_no_offset():
    marker_pos = np.array([0.4, 0.0, 0.3])
    marker_R = np.eye(3)
    center_no_offset = head_sphere_center_from_marker(marker_pos, marker_R, radius=0.08)
    center_zero_offset = head_sphere_center_from_marker(
        marker_pos, marker_R, radius=0.08, offset=[0.0, 0.0, 0.0]
    )
    np.testing.assert_allclose(center_no_offset, center_zero_offset)


def test_sphere_center_offset_applied_in_world_axes_when_marker_identity():
    # Marker with identity rotation -- its local frame IS world frame, so the
    # offset should land exactly where it's written.
    marker_pos = np.array([0.4, 0.0, 0.3])
    marker_R = np.eye(3)
    center = head_sphere_center_from_marker(
        marker_pos, marker_R, radius=0.08, offset=[0.01, -0.02, 0.03]
    )
    # Base (no offset) center is [0.4, 0.0, 0.22] (see the no-offset test above).
    np.testing.assert_allclose(center, [0.41, -0.02, 0.25])


def test_sphere_center_offset_rotated_by_tilted_marker():
    # Same 90-deg-about-Y marker as the tilted-axis test above: its local X axis
    # (column 0) points along world -Z, local Y (column 1) is world Y, local Z
    # (column 2) is world +X. An offset written in marker-local axes must come
    # out rotated into world axes, not added raw.
    marker_pos = np.array([0.5, 0.0, 0.3])
    marker_R = np.array([
        [0.0, 0.0, 1.0],
        [0.0, 1.0, 0.0],
        [-1.0, 0.0, 0.0],
    ])
    # Base (no offset) center is [0.4, 0.0, 0.3] (see the tilted-axis test above).
    center = head_sphere_center_from_marker(
        marker_pos, marker_R, radius=0.1, offset=[0.02, 0.0, 0.0]
    )
    np.testing.assert_allclose(center, [0.4, 0.0, 0.3 - 0.02], atol=1e-12)


def test_sphere_center_anchor_distance_none_matches_radius_coupled_behavior():
    marker_pos = np.array([0.4, 0.0, 0.3])
    marker_R = np.eye(3)
    center_default = head_sphere_center_from_marker(marker_pos, marker_R, radius=0.08)
    center_explicit_none = head_sphere_center_from_marker(
        marker_pos, marker_R, radius=0.08, anchor_distance=None
    )
    np.testing.assert_allclose(center_default, center_explicit_none)


def test_sphere_center_anchor_distance_decouples_from_radius():
    # With identity rotation, anchor_distance directly sets the Z shift,
    # independent of whatever radius is passed.
    marker_pos = np.array([0.4, 0.0, 0.3])
    marker_R = np.eye(3)
    center = head_sphere_center_from_marker(
        marker_pos, marker_R, radius=0.05, anchor_distance=0.09
    )
    np.testing.assert_allclose(center, [0.4, 0.0, 0.21])


def test_sphere_center_unchanged_when_radius_varies_but_anchor_distance_fixed():
    # The whole point of decoupling: changing radius alone (anchor_distance
    # held fixed) must not move the center at all.
    marker_pos = np.array([0.4, 0.0, 0.3])
    marker_R = np.eye(3)
    center_small_radius = head_sphere_center_from_marker(
        marker_pos, marker_R, radius=0.05, anchor_distance=0.09
    )
    center_large_radius = head_sphere_center_from_marker(
        marker_pos, marker_R, radius=0.15, anchor_distance=0.09
    )
    np.testing.assert_allclose(center_small_radius, center_large_radius)


def test_sphere_center_anchor_distance_and_offset_combine():
    # anchor_distance replaces the radius-based depth; offset is still added
    # on top of that, both rotated into the tilted marker's frame.
    marker_pos = np.array([0.5, 0.0, 0.3])
    marker_R = np.array([
        [0.0, 0.0, 1.0],
        [0.0, 1.0, 0.0],
        [-1.0, 0.0, 0.0],
    ])
    center = head_sphere_center_from_marker(
        marker_pos, marker_R, radius=0.05, offset=[0.02, 0.0, 0.0], anchor_distance=0.1,
    )
    # anchor_distance=0.1 along local Z (-> world +X... here col2=[1,0,0], so
    # shift is -0.1*[1,0,0]); offset [0.02,0,0] along local X (col0=[0,0,-1])
    # adds -0.02 to world Z, matching the tilted-axis convention already
    # verified in the offset-only and anchor-only tests above.
    np.testing.assert_allclose(center, [0.5 - 0.1, 0.0, 0.3 - 0.02], atol=1e-12)


def test_stack_count_zero_matches_single_obstacle():
    """stack_count=0 (default) must reproduce today's single-sphere
    behavior exactly -- a list of exactly one HeadObstacle, identical to
    calling build_head_obstacle directly."""
    center = np.array([0.36, 0.0, 0.10])
    obstacles = build_head_obstacle_stack(center, 0.10, margin_m=0.02, stack_count=0)
    assert len(obstacles) == 1
    expected = build_head_obstacle(center, 0.10, margin_m=0.02)
    np.testing.assert_allclose(obstacles[0].placement.translation, expected.placement.translation)
    assert obstacles[0].true_shape.radius == pytest.approx(expected.true_shape.radius)
    assert obstacles[0].padded_shape.radius == pytest.approx(expected.padded_shape.radius)


def test_stack_count_two_returns_three_obstacles_at_expected_centers():
    """stack_count=2 returns [primary, +1*spacing, +2*spacing] along axis."""
    center = np.array([0.36, 0.0, 0.10])
    axis = np.array([0.0, 0.0, -1.0])
    spacing = 0.08
    obstacles = build_head_obstacle_stack(
        center, 0.10, margin_m=0.0, stack_count=2, stack_spacing_m=spacing, axis=axis,
    )
    assert len(obstacles) == 3
    np.testing.assert_allclose(obstacles[0].placement.translation, center)
    np.testing.assert_allclose(obstacles[1].placement.translation, center + spacing * axis)
    np.testing.assert_allclose(obstacles[2].placement.translation, center + 2 * spacing * axis)
    # Every obstacle shares the same radius as the primary.
    for obs in obstacles:
        assert obs.true_shape.radius == pytest.approx(0.10)


def test_stack_axis_is_normalized():
    """A non-unit axis vector is normalized before spacing is applied, so
    stack_spacing_m always means the real physical center-to-center
    distance regardless of the axis vector's own magnitude."""
    center = np.array([0.36, 0.0, 0.10])
    spacing = 0.08
    obstacles = build_head_obstacle_stack(
        center, 0.10, margin_m=0.0, stack_count=1, stack_spacing_m=spacing,
        axis=np.array([0.0, 0.0, -5.0]),  # same direction as [0,0,-1], 5x magnitude
    )
    np.testing.assert_allclose(obstacles[1].placement.translation, center + spacing * np.array([0.0, 0.0, -1.0]))


def test_stack_default_axis_is_world_negative_z():
    """axis=None (not provided) falls back to world -Z."""
    center = np.array([0.36, 0.0, 0.10])
    spacing = 0.08
    obstacles = build_head_obstacle_stack(
        center, 0.10, margin_m=0.0, stack_count=1, stack_spacing_m=spacing, axis=None,
    )
    np.testing.assert_allclose(obstacles[1].placement.translation, center + [0.0, 0.0, -spacing])


def test_stacked_spheres_overlap_when_spacing_less_than_two_radius():
    """Consecutive spheres must overlap (center distance < sum of radii) when
    stack_spacing_m < 2*radius, so no gap exists for an RRT edge to slip
    through between them -- the whole point of stacking."""
    center = np.array([0.36, 0.0, 0.10])
    radius = 0.10
    spacing = 0.15  # < 2*radius = 0.20
    obstacles = build_head_obstacle_stack(
        center, radius, margin_m=0.0, stack_count=1, stack_spacing_m=spacing,
        axis=np.array([0.0, 0.0, -1.0]),
    )
    dist = np.linalg.norm(obstacles[1].placement.translation - obstacles[0].placement.translation)
    assert dist < obstacles[0].true_shape.radius + obstacles[1].true_shape.radius


def test_floor_shapes_are_halfspaces():
    floor = build_floor_obstacle(0.05, margin_m=0.02)
    assert isinstance(floor.true_shape, hppfcl.Halfspace)
    assert isinstance(floor.padded_shape, hppfcl.Halfspace)


def test_floor_true_shape_d_equals_z_min():
    floor = build_floor_obstacle(0.05, margin_m=0.02)
    assert floor.true_shape.d == pytest.approx(0.05)


def test_floor_padded_shape_d_is_z_min_plus_margin():
    """Padded (transit) sits HIGHER than true (goal) -- opposite-feeling
    from the sphere case at first glance, but the same underlying
    convention (padded = more conservative): a floor has no goal
    exception, so raising the transit floor by margin_m is what makes it
    the more conservative of the two."""
    floor = build_floor_obstacle(0.05, margin_m=0.02)
    assert floor.padded_shape.d == pytest.approx(0.07)


def test_floor_margin_zero_matches_true_and_padded():
    floor = build_floor_obstacle(0.05, margin_m=0.0)
    assert floor.padded_shape.d == pytest.approx(floor.true_shape.d)


def test_floor_default_normal_is_world_z():
    floor = build_floor_obstacle(0.05)
    np.testing.assert_allclose(floor.true_shape.n, [0.0, 0.0, 1.0])


def test_floor_custom_normal_is_normalized():
    floor = build_floor_obstacle(0.05, normal=[0.0, 0.0, 5.0])
    np.testing.assert_allclose(floor.true_shape.n, [0.0, 0.0, 1.0])


def test_floor_placement_is_trivial():
    """The plane's location is carried entirely in each shape's own `d`,
    not in `placement` (unlike the sphere builders) -- placement should be
    identity/zero regardless of z_min."""
    floor = build_floor_obstacle(0.42)
    np.testing.assert_allclose(floor.placement.translation, np.zeros(3))
    np.testing.assert_allclose(floor.placement.rotation, np.eye(3))
