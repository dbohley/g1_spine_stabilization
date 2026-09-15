"""Unit tests for the `squeeze` (palm-press depth) parameter of compute_hand_targets.

The palm-force press loop presses the two palms together by shrinking the
hand-to-hand separation via `squeeze`. These tests pin the geometry:
symmetric inward motion, separation reduced by 2*squeeze, backward
compatibility at squeeze=0, and clamping when squeeze exceeds the half-width.
"""

import numpy as np
from scipy.spatial.transform import Rotation

from g1_spine_stabilization.trajectory.targets import (
    BOX_CENTER_DEFAULT,
    BOX_HALF_WIDTH_DEFAULT,
    compute_hand_targets,
    apply_palm_tilt,
)


def test_squeeze_zero_matches_no_squeeze():
    """squeeze=0.0 must reproduce the original geometry byte-for-byte."""
    box_center = np.array([0.35, 0.0, 0.10])
    for theta in (0.0, 0.3, -0.7):
        L0, R0 = compute_hand_targets(box_center, 0.11, theta)
        L1, R1 = compute_hand_targets(box_center, 0.11, theta, squeeze=0.0)
        np.testing.assert_array_equal(L0, L1)
        np.testing.assert_array_equal(R0, R1)


def test_squeeze_reduces_separation_by_twice_s():
    """Hand-to-hand distance shrinks by exactly 2*squeeze (both palms move in)."""
    box_center = np.array([0.35, 0.0, 0.10])
    hw = 0.11
    s = 0.02
    L0, R0 = compute_hand_targets(box_center, hw, 0.0)
    Ls, Rs = compute_hand_targets(box_center, hw, 0.0, squeeze=s)
    sep0 = np.linalg.norm(L0[:3, 3] - R0[:3, 3])
    seps = np.linalg.norm(Ls[:3, 3] - Rs[:3, 3])
    np.testing.assert_allclose(sep0 - seps, 2 * s, atol=1e-9)


def test_squeeze_is_symmetric_about_box_center():
    """Both palms move inward by `s`; midpoint stays at box_center."""
    box_center = np.array([0.35, 0.02, 0.10])
    hw = 0.11
    s = 0.015
    Ls, Rs = compute_hand_targets(box_center, hw, 0.0, squeeze=s)
    mid = 0.5 * (Ls[:3, 3] + Rs[:3, 3])
    np.testing.assert_allclose(mid, box_center, atol=1e-9)
    # each palm is (hw - s) from the center along +/- Y at theta=0
    np.testing.assert_allclose(Ls[:3, 3][1] - box_center[1], hw - s, atol=1e-9)
    np.testing.assert_allclose(box_center[1] - Rs[:3, 3][1], hw - s, atol=1e-9)


def test_squeeze_rotates_with_theta():
    """The inward direction follows the object rotation about X."""
    box_center = np.array([0.35, 0.0, 0.10])
    hw = 0.11
    s = 0.02
    theta = 0.5
    Ls, Rs = compute_hand_targets(box_center, hw, theta, squeeze=s)
    # effective half-width along the rotated axis
    Rx = np.array([[1, 0, 0],
                   [0, np.cos(theta), -np.sin(theta)],
                   [0, np.sin(theta),  np.cos(theta)]])
    expected_L = box_center + Rx @ np.array([0.0, hw - s, 0.0])
    expected_R = box_center + Rx @ np.array([0.0, -(hw - s), 0.0])
    np.testing.assert_allclose(Ls[:3, 3], expected_L, atol=1e-9)
    np.testing.assert_allclose(Rs[:3, 3], expected_R, atol=1e-9)


def test_squeeze_clamps_at_zero_half_width():
    """squeeze larger than the half-width clamps both palms to the center."""
    box_center = np.array([0.35, 0.0, 0.10])
    hw = 0.05
    Ls, Rs = compute_hand_targets(box_center, hw, 0.0, squeeze=0.5)  # >> hw
    np.testing.assert_allclose(Ls[:3, 3], box_center, atol=1e-9)
    np.testing.assert_allclose(Rs[:3, 3], box_center, atol=1e-9)


def test_squeeze_leaves_orientation_untouched():
    """Only positions change with squeeze; orientations are unaffected."""
    box_center = np.array([0.35, 0.0, 0.10])
    L0, R0 = compute_hand_targets(box_center, BOX_HALF_WIDTH_DEFAULT, 0.4)
    Ls, Rs = compute_hand_targets(box_center, BOX_HALF_WIDTH_DEFAULT, 0.4, squeeze=0.02)
    np.testing.assert_array_equal(L0[:3, :3], Ls[:3, :3])
    np.testing.assert_array_equal(R0[:3, :3], Rs[:3, :3])


def test_negative_squeeze_widens_for_pregrasp_standoff():
    """A negative squeeze (the Phase 1a pre-grasp standoff) makes the hands stand
    off WIDER than the grasp pose, by 2*offset."""
    box_center = np.array([0.35, 0.0, 0.10])
    hw = 0.055
    offset = 0.04
    Lg, Rg = compute_hand_targets(box_center, hw, 0.0)                    # grasp
    Lp, Rp = compute_hand_targets(box_center, hw, 0.0, squeeze=-offset)   # standoff
    sep_grasp = np.linalg.norm(Lg[:3, 3] - Rg[:3, 3])
    sep_pre = np.linalg.norm(Lp[:3, 3] - Rp[:3, 3])
    assert sep_pre > sep_grasp
    np.testing.assert_allclose(sep_pre - sep_grasp, 2 * offset, atol=1e-9)


def test_straight_in_close_moves_only_along_inter_palm_axis():
    """Phase 1b close: sweeping squeeze from -offset to 0 at theta=0 moves each
    palm PURELY along Y (X and Z stay constant) -- the 'one direction' guarantee."""
    box_center = np.array([0.35, 0.0, 0.10])
    hw = 0.055
    offset = 0.04
    L_positions = []
    R_positions = []
    for alpha in np.linspace(0.0, 1.0, 11):
        squeeze = (1 - alpha) * (-offset)  # -offset -> 0, matching Phase 1b
        L, R = compute_hand_targets(box_center, hw, 0.0, squeeze=squeeze)
        L_positions.append(L[:3, 3])
        R_positions.append(R[:3, 3])
    L_positions = np.array(L_positions)
    R_positions = np.array(R_positions)
    # X and Z constant across the whole close for both palms
    np.testing.assert_allclose(L_positions[:, 0], L_positions[0, 0], atol=1e-12)
    np.testing.assert_allclose(L_positions[:, 2], L_positions[0, 2], atol=1e-12)
    np.testing.assert_allclose(R_positions[:, 0], R_positions[0, 0], atol=1e-12)
    np.testing.assert_allclose(R_positions[:, 2], R_positions[0, 2], atol=1e-12)
    # Y moves monotonically inward (left palm decreasing, right palm increasing)
    assert np.all(np.diff(L_positions[:, 1]) < 0)
    assert np.all(np.diff(R_positions[:, 1]) > 0)


def test_grasp_frame_r_none_matches_original_behavior():
    """grasp_frame_R=None (default) must reproduce today's pelvis-Y
    geometry byte-for-byte -- regression guard for every existing caller
    that doesn't pass it."""
    box_center = np.array([0.35, 0.02, 0.10])
    hw = 0.11
    for theta in (0.0, 0.3, -0.7):
        for squeeze in (0.0, 0.02, -0.04):
            L0, R0 = compute_hand_targets(box_center, hw, theta, squeeze=squeeze)
            L1, R1 = compute_hand_targets(box_center, hw, theta, squeeze=squeeze,
                                          grasp_frame_R=None)
            np.testing.assert_array_equal(L0, L1)
            np.testing.assert_array_equal(R0, R1)


def test_grasp_frame_r_identity_matches_none():
    """grasp_frame_R=np.eye(3) is a true no-op, identical to leaving it
    unset -- identity is not treated specially/differently from None."""
    box_center = np.array([0.35, 0.0, 0.10])
    hw = 0.11
    L0, R0 = compute_hand_targets(box_center, hw, 0.3, squeeze=0.02)
    L1, R1 = compute_hand_targets(box_center, hw, 0.3, squeeze=0.02,
                                  grasp_frame_R=np.eye(3))
    np.testing.assert_allclose(L0, L1, atol=1e-12)
    np.testing.assert_allclose(R0, R1, atol=1e-12)


def test_grasp_frame_r_rotates_position_and_orientation_together():
    """A non-trivial grasp_frame_R must move BOTH the arc axis (position)
    AND the base orientations (R_L_BASE/R_R_BASE) consistently -- the main
    risk this parameter's design guards against is orientation lagging
    behind position (palms arriving at the right spot but still facing the
    old pelvis-relative direction instead of the object's own)."""
    from g1_spine_stabilization.trajectory.targets import R_L_BASE, R_R_BASE

    box_center = np.array([0.35, 0.0, 0.10])
    hw = 0.11
    # 90 degree rotation about pelvis Z: object's local Y (column 1) now
    # points along world -X instead of world +Y.
    c, s = np.cos(np.pi / 2), np.sin(np.pi / 2)
    G = np.array([
        [c, -s, 0.0],
        [s,  c, 0.0],
        [0.0, 0.0, 1.0],
    ])
    L, R = compute_hand_targets(box_center, hw, 0.0, grasp_frame_R=G)

    axis = G[:, 1]
    expected_L_pos = box_center + hw * axis
    expected_R_pos = box_center - hw * axis
    np.testing.assert_allclose(L[:3, 3], expected_L_pos, atol=1e-9)
    np.testing.assert_allclose(R[:3, 3], expected_R_pos, atol=1e-9)

    # Orientation must be re-based through G too (at theta=0, R == identity).
    np.testing.assert_allclose(L[:3, :3], G @ R_L_BASE, atol=1e-12)
    np.testing.assert_allclose(R[:3, :3], G @ R_R_BASE, atol=1e-12)


# ---------------------------------------------------------------------
# apply_palm_tilt -- local-frame palm orientation offset
# ---------------------------------------------------------------------

def test_apply_palm_tilt_zero_is_exact_noop():
    """All-zero tilts (the default) must reproduce the input byte-for-byte --
    today's exact behavior for every existing caller that doesn't pass tilts."""
    box_center = np.array([0.35, 0.0, 0.10])
    L0, R0 = compute_hand_targets(box_center, 0.11, 0.3, squeeze=0.02)
    L1, R1 = apply_palm_tilt(L0, R0)
    np.testing.assert_array_equal(L0, L1)
    np.testing.assert_array_equal(R0, R1)


def test_apply_palm_tilt_rotates_only_the_given_hand():
    """A tilt on one hand must not touch the other hand's rotation, and must
    never touch either hand's position."""
    box_center = np.array([0.35, 0.0, 0.10])
    L0, R0 = compute_hand_targets(box_center, 0.11, 0.0)
    L1, R1 = apply_palm_tilt(L0, R0, tilt_left_deg=(5.0, 0.0, 0.0))
    np.testing.assert_array_equal(L0[:3, 3], L1[:3, 3])
    np.testing.assert_array_equal(R0[:3, 3], R1[:3, 3])
    np.testing.assert_array_equal(R0[:3, :3], R1[:3, :3])
    assert not np.allclose(L0[:3, :3], L1[:3, :3])


def test_apply_palm_tilt_composes_as_local_frame_right_multiply():
    """The tilt must compose as R_original @ tilt_matrix for each hand
    (a body-frame rotation appended as the rightmost factor), even on top of
    an already-nonzero theta/squeeze -- the property that makes this safe to
    bolt on without touching compute_hand_targets itself."""
    box_center = np.array([0.35, 0.0, 0.10])
    L0, R0 = compute_hand_targets(box_center, 0.11, 0.4, squeeze=0.01)
    tilt_left = (3.0, -2.0, 7.0)
    tilt_right = (-4.0, 1.5, 0.0)
    L1, R1 = apply_palm_tilt(L0, R0, tilt_left_deg=tilt_left, tilt_right_deg=tilt_right)
    L_tilt_mat = Rotation.from_euler("XYZ", tilt_left, degrees=True).as_matrix()
    R_tilt_mat = Rotation.from_euler("XYZ", tilt_right, degrees=True).as_matrix()
    np.testing.assert_allclose(L1[:3, :3], L0[:3, :3] @ L_tilt_mat, atol=1e-12)
    np.testing.assert_allclose(R1[:3, :3], R0[:3, :3] @ R_tilt_mat, atol=1e-12)


def test_apply_palm_tilt_roll_preserves_local_x_axis():
    """Rolling about the hand's own local X (the inter-palm press/squeeze
    axis, column 0) must leave that same axis direction unchanged -- it IS
    the rotation axis, so the press direction is never disturbed by a roll
    tilt regardless of magnitude or sign."""
    box_center = np.array([0.35, 0.0, 0.10])
    L0, R0 = compute_hand_targets(box_center, 0.11, 0.0)
    L1, R1 = apply_palm_tilt(L0, R0, tilt_left_deg=(15.0, 0.0, 0.0),
                             tilt_right_deg=(-15.0, 0.0, 0.0))
    np.testing.assert_allclose(L1[:3, 0], L0[:3, 0], atol=1e-9)
    np.testing.assert_allclose(R1[:3, 0], R0[:3, 0], atol=1e-9)
