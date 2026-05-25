"""Perception frame composition tests.
Math-only — no camera, no detector."""
import numpy as np
import pytest

from g1_spine_stabilization.perception.frames import (
    optical_to_d435_from_config,
    compose_to_pelvis,
)


def test_optical_to_d435_zero_rpy_is_identity_rotation():
    cfg = {"xyz": [0.0, 0.0, 0.0], "rpy": [0.0, 0.0, 0.0]}
    T = optical_to_d435_from_config(cfg)
    assert np.allclose(T, np.eye(4))


def test_optical_to_d435_translation_only():
    cfg = {"xyz": [0.1, 0.2, 0.3], "rpy": [0.0, 0.0, 0.0]}
    T = optical_to_d435_from_config(cfg)
    assert np.allclose(T[:3, 3], [0.1, 0.2, 0.3])
    assert np.allclose(T[:3, :3], np.eye(3))


def test_compose_to_pelvis_identity_chain():
    pose = np.eye(4); pose[:3, 3] = [1, 2, 3]
    result = compose_to_pelvis(pose, np.eye(4), np.eye(4))
    assert np.allclose(result, pose)


def test_compose_to_pelvis_order_matters():
    """Translation should compose left-to-right: T_d435_to_pelvis applied last."""
    pose = np.eye(4); pose[:3, 3] = [1, 0, 0]
    T_opt_to_d435 = np.eye(4); T_opt_to_d435[:3, 3] = [0, 1, 0]
    T_d435_to_pelvis = np.eye(4); T_d435_to_pelvis[:3, 3] = [0, 0, 1]
    result = compose_to_pelvis(pose, T_opt_to_d435, T_d435_to_pelvis)
    # All translations stack since all rotations are identity.
    assert np.allclose(result[:3, 3], [1, 1, 1])


@pytest.mark.skipif(
    pytest.importorskip("pinocchio", reason="pinocchio not installed") is None,
    reason="pinocchio not installed",
)
def test_d435_to_pelvis_returns_valid_se3():
    """Smoke test: the FK lookup returns a sensible 4x4 transform."""
    from g1_spine_stabilization.perception.frames import (
        d435_to_pelvis_from_pinocchio
    )
    T = d435_to_pelvis_from_pinocchio()
    assert T.shape == (4, 4)
    # Rotation should be orthonormal
    R = T[:3, :3]
    assert np.allclose(R @ R.T, np.eye(3), atol=1e-6)
    assert np.isclose(np.linalg.det(R), 1.0, atol=1e-6)
    # Translation should be in a reasonable range (head is above pelvis)
    assert T[2, 3] > 0.1  # d435 is above the pelvis
