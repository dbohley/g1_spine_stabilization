"""Hardware-free unit tests for PoseNoiseFilter (marker pose noise filter).

Pure math -- no camera, no tracker, no robot. Mirrors
tests/test_noise_filtered_source.py's style (isolate the median stage with
ema_alpha=1.0, isolate the EMA stage with median_window=1).
"""
import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from g1_spine_stabilization.perception.pose_filter import PoseNoiseFilter


def _pose(pos, R):
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = pos
    return T


def _angle_from(R_a, R_b):
    """Rotation angle (radians) between two rotation matrices."""
    return Rotation.from_matrix(R_a.T @ R_b).magnitude()


def test_median_window_one_and_alpha_one_is_identity_passthrough():
    poses = [
        _pose([0.1, 0.2, 0.3], Rotation.from_euler("xyz", [0.1, 0.0, 0.0]).as_matrix()),
        _pose([0.4, -0.1, 0.0], Rotation.from_euler("xyz", [0.0, 0.2, 0.0]).as_matrix()),
        _pose([1.0, 1.0, 1.0], np.eye(3)),
    ]
    f = PoseNoiseFilter(median_window=1, ema_alpha=1.0)
    for p in poses:
        out = f.filter(p)
        np.testing.assert_allclose(out[:3, 3], p[:3, 3], atol=1e-9)
        np.testing.assert_allclose(out[:3, :3], p[:3, :3], atol=1e-6)


def test_median_rejects_single_frame_position_outlier():
    ref_pos = np.array([0.5, 0.0, 0.1])
    ref_R = np.eye(3)
    spike_pos = ref_pos + np.array([2.0, 0.0, 0.0])   # implausible 2m jump
    f = PoseNoiseFilter(median_window=5, ema_alpha=1.0)  # ema_alpha=1 isolates median

    outputs = []
    for pos in [ref_pos] * 5 + [spike_pos] + [ref_pos] * 4:
        outputs.append(f.filter(_pose(pos, ref_R))[:3, 3])

    spike_output = outputs[5]
    assert np.linalg.norm(spike_output - ref_pos) < 0.1, (
        f"median-of-5 should reject the 2m position outlier, got {spike_output}"
    )


def test_median_rejects_single_frame_rotation_outlier():
    ref_R = np.eye(3)
    spike_R = Rotation.from_euler("xyz", [np.pi / 2, 0.0, 0.0]).as_matrix()  # 90 deg outlier
    f = PoseNoiseFilter(median_window=5, ema_alpha=1.0)

    outputs = []
    for R in [ref_R] * 5 + [spike_R] + [ref_R] * 4:
        outputs.append(f.filter(_pose([0.0, 0.0, 0.0], R))[:3, :3])

    spike_output = outputs[5]
    assert _angle_from(ref_R, spike_output) < np.radians(10), (
        f"median-of-5 should reject the 90deg rotation outlier, got "
        f"{np.degrees(_angle_from(ref_R, spike_output)):.1f} deg from reference"
    )


def test_quaternion_hemisphere_continuity_across_sign_flip():
    """Two quaternion representations of the SAME physical rotation (q and
    -q) must not be treated as a huge jump -- without the hemisphere-
    continuity fix, feeding q then -q would corrupt the median/EMA with a
    near-antipodal pair representing identical physical orientations."""
    R1 = Rotation.from_euler("xyz", [0.3, -0.2, 0.5]).as_matrix()
    q1 = Rotation.from_matrix(R1).as_quat()
    R2 = Rotation.from_quat(-q1).as_matrix()
    # Sanity: -q1 really does represent the identical physical rotation.
    np.testing.assert_allclose(R1, R2, atol=1e-9)

    f = PoseNoiseFilter(median_window=3, ema_alpha=1.0)
    f.filter(_pose([0.0, 0.0, 0.0], R1))
    out2 = f.filter(_pose([0.0, 0.0, 0.0], R2))
    out3 = f.filter(_pose([0.0, 0.0, 0.0], R1))

    assert _angle_from(R1, out2[:3, :3]) < np.radians(1.0)
    assert _angle_from(R1, out3[:3, :3]) < np.radians(1.0)


def test_ema_reduces_variance_of_small_position_jitter():
    rng = np.random.default_rng(42)
    n = 200
    base = np.array([0.4, 0.0, 0.15])
    positions = [base + rng.normal(0.0, 0.01, size=3) for _ in range(n)]
    f = PoseNoiseFilter(median_window=1, ema_alpha=0.1)  # median_window=1 isolates EMA
    outputs = np.array([f.filter(_pose(p, np.eye(3)))[:3, 3] for p in positions])
    raw = np.array(positions)
    assert np.var(outputs) < np.var(raw)


def test_output_is_always_a_valid_rotation_matrix():
    rng = np.random.default_rng(7)
    f = PoseNoiseFilter(median_window=3, ema_alpha=0.4)
    for _ in range(20):
        rpy = rng.normal(0.0, 0.3, size=3)
        R = Rotation.from_euler("xyz", rpy).as_matrix()
        out = f.filter(_pose([0.0, 0.0, 0.0], R))
        R_out = out[:3, :3]
        np.testing.assert_allclose(R_out @ R_out.T, np.eye(3), atol=1e-6)
        assert np.linalg.det(R_out) == pytest.approx(1.0, abs=1e-6)


def test_constructor_rejects_even_or_nonpositive_window():
    with pytest.raises(ValueError):
        PoseNoiseFilter(median_window=4)
    with pytest.raises(ValueError):
        PoseNoiseFilter(median_window=0)
    with pytest.raises(ValueError):
        PoseNoiseFilter(median_window=-1)
