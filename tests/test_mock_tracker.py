"""MockMarkerTracker API tests. No hardware, no OpenCV needed."""
import numpy as np

from g1_spine_stabilization.perception.tracker import MockMarkerTracker
from g1_spine_stabilization.perception.types import MarkerObservation


def _obs(marker_id, x=0.0, t=0.0):
    pose = np.eye(4); pose[:3, 3] = [x, 0, 0]
    return MarkerObservation(
        marker_id=marker_id, pose_in_pelvis=pose,
        timestamp=t, reprojection_error=0.5,
    )


def test_mock_tracker_returns_scripted_observations_in_order():
    obs0 = _obs(0, x=0.1, t=0.0)
    obs1 = _obs(1, x=0.2, t=0.1)
    tracker = MockMarkerTracker([obs0, obs1])
    assert tracker.step() == [obs0]
    assert tracker.step() == [obs1]
    assert tracker.step() == []


def test_mock_tracker_get_latest_returns_most_recent():
    tracker = MockMarkerTracker([_obs(0, x=0.1), _obs(0, x=0.2)])
    tracker.step()
    assert tracker.get_latest(0).pose_in_pelvis[0, 3] == 0.1
    tracker.step()
    assert tracker.get_latest(0).pose_in_pelvis[0, 3] == 0.2


def test_mock_tracker_get_latest_returns_none_for_unseen_id():
    tracker = MockMarkerTracker([_obs(0)])
    assert tracker.get_latest(99) is None
