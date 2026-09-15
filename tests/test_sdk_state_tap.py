"""
Hardware-free tests for diagnostics/sdk_state_tap.py -- the read-only
polling tap into the vendored SDK's internal dry-run arm state.
"""

import time

import numpy as np
import pytest

from g1_spine_stabilization.diagnostics.run_log import RUN_LOG
from g1_spine_stabilization.diagnostics.sdk_state_tap import start_sdk_state_tap


@pytest.fixture(autouse=True)
def _reset_run_log():
    RUN_LOG.stop()
    yield
    RUN_LOG.stop()


class _FakeArm:
    def __init__(self):
        self._q_target = np.zeros(14)
        self._cmd_q_state = np.zeros(14)
        self._sim_q = np.zeros(14)
        self._arm_velocity_limit = 30.0


class _FakeRobot:
    def __init__(self, arm=None):
        self.arm = arm


def test_tap_degrades_gracefully_when_robot_has_no_arm():
    robot = _FakeRobot(arm=None)
    stop_event = start_sdk_state_tap(robot)
    assert stop_event.is_set()  # immediately disabled, not crashed


def test_tap_degrades_gracefully_when_arm_lacks_expected_attrs():
    class _BareArm:
        pass

    robot = _FakeRobot(arm=_BareArm())
    stop_event = start_sdk_state_tap(robot)
    assert stop_event.is_set()


def test_tap_logs_sdk_internal_ticks(tmp_path):
    path = tmp_path / "run.jsonl"
    RUN_LOG.start(str(path))
    robot = _FakeRobot(arm=_FakeArm())
    stop_event = start_sdk_state_tap(robot, interval_s=0.005)
    time.sleep(0.05)
    stop_event.set()
    time.sleep(0.02)  # let the thread observe the stop event
    RUN_LOG.stop()

    lines = [l for l in path.read_text().strip().split("\n") if l]
    assert len(lines) > 0
    import json
    record = json.loads(lines[0])
    assert record["source"] == "sdk_internal"
    assert record["event"] == "tick"
    assert record["q_target"] == [0.0] * 14
    assert record["vel_limit"] == 30.0
