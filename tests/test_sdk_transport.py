"""Hardware-free tests for the SDK transport layer (dry_run=True).

These tests validate the G1InspireRobot lifecycle and the joint-target
hand-off without a physical robot or DDS network.
"""
import numpy as np
import pytest


@pytest.fixture(scope="module")
def dry_robot():
    """Start a dry_run robot, yield it, then shut it down."""
    from g1_inspire_sdk import G1InspireRobot
    robot = G1InspireRobot(dry_run=True, arm_side="both", hand_side="both")
    robot.connect()
    robot.initialize()
    robot.start_streaming()
    yield robot
    robot.shutdown()


def test_robot_dry_run_lifecycle():
    """connect → initialize → start_streaming → shutdown completes without error."""
    from g1_inspire_sdk import G1InspireRobot
    robot = G1InspireRobot(dry_run=True, arm_side="both", hand_side="both")
    robot.connect()
    robot.initialize()
    robot.start_streaming()
    robot.shutdown()


def test_get_state_returns_q_arm_shape(dry_robot):
    """get_state() returns a dict with state['arm']['q'] of shape (14,)."""
    state = dry_robot.get_state()
    assert "arm" in state, f"get_state() missing 'arm' key; got keys: {list(state.keys())}"
    q = np.asarray(state["arm"]["q"])
    assert q.shape == (14,), f"Expected shape (14,), got {q.shape}"


def test_step_with_q_arm_does_not_raise(dry_robot):
    """step(q_arm=zeros(14)) does not raise after start_streaming."""
    dry_robot.step(q_arm=np.zeros(14))


def test_hand_commands_do_not_raise(dry_robot):
    """close_fist() and open() do not raise in dry_run mode."""
    dry_robot.hand.close_fist()
    dry_robot.hand.open()
