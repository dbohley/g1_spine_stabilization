"""Hardware-free tests for the SDK transport layer (dry_run=True).

These tests validate the G1InspireRobot lifecycle and the joint-target
hand-off without a physical robot or DDS network.
"""
import os
import pickle

import numpy as np
import pytest

from g1_spine_stabilization.controllers.sdk_pose_tracking import run_pose_tracking_sdk
from g1_spine_stabilization.data_recording.recorder import RunRecorder
from g1_spine_stabilization.kinematics.ik_solver import G1HandIK
from g1_spine_stabilization.trajectory.sources import SineSource
from g1_spine_stabilization.utils.config import load_config


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


@pytest.fixture(scope="module")
def ik():
    """Shared IK solver -- cold start (~10s) amortized across this file's tests."""
    return G1HandIK(gravity_comp_enabled=True)


@pytest.fixture
def cfg_fast():
    """Real config with the interpolation/settle durations shrunk for test speed."""
    cfg = load_config()
    cfg["control"]["interpolation_time"] = 0.06
    # Phase 1b (straight-in close) defaults to 1.5s; shrink it too so tests stay fast.
    cfg.setdefault("grasp_approach", {})["close_time"] = 0.06
    cfg.setdefault("shutdown", {})["settle_time"] = 0.05
    return cfg


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


def test_pose_tracking_sdk_settle_dry_run(dry_robot, ik, cfg_fast, capsys):
    """run_pose_tracking_sdk completes and the new settle pause doesn't hang/raise."""
    source = SineSource(
        box_center=np.array([0.35, 0.0, 0.1]),
        amplitude=cfg_fast["trajectory"]["sine"]["amplitude"],
        freq=cfg_fast["trajectory"]["sine"]["frequency"],
    )
    run_pose_tracking_sdk(dry_robot, ik, cfg_fast, source, duration=0.05)
    out = capsys.readouterr().out
    assert "Settling for" in out
    assert "Done." in out


def test_pose_tracking_sdk_phase1_starts_from_actual_current_pose(
    dry_robot, ik, cfg_fast, monkeypatch
):
    """Phase 1's first commanded EE target (alpha=0) must equal ik.fk(current_q)
    for whatever q get_state() reports, NOT the hardcoded zero-config
    L_EE_DEFAULT/R_EE_DEFAULT constants (regression test for the startup-jerk fix)."""
    from g1_spine_stabilization.controllers.sdk_pose_tracking import ARM_DEFAULT_Q
    from g1_spine_stabilization.trajectory.targets import L_EE_DEFAULT

    fake_q = ARM_DEFAULT_Q.copy()  # differs from zero-config FK by ~13mm (documented in CLAUDE.md)
    monkeypatch.setattr(dry_robot, "get_state", lambda: {"arm": {"q": fake_q}})
    L_expected, _ = ik.fk(fake_q)
    expected_L_pos = L_expected.translation.copy()

    calls = []
    original_solve_ik = ik.solve_ik

    def recording_solve_ik(L_target, R_target, *args, **kwargs):
        calls.append(L_target.copy())
        return original_solve_ik(L_target, R_target, *args, **kwargs)

    monkeypatch.setattr(ik, "solve_ik", recording_solve_ik)

    source = SineSource(
        box_center=np.array([0.35, 0.0, 0.1]),
        amplitude=cfg_fast["trajectory"]["sine"]["amplitude"],
        freq=cfg_fast["trajectory"]["sine"]["frequency"],
    )
    run_pose_tracking_sdk(dry_robot, ik, cfg_fast, source, duration=0.05)

    first_L_target_pos = calls[0][:3, 3]
    np.testing.assert_allclose(first_L_target_pos, expected_L_pos, atol=1e-9)
    assert not np.allclose(first_L_target_pos, L_EE_DEFAULT, atol=1e-4), (
        "Phase 1's first target should NOT be the hardcoded zero-config constant"
    )


def test_pose_tracking_sdk_phase1_keyboard_interrupt_skips_to_phase3(
    dry_robot, ik, cfg_fast, monkeypatch, capsys
):
    """KeyboardInterrupt during Phase 1 must not raise -- and Phase 3 must still run."""
    source = SineSource(
        box_center=np.array([0.35, 0.0, 0.1]),
        amplitude=cfg_fast["trajectory"]["sine"]["amplitude"],
        freq=cfg_fast["trajectory"]["sine"]["frequency"],
    )

    call_count = {"n": 0}
    original_solve_ik = ik.solve_ik

    def fault_solve_ik(*args, **kwargs):
        call_count["n"] += 1
        if call_count["n"] == 1:
            raise KeyboardInterrupt()
        return original_solve_ik(*args, **kwargs)

    monkeypatch.setattr(ik, "solve_ik", fault_solve_ik)

    # Should NOT raise -- Phase 1's KeyboardInterrupt is swallowed, matching
    # Phase 2's existing precedent, and execution falls through to Phase 3.
    run_pose_tracking_sdk(dry_robot, ik, cfg_fast, source, duration=0.05)

    out = capsys.readouterr().out
    assert "exiting Phase 1" in out
    assert "Phase 3" in out
    assert "Settling for" in out


def test_pose_tracking_sdk_phase3_keyboard_interrupt_still_settles(
    dry_robot, ik, cfg_fast, monkeypatch, capsys
):
    """KeyboardInterrupt during Phase 3's own return-motion loop must not
    raise out of run_pose_tracking_sdk -- and the settle pause before
    shutdown must still run (regression test for the same swallowed-interrupt
    gap fixed in solve_ik(), now also guarded with a try/except around
    Phase 3 for consistency with Phase 1/2)."""
    source = SineSource(
        box_center=np.array([0.35, 0.0, 0.1]),
        amplitude=cfg_fast["trajectory"]["sine"]["amplitude"],
        freq=cfg_fast["trajectory"]["sine"]["frequency"],
    )

    call_count = {"n": 0}
    original_solve_ik = ik.solve_ik

    dt = cfg_fast["control"]["dt"]
    phase1_calls = (int(cfg_fast["control"]["interpolation_time"] / dt)
                    + max(1, int(cfg_fast["grasp_approach"]["close_time"] / dt)))

    def fault_solve_ik(*args, **kwargs):
        call_count["n"] += 1
        # Phase 1 = Phase 1a (interpolation_time) + Phase 1b (close_time) solve_ik
        # calls; duration=0.0 makes Phase 2 make zero calls (its elapsed-time check
        # breaks the loop before the first solve_ik call), so the
        # (phase1_calls+1)-th call is deterministically the first call in Phase 3.
        if call_count["n"] == phase1_calls + 1:
            raise KeyboardInterrupt()
        return original_solve_ik(*args, **kwargs)

    monkeypatch.setattr(ik, "solve_ik", fault_solve_ik)

    # Should NOT raise -- Phase 3 catches KeyboardInterrupt and still settles.
    run_pose_tracking_sdk(dry_robot, ik, cfg_fast, source, duration=0.0)

    out = capsys.readouterr().out
    assert "Phase 3" in out
    assert "exiting Phase 3 early" in out
    assert "Settling for" in out
    assert "Done." in out


def _record_hand_commands(monkeypatch, robot):
    """Monkeypatch robot.hand.set_joint_targets to record every q_left kwarg."""
    recorded = []
    original = robot.hand.set_joint_targets

    def recording(*args, **kwargs):
        recorded.append(kwargs.get("q_left"))
        return original(*args, **kwargs)

    monkeypatch.setattr(robot.hand, "set_joint_targets", recording)
    return recorded


def test_pose_tracking_sdk_grips_and_releases_hands(dry_robot, ik, cfg_fast, monkeypatch):
    """Phase 2 must command a closed-ish grip (grip_force.enabled defaults
    True), and the hands must end up released (open) again before Phase 3 --
    the guaranteed-release counterpart to grasp_sequence.py's arm-return,
    now applied to the fingers during pose tracking."""
    from g1_spine_stabilization.controllers.grip_force import HAND_OPEN_POSE

    source = SineSource(
        box_center=np.array([0.35, 0.0, 0.1]),
        amplitude=cfg_fast["trajectory"]["sine"]["amplitude"],
        freq=cfg_fast["trajectory"]["sine"]["frequency"],
    )
    recorded = _record_hand_commands(monkeypatch, dry_robot)

    run_pose_tracking_sdk(dry_robot, ik, cfg_fast, source, duration=0.05)

    assert len(recorded) > 0
    assert any(not np.allclose(q, HAND_OPEN_POSE, atol=1e-3) for q in recorded), (
        "expected at least one closed-ish hand command while gripping"
    )
    np.testing.assert_allclose(recorded[-1], HAND_OPEN_POSE, atol=1e-3)


def test_pose_tracking_sdk_grip_disabled_skips_hand_commands(dry_robot, ik, cfg_fast, monkeypatch):
    """Arm-only behavior (zero hand commands anywhere in the run) requires BOTH
    grip_force AND palm_press disabled -- palm-press mode still closes the
    fingers to hold the object while it regulates force via the wrist press."""
    cfg_fast["grip_force"]["enabled"] = False
    cfg_fast.setdefault("palm_press", {})["enabled"] = False

    source = SineSource(
        box_center=np.array([0.35, 0.0, 0.1]),
        amplitude=cfg_fast["trajectory"]["sine"]["amplitude"],
        freq=cfg_fast["trajectory"]["sine"]["frequency"],
    )
    recorded = _record_hand_commands(monkeypatch, dry_robot)

    run_pose_tracking_sdk(dry_robot, ik, cfg_fast, source, duration=0.05)

    assert len(recorded) == 0


def test_pose_tracking_sdk_releases_hands_after_interrupt_during_tracking(
    dry_robot, ik, cfg_fast, monkeypatch
):
    """A KeyboardInterrupt landing inside Phase 2 must still release the
    grip before Phase 3 -- the guaranteed-release block runs regardless of
    how Phase 2 ended."""
    from g1_spine_stabilization.controllers.grip_force import HAND_OPEN_POSE

    source = SineSource(
        box_center=np.array([0.35, 0.0, 0.1]),
        amplitude=cfg_fast["trajectory"]["sine"]["amplitude"],
        freq=cfg_fast["trajectory"]["sine"]["frequency"],
    )

    call_count = {"n": 0}
    original_solve_ik = ik.solve_ik

    dt = cfg_fast["control"]["dt"]
    phase1_calls = (int(cfg_fast["control"]["interpolation_time"] / dt)
                    + max(1, int(cfg_fast["grasp_approach"]["close_time"] / dt)))

    def fault_solve_ik(*args, **kwargs):
        call_count["n"] += 1
        # Phase 1 = Phase 1a (interpolation_time) + Phase 1b (close_time) solve_ik
        # calls; Phase 1.5/the release block never call solve_ik, so the
        # (phase1_calls+1)-th call is the first call inside Phase 2's tracking loop.
        if call_count["n"] == phase1_calls + 1:
            raise KeyboardInterrupt()
        return original_solve_ik(*args, **kwargs)

    monkeypatch.setattr(ik, "solve_ik", fault_solve_ik)
    recorded = _record_hand_commands(monkeypatch, dry_robot)

    run_pose_tracking_sdk(dry_robot, ik, cfg_fast, source, duration=0.05)

    assert len(recorded) > 0
    np.testing.assert_allclose(recorded[-1], HAND_OPEN_POSE, atol=1e-3)


def test_pose_tracking_sdk_saves_recording_after_interrupt_during_tracking(
    dry_robot, ik, cfg_fast, monkeypatch, tmp_path
):
    """A KeyboardInterrupt landing inside Phase 2 must still save the
    recorder's accumulated samples -- recorder.save() runs in a `finally`
    wrapping Phase 2's existing try/except, so it runs regardless of how
    Phase 2 ended."""
    source = SineSource(
        box_center=np.array([0.35, 0.0, 0.1]),
        amplitude=cfg_fast["trajectory"]["sine"]["amplitude"],
        freq=cfg_fast["trajectory"]["sine"]["frequency"],
    )
    recorder = RunRecorder(
        origin="robot", source_label="sine",
        output_path=str(tmp_path / "run.pkl"),
    )

    call_count = {"n": 0}
    original_solve_ik = ik.solve_ik

    dt = cfg_fast["control"]["dt"]
    phase1_calls = (int(cfg_fast["control"]["interpolation_time"] / dt)
                    + max(1, int(cfg_fast["grasp_approach"]["close_time"] / dt)))

    def fault_solve_ik(*args, **kwargs):
        call_count["n"] += 1
        if call_count["n"] == phase1_calls + 1:   # first call inside Phase 2
            raise KeyboardInterrupt()
        return original_solve_ik(*args, **kwargs)

    monkeypatch.setattr(ik, "solve_ik", fault_solve_ik)

    run_pose_tracking_sdk(dry_robot, ik, cfg_fast, source, duration=0.05, recorder=recorder)

    assert os.path.exists(recorder.output_path)
    with open(recorder.output_path, "rb") as f:
        payload = pickle.load(f)
    assert payload["origin"] == "robot"
