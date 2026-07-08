"""Hardware-free tests for the guaranteed safe-return-before-shutdown behavior.

Exercises g1_spine_stabilization.controllers.grasp_sequence.run_grasp_phases
with dry_run=True and injected faults, verifying that hand-release (Phase D)
and arm-return-to-default (Phase R) always run even when Phase 0/A/B/C raises
(including KeyboardInterrupt) -- the concrete gap that used to let
scripts/grasp_demo.py skip straight to the SDK's fixed shutdown-damping
command from an actively-loaded grasp pose.
"""
import numpy as np
import pytest

from g1_spine_stabilization.controllers.grasp_sequence import run_grasp_phases
from g1_spine_stabilization.controllers.grip_force import GraspController
from g1_spine_stabilization.kinematics.ik_solver import G1HandIK
from g1_spine_stabilization.utils.config import load_config


@pytest.fixture(scope="module")
def ik():
    """Shared IK solver -- cold start (~10s) amortized across this file's tests."""
    return G1HandIK(gravity_comp_enabled=True)


@pytest.fixture
def cfg_fast():
    """Real config with the interpolation/settle durations shrunk for test speed."""
    cfg = load_config()
    cfg["control"]["interpolation_time"] = 0.06  # a few IK steps, not 3s
    cfg.setdefault("shutdown", {})["settle_time"] = 0.05
    return cfg


@pytest.fixture
def dry_robot():
    """Fresh dry_run robot per test: connect -> initialize -> start_streaming -> shutdown."""
    from g1_inspire_sdk import G1InspireRobot
    robot = G1InspireRobot(dry_run=True, arm_side="both", hand_side="both")
    robot.connect()
    robot.initialize()
    robot.start_streaming()
    yield robot
    robot.shutdown()


def _controllers(cfg_fast):
    ctrl_l = GraspController(cfg_fast["grip_force"])
    ctrl_r = GraspController(cfg_fast["grip_force"])
    return ctrl_l, ctrl_r


def test_grasp_phases_completes_normally_dry_run(dry_robot, ik, cfg_fast, capsys):
    """Baseline: full 0/A/B/C/D/R sequence completes without error in dry-run."""
    ctrl_l, ctrl_r = _controllers(cfg_fast)
    run_grasp_phases(
        dry_robot, ik, cfg_fast,
        ctrl_l=ctrl_l, ctrl_r=ctrl_r, do_left=True, do_right=True,
        hold_time=0.05, rate=50.0, preview_enabled=False, dry_run=True,
    )
    out = capsys.readouterr().out
    assert "Phase: release" in out
    assert "Phase: arm return to default" in out
    assert "Settling for" in out


def test_grasp_phases_recovers_from_exception_during_hold(dry_robot, ik, cfg_fast, monkeypatch, capsys):
    """A fault inside the force-feedback hold (Phase C) must not skip release/return."""
    ctrl_l, ctrl_r = _controllers(cfg_fast)

    def fault_step(*args, **kwargs):
        raise RuntimeError("simulated force-feedback fault")

    monkeypatch.setattr(ctrl_l, "step", fault_step)

    with pytest.raises(RuntimeError, match="simulated force-feedback fault"):
        run_grasp_phases(
            dry_robot, ik, cfg_fast,
            ctrl_l=ctrl_l, ctrl_r=ctrl_r, do_left=True, do_right=True,
            hold_time=1.0, rate=50.0, preview_enabled=False, dry_run=True,
        )

    out = capsys.readouterr().out
    assert "WARNING" in out
    assert "Phase: release" in out
    assert "Phase: arm return to default" in out
    assert "Settling for" in out


def test_grasp_phases_recovers_from_keyboard_interrupt_during_phase0(dry_robot, ik, cfg_fast, monkeypatch, capsys):
    """A KeyboardInterrupt before grasp_arm_q is ever set must still release/return
    safely -- exercises the `hold_arm_q = current_q` fallback (grasp_arm_q is None)."""
    ctrl_l, ctrl_r = _controllers(cfg_fast)

    call_count = {"n": 0}
    original_solve_ik = ik.solve_ik

    def fault_solve_ik(*args, **kwargs):
        call_count["n"] += 1
        if call_count["n"] == 1:
            raise KeyboardInterrupt()
        return original_solve_ik(*args, **kwargs)

    monkeypatch.setattr(ik, "solve_ik", fault_solve_ik)

    with pytest.raises(KeyboardInterrupt):
        run_grasp_phases(
            dry_robot, ik, cfg_fast,
            ctrl_l=ctrl_l, ctrl_r=ctrl_r, do_left=True, do_right=True,
            hold_time=0.05, rate=50.0, preview_enabled=False, dry_run=True,
        )

    out = capsys.readouterr().out
    assert "WARNING" in out
    assert "Phase: release" in out
    assert "Phase: arm return to default" in out
    assert "Settling for" in out


def test_grasp_phases_settles_after_interrupt_during_phase_r(dry_robot, ik, cfg_fast, monkeypatch, capsys):
    """A KeyboardInterrupt during Phase R's own return-motion loop must still
    reach the settle-before-shutdown pause -- previously the loop's exception
    jumped straight to the outer except, skipping the settle sleep entirely
    (see docs/followups.md)."""
    ctrl_l, ctrl_r = _controllers(cfg_fast)

    n_phase0_steps = int(cfg_fast["control"]["interpolation_time"] / cfg_fast["control"]["dt"])
    call_count = {"n": 0}
    original_solve_ik = ik.solve_ik

    def fault_solve_ik(*args, **kwargs):
        call_count["n"] += 1
        if call_count["n"] == n_phase0_steps + 1:  # first call inside Phase R
            raise KeyboardInterrupt()
        return original_solve_ik(*args, **kwargs)

    monkeypatch.setattr(ik, "solve_ik", fault_solve_ik)

    with pytest.raises(KeyboardInterrupt):
        run_grasp_phases(
            dry_robot, ik, cfg_fast,
            ctrl_l=ctrl_l, ctrl_r=ctrl_r, do_left=True, do_right=True,
            hold_time=0.05, rate=50.0, preview_enabled=False, dry_run=True,
        )

    out = capsys.readouterr().out
    assert "Phase: arm return to default" in out
    assert "Settling for" in out


def test_settle_time_config_key():
    """configs/default.yaml must define a positive shutdown.settle_time."""
    cfg = load_config()
    settle_time = cfg.get("shutdown", {}).get("settle_time")
    assert settle_time is not None
    assert isinstance(settle_time, (int, float))
    assert settle_time > 0
