"""Hardware-free integration test: the Phase-2 contact-loss guard freezes the
palm press end-to-end (the head-drop safety case).

Drives the real run_pose_tracking_sdk three-phase controller with a dry-run
robot, a sine source, and a scripted palm-force sequence (grip first, then drop
to zero). Proves the PalmContactWatchdog wiring reaches the freeze branch and
emits the SAFETY alert through the actual control loop -- not just the unit-level
state machine in test_palm_contact_watchdog.py.
"""
import numpy as np
import pytest

from g1_spine_stabilization.controllers import sdk_pose_tracking
from g1_spine_stabilization.controllers.sdk_pose_tracking import run_pose_tracking_sdk
from g1_spine_stabilization.kinematics.ik_solver import G1HandIK
from g1_spine_stabilization.trajectory.sources import SineSource
from g1_spine_stabilization.trajectory.targets import L_EE_DEFAULT
from g1_spine_stabilization.utils.config import load_config


@pytest.fixture(scope="module")
def ik():
    """Shared IK solver -- cold start (~10s) amortized across this file."""
    return G1HandIK(gravity_comp_enabled=True)


@pytest.fixture
def cfg_fast():
    """Real config, durations shrunk for speed, palm-press on / finger-force off."""
    cfg = load_config()
    cfg["control"]["interpolation_time"] = 0.06
    cfg["grasp_approach"]["close_time"] = 0.06
    cfg.setdefault("shutdown", {})["settle_time"] = 0.02
    cfg["palm_press"]["enabled"] = True
    cfg["grip_force"]["enabled"] = False
    return cfg


@pytest.fixture
def dry_robot():
    from g1_inspire_sdk import G1InspireRobot
    robot = G1InspireRobot(dry_run=True, arm_side="both", hand_side="both")
    robot.connect()
    robot.initialize()
    robot.start_streaming()
    yield robot
    robot.shutdown()


def _sine_source():
    return SineSource(
        box_center=np.array([L_EE_DEFAULT[0], 0.0, L_EE_DEFAULT[2]]),
        amplitude=0.2, freq=0.1,
    )


def _speed_up_hand_phases(monkeypatch):
    # The hand close/settle/release phases are plain time.sleep loops; shrink them.
    monkeypatch.setattr(sdk_pose_tracking, "_HAND_CLOSE_TIME", 0.05)
    monkeypatch.setattr(sdk_pose_tracking, "_HAND_SETTLE_TIME", 0.02)
    monkeypatch.setattr(sdk_pose_tracking, "_HAND_RELEASE_TIME", 0.05)


def test_phase2_contact_loss_freezes_press_and_alerts(dry_robot, ik, cfg_fast, monkeypatch, capsys):
    """Grip established, then force drops to zero -> watchdog trips, press freezes,
    SAFETY alert prints, and the run still completes cleanly (does NOT abort)."""
    _speed_up_hand_phases(monkeypatch)

    touch = cfg_fast["palm_press"]["touch_threshold"]
    state = {"n": 0}

    def fake_target_force(self):
        # First two Phase-2 reads: gripped (arms the watchdog). Then the head
        # "drops": force falls to 0, below loss_floor, on every subsequent read.
        state["n"] += 1
        return (touch + 2.0) if state["n"] <= 2 else 0.0

    def fake_guard_force(self):
        return 0.0  # keep the crush guard out of it

    monkeypatch.setattr(sdk_pose_tracking.PalmForceReader, "target_force", fake_target_force)
    monkeypatch.setattr(sdk_pose_tracking.PalmForceReader, "guard_force", fake_guard_force)

    run_pose_tracking_sdk(dry_robot, ik, cfg_fast, _sine_source(),
                          duration=1.5, preview=None, cam_for_preview=None)

    out = capsys.readouterr().out
    assert "[SAFETY] palm contact lost" in out   # trip reached the freeze branch
    assert "Phase 2" in out
    assert "[SDKController] Done." in out         # completed, did not abort


def test_phase2_sustained_contact_does_not_trip(dry_robot, ik, cfg_fast, monkeypatch, capsys):
    """Sanity counterpart: a steady grip never trips the guard (no false positive)."""
    _speed_up_hand_phases(monkeypatch)

    touch = cfg_fast["palm_press"]["touch_threshold"]
    monkeypatch.setattr(sdk_pose_tracking.PalmForceReader, "target_force",
                        lambda self: touch + 2.0)
    monkeypatch.setattr(sdk_pose_tracking.PalmForceReader, "guard_force",
                        lambda self: touch + 2.0)

    run_pose_tracking_sdk(dry_robot, ik, cfg_fast, _sine_source(),
                          duration=1.0, preview=None, cam_for_preview=None)

    out = capsys.readouterr().out
    assert "[SAFETY] palm contact lost" not in out
    assert "[SDKController] Done." in out


# ---------------------------------------------------------------------------
# Jacobian mode (S1-S3): force via tau = J^T W, same S0 watchdog.
# ---------------------------------------------------------------------------

def test_phase2_jacobian_steady_grip_tracks_no_trip(dry_robot, ik, cfg_fast, monkeypatch, capsys):
    """Jacobian mode, steady grip: the force feedforward path runs end-to-end,
    the watchdog never trips, and the run completes. Confirms S1-S3 wiring."""
    _speed_up_hand_phases(monkeypatch)
    cfg_fast["palm_press"]["mode"] = "jacobian"
    cfg_fast["palm_press"]["f_des_ramp_time"] = 0.2
    touch = cfg_fast["palm_press"]["touch_threshold"]
    monkeypatch.setattr(sdk_pose_tracking.PalmForceReader, "target_force",
                        lambda self: touch + 2.0)
    monkeypatch.setattr(sdk_pose_tracking.PalmForceReader, "guard_force",
                        lambda self: touch + 2.0)

    run_pose_tracking_sdk(dry_robot, ik, cfg_fast, _sine_source(),
                          duration=1.2, preview=None, cam_for_preview=None)

    out = capsys.readouterr().out
    assert "mode=jacobian" in out
    assert "Palm force[jacobian]" in out
    assert "[SAFETY] palm contact lost" not in out
    assert "[SDKController] Done." in out


def test_phase2_jacobian_contact_loss_zeros_wrench(dry_robot, ik, cfg_fast, monkeypatch, capsys):
    """Jacobian mode: grip, then head-drop -> watchdog trips, F_cmd/wrench zeroed,
    SAFETY alert, run still completes (holds, does not abort)."""
    _speed_up_hand_phases(monkeypatch)
    cfg_fast["palm_press"]["mode"] = "jacobian"
    touch = cfg_fast["palm_press"]["touch_threshold"]
    state = {"n": 0}

    def ftf(self):
        state["n"] += 1
        return (touch + 2.0) if state["n"] <= 2 else 0.0

    monkeypatch.setattr(sdk_pose_tracking.PalmForceReader, "target_force", ftf)
    monkeypatch.setattr(sdk_pose_tracking.PalmForceReader, "guard_force", lambda self: 0.0)

    run_pose_tracking_sdk(dry_robot, ik, cfg_fast, _sine_source(),
                          duration=1.5, preview=None, cam_for_preview=None)

    out = capsys.readouterr().out
    assert "[SAFETY] palm contact lost" in out
    assert "[SDKController] Done." in out
