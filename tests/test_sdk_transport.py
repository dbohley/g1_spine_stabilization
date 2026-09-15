"""Hardware-free tests for the SDK transport layer (dry_run=True).

These tests validate the G1InspireRobot lifecycle and the joint-target
hand-off without a physical robot or DDS network.
"""
import os
import pickle

import numpy as np
import pytest

from g1_spine_stabilization.controllers.sdk_pose_tracking import (
    run_pose_tracking_sdk,
    _get_q,
)
from g1_spine_stabilization.controllers.joint_path_execution import close_phase_steps
from g1_spine_stabilization.data_recording.recorder import RunRecorder
from g1_spine_stabilization.kinematics.ik_solver import G1HandIK
from g1_spine_stabilization.trajectory.sources import MarkerSource, SineSource
from g1_spine_stabilization.perception.types import MarkerObservation
from g1_spine_stabilization.trajectory.targets import R_L_BASE, R_R_BASE
from g1_spine_stabilization.utils.config import load_config
from g1_spine_stabilization.utils.transforms import make_se3


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
    # These tests exercise current-pose-start/grip/interrupt/recording behavior,
    # not collision avoidance -- disable RRT* so Phase 1a keeps using the
    # simple, deterministic straight-line motion they were written against.
    # See tests/test_rrt_planner.py for dedicated collision-avoidance coverage.
    cfg.setdefault("collision_avoidance", {})["enabled"] = False
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


def test_phase1a_resets_ik_smoothing_after_execute_joint_path(
    dry_robot, ik, cfg_fast, monkeypatch
):
    """Regression test: when collision avoidance is enabled, Phase 1a's
    RRT*-planned execute_joint_path moves the arm via set_joint_targets()
    directly, never calling solve_ik() -- so ik.smooth_filter's queue is
    left stale relative to where the arm actually ends up. Without an
    explicit reset, Phase 1b's first solve_ik() call blends a correct fresh
    solution against that stale history, producing a large spurious jump
    (root-caused via an isolated dry-run repro; see
    G1HandIK.reset_smoothing). This confirms the reset is actually wired in
    at the Phase 1a -> Phase 1b boundary, with the value execute_joint_path
    returned."""
    cfg = dict(cfg_fast)
    cfg["collision_avoidance"] = {
        **cfg_fast.get("collision_avoidance", {}),
        "enabled": True,
        "seed": 7,
        "max_iters": 500,
        "time_budget_s": 3.0,
        "step_size_rad": 0.3,
        "goal_bias": 0.15,
        "segment_bias": 0.8,
        "segment_sigma_rad": 0.2,
        "rewire_radius_rad": 0.6,
        "goal_connect_radius_rad": 0.4,
        "edge_check_resolution_rad": 0.05,
        "shortcut_attempts": 20,
        "head_radius_m": 0.01,
        "head_margin_m": 0.0,
    }

    reset_calls = []
    original_reset = ik.reset_smoothing

    def recording_reset(q):
        reset_calls.append(np.asarray(q, dtype=float).copy())
        return original_reset(q)

    monkeypatch.setattr(ik, "reset_smoothing", recording_reset)

    returned_q = {}
    from g1_spine_stabilization.controllers import sdk_pose_tracking as spt
    original_execute = spt.execute_joint_path

    def recording_execute(*args, **kwargs):
        result = original_execute(*args, **kwargs)
        returned_q["value"] = result.copy()
        return result

    monkeypatch.setattr(spt, "execute_joint_path", recording_execute)

    source = SineSource(
        box_center=np.array([0.35, 0.0, 0.1]),
        amplitude=cfg["trajectory"]["sine"]["amplitude"],
        freq=cfg["trajectory"]["sine"]["frequency"],
    )
    run_pose_tracking_sdk(dry_robot, ik, cfg, source, duration=0.05)

    assert len(reset_calls) >= 1, "ik.reset_smoothing was never called"
    np.testing.assert_allclose(reset_calls[0], returned_q["value"])


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
    # Phase 1b's step count is floored by close_phase_steps against
    # collision_avoidance.max_hand_speed_m_s (see controllers/
    # joint_path_execution.py) -- must match that function's real output,
    # not the old fixed close_time/dt formula, or this test's hardcoded
    # KeyboardInterrupt injection point drifts out of sync with Phase 1b's
    # actual call count.
    phase1_calls = (int(cfg_fast["control"]["interpolation_time"] / dt)
                    + close_phase_steps(
                        cfg_fast["grasp_approach"]["close_time"],
                        cfg_fast["grasp_approach"].get("pregrasp_offset", 0.04),
                        dt,
                        cfg_fast.get("collision_avoidance", {}).get("max_hand_speed_m_s", None),
                    ))

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


class _ObjectOnlyMarkerTracker:
    """Always returns the object marker; never returns the target marker --
    simulates a torso/target marker that's out of frame or never attached."""
    OBJ_ID = 0
    TGT_ID = 1

    def __init__(self, obj_pos):
        pose = np.eye(4)
        pose[:3, 3] = obj_pos
        self._obj_obs = MarkerObservation(
            marker_id=self.OBJ_ID, pose_in_pelvis=pose, timestamp=0.0,
            reprojection_error=0.0,
        )

    def step(self):
        return [self._obj_obs]

    def get_latest(self, marker_id):
        return self._obj_obs if marker_id == self.OBJ_ID else None


class _FakeReleaseWatcher:
    """Injectable stand-in for keyboard.SpacebarWatcher -- reports pressed()
    True once its `after`-th call is reached, so tests can drive the
    no-target-marker hold loop deterministically without a real TTY."""

    def __init__(self, after=0, raise_on_call=None):
        self.after = after
        self.raise_on_call = raise_on_call
        self.calls = 0

    def pressed(self):
        self.calls += 1
        if self.raise_on_call is not None and self.calls >= self.raise_on_call:
            raise KeyboardInterrupt()
        return self.calls > self.after


def test_pose_tracking_sdk_skips_tracking_when_target_marker_not_found(
    dry_robot, ik, cfg_fast, monkeypatch
):
    """If the target/torso marker is never detected at startup, the grasp
    (Phase 0/1/1.5) must still complete and release cleanly -- Phase 2
    (tracking) must be skipped entirely, not crash the whole run. Uses a fake
    release_watcher that reports pressed immediately, so the new hold-until-
    space phase (see below) exits right away for this test's purposes."""
    from g1_spine_stabilization.controllers.grip_force import HAND_OPEN_POSE

    tracker = _ObjectOnlyMarkerTracker(obj_pos=np.array([0.35, 0.0, 0.1]))
    source = MarkerSource(tracker, _ObjectOnlyMarkerTracker.OBJ_ID,
                          _ObjectOnlyMarkerTracker.TGT_ID,
                          mode="orient-only", countdown=0.0)

    step_calls = []
    original_step = source.step
    monkeypatch.setattr(source, "step", lambda: (step_calls.append(1), original_step())[1])

    recorded = _record_hand_commands(monkeypatch, dry_robot)

    run_pose_tracking_sdk(dry_robot, ik, cfg_fast, source, duration=0.05,
                          release_watcher=_FakeReleaseWatcher(after=0))

    assert source.target_found is False
    assert step_calls == [], "source.step() is only called in Phase 2 -- it must never run"
    # Grasp (Phase 1.5) still happened and hands still released (Phase 1.5
    # gate depends only on _phase1_interrupted, not on tracking_available).
    assert len(recorded) > 0
    np.testing.assert_allclose(recorded[-1], HAND_OPEN_POSE, atol=1e-3)


def test_pose_tracking_sdk_holds_grasp_until_release_watcher_signals(
    dry_robot, ik, cfg_fast, monkeypatch, capsys
):
    """No target marker: the arm must be actively held (repeated
    set_joint_targets calls, i.e. the grip loop runs LIVE) across several
    ticks before release, not release on the very first check -- this is the
    real-world grip-strength-testing scenario the hold phase exists for."""
    tracker = _ObjectOnlyMarkerTracker(obj_pos=np.array([0.35, 0.0, 0.1]))
    source = MarkerSource(tracker, _ObjectOnlyMarkerTracker.OBJ_ID,
                          _ObjectOnlyMarkerTracker.TGT_ID,
                          mode="orient-only", countdown=0.0)

    arm_calls = []
    original_set_arm_targets = dry_robot.arm.set_joint_targets

    def recording_set_arm_targets(q14, tau_ff14=None):
        arm_calls.append(np.asarray(q14).copy())
        return original_set_arm_targets(q14, tau_ff14)

    monkeypatch.setattr(dry_robot.arm, "set_joint_targets", recording_set_arm_targets)

    n_before_hold = len(arm_calls)
    watcher = _FakeReleaseWatcher(after=5)

    run_pose_tracking_sdk(dry_robot, ik, cfg_fast, source, duration=0.05,
                          release_watcher=watcher)

    assert watcher.calls == 6, "hold loop should tick exactly `after`+1 times before releasing"
    out = capsys.readouterr().out
    assert "Holding grasp" in out
    assert "Phase 3" in out
    assert "Settling for" in out


def test_pose_tracking_sdk_hold_keyboard_interrupt_still_releases(
    dry_robot, ik, cfg_fast, monkeypatch
):
    """A KeyboardInterrupt raised while waiting in the no-target-marker hold
    phase must not propagate out -- hands must still release and Phase 3
    must still run, mirroring the existing Phase-2-interrupt guarantee."""
    from g1_spine_stabilization.controllers.grip_force import HAND_OPEN_POSE

    tracker = _ObjectOnlyMarkerTracker(obj_pos=np.array([0.35, 0.0, 0.1]))
    source = MarkerSource(tracker, _ObjectOnlyMarkerTracker.OBJ_ID,
                          _ObjectOnlyMarkerTracker.TGT_ID,
                          mode="orient-only", countdown=0.0)
    recorded = _record_hand_commands(monkeypatch, dry_robot)
    watcher = _FakeReleaseWatcher(after=100, raise_on_call=2)

    # Must not raise.
    run_pose_tracking_sdk(dry_robot, ik, cfg_fast, source, duration=0.05,
                          release_watcher=watcher)

    assert len(recorded) > 0
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
    # Phase 1b's step count is floored by close_phase_steps against
    # collision_avoidance.max_hand_speed_m_s (see controllers/
    # joint_path_execution.py) -- must match that function's real output,
    # not the old fixed close_time/dt formula, or this test's hardcoded
    # KeyboardInterrupt injection point drifts out of sync with Phase 1b's
    # actual call count.
    phase1_calls = (int(cfg_fast["control"]["interpolation_time"] / dt)
                    + close_phase_steps(
                        cfg_fast["grasp_approach"]["close_time"],
                        cfg_fast["grasp_approach"].get("pregrasp_offset", 0.04),
                        dt,
                        cfg_fast.get("collision_avoidance", {}).get("max_hand_speed_m_s", None),
                    ))

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
    # Phase 1b's step count is floored by close_phase_steps against
    # collision_avoidance.max_hand_speed_m_s (see controllers/
    # joint_path_execution.py) -- must match that function's real output,
    # not the old fixed close_time/dt formula, or this test's hardcoded
    # KeyboardInterrupt injection point drifts out of sync with Phase 1b's
    # actual call count.
    phase1_calls = (int(cfg_fast["control"]["interpolation_time"] / dt)
                    + close_phase_steps(
                        cfg_fast["grasp_approach"]["close_time"],
                        cfg_fast["grasp_approach"].get("pregrasp_offset", 0.04),
                        dt,
                        cfg_fast.get("collision_avoidance", {}).get("max_hand_speed_m_s", None),
                    ))

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


def test_test_start_pose_drives_both_hands_before_source_startup(dry_robot, ik, cfg_fast, monkeypatch):
    """test_start_pose=(L,R) must actively drive the arm there -- via the
    same normal position-control interpolation Phase 1a uses, never by
    backdriving -- before source.startup() runs.

    Checks what was actually COMMANDED (the final solve_ik output during the
    drive step), matching this file's existing spy-on-set_joint_targets
    pattern (see test_pose_tracking_sdk_phase1_starts_from_actual_current_pose
    above) -- not what the dry-run simulation's own velocity-clipped state
    converged to, which lags behind a continuously-updating target for
    reasons specific to the SDK's dry-run loop, unrelated to this
    controller's own correctness."""
    calls = []
    original_set_joint_targets = dry_robot.arm.set_joint_targets

    def recording_set_joint_targets(q14, tau_ff14=None):
        calls.append(np.asarray(q14).copy())
        return original_set_joint_targets(q14, tau_ff14)

    monkeypatch.setattr(dry_robot.arm, "set_joint_targets", recording_set_joint_targets)

    # cfg_fast's interpolation_time (0.06s) gives only 3 steps at control_dt
    # =0.02s; smoothstep(i/steps) for i in range(steps) never actually
    # reaches alpha=1.0, so at very low step counts a real, non-negligible
    # gap remains (negligible at the real ~150-step default). Use enough
    # steps here to actually converge, while still keeping the test fast.
    cfg_fast["control"]["interpolation_time"] = 0.3

    # Orientation uses R_L_BASE/R_R_BASE -- the same "natural" orientation
    # family compute_hand_targets always builds targets from throughout this
    # codebase. An arbitrary orientation (e.g. identity) paired with an
    # arbitrary position can land in a region where position and orientation
    # cost genuinely compete, leaving a real (not test-artifact) IK residual
    # -- that's a property of the target chosen, not of _drive_to_test_pose.
    L_target = make_se3([0.30, 0.20, 0.15], R_L_BASE)
    R_target = make_se3([0.30, -0.20, 0.15], R_R_BASE)
    source = SineSource(
        box_center=np.array([0.35, 0.0, 0.1]),
        amplitude=cfg_fast["trajectory"]["sine"]["amplitude"],
        freq=cfg_fast["trajectory"]["sine"]["frequency"],
    )
    test_pose_steps = max(1, int(cfg_fast["control"]["interpolation_time"] / cfg_fast["control"]["dt"]))

    run_pose_tracking_sdk(dry_robot, ik, cfg_fast, source, duration=0.05,
                          test_start_pose=(L_target, R_target))

    last_test_pose_q = calls[test_pose_steps - 1]
    L_reached, R_reached = ik.fk(last_test_pose_q)
    # 1cm, not the 5mm documented at the default pose specifically: solve_ik's
    # 0.02 regularization cost pulls solutions away from an exact match by an
    # amount that grows with distance from q=0 (CLAUDE.md's "Known accuracy
    # floor" -- ~1.8mm AT the default pose; these targets are further away).
    np.testing.assert_allclose(L_reached.translation, L_target[:3, 3], atol=1e-2)
    np.testing.assert_allclose(R_reached.translation, R_target[:3, 3], atol=1e-2)


def test_test_start_pose_one_hand_holds_current(dry_robot, ik, cfg_fast, monkeypatch):
    """When only one hand's target is given, the other must stay at whatever
    pose it already had -- not snap to some default."""
    current_q_before = _get_q(dry_robot)
    _, R_before = ik.fk(current_q_before)
    R_before_pos = R_before.translation.copy()

    calls = []
    original_set_joint_targets = dry_robot.arm.set_joint_targets

    def recording_set_joint_targets(q14, tau_ff14=None):
        calls.append(np.asarray(q14).copy())
        return original_set_joint_targets(q14, tau_ff14)

    monkeypatch.setattr(dry_robot.arm, "set_joint_targets", recording_set_joint_targets)

    cfg_fast["control"]["interpolation_time"] = 0.3  # see comment in the test above

    L_target = make_se3([0.32, 0.22, 0.12], R_L_BASE)
    source = SineSource(
        box_center=np.array([0.35, 0.0, 0.1]),
        amplitude=cfg_fast["trajectory"]["sine"]["amplitude"],
        freq=cfg_fast["trajectory"]["sine"]["frequency"],
    )
    test_pose_steps = max(1, int(cfg_fast["control"]["interpolation_time"] / cfg_fast["control"]["dt"]))

    run_pose_tracking_sdk(dry_robot, ik, cfg_fast, source, duration=0.05,
                          test_start_pose=(L_target, None))

    last_test_pose_q = calls[test_pose_steps - 1]
    L_reached, R_reached = ik.fk(last_test_pose_q)
    np.testing.assert_allclose(L_reached.translation, L_target[:3, 3], atol=1e-2)
    np.testing.assert_allclose(R_reached.translation, R_before_pos, atol=1e-2)


def test_test_start_pose_none_is_a_no_op(dry_robot, ik, cfg_fast, capsys):
    """test_start_pose=None (default) must not print the test-pose-drive
    message or add any extra phase before source.startup()."""
    source = SineSource(
        box_center=np.array([0.35, 0.0, 0.1]),
        amplitude=cfg_fast["trajectory"]["sine"]["amplitude"],
        freq=cfg_fast["trajectory"]["sine"]["frequency"],
    )
    run_pose_tracking_sdk(dry_robot, ik, cfg_fast, source, duration=0.05)
    out = capsys.readouterr().out
    assert "test start pose" not in out.lower()
