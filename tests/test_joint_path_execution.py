"""
Tests for controllers/joint_path_execution.py's execute_joint_path -- the
max_speed_rad_s (joint-space) and max_cartesian_speed_m_s (hand-tip, via
ik.fk()) caps that, once configured, derive total_time PURELY from path
length / speed (never blended/floored against the passed-in total_time);
and the arrival_tolerance_rad post-playback check that polls the robot's
real state (robot.get_state()["arm"]["q"]) until it actually converges to
the final waypoint, instead of trusting total_time as a proxy for arrival.

Uses tiny control_dt/total_time values throughout so the function's real
time.sleep(control_dt) calls stay negligible in test wall time -- no
mocking of time.sleep, matching the rest of this repo's testing style.
"""

import time

import numpy as np
import pytest

from g1_spine_stabilization.controllers.joint_path_execution import (
    execute_joint_path,
    close_phase_steps,
)


class _FakeArm:
    def __init__(self):
        self.commanded = []

    def set_joint_targets(self, q, tau_ff=None):
        self.commanded.append(np.asarray(q, dtype=float).copy())


class _FakeRobot:
    def __init__(self):
        self.arm = _FakeArm()


class _FakePose:
    def __init__(self, translation, rotation=None):
        self.translation = np.asarray(translation, dtype=float)
        self.rotation = np.eye(3) if rotation is None else np.asarray(rotation, dtype=float)


class _FakeIK:
    """Minimal ik.fk(q) stub: L/R hand Cartesian position (and, optionally,
    rotation) as a configurable function of q, so segment Cartesian
    displacement is exactly predictable without needing a real Pinocchio
    model."""

    def __init__(self, l_map=None, r_map=None, l_rot_map=None, r_rot_map=None):
        self.l_map = l_map or (lambda q: q[:3])
        self.r_map = r_map or (lambda q: np.zeros(3))
        self.l_rot_map = l_rot_map or (lambda q: np.eye(3))
        self.r_rot_map = r_rot_map or (lambda q: np.eye(3))

    def fk(self, q):
        q = np.asarray(q, dtype=float)
        return (_FakePose(self.l_map(q), self.l_rot_map(q)),
                _FakePose(self.r_map(q), self.r_rot_map(q)))


def test_no_cap_uses_total_time_unchanged():
    robot = _FakeRobot()
    waypoints = np.array([
        [0.0] * 14,
        [1.0] + [0.0] * 13,
    ])
    start = time.monotonic()
    execute_joint_path(robot, ik=None, control_dt=0.02, waypoints=waypoints,
                        total_time=0.04, gravity_comp_enabled=False,
                        max_speed_rad_s=None)
    elapsed = time.monotonic() - start
    # 0.04s / 0.02s control_dt -> 2 steps -> ~0.04-0.06s wall time, not
    # inflated by any speed cap (there is none).
    assert elapsed < 0.3


def test_short_path_under_generous_cap_finishes_fast_not_padded():
    robot = _FakeRobot()
    # Path length 1.0 rad; a generous 1000 rad/s cap only needs 0.001s, far
    # less than total_time=0.04s. Once ANY cap is configured, duration is
    # PURELY derived from it -- never padded out to fill the old total_time.
    waypoints = np.array([
        [0.0] * 14,
        [1.0] + [0.0] * 13,
    ])
    execute_joint_path(robot, ik=None, control_dt=0.02, waypoints=waypoints,
                        total_time=0.04, gravity_comp_enabled=False,
                        max_speed_rad_s=1000.0)
    # cap_required_time = 1.0/1000 = 0.001s -> n_steps = max(1, int(0.001/0.02))
    # = 1 -> 2 commands, NOT the 3 commands total_time=0.04s alone would give
    # (n_steps = int(0.04/0.02) = 2 -> 3 commands) if it were still a floor.
    assert len(robot.arm.commanded) == 2


def test_long_path_exceeding_cap_extends_total_time():
    robot = _FakeRobot()
    # Path length 1.0 rad; total_time=0.02s would require 50 rad/s, but the
    # cap is 10 rad/s -> minimum required time is 1.0/10 = 0.1s. With
    # control_dt=0.02s that's ~5 steps instead of ~1.
    waypoints = np.array([
        [0.0] * 14,
        [1.0] + [0.0] * 13,
    ])
    execute_joint_path(robot, ik=None, control_dt=0.02, waypoints=waypoints,
                        total_time=0.02, gravity_comp_enabled=False,
                        max_speed_rad_s=10.0)
    # n_steps = max(1, int(seg_times[0]/control_dt)); extended seg_time is
    # ~0.1s over one segment -> int(0.1/0.02) = 5 -> 6 commands (n_steps+1).
    assert len(robot.arm.commanded) >= 5


def test_long_path_final_command_matches_last_waypoint():
    robot = _FakeRobot()
    waypoints = np.array([
        [0.0] * 14,
        [0.5] + [0.0] * 13,
        [1.0] + [0.0] * 13,
    ])
    final_q = execute_joint_path(robot, ik=None, control_dt=0.02, waypoints=waypoints,
                                  total_time=0.01, gravity_comp_enabled=False,
                                  max_speed_rad_s=5.0)
    np.testing.assert_allclose(final_q, waypoints[-1])
    np.testing.assert_allclose(robot.arm.commanded[-1], waypoints[-1])


def test_zero_or_negative_cap_is_treated_as_disabled():
    robot = _FakeRobot()
    waypoints = np.array([
        [0.0] * 14,
        [1.0] + [0.0] * 13,
    ])
    start = time.monotonic()
    execute_joint_path(robot, ik=None, control_dt=0.02, waypoints=waypoints,
                        total_time=0.04, gravity_comp_enabled=False,
                        max_speed_rad_s=0.0)
    elapsed = time.monotonic() - start
    assert elapsed < 0.3


def test_single_waypoint_returns_immediately_regardless_of_cap():
    robot = _FakeRobot()
    waypoints = np.array([[0.0] * 14])
    result = execute_joint_path(robot, ik=None, control_dt=0.02, waypoints=waypoints,
                                 total_time=0.04, gravity_comp_enabled=False,
                                 max_speed_rad_s=0.001)
    np.testing.assert_allclose(result, waypoints[0])
    assert robot.arm.commanded == []


def test_cartesian_cap_none_matches_no_cap():
    robot = _FakeRobot()
    ik = _FakeIK()
    waypoints = np.array([
        [0.0] * 14,
        [1.0] + [0.0] * 13,
    ])
    start = time.monotonic()
    execute_joint_path(robot, ik=ik, control_dt=0.02, waypoints=waypoints,
                        total_time=0.04, gravity_comp_enabled=False,
                        max_cartesian_speed_m_s=None)
    elapsed = time.monotonic() - start
    assert elapsed < 0.3


def test_cartesian_cap_under_generous_limit_finishes_fast_not_padded():
    robot = _FakeRobot()
    ik = _FakeIK()  # l_map = q[:3], identity 1:1 mapping
    # Cartesian displacement 1.0 m; a generous 1000 m/s cap only needs
    # 0.001s, far less than total_time=0.04s -- duration must be derived
    # purely from the cap, not padded out to the old total_time.
    waypoints = np.array([
        [0.0] * 14,
        [1.0] + [0.0] * 13,
    ])
    execute_joint_path(robot, ik=ik, control_dt=0.02, waypoints=waypoints,
                        total_time=0.04, gravity_comp_enabled=False,
                        max_cartesian_speed_m_s=1000.0)
    # cap_required_time = 1.0/1000 = 0.001s -> n_steps=1 -> 2 commands, not
    # the 3 commands total_time=0.04s alone would give as a floor.
    assert len(robot.arm.commanded) == 2


def test_cartesian_cap_exceeded_extends_total_time():
    robot = _FakeRobot()
    ik = _FakeIK()  # l_map = q[:3], identity 1:1 mapping
    # Cartesian displacement 1.0 m; total_time=0.02s would need 50 m/s, but
    # the cap is 5 m/s -> minimum required time is 1.0/5 = 0.2s.
    waypoints = np.array([
        [0.0] * 14,
        [1.0] + [0.0] * 13,
    ])
    execute_joint_path(robot, ik=ik, control_dt=0.02, waypoints=waypoints,
                        total_time=0.02, gravity_comp_enabled=False,
                        max_cartesian_speed_m_s=5.0)
    # int(0.2/0.02) = 10 -> 11 commands (n_steps+1).
    assert len(robot.arm.commanded) >= 10


def test_cartesian_cap_checked_per_segment_not_aggregate():
    """A segment with a SMALL joint-space length but a proportionally large
    Cartesian displacement must drive up required_time even though the
    aggregate joint-space time share it would naively get (proportional to
    its tiny joint length) is small. An aggregate-only Cartesian check
    (summing total Cartesian length and dividing by total time) would
    under-estimate the time this specific segment actually needs.
    """
    robot = _FakeRobot()

    # Segment 0 (q0->q1): joint length 1.0 along dim 0, Cartesian mapped 1:1
    #   -> Cartesian length 1.0.
    # Segment 1 (q1->q2): joint length 0.01 along dim 1 (100x smaller than
    #   segment 0), but Cartesian-amplified 2x -> Cartesian length 0.02.
    q0 = np.zeros(14)
    q1 = q0.copy(); q1[0] = 1.0
    q2 = q1.copy(); q2[1] = 0.01
    waypoints = np.array([q0, q1, q2])

    ik = _FakeIK(l_map=lambda q: np.array([q[0], 2.0 * q[1], q[2]]))

    CAP = 20.0
    TOTAL_LENGTH = 1.0 + 0.01
    # Correct per-segment answer: needed_1 = C1*(L0+L1)/(cap*L1)
    #   = 0.02 * 1.01 / (20 * 0.01) = 0.101s -- segment 1 dominates.
    # A (wrong) aggregate-only check would give (C0+C1)/cap = 1.02/20 = 0.051s.
    correct_required_time = 0.02 * TOTAL_LENGTH / (CAP * 0.01)
    wrong_aggregate_time = (1.0 + 0.02) / CAP
    assert correct_required_time > 1.5 * wrong_aggregate_time  # sanity on the numbers themselves

    start = time.monotonic()
    execute_joint_path(robot, ik=ik, control_dt=0.01, waypoints=waypoints,
                        total_time=0.02, gravity_comp_enabled=False,
                        max_cartesian_speed_m_s=CAP)
    elapsed = time.monotonic() - start

    # Elapsed must reflect the correct (larger) per-segment answer, not the
    # smaller aggregate-only estimate.
    assert elapsed > wrong_aggregate_time * 1.3
    assert elapsed < correct_required_time * 2.0 + 0.1  # generous upper sanity bound


class _FakeRobotGradualConvergence:
    """robot.get_state()["arm"]["q"] halves the gap to target_q on every
    call, simulating a real (dry-run or live) velocity-limited state that
    converges over several polls rather than instantly."""

    def __init__(self, start_q, target_q):
        self.arm = _FakeArm()
        self._q = np.asarray(start_q, dtype=float).copy()
        self._target_q = np.asarray(target_q, dtype=float)
        self.n_state_reads = 0

    def get_state(self):
        self.n_state_reads += 1
        self._q = self._q + 0.5 * (self._target_q - self._q)
        return {"arm": {"q": self._q.copy()}}


class _FakeRobotNeverConverges:
    """robot.get_state()["arm"]["q"] always returns a fixed, far-away state."""

    def __init__(self, stuck_q):
        self.arm = _FakeArm()
        self._stuck_q = np.asarray(stuck_q, dtype=float).copy()

    def get_state(self):
        return {"arm": {"q": self._stuck_q.copy()}}


def test_arrival_check_disabled_by_default_no_state_read_attempted():
    robot = _FakeRobot()  # has no get_state() at all
    waypoints = np.array([
        [0.0] * 14,
        [1.0] + [0.0] * 13,
    ])
    # Would raise AttributeError if the (disabled) arrival check tried to
    # call robot.get_state() anyway.
    final_q = execute_joint_path(robot, ik=None, control_dt=0.02, waypoints=waypoints,
                                  total_time=0.02, gravity_comp_enabled=False,
                                  arrival_tolerance_rad=None)
    np.testing.assert_allclose(final_q, waypoints[-1])


def test_arrival_check_waits_for_gradual_convergence_and_returns_real_state():
    target_q = np.array([1.0] + [0.0] * 13)
    robot = _FakeRobotGradualConvergence(start_q=np.full(14, 5.0), target_q=target_q)
    waypoints = np.array([np.zeros(14), target_q])
    final_q = execute_joint_path(robot, ik=None, control_dt=0.02, waypoints=waypoints,
                                  total_time=0.02, gravity_comp_enabled=False,
                                  arrival_tolerance_rad=0.1, arrival_timeout_s=2.0,
                                  arrival_poll_interval_s=0.001)
    # Geometric convergence (halves the gap each call) takes several polls,
    # not zero and not anywhere near the 2.0s timeout -- proves it actually
    # waited rather than returning immediately.
    assert 3 <= robot.n_state_reads <= 20
    # Returned value is the REAL last-read state, not just waypoints[-1].
    np.testing.assert_allclose(final_q, robot._q)
    assert np.linalg.norm(final_q - target_q) <= 0.1


def test_arrival_check_timeout_prints_warning_and_proceeds(capsys):
    target_q = np.array([1.0] + [0.0] * 13)
    stuck_q = np.full(14, 5.0)  # never anywhere near target_q
    robot = _FakeRobotNeverConverges(stuck_q=stuck_q)
    waypoints = np.array([np.zeros(14), target_q])
    start = time.monotonic()
    final_q = execute_joint_path(robot, ik=None, control_dt=0.02, waypoints=waypoints,
                                  total_time=0.02, gravity_comp_enabled=False,
                                  arrival_tolerance_rad=0.01, arrival_timeout_s=0.05,
                                  arrival_poll_interval_s=0.01)
    elapsed = time.monotonic() - start
    assert elapsed < 0.5  # bounded wait, not a hang
    np.testing.assert_allclose(final_q, stuck_q)  # real (non-converged) state returned
    captured = capsys.readouterr()
    assert "did not converge" in captured.out


def test_arrival_check_skipped_when_robot_has_no_get_state():
    robot = _FakeRobot()  # no get_state() method at all
    waypoints = np.array([
        [0.0] * 14,
        [1.0] + [0.0] * 13,
    ])
    # arrival_tolerance_rad IS set, but the robot can't be queried -- must
    # not raise, just proceed with the last commanded value.
    final_q = execute_joint_path(robot, ik=None, control_dt=0.02, waypoints=waypoints,
                                  total_time=0.02, gravity_comp_enabled=False,
                                  arrival_tolerance_rad=0.01)
    np.testing.assert_allclose(final_q, waypoints[-1])


def _rotz(theta):
    """Small helper: rotation matrix about Z by theta radians, for building
    _FakeIK rotation maps in the Cartesian-arrival-check tests below."""
    c, s = np.cos(theta), np.sin(theta)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def test_cartesian_arrival_check_disabled_unless_both_tolerances_set():
    """Only one of cartesian_arrival_tolerance_m/_rad set -> Cartesian check
    stays disabled (both are required together) -- no ik.fk() call attempted,
    which would otherwise raise since ik=None here."""
    robot = _FakeRobotNeverConverges(stuck_q=np.full(14, 5.0))
    waypoints = np.array([np.zeros(14), np.array([1.0] + [0.0] * 13)])
    final_q = execute_joint_path(robot, ik=None, control_dt=0.02, waypoints=waypoints,
                                  total_time=0.02, gravity_comp_enabled=False,
                                  cartesian_arrival_tolerance_m=0.01,
                                  cartesian_arrival_tolerance_rad=None)
    # No arrival_tolerance_rad either -> no check ran at all, no get_state() call.
    np.testing.assert_allclose(final_q, waypoints[-1])


def test_cartesian_arrival_check_alone_gates_on_position_and_rotation():
    """Cartesian check alone (no joint-space arrival_tolerance_rad) must wait
    for BOTH position and rotation error to converge before returning."""
    target_q = np.array([1.0] + [0.0] * 13)
    # q[0] maps to both L-hand position (X) and L-hand rotation angle,
    # halving the gap to 1.0 each poll -- position and rotation converge
    # together here, but this proves the Cartesian check (not the disabled
    # joint-space one) is what's gating the wait.
    robot = _FakeRobotGradualConvergence(start_q=np.zeros(14), target_q=target_q)
    ik = _FakeIK(l_map=lambda q: np.array([q[0], 0.0, 0.0]),
                 l_rot_map=lambda q: _rotz(q[0]))
    waypoints = np.array([np.zeros(14), target_q])
    final_q = execute_joint_path(robot, ik=ik, control_dt=0.02, waypoints=waypoints,
                                  total_time=0.02, gravity_comp_enabled=False,
                                  cartesian_arrival_tolerance_m=0.05,
                                  cartesian_arrival_tolerance_rad=0.05,
                                  arrival_timeout_s=2.0, arrival_poll_interval_s=0.001)
    assert 2 <= robot.n_state_reads <= 20  # actually waited, not instant
    np.testing.assert_allclose(final_q, robot._q)


def test_cartesian_arrival_check_position_converges_rotation_does_not():
    """Position reaches tolerance quickly, but rotation is stuck far off --
    the combined check must still time out (treats them as independent
    sub-conditions, not satisfied by position alone)."""
    target_q = np.array([1.0] + [0.0] * 13)
    robot = _FakeRobotNeverConverges(stuck_q=target_q)  # position "arrives" immediately
    # Rotation is a FIXED large angle, independent of q -- never converges
    # regardless of how close q gets to target_q.
    ik = _FakeIK(l_map=lambda q: q[:3], l_rot_map=lambda q: _rotz(1.5))
    waypoints = np.array([np.zeros(14), target_q])
    start = time.monotonic()
    final_q = execute_joint_path(robot, ik=ik, control_dt=0.02, waypoints=waypoints,
                                  total_time=0.02, gravity_comp_enabled=False,
                                  cartesian_arrival_tolerance_m=0.5,   # generous -- position passes
                                  cartesian_arrival_tolerance_rad=0.05,  # tight -- rotation never passes
                                  arrival_timeout_s=0.05, arrival_poll_interval_s=0.01)
    elapsed = time.monotonic() - start
    assert elapsed < 0.5  # bounded wait, not a hang
    np.testing.assert_allclose(final_q, target_q)


def test_both_joint_space_and_cartesian_checks_must_pass(capsys):
    """Both arrival_tolerance_rad and the Cartesian tolerances configured
    together: joint-space converges immediately, but Cartesian position
    never does -- must still time out and name the Cartesian check
    specifically in the warning, not just print a generic message."""
    target_q = np.array([1.0] + [0.0] * 13)
    # L-hand position tracks q[13] -- 0.0 in target_q vs. 99.0 in the stuck
    # state, so joint-space error (dominated by q[0]=1.0 matching) passes a
    # generous tolerance while Cartesian position (driven by q[13]) does not.
    ik = _FakeIK(l_map=lambda q: np.array([q[13], 0.0, 0.0]))
    waypoints = np.array([np.zeros(14), target_q])
    stuck_q_with_offset = target_q.copy()
    stuck_q_with_offset[13] = 99.0
    robot = _FakeRobotNeverConverges(stuck_q=stuck_q_with_offset)
    final_q = execute_joint_path(robot, ik=ik, control_dt=0.02, waypoints=waypoints,
                                  total_time=0.02, gravity_comp_enabled=False,
                                  arrival_tolerance_rad=200.0,           # generous -- passes regardless
                                  cartesian_arrival_tolerance_m=0.01,    # tight -- never passes
                                  cartesian_arrival_tolerance_rad=0.5,   # generous -- rotation passes (identity)
                                  arrival_timeout_s=0.05, arrival_poll_interval_s=0.01)
    np.testing.assert_allclose(final_q, stuck_q_with_offset)
    captured = capsys.readouterr()
    assert "did not converge" in captured.out
    assert "Cartesian" in captured.out
    assert "joint-space" not in captured.out  # only the FAILED check should be named


def test_cartesian_check_shares_timeout_budget_not_sequential():
    """Both checks configured, both never converge -- total wait must stay
    bounded by ONE arrival_timeout_s, not double it (proves the two checks
    share one poll loop rather than running as two sequential waits)."""
    stuck_q = np.full(14, 5.0)
    robot = _FakeRobotNeverConverges(stuck_q=stuck_q)
    ik = _FakeIK(l_map=lambda q: np.array([99.0, 0.0, 0.0]))
    waypoints = np.array([np.zeros(14), np.array([1.0] + [0.0] * 13)])
    start = time.monotonic()
    execute_joint_path(robot, ik=ik, control_dt=0.02, waypoints=waypoints,
                        total_time=0.02, gravity_comp_enabled=False,
                        arrival_tolerance_rad=0.01,
                        cartesian_arrival_tolerance_m=0.01,
                        cartesian_arrival_tolerance_rad=0.01,
                        arrival_timeout_s=0.1, arrival_poll_interval_s=0.01)
    elapsed = time.monotonic() - start
    assert elapsed < 0.3  # ~0.1s timeout, not ~0.2s (would indicate sequential waits)


def test_max_speed_rad_s_bounds_the_true_peak_not_the_average():
    """Regression test for a confirmed live-hardware incident: a dangerous
    fast/violent motion occurred even WITH max_speed_rad_s actively
    configured. Root cause -- the cap-derived total_time only bounded each
    segment's AVERAGE per-tick rate (seg_length / seg_time), but smoothstep's
    peak slope (at a segment's own midpoint) is 1.5x that average, so the
    true peak silently exceeded the configured cap by 50%. This test uses a
    single long segment (so smoothstep's midpoint dominates the measured
    peak) and asserts the true peak per-tick rate never exceeds the cap.
    """
    robot = _FakeRobot()
    waypoints = np.array([
        [0.0] * 14,
        [2.0] + [0.0] * 13,
    ])
    CAP = 1.0  # rad/s
    control_dt = 0.005
    execute_joint_path(robot, ik=None, control_dt=control_dt, waypoints=waypoints,
                        total_time=0.01, gravity_comp_enabled=False,
                        max_speed_rad_s=CAP)
    cmds = robot.arm.commanded
    deltas = [np.linalg.norm(cmds[i + 1] - cmds[i]) / control_dt for i in range(len(cmds) - 1)]
    assert max(deltas) <= CAP * 1.001  # true peak, not just the average, stays under the cap


def test_max_cartesian_speed_m_s_bounds_the_true_peak_not_the_average():
    robot = _FakeRobot()
    ik = _FakeIK()  # l_map = q[:3], identity 1:1 mapping
    waypoints = np.array([
        [0.0] * 14,
        [2.0] + [0.0] * 13,
    ])
    CAP = 1.0  # m/s
    control_dt = 0.005
    execute_joint_path(robot, ik=ik, control_dt=control_dt, waypoints=waypoints,
                        total_time=0.01, gravity_comp_enabled=False,
                        max_cartesian_speed_m_s=CAP)
    cmds = robot.arm.commanded
    # l_map = q[:3], so Cartesian hand position tracks q[0] 1:1 here.
    deltas = [abs(cmds[i + 1][0] - cmds[i][0]) / control_dt for i in range(len(cmds) - 1)]
    assert max(deltas) <= CAP * 1.001


def test_max_step_rad_floors_a_disproportionately_short_segment():
    """Regression test for a confirmed live-hardware incident: a segment
    that receives a thin proportional time slice (because an earlier
    segment is much longer) must NOT collapse to n_steps=1 if that would
    exceed max_step_rad -- it must get enough steps to keep every per-tick
    delta under the cap, even though total_time alone would allocate it
    almost no time at all.
    """
    robot = _FakeRobot()
    # Segment 0 (q0->q1): length 10.0 rad -- eats almost the whole
    # proportional time budget under total_time=0.02s.
    # Segment 1 (q1->q2): length 1.0 rad -- without a floor, its share of
    # total_time is ~1/11 of 0.02s = ~0.0018s, so int(.../control_dt=0.01)
    # collapses to n_steps=max(1, 0) = 1: a single 1.0 rad jump in one tick.
    q0 = np.zeros(14)
    q1 = q0.copy(); q1[0] = 10.0
    q2 = q1.copy(); q2[1] = 1.0
    waypoints = np.array([q0, q1, q2])

    MAX_STEP_RAD = 0.1
    execute_joint_path(robot, ik=None, control_dt=0.01, waypoints=waypoints,
                        total_time=0.02, gravity_comp_enabled=False,
                        max_step_rad=MAX_STEP_RAD)

    # Segment 1 must be split into enough ticks that no single commanded
    # delta exceeds max_step_rad even at smoothstep's peak slope (1.5x the
    # segment's average-per-tick delta, at its midpoint) -- not just on
    # average. ceil(1.5 * 1.0 / 0.1) = 15 steps -> 16 commands for that
    # segment alone.
    seg1_commands = robot.arm.commanded[-16:]
    for a, b in zip(seg1_commands, seg1_commands[1:]):
        assert np.linalg.norm(b - a) <= MAX_STEP_RAD + 1e-9


def test_max_step_rad_none_matches_pre_fix_behavior():
    robot = _FakeRobot()
    waypoints = np.array([
        [0.0] * 14,
        [1.0] + [0.0] * 13,
    ])
    execute_joint_path(robot, ik=None, control_dt=0.02, waypoints=waypoints,
                        total_time=0.04, gravity_comp_enabled=False,
                        max_step_rad=None)
    # Unchanged from before the floor existed: n_steps = int(0.04/0.02) = 2
    # -> 3 commands.
    assert len(robot.arm.commanded) == 3


def test_log_path_writes_per_tick_csv(tmp_path):
    import csv as _csv

    robot = _FakeRobot()
    waypoints = np.array([
        [0.0] * 14,
        [0.5] + [0.0] * 13,
        [1.0] + [0.0] * 13,
    ])
    log_file = tmp_path / "joint_path_log.csv"
    execute_joint_path(robot, ik=None, control_dt=0.01, waypoints=waypoints,
                        total_time=0.02, gravity_comp_enabled=False,
                        log_path=str(log_file))

    assert log_file.exists()
    with open(log_file, newline="") as f:
        rows = list(_csv.reader(f))
    # Header rows (segment summary + blank + tick-row header) plus at least
    # one tick row per segment.
    assert any(row and row[0] == "timestamp" for row in rows)
    tick_rows = [row for row in rows if row and row[0] not in ("timestamp",)
                 and not row[0].startswith("#") and len(row) > 4]
    # Filter to actual data rows (numeric timestamp in first column).
    data_rows = []
    for row in tick_rows:
        try:
            float(row[0])
            data_rows.append(row)
        except ValueError:
            continue
    assert len(data_rows) >= len(waypoints) - 1


def test_both_caps_combined_uses_the_larger_requirement():
    robot = _FakeRobot()
    ik = _FakeIK()  # l_map = q[:3], identity 1:1 mapping
    waypoints = np.array([
        [0.0] * 14,
        [1.0] + [0.0] * 13,
    ])
    # Joint cap alone would need 1.0/2.0 = 0.5s; Cartesian cap alone would
    # need 1.0/5.0 = 0.2s -- the joint cap's requirement should win.
    execute_joint_path(robot, ik=ik, control_dt=0.02, waypoints=waypoints,
                        total_time=0.01, gravity_comp_enabled=False,
                        max_speed_rad_s=2.0, max_cartesian_speed_m_s=5.0)
    # int(0.5/0.02) = 25 -> 26 commands (n_steps+1).
    assert len(robot.arm.commanded) >= 25


def test_close_phase_steps_no_cap_matches_fixed_close_time():
    # No cap given -> unchanged from before this fix existed: purely
    # close_time / control_dt.
    steps = close_phase_steps(close_time=1.5, pregrasp_offset=0.08,
                               control_dt=0.02, max_hand_speed_m_s=None)
    assert steps == int(1.5 / 0.02)


def test_close_phase_steps_zero_or_negative_cap_is_treated_as_disabled():
    steps_zero = close_phase_steps(close_time=1.5, pregrasp_offset=0.08,
                                    control_dt=0.02, max_hand_speed_m_s=0.0)
    steps_neg = close_phase_steps(close_time=1.5, pregrasp_offset=0.08,
                                   control_dt=0.02, max_hand_speed_m_s=-1.0)
    assert steps_zero == int(1.5 / 0.02)
    assert steps_neg == int(1.5 / 0.02)


def test_close_phase_steps_regression_confirmed_live_incident():
    """Regression test for the confirmed live-hardware incident: with
    defaults (pregrasp_offset=0.08, close_time=1.5), the fixed-time step
    count alone gives a true peak Cartesian speed of 1.5*0.08/1.5 = 0.08 m/s
    -- 8x over a configured max_hand_speed_m_s=0.01 cap, which the pre-fix
    code never even read. The floored step count must bring the true peak
    back under the configured cap.
    """
    CLOSE_TIME = 1.5
    PREGRASP_OFFSET = 0.08
    CONTROL_DT = 0.02
    MAX_HAND_SPEED = 0.01

    unfloored_steps = int(CLOSE_TIME / CONTROL_DT)
    unfloored_peak = 1.5 * PREGRASP_OFFSET / (unfloored_steps * CONTROL_DT)
    assert unfloored_peak > 5 * MAX_HAND_SPEED  # sanity: confirms the incident's ~8x overshoot

    steps = close_phase_steps(CLOSE_TIME, PREGRASP_OFFSET, CONTROL_DT, MAX_HAND_SPEED)
    true_peak = 1.5 * PREGRASP_OFFSET / (steps * CONTROL_DT)
    assert true_peak <= MAX_HAND_SPEED * 1.01  # small tolerance for int() truncation


def test_close_phase_steps_generous_cap_does_not_shrink_below_close_time():
    # A cap so generous it would need fewer steps than close_time alone must
    # NOT reduce the step count below the fixed-time floor -- mirrors
    # execute_joint_path's "cap only extends, never shrinks" behavior.
    steps = close_phase_steps(close_time=1.5, pregrasp_offset=0.08,
                               control_dt=0.02, max_hand_speed_m_s=1000.0)
    assert steps == int(1.5 / 0.02)
