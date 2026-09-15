"""
Hardware-free tests for diagnostics/run_log.py -- the single global,
wall-clock-anchored JSONL logger built to root-cause a smooth-then-violent
motion bug by capturing real per-tick data instead of theorizing about a
mechanism (see run_log.py's module docstring).
"""

import json

import pytest

from g1_spine_stabilization.diagnostics.run_log import RUN_LOG


@pytest.fixture(autouse=True)
def _reset_run_log():
    """Ensure RUN_LOG starts and ends each test closed, regardless of
    whether the test itself calls stop() -- tests share the RUN_LOG
    singleton across the whole test session."""
    RUN_LOG.stop()
    yield
    RUN_LOG.stop()


def test_log_before_start_is_a_noop(tmp_path):
    # No start() call at all -- must not raise, must not create a file.
    RUN_LOG.log("phase1b", "tick", i=0)
    assert not RUN_LOG.is_active


def test_start_log_stop_writes_valid_jsonl(tmp_path):
    path = tmp_path / "run.jsonl"
    RUN_LOG.start(str(path))
    assert RUN_LOG.is_active
    RUN_LOG.log("phase1b", "tick", i=0, alpha=0.5, sol_q=[0.0] * 14)
    RUN_LOG.log("phase1a", "plan", n_waypoints=3, fast_path=False)
    RUN_LOG.stop()
    assert not RUN_LOG.is_active

    lines = path.read_text().strip().split("\n")
    assert len(lines) == 2
    record0 = json.loads(lines[0])
    record1 = json.loads(lines[1])

    for record in (record0, record1):
        assert "t_wall" in record
        assert "t_mono" in record
        assert isinstance(record["t_wall"], float)
        assert isinstance(record["t_mono"], float)

    assert record0["source"] == "phase1b"
    assert record0["event"] == "tick"
    assert record0["i"] == 0
    assert record0["alpha"] == 0.5
    assert record0["sol_q"] == [0.0] * 14

    assert record1["source"] == "phase1a"
    assert record1["event"] == "plan"
    assert record1["n_waypoints"] == 3
    assert record1["fast_path"] is False


def test_log_after_stop_is_a_noop(tmp_path):
    path = tmp_path / "run.jsonl"
    RUN_LOG.start(str(path))
    RUN_LOG.stop()
    # Should not raise, should not reopen the file.
    RUN_LOG.log("phase3", "tick", i=1)
    assert not RUN_LOG.is_active


def test_numpy_array_fields_are_serialized_via_tolist(tmp_path):
    import numpy as np

    path = tmp_path / "run.jsonl"
    RUN_LOG.start(str(path))
    RUN_LOG.log("sdk_internal", "tick", q_target=np.zeros(14), sim_q=None)
    RUN_LOG.stop()

    record = json.loads(path.read_text().strip())
    assert record["q_target"] == [0.0] * 14
    assert record["sim_q"] is None


def test_restart_closes_previous_file(tmp_path):
    path1 = tmp_path / "run1.jsonl"
    path2 = tmp_path / "run2.jsonl"
    RUN_LOG.start(str(path1))
    RUN_LOG.log("phase1a", "plan", n_waypoints=2)
    RUN_LOG.start(str(path2))  # implicitly closes path1
    RUN_LOG.log("phase1b", "tick", i=0)
    RUN_LOG.stop()

    assert json.loads(path1.read_text().strip())["source"] == "phase1a"
    assert json.loads(path2.read_text().strip())["source"] == "phase1b"
