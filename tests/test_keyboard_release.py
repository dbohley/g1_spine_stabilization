"""Hardware/TTY-free tests for utils/keyboard.py's non-blocking spacebar
detection, used by the no-target-marker hold phase in sdk_pose_tracking.py.
"""
import pytest

from g1_spine_stabilization.utils.keyboard import SpacebarWatcher, _contains_space


def test_contains_space_true_for_space_byte():
    assert _contains_space(b' ') is True
    assert _contains_space(b'a b') is True


def test_contains_space_false_without_space():
    assert _contains_space(b'') is False
    assert _contains_space(b'abc\n') is False


def test_watcher_degrades_safely_without_a_tty(capsys):
    """pytest's captured stdin isn't a real TTY -- pressed() must never crash
    or hang, and must simply always report False (Ctrl-C is then the only
    way to end a hold), with one warning printed."""
    with SpacebarWatcher() as watcher:
        assert watcher.pressed() is False
        assert watcher.pressed() is False  # repeated calls stay safe
    out = capsys.readouterr().out
    assert "not a terminal" in out


def test_watcher_exit_is_a_no_op_without_a_tty():
    """__exit__ must not raise even though __enter__ never touched termios."""
    watcher = SpacebarWatcher()
    watcher.__enter__()
    watcher.__exit__(None, None, None)  # must not raise
