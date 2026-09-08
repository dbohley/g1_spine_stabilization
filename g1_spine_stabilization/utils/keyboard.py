"""Non-blocking single-keypress detection (Linux only).

Lets a human end a held grasp by pressing space without blocking the control
loop that keeps grip/force regulation (and its safety watchdogs) running
during the hold -- a blocking input() would freeze exactly that loop for the
whole hold. See controllers/sdk_pose_tracking.py's no-target-marker hold
phase, which is the sole consumer of this module.
"""
import os
import select
import sys


def _contains_space(data: bytes) -> bool:
    """True if any byte in data is a space character. Pure/testable without a TTY."""
    return b' ' in data


class SpacebarWatcher:
    """Context manager: puts stdin into cbreak mode so pressed() can detect a
    bare spacebar press without blocking or requiring Enter.

    Degrades safely when stdin isn't a real TTY (e.g. under pytest, or a
    piped/headless invocation): pressed() always returns False in that case,
    so the only way to end a hold is Ctrl-C -- no crash, no hang.
    """

    def __init__(self):
        self._is_tty = False
        self._fd = None
        self._old_settings = None

    def __enter__(self):
        try:
            self._is_tty = sys.stdin.isatty()
        except Exception:
            self._is_tty = False
        if self._is_tty:
            import termios
            import tty
            self._fd = sys.stdin.fileno()
            self._old_settings = termios.tcgetattr(self._fd)
            tty.setcbreak(self._fd)
        else:
            print("[keyboard] stdin is not a terminal -- space-to-release is "
                  "unavailable; use Ctrl-C to end the hold instead.")
        return self

    def pressed(self) -> bool:
        """Non-blocking: True the instant a space character has been read."""
        if not self._is_tty:
            return False
        ready, _, _ = select.select([sys.stdin], [], [], 0)
        if not ready:
            return False
        data = os.read(self._fd, 64)
        return _contains_space(data)

    def __exit__(self, exc_type, exc_val, exc_tb):
        if self._is_tty and self._old_settings is not None:
            import termios
            termios.tcsetattr(self._fd, termios.TCSADRAIN, self._old_settings)
        return False
