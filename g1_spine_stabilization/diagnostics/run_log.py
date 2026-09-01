"""
Single global, wall-clock-anchored diagnostic log for an entire dry run.

Built to root-cause a still-unresolved "smooth, then violent" motion bug in
the RRT pre-grasp approach (Phase 1a) and the straight-in close (Phase 1b) --
two prior fix attempts were each based on reading code and reasoning about a
plausible worst case, not on watching an actual run. This module exists so
every command this codebase sends, every phase transition, and (via
diagnostics/sdk_state_tap.py) the vendored SDK's own internal dry-run state
can be captured into ONE file with a shared clock, instead of the several
separate, uncorrelated time bases (controller time.monotonic(), the
execute_joint_path CSV's time.time(), Rerun's own implicit timeline) that
existed before this.

Usage:
    from g1_spine_stabilization.diagnostics.run_log import RUN_LOG
    RUN_LOG.start("/path/to/log.jsonl")   # once, near the top of main()
    RUN_LOG.log("phase1b", "tick", i=3, sol_q=q.tolist())
    RUN_LOG.stop()                        # once, in a finally block

RUN_LOG.log() before start() (or after stop()) is a documented no-op, not an
error -- so any code that unconditionally calls RUN_LOG.log() stays safe to
import and run even when no one has opted into logging for this process.
"""

import json
import threading
import time


class _RunLog:
    def __init__(self):
        self._lock = threading.Lock()
        self._file = None

    def start(self, path):
        """Open `path` for JSONL writes. Safe to call at most once per
        process per run -- a second start() while already open closes the
        previous file first (documented, not asserted, since a script that
        re-enters main() in a test harness shouldn't crash)."""
        with self._lock:
            if self._file is not None:
                self._file.close()
            self._file = open(path, "w")

    def log(self, source, event, **fields):
        """Append one JSONL record: {t_wall, t_mono, source, event, **fields}.
        No-op if start() was never called (or stop() already ran) -- lets
        every call site call this unconditionally."""
        with self._lock:
            if self._file is None:
                return
            record = {
                "t_wall": time.time(),
                "t_mono": time.monotonic(),
                "source": source,
                "event": event,
            }
            record.update(fields)
            self._file.write(json.dumps(record, default=_json_default) + "\n")
            self._file.flush()

    def stop(self):
        """Close the log file, if open. Safe to call even if start() was
        never called (or stop() already ran)."""
        with self._lock:
            if self._file is not None:
                self._file.close()
                self._file = None

    @property
    def is_active(self):
        with self._lock:
            return self._file is not None


def _json_default(obj):
    tolist = getattr(obj, "tolist", None)
    if callable(tolist):
        return tolist()
    return str(obj)


RUN_LOG = _RunLog()
