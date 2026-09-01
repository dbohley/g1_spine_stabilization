"""
Read-only polling tap into the vendored g1_inspire_sdk's internal dry-run
arm state -- G1Arm._q_target / ._cmd_q_state / ._sim_q / ._arm_velocity_limit.

Why this exists: g1_arm.py's dry-run pipeline runs its own background
daemon thread (`_dry_run_loop`, 250Hz) that clips `_q_target` toward
`_cmd_q_state` at a velocity cap, completely independent of this repo's own
control-loop cadence. Nothing in this codebase (or the SDK itself) ever
logs `_cmd_q_state` -- the clipper's actual running state -- anywhere, so a
sustained lag between what a controller last commanded and what the
clipper has really reached is otherwise invisible. This tap makes that
state visible in the same RUN_LOG timeline as every command this repo
sends, without modifying the vendored SDK clone (read-only attribute
access only, never assignment, never monkeypatching).

Best-effort: polling a background thread's state from another thread can
miss individual 250Hz ticks; this is not a substitute for editing the SDK
(out of scope, git-pull-only per this repo's rules) and isn't trying to
catch every single tick -- polling at ~4ms (matching the SDK's own
_motor_dt) is enough to see sustained lag/catch-up behavior, which is the
question this tap exists to answer.
"""

import threading
import time

from g1_spine_stabilization.diagnostics.run_log import RUN_LOG


def start_sdk_state_tap(robot, interval_s=0.004):
    """Spawn a daemon thread polling robot.arm's internal dry-run state into
    RUN_LOG at ~interval_s. Returns the threading.Event used to stop it (call
    .set() to stop, though as a daemon thread it will also die with the
    process). No-op (returns a pre-set Event) if robot.arm lacks the expected
    attributes -- e.g. a future SDK version renames them, or this is called
    against a live (non-dry-run) robot where this internal state may not
    mean the same thing. Never raises."""
    stop_event = threading.Event()

    arm = getattr(robot, "arm", None)
    if arm is None or not hasattr(arm, "_q_target"):
        stop_event.set()
        return stop_event

    def _poll():
        while not stop_event.is_set():
            try:
                q_target = arm._q_target
                cmd_q_state = arm._cmd_q_state
                sim_q = arm._sim_q
                vel_limit = arm._arm_velocity_limit
                RUN_LOG.log(
                    "sdk_internal", "tick",
                    q_target=None if q_target is None else q_target.copy(),
                    cmd_q_state=None if cmd_q_state is None else cmd_q_state.copy(),
                    sim_q=None if sim_q is None else sim_q.copy(),
                    vel_limit=vel_limit,
                )
            except AttributeError:
                # Future SDK rename -- degrade to "tap disabled", not a crash.
                break
            time.sleep(interval_s)

    thread = threading.Thread(target=_poll, name="sdk-state-tap", daemon=True)
    thread.start()
    return stop_event
