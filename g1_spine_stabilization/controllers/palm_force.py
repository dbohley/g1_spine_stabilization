"""Palm tactile force sensing + Cartesian press control for the Inspire hands.

This module implements the palm-force grip architecture (see
``docs/palm_force_control.md``):

  * The FINGERS just hold a fixed grasp pose -- no force loop on the fingers.
  * The PALM normal force is regulated by *pressing the two palms together*: an
    outer Cartesian loop drives a scalar press depth ``s`` (meters) that shrinks
    the hand-to-hand IK-target separation in ``compute_hand_targets``; the
    existing IK moves the wrist/elbow to realize the press.

Two pieces:

``PalmForceReader``
    Reduces the central-palm 14x8 tactile array to a per-hand normal force in
    Newtons. Reads live from the DDS topic the SDK already publishes
    (``rt/inspire_hand/touch/{l,r}``) whenever ``G1InspireRobot`` is connected --
    it does NOT open a second Modbus master (which would contend with the SDK's
    own reader loop). ``dry_run`` returns 0 N so the preview/headless path runs
    unchanged.

``PalmPressController``
    The outer press loop -- a bounded scalar integral law converging to a target
    palm force, with a crush guard. Paper-ready derivation in the doc.

Units: forces in Newtons (via ``counts_per_newton`` calibration from
``scripts/read_palm_force.py --calibrate``); press depth ``s`` and ``s_max`` in
meters; gain ``kappa`` in m/N.
"""

from __future__ import annotations

import threading
import time

import numpy as np

from g1_spine_stabilization.trajectory.targets import press_axis

# Central palm tactile matrix shape (register 4900, 112 shorts). Mirrors the
# data_sheet entry in inspire_sdkpy/inspire_hand_defaut.py.
PALM_SHAPE = (14, 8)

_SIDE_TO_LR = {"left": "l", "right": "r"}


def palm_force_newtons(palm, baseline, counts_per_newton, noise_threshold=0.0):
    """Reduce a 14x8 raw-count palm matrix to a normal force in Newtons.

    Baseline-subtracted and clipped non-negative, matching the reduction
    convention printed by ``scripts/read_palm_force.py``.

    Args:
        palm:              14x8 array of raw int16 taxel counts.
        baseline:          14x8 per-cell no-contact offset (counts).
        counts_per_newton: calibration scale; summed counts per Newton.
        noise_threshold:   counts above baseline to count a cell as "active".

    Returns:
        (force_newtons, peak_counts, active_cells)
        - force_newtons: summed contact counts / counts_per_newton  [N]
        - peak_counts:   strongest single cell (raw counts, NOT converted --
                         counts_per_newton calibrates the SUM, not one cell)
        - active_cells:  number of cells above the noise threshold
    """
    d = np.clip(np.asarray(palm, dtype=float) - np.asarray(baseline, dtype=float), 0.0, None)
    cpn = max(float(counts_per_newton), 1e-9)
    force_newtons = float(d.sum()) / cpn
    peak_counts = float(d.max())
    active_cells = int((d > float(noise_threshold)).sum())
    return force_newtons, peak_counts, active_cells


def fit_calibration_series(forces_n, counts_totals):
    """Least-squares line fit of summed palm counts vs. known applied force,
    from a multi-point calibration series (scripts/read_palm_force.py's
    ``--calibrate-series``): ``counts ~= slope*force + intercept``, so
    ``slope`` is the ``counts_per_newton`` estimate.

    This project gets force from the palm sensor by summing all 112 raw
    taxel counts and dividing by one calibration scalar -- NOT by an
    explicit per-cell area computation (no per-cell physical dimensions are
    available). ``docs/palm_force_control.md`` argues this is valid because
    each cell's local pressure reading, summed over a fixed geometry, is
    itself proportional to total force -- but that's an assumption, not a
    measurement. A single-point calibration can't tell the difference
    between "truly linear" and "just one sample on a curve"; a multi-point
    fit can: a near-zero intercept and R^2 close to 1 across several known
    loads IS the empirical check that the no-area-division approach holds
    over that range. ``per_point_ratio`` (each point's own counts/force,
    computed independently of the fit) is a second, assumption-free way to
    see the same thing -- if the sensor were nonlinear, these would drift
    with load even though the least-squares fit still returns *a* line.

    Args:
        forces_n:      known applied forces (N), one per calibration point.
        counts_totals: matching mean baseline-subtracted summed counts.

    Returns:
        dict with ``slope`` (counts_per_newton), ``intercept``, ``r2``
        (1.0 = perfect fit), and ``per_point_ratio`` (counts_totals /
        forces_n, elementwise, as a plain list).

    Raises:
        ValueError: fewer than 2 points (a line needs 2), or mismatched shapes.
    """
    forces_n = np.asarray(forces_n, dtype=float)
    counts_totals = np.asarray(counts_totals, dtype=float)
    if forces_n.shape != counts_totals.shape:
        raise ValueError("forces_n and counts_totals must be the same shape")
    if forces_n.size < 2:
        raise ValueError("need at least 2 calibration points to fit a line")

    slope, intercept = np.polyfit(forces_n, counts_totals, 1)
    predicted = slope * forces_n + intercept
    ss_res = float(np.sum((counts_totals - predicted) ** 2))
    ss_tot = float(np.sum((counts_totals - counts_totals.mean()) ** 2))
    r2 = 1.0 if ss_tot == 0.0 else 1.0 - ss_res / ss_tot
    return {
        "slope": float(slope),
        "intercept": float(intercept),
        "r2": float(r2),
        "per_point_ratio": (counts_totals / forces_n).tolist(),
    }


class PalmForceReader:
    """Per-hand palm normal force (Newtons) from the SDK's DDS touch topic.

    Live mode subscribes to ``rt/inspire_hand/touch/{l,r}`` and reduces the
    latest ``palm_touch`` matrix each query. Dry-run returns 0 N. The DDS
    ``ChannelFactory`` must already be initialized (it is, once the robot has
    ``connect()``-ed), so this reader is a pure consumer -- no network interface,
    no Modbus.
    """

    def __init__(
        self,
        sides,
        *,
        dry_run: bool,
        counts_per_newton: float,
        baseline_frames: int = 20,
        noise_threshold: float = 0.0,
        signal: str = "mean",
    ) -> None:
        self.sides = [s for s in sides]
        self.dry_run = bool(dry_run)
        self.counts_per_newton = float(counts_per_newton)
        self.baseline_frames = int(baseline_frames)
        self.noise_threshold = float(noise_threshold)
        if signal not in ("min", "mean"):
            raise ValueError(f"signal must be 'min' or 'mean', got {signal!r}")
        self.signal = signal

        self._lock = threading.Lock()
        self._latest = {s: None for s in self.sides}
        self._baseline = {s: np.zeros(PALM_SHAPE) for s in self.sides}
        self._subs: dict = {}

        if not self.dry_run:
            self._init_subscribers()

    # ------------------------------------------------------------------
    # DDS wiring (live only) -- lazy imports so dry-run needs no SDK.
    # ------------------------------------------------------------------
    def _init_subscribers(self) -> None:
        from unitree_sdk2py.core.channel import ChannelSubscriber  # noqa: PLC0415
        from inspire_sdkpy import inspire_dds  # noqa: PLC0415

        for s in self.sides:
            lr = _SIDE_TO_LR[s]
            sub = ChannelSubscriber(f"rt/inspire_hand/touch/{lr}", inspire_dds.inspire_hand_touch)
            sub.Init(self._make_callback(s), 10)
            self._subs[s] = sub
        print(f"[PalmForce] Subscribed to touch topics for {self.sides}")

    def _make_callback(self, side: str):
        def _cb(msg) -> None:
            try:
                arr = np.asarray(msg.palm_touch, dtype=float).reshape(PALM_SHAPE)
            except Exception:
                return
            with self._lock:
                self._latest[side] = arr
        return _cb

    def _get_matrix(self, side: str):
        with self._lock:
            return self._latest[side]

    # ------------------------------------------------------------------
    # Baseline + queries
    # ------------------------------------------------------------------
    def capture_baseline(self, frames: int | None = None, sleep: float = 0.02) -> None:
        """Average no-contact frames per side into the per-cell baseline.

        Call once, with the palms not yet pressing (squeeze=0), before the press
        loop starts. No-op in dry-run.
        """
        if self.dry_run:
            return
        n = int(self.baseline_frames if frames is None else frames)
        accum = {s: np.zeros(PALM_SHAPE) for s in self.sides}
        count = {s: 0 for s in self.sides}
        for _ in range(n):
            for s in self.sides:
                m = self._get_matrix(s)
                if m is not None:
                    accum[s] += m
                    count[s] += 1
            time.sleep(sleep)
        with self._lock:
            for s in self.sides:
                if count[s] > 0:
                    self._baseline[s] = accum[s] / count[s]
        print(f"[PalmForce] Baseline captured over {n} frames: "
              + ", ".join(f"{s}={count[s]} samples" for s in self.sides))

    def force(self, side: str) -> float:
        """Baseline-subtracted total normal force for one hand, in Newtons."""
        if self.dry_run:
            return 0.0
        m = self._get_matrix(side)
        if m is None:
            return 0.0
        f, _, _ = palm_force_newtons(m, self._baseline[side], self.counts_per_newton, self.noise_threshold)
        return f

    def peak_counts(self, side: str) -> float:
        """Strongest single palm cell (raw counts), for diagnostics."""
        if self.dry_run:
            return 0.0
        m = self._get_matrix(side)
        if m is None:
            return 0.0
        _, pk, _ = palm_force_newtons(m, self._baseline[side], self.counts_per_newton, self.noise_threshold)
        return pk

    def target_force(self) -> float:
        """Combined force used as the press-loop feedback (min or mean of sides).

        ``mean`` (default) drives the PAIR's average toward the setpoint --
        this is the fix for a live finding where ``min`` kept driving the
        weaker-reading hand toward the target while the other ran up toward
        the crush-guard ceiling with no corresponding benefit, instead of the
        two settling near the requested target together. ``min`` guarantees
        BOTH palms individually reach the setpoint before the loop settles,
        but is unreachable if the two hands have real per-hand asymmetry
        (contact geometry, calibration) -- pass ``signal="min"`` explicitly
        if that guarantee is actually what's wanted.
        """
        vals = [self.force(s) for s in self.sides]
        if not vals:
            return 0.0
        return float(min(vals)) if self.signal == "min" else float(np.mean(vals))

    def guard_force(self) -> float:
        """Combined force used for the crush guard: the harder-pressed palm.

        Always the max of the two hands, regardless of ``signal`` -- the
        crush guard is a safety concept (never let EITHER hand over-press),
        independent of how the target-tracking signal is aggregated.
        """
        vals = [self.force(s) for s in self.sides]
        return float(max(vals)) if vals else 0.0

    def update(self, theta: float = 0.0, grasp_frame_R=None) -> None:
        """No-op -- this reader is fed by a background DDS callback, not a
        per-tick push. Exists only so callers (sdk_pose_tracking.py's
        _grip_and_hold_tick) can call update() unconditionally without
        branching on which reader type is active; WristWrenchForceReader
        below is the one that actually needs this call."""
        return

    def close(self) -> None:
        for sub in self._subs.values():
            try:
                sub.Close()
            except Exception:
                pass
        self._subs.clear()


class WristWrenchForceReader:
    """Per-hand grip-force estimate derived from measured joint torque, NOT a
    hardware wrist force/torque sensor -- confirmed none exists on this robot
    or in the vendored SDK (the DDS ``LowState_``/``MotorState_`` messages
    carry only a per-motor ``tau_est`` scalar, no wrench field; the SDK's own
    internal "wrist force" vocabulary, e.g. ``_ik/admittance.py``, already
    means this same derived quantity, never a hardware channel). The existing
    ATI Axia80 F/T sensor in this repo (``sensors/force_torque.py``) is a
    different, separate device mounted on a human neck-rig, wired only into
    the ``--record`` data-logging path -- not usable here.

    Estimate: ``tau_ext = tau_measured - gravity_torque(q)``, then
    ``W_ext ~= pinv(J(q)^T) @ tau_ext`` per arm (``G1HandIK.ee_jacobians``,
    ``LOCAL_WORLD_ALIGNED`` frame -- each arm's Jacobian is nonzero only in
    its own 7 columns, so the two arms' estimates never mix), then the FORCE
    part of each arm's estimated wrench is projected onto the inter-palm
    press axis (``trajectory.targets.press_axis``, the SAME axis
    ``compute_press_wrenches`` uses to command a press the other direction).

    Sign convention -- TWO independent sign considerations, kept deliberately
    separate below so a wrong one can be fixed without touching the other:

    (1) +u/-u projection convention (derived once, unit-tested, unaffected by
    (2) below): ``compute_press_wrenches`` defines W as the wrench the palm
    exerts ON the object -- left ``-F*u``, right ``+F*u``. By Newton's third
    law the object's reaction ON the hand is the negative of that: ``+F*u``
    on the left hand, ``-F*u`` on the right -- i.e. for a genuine symmetric
    inward press, the two hands' RAW estimated reaction wrenches point in
    OPPOSITE world directions. Dotting the left hand's raw wrench with
    ``+u`` and the right hand's with ``-u`` turns those opposite raw signs
    into a matching POSITIVE reading for both hands when both are genuinely
    pressing inward -- mirroring ``PalmForceReader.force()``'s
    always-non-negative convention.

    (2) tau_est vs. gravity_torque(q) sign -- an SDK/hardware-specific
    convention, NOT derivable from the math above, and NOT something a
    synthetic-Jacobian unit test can catch (it only proves (1) is internally
    self-consistent, not that the absolute sign matches real hardware).
    Live-tested 2026-09-08: this robot's ``tau_est`` reports the OPPOSITE
    sign from Pinocchio's ``gravity_torque(q)`` -- confirmed by both hands
    reading increasingly NEGATIVE together as real squeeze/contact
    genuinely increased (a per-hand (1)-type mixup would show one hand
    positive and the other negative instead of both moving the same way).
    Corrected by negating in ``_estimate`` (``tau_ext = gravity_torque(q) -
    tau``, not ``tau - gravity_torque(q)``).

    Getting either backwards would make the crush guard silently wrong, not
    just noisy -- see tests/test_palm_force_reader.py's sign-convention
    regression tests, and re-confirm live after any hardware/SDK change that
    could touch how tau_est is reported.

    CAUTION: torque-derived force is inherently noisier and more
    model-dependent than a dedicated sensor (gravity-model error, Jacobian
    conditioning near singularities, joint friction/tau_est bias).
    ``force_max``/``touch_threshold``/``loss_floor`` tuned against the palm
    sensor's Newton values are NOT guaranteed to mean the same real-world
    force here even though both report "Newtons" -- re-validate live, don't
    assume equivalence. See docs/palm_force_control.md.

    Mirrors PalmForceReader's public surface (force/target_force/
    guard_force/capture_baseline/close) so it is a drop-in replacement at its
    one construction site in sdk_pose_tracking.py. The one real difference:
    this reader has no background thread, so it must be fed this tick's
    theta/grasp_frame_R explicitly via update() before force()/target_force()/
    guard_force() reflect anything current.
    """

    def __init__(self, robot, ik, *, signal: str = "mean") -> None:
        if signal not in ("min", "mean"):
            raise ValueError(f"signal must be 'min' or 'mean', got {signal!r}")
        self.robot = robot
        self.ik = ik
        self.signal = signal
        self._bias = np.zeros(2)     # [left, right] no-load offset from capture_baseline
        self._latest = np.zeros(2)   # [left, right], bias-corrected, in Newtons

    def _estimate(self, theta: float, grasp_frame_R=None) -> np.ndarray:
        """Raw (pre-bias-correction) [F_left, F_right] estimate for the given
        tick's theta/grasp_frame_R. Degrades to zeros (with a printed
        diagnostic) instead of raising if arm state isn't available (e.g.
        dry-run) -- mirrors PalmForceReader's dry_run -> 0N behavior."""
        try:
            state = self.robot.get_state()["arm"]
            q = np.asarray(state["q"], dtype=float)
            tau = np.asarray(state["tau"], dtype=float)
        except Exception as exc:
            print(f"[WristWrenchForce] no arm state available ({exc!r}) -- reading 0N.")
            return np.zeros(2)

        # Live-tested 2026-09-08: tau_est (the SDK's measured joint torque)
        # turned out to be in the OPPOSITE sign convention from Pinocchio's
        # gravity_torque(q) on this hardware -- confirmed live: both hands
        # read increasingly NEGATIVE together as real squeeze/contact
        # genuinely increased (ruling out a per-hand +u/-u mixup, which
        # would show one hand positive and the other negative instead of
        # both moving the same way). Negate here, upstream of the
        # projection, so the already independently-verified +u/-u
        # sign-convention derivation below (and its own unit test) stays
        # untouched and correct in its own right -- this fixes a different,
        # SDK-specific convention mismatch, not that derivation.
        tau_ext = self.ik.gravity_torque(q) - tau
        J_L, J_R = self.ik.ee_jacobians(q)
        W_L = np.linalg.pinv(J_L[:, :7].T) @ tau_ext[:7]
        W_R = np.linalg.pinv(J_R[:, 7:].T) @ tau_ext[7:]
        u = press_axis(theta, grasp_frame_R)
        F_L = float(W_L[:3] @ u)
        F_R = float(W_R[:3] @ -u)
        return np.array([F_L, F_R])

    def capture_baseline(self, frames: int = 20, sleep: float = 0.02,
                          theta: float = 0.0, grasp_frame_R=None) -> None:
        """Average `frames` no-external-load readings into a per-hand bias,
        subtracted by update() going forward. Call once, with the palms not
        yet pressing (mirrors PalmForceReader.capture_baseline's contract) --
        tau_ext should already be ~0 with no external contact in principle,
        but this covers real-world gravity-model residual error too."""
        accum = np.zeros(2)
        for _ in range(max(int(frames), 1)):
            accum += self._estimate(theta, grasp_frame_R)
            time.sleep(sleep)
        self._bias = accum / max(int(frames), 1)
        print(f"[WristWrenchForce] Baseline captured over {frames} frames: "
              f"left={self._bias[0]:+.2f}N right={self._bias[1]:+.2f}N")

    def update(self, theta: float, grasp_frame_R=None) -> None:
        """Refresh this tick's [left, right] estimate. Must be called before
        force()/target_force()/guard_force() to reflect anything current --
        this reader has no background thread pushing updates on its own."""
        self._latest = self._estimate(theta, grasp_frame_R) - self._bias

    def force(self, side: str) -> float:
        idx = 0 if side == "left" else 1
        return float(self._latest[idx])

    def target_force(self) -> float:
        """See PalmForceReader.target_force -- identical mean/min semantics,
        same reasoning for why mean is the default.

        NOTE the clip at 0 below, which PalmForceReader does not need: that
        reader's palm_force_newtons() already clips per-cell counts
        non-negative, so its per-hand force can never be negative. This
        estimator is torque-derived and SIGNED -- gravity-model residual,
        tau_est bias and Jacobian conditioning routinely push an unloaded
        hand's reading a few Newtons NEGATIVE.

        Live-found bug this fixes: a negative reading on one hand dragged the
        *mean* below target even when the other hand was already at or above
        it (left +5N, right -2N -> mean 1.5N against a 3N target), so the
        press loop read "under target" and kept driving the shared press
        depth s inward. Because s is a single scalar shared by both palms,
        that inward drive shoved the whole grasp frame toward the
        weak-reading side -- the aggressive one-sided motion reported live.
        A hand cannot pull the pair inward by reading negative; clipping at 0
        makes "no measurable contact" mean exactly that, not negative
        evidence of contact.
        """
        vals = np.clip(self._latest, 0.0, None)
        return float(np.min(vals)) if self.signal == "min" else float(np.mean(vals))

    def guard_force(self) -> float:
        """Always the max of the two hands -- see PalmForceReader.guard_force.

        Deliberately NOT clipped at 0: the guard only ever fires on readings
        above force_max, so a negative value is already inert here, and
        leaving it unclipped keeps the raw estimate visible for diagnostics.
        """
        return float(np.max(self._latest))

    def close(self) -> None:
        return


def clamp_target_force(requested: float, force_min: float, force_max: float):
    """Clamp a requested palm-press force_target (N) into the configured
    [force_min, force_max] safety bounds -- the same bounds PalmPressController
    already enforces at runtime via its release-only crush guard, applied here
    up front so a CLI override can't even ask for an unsafe target.

    Returns (effective_value, warning) -- warning is None unless requested was
    above force_max (the crush-guard ceiling), in which case it's a short
    human-readable string explaining the clamp.
    """
    requested = float(requested)
    if requested > force_max:
        return float(force_max), (
            f"requested target force {requested:.2f}N exceeds the configured "
            f"crush-guard ceiling force_max={force_max:.2f}N -- clamping to "
            f"{force_max:.2f}N. See docs/palm_force_control.md for how force_max "
            f"is set (pull-test)."
        )
    if requested < force_min:
        return float(force_min), (
            f"requested target force {requested:.2f}N is below the configured "
            f"hold floor force_min={force_min:.2f}N -- clamping to "
            f"{force_min:.2f}N."
        )
    return requested, None


class PalmPressController:
    """Outer Cartesian press loop: bounded scalar integral law on press depth s.

        F_des     = clip(force_target, [force_min, force_max])
        s_{k+1}   = clip_[0, s_max]( s_k + kappa * (F_des - F) )
        crush guard: if F_guard > force_max, force a non-positive (retreating)
                     increment regardless of the target error.

    Fixed point (unsaturated, no crush): F = F_des. See
    ``docs/palm_force_control.md`` for the derivation, stability bound, and how
    ``s`` feeds ``compute_hand_targets``.

    Units: forces in Newtons; ``s``/``s_max`` in meters; ``kappa`` in m/N.
    """

    def __init__(
        self,
        *,
        kappa: float,
        s_max: float,
        force_min: float,
        force_target: float,
        force_max: float,
        step_increment: float | None = None,
        settle_time: float = 0.0,
    ) -> None:
        if force_min > force_max:
            raise ValueError(f"force_min ({force_min}) must be <= force_max ({force_max})")
        if step_increment is not None and step_increment <= 0.0:
            raise ValueError(f"step_increment ({step_increment}) must be > 0 or None")
        if settle_time < 0.0:
            raise ValueError(f"settle_time ({settle_time}) must be >= 0")
        self.kappa = float(kappa)
        self.s_max = float(s_max)
        self.force_min = float(force_min)
        self.force_max = float(force_max)
        self.force_target = float(np.clip(force_target, force_min, force_max))
        self.step_increment = None if step_increment is None else float(step_increment)
        self.settle_time = float(settle_time)
        self.s = 0.0
        self._next_advance_t = None

    def reset(self) -> None:
        self.s = 0.0
        self._next_advance_t = None

    def step(self, F: float, F_guard: float | None = None, now: float | None = None) -> float:
        """Advance the press depth one tick and return the new ``s`` (meters).

        Two cadences, selected by ``step_increment``:

        * ``None`` (default): the original continuous integral law -- every
          call moves ``s`` by ``kappa * error``.
        * set: **step-and-settle**. ``s`` advances by at most
          ``step_increment`` metres, then HOLDS for ``settle_time`` seconds
          before it may advance again. Between advances the measured force is
          still read (and the crush guard still acts, see below), so each
          increment is judged against a force reading that actually reflects
          the *previous* increment.

        Why step-and-settle exists: the press depth is realised through the IK
        servo and the arm's own dynamics, and (under
        ``force_source: wrist_wrench``) sensed through a torque estimate that
        lags real contact. Advancing every tick at the ~6-10Hz loop rate
        commands new inward motion before the previous motion has shown up in
        the reading, so the loop integrates against stale force and overshoots
        -- it keeps closing because it has not yet *seen* the contact it
        already created. Pausing between increments is what makes the
        measurement meaningful.

        The crush guard is deliberately exempt from the settle hold: retreating
        from an over-pressed palm must never wait on a timer.

        Args:
            F:       combined measured palm force (Newtons) -- the target signal.
            F_guard: combined force for the crush guard (e.g. max of the two
                     palms). Defaults to ``F``.
            now:     wall-clock seconds for the settle timer; defaults to
                     time.monotonic(). Injected explicitly in tests.
        """
        F = float(F)
        F_guard = F if F_guard is None else float(F_guard)
        now = time.monotonic() if now is None else float(now)

        update = self.kappa * (self.force_target - F)
        crushing = F_guard > self.force_max
        if crushing:
            # Release-only crush guard: cap the increment at a retreating value
            # so an over-pressed palm always backs off, even if the target-side
            # force is still below setpoint.
            update = min(update, self.kappa * (self.force_max - F_guard))

        if self.step_increment is not None:
            # Bound how far one advance may travel, so a large force error
            # cannot be spent as one big lunge inward.
            update = float(np.clip(update, -self.step_increment, self.step_increment))
            if update > 0.0 and not crushing:
                # Inward advances wait out the settle interval; retreats do not.
                if self._next_advance_t is not None and now < self._next_advance_t:
                    return self.s
                self._next_advance_t = now + self.settle_time

        self.s = float(np.clip(self.s + update, 0.0, self.s_max))
        return self.s


class PalmContactWatchdog:
    """Contact-loss safety guard for the palm-press grip (SDK-6 head-drop guard).

    The press loops (both the position ``PalmPressController`` and the later
    Jacobian force controller) have a shared failure mode: if the grasped object
    -- a head, in the spine-stabilization context -- drops or slips out, the
    measured palm force falls to ~0, the loop reads a large below-setpoint error,
    and it keeps driving the palms *inward* against nothing. The position loop
    claps the palms in to ``squeeze_max`` and holds there; the force loop winds
    the commanded wrench up to its max and pushes the hands together with nothing
    to react against. Neither should keep squeezing a head that is no longer
    there.

    This watchdog is a pure state machine on the combined (min-of-two-palms)
    force signal, independent of which press controller is active:

      * **Arm** once the grip is real: the min-palm force crosses
        ``touch_threshold`` (both palms loaded). Before arming it never freezes,
        so the approach/close-in press can build first contact normally. In
        dry-run the reader returns 0 N, so it never arms and this whole guard is
        a no-op (the preview path is unchanged).
      * **Freeze** the instant a below-``loss_floor`` reading appears after
        arming (``press_frozen`` True) -- the press must not advance inward while
        contact looks lost, even before the trip latches. A transient dip that
        recovers next frame simply resumes pressing.
      * **Trip** once the force stays below ``loss_floor`` for ``loss_frames``
        consecutive frames -- a confirmed loss. The caller freezes the press,
        holds the current pose stiffly via the unchanged ``kp`` servo, and
        alerts the human.

    ``latch`` controls whether that trip is permanent:

      * ``latch=True`` (class default, the original behavior): ``tripped``
        stays True for the rest of the run even if contact fully returns.
        Only ``reset()`` clears it, and nothing calls ``reset()`` mid-run.
      * ``latch=False``: ``tripped`` clears as soon as force recovers above
        ``loss_floor``, so a spurious trip costs one dip's worth of frozen
        press rather than the whole remaining run. This is what the live
        config selects (``palm_press.watchdog_latch``) -- see that key's
        comment for why: ``force_source: wrist_wrench`` is a noisy,
        torque-derived estimate, not the palm tactile sensor, so a
        momentary below-floor reading is far more likely to be sensor noise
        than a genuinely dropped head. A confirmed loss should flag loudly
        and stop the press advancing, not end the operator's run.

    ``loss_floor <= touch_threshold`` (hysteresis) so arming and loss detection
    don't chatter. This is complementary to the vision-based ``object_lost``
    signal (marker gone): a head that drops but is still visible, or slips within
    the hands, shows up here as a *force* drop, not a marker loss.
    """

    def __init__(
        self,
        *,
        touch_threshold: float,
        loss_floor: float,
        loss_frames: int,
        latch: bool = True,
    ) -> None:
        if loss_floor > touch_threshold:
            raise ValueError(
                f"loss_floor ({loss_floor}) must be <= touch_threshold "
                f"({touch_threshold}) for hysteresis"
            )
        self.touch_threshold = float(touch_threshold)
        self.loss_floor = float(loss_floor)
        self.loss_frames = int(loss_frames)
        self.latch = bool(latch)
        self.armed = False
        self.tripped = False
        self._loss_count = 0
        self._below_floor = False

    def reset(self) -> None:
        """Clear all state. Call before re-grasping."""
        self.armed = False
        self.tripped = False
        self._loss_count = 0
        self._below_floor = False

    def update(self, F: float) -> None:
        """Advance the guard one tick with the combined palm force F (N).

        Returns nothing; read ``tripped``/``press_frozen`` after calling.
        Callers wanting to log loss/recovery edges should compare ``tripped``
        across calls (see sdk_pose_tracking.py's _grip_and_hold_tick).
        """
        F = float(F)
        if not self.armed:
            # Not yet gripped: never freeze (let first contact build), arm on touch.
            self._below_floor = False
            if F >= self.touch_threshold:
                self.armed = True
            return

        self._below_floor = F < self.loss_floor
        if self._below_floor:
            if self.tripped:
                return
            self._loss_count += 1
            if self._loss_count >= self.loss_frames:
                self.tripped = True
        else:
            # Force is back above the floor. Always clear the consecutive-frame
            # counter; additionally clear a confirmed trip when not latching, so
            # a recovered grip resumes pressing instead of staying frozen for
            # the rest of the run (see the class docstring's `latch` section).
            self._loss_count = 0
            if not self.latch:
                self.tripped = False

    @property
    def press_frozen(self) -> bool:
        """True when the press must NOT advance inward (contact lost or suspected).

        True whenever ``tripped``; also true transiently on any below-floor frame
        after arming, so inward creep stops immediately, before the trip confirms.
        With ``latch=False`` both conditions clear once force recovers above
        ``loss_floor``.
        """
        return self.tripped or (self.armed and self._below_floor)


class PressSeatDetector:
    """Declares the palm press CONVERGED, after which the press depth is held
    fixed for the rest of the run ("seat-and-freeze").

    Why this exists: the press depth ``s`` is not a private side channel of the
    force loop -- it is fed straight into ``compute_hand_targets(...,
    squeeze=s, ...)``, i.e. into the very same IK hand targets the Phase 2
    tracking controller is trying to hit. A force loop that keeps rewriting
    ``s`` every tick therefore fights the tracker on the press axis, and the
    force loop wins. With ``palm_press.force_source: wrist_wrench`` (a noisy,
    torque-derived estimate, not the palm tactile sensor) it is fighting the
    tracker with *noise*.

    The press loop's real job is to ESTABLISH the grip. Once the measured force
    has reached the target, that depth IS the grip position: save it, hold it,
    and let tracking own the hand targets from then on. Note that freezing
    ``s`` does NOT freeze the hands -- ``s`` keeps flowing through
    ``compute_hand_targets`` with the live ``theta``/``box_center``, so the
    grip geometry still rotates with the tracked object. Only the force-driven
    *adjustment* of the palm separation stops.

    The criterion is deliberately NOT "force first crossed the target": a
    single noise spike on the ``wrist_wrench`` estimate would then seat the
    grip early, at too shallow a depth, and there is no un-seating. Requiring
    ``frames`` CONSECUTIVE in-band readings makes a spike cost a counter reset
    rather than a bad grip.

    Once ``seated`` is True it stays True, even if force later leaves the band
    (contact lost, object slipping). That is intentional and is the difference
    from ``PalmContactWatchdog``'s ``latch=False``: a lost grip must flag
    loudly (the watchdog's ``[SAFETY]`` prints still do that) but must NOT hand
    control of the hand targets back to the force loop mid-tracking. Only
    ``reset()`` -- called at release, before a re-grasp -- clears it.
    """

    def __init__(
        self,
        *,
        force_target: float,
        tolerance: float,
        frames: int,
        per_hand_min: float | None = None,
    ) -> None:
        if tolerance < 0.0:
            raise ValueError(f"tolerance ({tolerance}) must be >= 0")
        if frames < 1:
            raise ValueError(f"frames ({frames}) must be >= 1")
        self.force_target = float(force_target)
        self.tolerance = float(tolerance)
        self.frames = int(frames)
        self.per_hand_min = None if per_hand_min is None else float(per_hand_min)
        self.seated = False
        self._in_band_count = 0

    def reset(self) -> None:
        """Clear all state. Call before re-grasping."""
        self.seated = False
        self._in_band_count = 0

    def update(self, F: float, per_hand=None) -> None:
        """Advance the detector one tick with the combined palm force F (N).

        Returns nothing; read ``seated`` after calling. Cheap to call after
        seating (it short-circuits), so callers need not branch.

        Args:
            F:        the combined (mean/min) force -- the same signal the
                      press loop regulates against.
            per_hand: optional [F_left, F_right]. When given alongside a
                      configured ``per_hand_min``, BOTH hands must
                      individually read at least ``per_hand_min`` before the
                      press can seat.

        Why per_hand exists: with ``signal: mean``, the combined force can sit
        exactly on target while the two hands are wildly asymmetric -- left
        +6N and right 0N averages to the same 3N as a balanced +3/+3. Seating
        there freezes a lopsided grip as the hold position, which is the
        asymmetry this gate exists to reject. Checking the aggregate alone
        cannot distinguish the two cases; only the per-hand values can.
        """
        if self.seated:
            return
        in_band = abs(float(F) - self.force_target) <= self.tolerance
        if in_band and self.per_hand_min is not None and per_hand is not None:
            in_band = bool(np.all(np.asarray(per_hand, dtype=float) >= self.per_hand_min))
        if in_band:
            self._in_band_count += 1
            if self._in_band_count >= self.frames:
                self.seated = True
        else:
            self._in_band_count = 0


class PalmForceController:
    """Force-domain palm controller for the Jacobian (Road-1) press mode.

    Output is a commanded press-force MAGNITUDE ``F_cmd`` (Newtons), NOT a press
    depth ``s`` (meters). The magnitude is turned into an inward inter-palm wrench
    by ``trajectory.targets.compute_press_wrenches`` and injected as joint torque
    feedforward via ``G1HandIK.ee_jacobians`` (``tau = J^T W``) -- so the grip
    force comes from a commanded torque, not from a position error against a
    stiff servo (that is ``PalmPressController``, the ``mode: position`` fallback).

    Structure: **feedforward + trim** (the doc's Road 1):

        F_des_k = force_target * clip((t - t0)/ramp_time, 0, 1)    # ramp from 0
        trim   += kappa_f * (F_des_k - F_meas)                     # integral trim
        F_cmd   = clip( (F_des_k if feedforward else 0) + trim, 0, force_max )

    The feedforward term ``F_des`` makes the press respond at the 250 Hz servo
    rate (its real benefit over the slow ~6-10 Hz position loop); the ``trim``
    integral cancels the object-dependent contact attenuation
    ``F_delivered = F_cmd * k_e/(k_e + k_cart)`` so the steady state is
    ``F_meas = F_des`` regardless of pad/object compliance. Ramping ``F_des`` from
    zero is a soft-start (never step a torque onto contact). ``trim`` and the
    output are both clamped, so even a stuck ``F_meas=0`` (contact lost) can only
    drive ``F_cmd`` to ``force_max`` -- and the caller zeros ``F_cmd`` on a
    ``PalmContactWatchdog`` trip well before that.

    Units: forces in Newtons; ``kappa_f`` in N per step per N error; times in s.
    """

    def __init__(
        self,
        *,
        kappa_f: float,
        force_target: float,
        force_max: float,
        f_des_ramp_time: float = 0.0,
        feedforward: bool = True,
    ) -> None:
        if force_target < 0.0:
            raise ValueError(f"force_target ({force_target}) must be >= 0")
        if force_max < 0.0:
            raise ValueError(f"force_max ({force_max}) must be >= 0")
        self.kappa_f = float(kappa_f)
        self.force_target = float(force_target)
        self.force_max = float(force_max)
        self.f_des_ramp_time = float(f_des_ramp_time)
        self.feedforward = bool(feedforward)
        self._trim = 0.0
        self.F_cmd = 0.0
        self._t0 = None

    def reset(self) -> None:
        """Zero the commanded force, the trim integrator, and the ramp clock."""
        self._trim = 0.0
        self.F_cmd = 0.0
        self._t0 = None

    def f_des(self, now: float) -> float:
        """Ramped force setpoint at wall-clock time ``now`` (seeds t0 on first call)."""
        if self._t0 is None:
            self._t0 = now
        if self.f_des_ramp_time <= 0.0:
            return self.force_target
        frac = (now - self._t0) / self.f_des_ramp_time
        return self.force_target * min(max(frac, 0.0), 1.0)

    def step(self, F_meas: float, now: float | None = None) -> float:
        """Advance one tick and return the commanded press force ``F_cmd`` (N).

        Args:
            F_meas: combined measured palm force (N) -- the target signal
                    (e.g. min of the two palms from ``PalmForceReader``).
            now:    wall-clock seconds for the ramp; defaults to time.monotonic().
                    Injected explicitly in tests for determinism.
        """
        now = time.monotonic() if now is None else float(now)
        F_des = self.f_des(now)
        self._trim = float(np.clip(
            self._trim + self.kappa_f * (F_des - float(F_meas)),
            -self.force_max, self.force_max,
        ))
        base = F_des if self.feedforward else 0.0
        self.F_cmd = float(np.clip(base + self._trim, 0.0, self.force_max))
        return self.F_cmd
