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
        signal: str = "min",
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

        ``min`` (default) guarantees BOTH palms reach the setpoint before the
        loop settles; ``mean`` balances them.
        """
        vals = [self.force(s) for s in self.sides]
        if not vals:
            return 0.0
        return float(min(vals)) if self.signal == "min" else float(np.mean(vals))

    def guard_force(self) -> float:
        """Combined force used for the crush guard: the harder-pressed palm."""
        vals = [self.force(s) for s in self.sides]
        return float(max(vals)) if vals else 0.0

    def close(self) -> None:
        for sub in self._subs.values():
            try:
                sub.Close()
            except Exception:
                pass
        self._subs.clear()


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
    ) -> None:
        if force_min > force_max:
            raise ValueError(f"force_min ({force_min}) must be <= force_max ({force_max})")
        self.kappa = float(kappa)
        self.s_max = float(s_max)
        self.force_min = float(force_min)
        self.force_max = float(force_max)
        self.force_target = float(np.clip(force_target, force_min, force_max))
        self.s = 0.0

    def reset(self) -> None:
        self.s = 0.0

    def step(self, F: float, F_guard: float | None = None) -> float:
        """Advance the press depth one tick and return the new ``s`` (meters).

        Args:
            F:       combined measured palm force (Newtons) -- the target signal
                     (e.g. min of the two palms).
            F_guard: combined force for the crush guard (e.g. max of the two
                     palms). Defaults to ``F``.
        """
        F = float(F)
        F_guard = F if F_guard is None else float(F_guard)

        update = self.kappa * (self.force_target - F)
        if F_guard > self.force_max:
            # Release-only crush guard: cap the increment at a retreating value
            # so an over-pressed palm always backs off, even if the target-side
            # force is still below setpoint.
            update = min(update, self.kappa * (self.force_max - F_guard))

        self.s = float(np.clip(self.s + update, 0.0, self.s_max))
        return self.s
