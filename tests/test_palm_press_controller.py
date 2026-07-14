"""Unit tests for PalmPressController — the outer Cartesian press loop.

Covers the bounded scalar integral law: convergence to force_target against a
simple contact plant, saturation at [0, s_max], the crush-guard retreat, target
clamping into [force_min, force_max], and reset.
"""

import numpy as np
import pytest

from g1_spine_stabilization.controllers.palm_force import PalmPressController


def _make(**kw):
    defaults = dict(kappa=1e-3, s_max=0.05, force_min=0.0, force_target=5.0, force_max=20.0)
    defaults.update(kw)
    return PalmPressController(**defaults)


def test_force_target_clamped_into_bounds():
    hi = _make(force_target=100.0, force_max=20.0)
    assert hi.force_target == 20.0
    lo = _make(force_min=2.0, force_target=0.5)
    assert lo.force_target == 2.0


def test_starts_at_zero_and_resets():
    c = _make()
    assert c.s == 0.0
    c.step(0.0)  # F below target -> s grows
    assert c.s > 0.0
    c.reset()
    assert c.s == 0.0


def test_saturates_at_s_max_when_force_stays_below_target():
    c = _make(kappa=1e-2, s_max=0.03, force_target=5.0)
    for _ in range(1000):
        c.step(0.0)  # never reaches target -> integrator winds up, clips at s_max
    assert c.s == pytest.approx(0.03)


def test_never_negative():
    c = _make(force_target=0.0)  # no press wanted
    for _ in range(100):
        c.step(10.0)  # F above target -> wants to retreat, but s floored at 0
    assert c.s == 0.0


def test_converges_to_target_against_linear_plant():
    """With a monotone contact plant F = stiffness * s, s converges so F -> target."""
    stiffness = 200.0  # N/m
    target = 5.0
    c = _make(kappa=5e-4, s_max=0.1, force_target=target)
    s = 0.0
    for _ in range(5000):
        F = stiffness * s
        s = c.step(F)
    F_final = stiffness * s
    assert F_final == pytest.approx(target, abs=0.05)
    assert s == pytest.approx(target / stiffness, abs=0.05 / stiffness)


def test_crush_guard_retreats_when_guard_exceeds_max():
    """If the guard force exceeds force_max, s decreases even below-target on F."""
    c = _make(kappa=1e-3, s_max=0.05, force_target=5.0, force_max=8.0)
    # wind s up first
    for _ in range(50):
        c.step(0.0)
    s_before = c.s
    assert s_before > 0.0
    # now the harder palm is way over force_max while the target-side reads 0
    c.step(0.0, F_guard=100.0)
    assert c.s < s_before


def test_no_guard_uses_F_for_guard():
    """When F_guard is omitted, F itself is the guard signal."""
    c = _make(kappa=1e-3, s_max=0.05, force_target=5.0, force_max=8.0)
    for _ in range(50):
        c.step(0.0)
    s_before = c.s
    c.step(100.0)  # F over max, no separate guard -> retreat
    assert c.s < s_before


def test_rejects_min_greater_than_max():
    with pytest.raises(ValueError):
        PalmPressController(kappa=1e-3, s_max=0.05, force_min=10.0, force_target=5.0, force_max=1.0)
