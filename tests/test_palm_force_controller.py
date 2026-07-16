"""Unit tests for PalmForceController — the force-domain (Jacobian-mode) loop.

Covers the feedforward+trim law: convergence to F_des against an attenuating
contact plant, saturation at force_max, non-negativity, the F_des soft-start
ramp, the feedforward-off (pure-integral) variant, and input validation.
Time is injected via `now` for determinism.
"""
import pytest

from g1_spine_stabilization.controllers.palm_force import PalmForceController


def _make(**kw):
    d = dict(kappa_f=0.5, force_target=5.0, force_max=20.0,
             f_des_ramp_time=0.0, feedforward=True)
    d.update(kw)
    return PalmForceController(**d)


def test_starts_at_zero_and_resets():
    c = _make()
    assert c.F_cmd == 0.0
    c.step(0.0, now=0.0)          # no ramp -> F_des=5, trim grows, F_cmd>0
    assert c.F_cmd > 0.0
    c.reset()
    assert c.F_cmd == 0.0 and c._trim == 0.0


def test_converges_to_target_against_attenuating_plant():
    """Contact attenuates: F_meas = g*F_cmd (g<1). Steady state F_meas == F_des,
    achieved by the trim (F_cmd > F_des)."""
    g = 0.6
    c = _make(kappa_f=0.2, force_target=5.0, force_max=50.0)
    F_meas = 0.0
    for k in range(3000):
        F_cmd = c.step(F_meas, now=float(k))
        F_meas = g * F_cmd
    assert F_meas == pytest.approx(5.0, abs=0.05)
    assert c.F_cmd == pytest.approx(5.0 / g, abs=0.1)   # trim raised F_cmd to overcome g


def test_saturates_at_force_max():
    c = _make(kappa_f=1.0, force_target=100.0, force_max=8.0)   # target above cap
    for k in range(200):
        c.step(0.0, now=float(k))
    assert c.F_cmd == pytest.approx(8.0)


def test_never_negative():
    c = _make(kappa_f=1.0, force_target=0.0, force_max=8.0)
    for k in range(100):
        c.step(10.0, now=float(k))    # F_meas above (zero) target -> wants negative, clamps at 0
    assert c.F_cmd == 0.0


def test_fdes_ramp_starts_at_zero_and_climbs():
    # kappa_f=0 isolates the ramp: F_cmd == F_des (feedforward, no trim).
    c = _make(kappa_f=0.0, force_target=5.0, f_des_ramp_time=2.0)
    assert c.step(0.0, now=0.0) == pytest.approx(0.0)   # t=0 -> 0
    assert c.step(0.0, now=1.0) == pytest.approx(2.5)   # halfway
    assert c.step(0.0, now=2.0) == pytest.approx(5.0)   # full
    assert c.step(0.0, now=9.0) == pytest.approx(5.0)   # clamped at target


def test_feedforward_off_is_pure_integral():
    c = _make(kappa_f=0.1, force_target=5.0, force_max=50.0, feedforward=False)
    # base term dropped: F_cmd = trim only = 0.1*(5-0) = 0.5 on the first step
    assert c.step(0.0, now=0.0) == pytest.approx(0.5)


def test_rejects_negative_config():
    with pytest.raises(ValueError):
        PalmForceController(kappa_f=0.1, force_target=-1.0, force_max=5.0)
    with pytest.raises(ValueError):
        PalmForceController(kappa_f=0.1, force_target=1.0, force_max=-5.0)
