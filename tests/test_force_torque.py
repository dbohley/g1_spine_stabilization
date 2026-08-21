"""Tests for the ATI F/T sensor reader + gravity compensation.

Hardware-free: GravityCompensator is pure math; ForceTorqueSensor is
exercised over a localhost UDP loopback with a fake sensor thread (no real
ATI hardware).
"""
import math
import socket
import struct
import threading
import time

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from g1_spine_stabilization.sensors.force_torque import (
    ForceTorqueSensor,
    ForceTorqueSource,
    GravityCompensator,
    parse_packet,
    build_command,
    rotation_from_rpy,
    sin_model_gravity_residual,
    RESPONSE_FMT,
)


# ---------------------------------------------------------------------
# parse_packet / build_command
# ---------------------------------------------------------------------

def test_parse_packet_scales_by_cpf_cpt():
    pkt = struct.pack(RESPONSE_FMT, 7, 8, 0, 2000, -4000, 6000, 100, -200, 300)
    wrench, status = parse_packet(pkt, cpf=1000.0, cpt=100.0)
    np.testing.assert_allclose(wrench, [2.0, -4.0, 6.0, 1.0, -2.0, 3.0])
    assert status == 0


def test_build_command_layout():
    assert build_command(0x0002) == struct.pack("!HHI", 0x1234, 0x0002, 0)


def test_rotation_from_rpy_identity():
    np.testing.assert_allclose(rotation_from_rpy([0.0, 0.0, 0.0]), np.eye(3), atol=1e-12)


# ---------------------------------------------------------------------
# sin_model_gravity_residual (experimental, debug-only -- gravity_comp_debug.py)
# ---------------------------------------------------------------------

def test_sin_model_residual_zero_at_reference():
    assert sin_model_gravity_residual(0.0, 7.5) == pytest.approx(0.0)


def test_sin_model_residual_recovers_reference_force_at_90_degrees():
    """The model was fit from exactly this data point: a 7.5N tared-force
    reading observed at a 90-degree torso rotation."""
    assert sin_model_gravity_residual(math.pi / 2, 7.5) == pytest.approx(7.5)


def test_sin_model_residual_is_a_real_sine_not_a_constant():
    assert sin_model_gravity_residual(math.pi / 4, 7.5) == pytest.approx(7.5 * math.sqrt(2) / 2)


# ---------------------------------------------------------------------
# GravityCompensator
# ---------------------------------------------------------------------

def test_compensator_none_before_reference():
    gc = GravityCompensator()
    assert gc.compensate(np.zeros(6), np.eye(3)) is None
    assert not gc.has_reference


def test_compensator_none_when_raw_none():
    gc = GravityCompensator()
    gc.set_reference(np.zeros(6), np.eye(3))
    assert gc.compensate(None, np.eye(3)) is None


def test_compensator_cancels_pure_gravity_at_reference_pose():
    gc = GravityCompensator()
    baseline = np.array([0.0, 0.0, -9.8, 0.1, -0.2, 0.3])  # weight + resting torque
    gc.set_reference(baseline, np.eye(3))
    # Same reading, same orientation -> external wrench is ~zero.
    comp = gc.compensate(baseline.copy(), np.eye(3))
    np.testing.assert_allclose(comp, np.zeros(6), atol=1e-9)


def test_compensator_force_is_orientation_aware():
    """A pure-gravity reading at a NEW orientation must still cancel to ~0 on
    the force channels -- proving the startup gravity vector is re-projected
    by the torso rotation, not just subtracted as a constant."""
    gc = GravityCompensator()
    F0 = np.array([0.0, 0.0, -9.8])
    T0 = np.array([0.05, -0.05, 0.0])
    gc.set_reference(np.concatenate([F0, T0]), np.eye(3))

    R_cur = Rotation.from_euler("xyz", [0.4, -0.3, 0.9]).as_matrix()
    # Gravity expressed in the sensor frame at the new orientation (M = I):
    F_grav_now = R_cur.T @ np.eye(3) @ F0
    raw = np.concatenate([F_grav_now, T0])   # pure gravity, no external pull
    comp = gc.compensate(raw, R_cur)
    np.testing.assert_allclose(comp[:3], np.zeros(3), atol=1e-9)


def test_compensator_force_respects_marker_to_sensor_rotation():
    M = Rotation.from_euler("xyz", [0.2, 0.5, -0.1]).as_matrix()
    gc = GravityCompensator(marker_to_sensor=M)
    F0 = np.array([1.0, 2.0, -9.0])
    T0 = np.zeros(3)
    gc.set_reference(np.concatenate([F0, T0]), np.eye(3))

    R_cur = Rotation.from_euler("xyz", [-0.6, 0.2, 1.1]).as_matrix()
    F_grav_now = M @ (R_cur.T @ np.eye(3)) @ M.T @ F0
    raw = np.concatenate([F_grav_now, T0])
    comp = gc.compensate(raw, R_cur)
    np.testing.assert_allclose(comp[:3], np.zeros(3), atol=1e-9)


def test_compensator_torque_is_tared_regardless_of_orientation():
    gc = GravityCompensator()
    T0 = np.array([0.3, -0.4, 0.5])
    gc.set_reference(np.concatenate([np.zeros(3), T0]), np.eye(3))
    external_T = np.array([1.0, 1.0, 1.0])
    R_cur = Rotation.from_euler("xyz", [1.0, 2.0, 3.0]).as_matrix()
    comp = gc.compensate(np.concatenate([np.zeros(3), T0 + external_T]), R_cur)
    np.testing.assert_allclose(comp[3:], external_T, atol=1e-9)


def test_compensator_falls_back_to_plain_tare_without_orientation():
    gc = GravityCompensator()
    baseline = np.array([1.0, 2.0, -9.0, 0.1, 0.2, 0.3])
    gc.set_reference(baseline, np.eye(3))
    raw = baseline + 5.0
    comp = gc.compensate(raw, None)   # no current orientation
    np.testing.assert_allclose(comp, raw - baseline, atol=1e-9)


def test_compensator_falls_back_when_no_reference_orientation():
    """Reference captured without a torso rotation -> can't do orientation-aware
    force, degrade to plain tare even if a current rotation is given."""
    gc = GravityCompensator()
    baseline = np.array([1.0, 2.0, -9.0, 0.1, 0.2, 0.3])
    gc.set_reference(baseline, None)
    raw = baseline + 2.0
    comp = gc.compensate(raw, np.eye(3))
    np.testing.assert_allclose(comp, raw - baseline, atol=1e-9)


def test_tare_none_before_reference():
    gc = GravityCompensator()
    assert gc.tare(np.zeros(6)) is None


def test_tare_none_when_raw_none():
    gc = GravityCompensator()
    gc.set_reference(np.zeros(6), np.eye(3))
    assert gc.tare(None) is None


def test_tare_is_plain_subtraction_regardless_of_orientation():
    """tare() takes no rotation argument at all -- it must return the same
    raw - baseline value no matter how the torso has rotated, unlike
    compensate() which re-projects the force channels."""
    gc = GravityCompensator()
    baseline = np.array([1.0, 2.0, -9.0, 0.1, 0.2, 0.3])
    gc.set_reference(baseline, np.eye(3))
    raw = baseline + 5.0
    np.testing.assert_allclose(gc.tare(raw), raw - baseline, atol=1e-9)


# ---------------------------------------------------------------------
# ForceTorqueSensor
# ---------------------------------------------------------------------

def test_sensor_disabled_returns_none_and_opens_no_socket():
    s = ForceTorqueSensor("192.168.1.1", enabled=False)
    s.start()
    assert s.read_raw() is None
    s.close()


class _FakeSensorServer:
    """A localhost UDP 'sensor' that replies to a start command with a stream
    of RDT packets carrying fixed raw counts."""

    def __init__(self, counts):
        self._counts = counts
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.bind(("127.0.0.1", 0))
        self._sock.settimeout(0.05)
        self.port = self._sock.getsockname()[1]
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, daemon=True)

    def start(self):
        self._thread.start()

    def _run(self):
        client = None
        pkt = struct.pack(RESPONSE_FMT, 1, 1, 0, *self._counts)
        while not self._stop.is_set():
            try:
                _, addr = self._sock.recvfrom(1024)
                client = addr
            except socket.timeout:
                pass
            if client is not None:
                try:
                    self._sock.sendto(pkt, client)
                except OSError:
                    pass

    def close(self):
        self._stop.set()
        self._thread.join(timeout=1.0)
        self._sock.close()


def test_sensor_streams_and_scales_over_loopback():
    server = _FakeSensorServer(counts=(2000, -4000, 6000, 100, -200, 300))
    server.start()
    sensor = ForceTorqueSensor("127.0.0.1", port=server.port,
                               cpf=1000.0, cpt=100.0, connect_timeout=2.0)
    try:
        sensor.start()
        # Give the stream a moment.
        deadline = time.monotonic() + 2.0
        reading = None
        while time.monotonic() < deadline:
            reading = sensor.read_raw()
            if reading is not None:
                break
            time.sleep(0.02)
        assert reading is not None, "no packet received over loopback"
        np.testing.assert_allclose(reading, [2.0, -4.0, 6.0, 1.0, -2.0, 3.0])
    finally:
        sensor.close()
        server.close()


# ---------------------------------------------------------------------
# ForceTorqueSource
# ---------------------------------------------------------------------

class _FakeReader:
    """Duck-typed in-memory stand-in for ForceTorqueSensor -- just enough of
    read_raw() to exercise ForceTorqueSource without a real socket."""

    def __init__(self, wrench):
        self._wrench = np.asarray(wrench, float)

    def read_raw(self):
        return self._wrench.copy()


def test_capture_reference_runs_even_with_gravity_comp_disabled():
    """Regression test for the bug being fixed: capture_reference() used to
    be a no-op whenever gravity_comp_enabled=False, so NO tare was ever
    captured. It must now always populate a reference."""
    baseline = np.array([1.0, 2.0, -9.0, 0.1, 0.2, 0.3])
    source = ForceTorqueSource(
        _FakeReader(baseline), GravityCompensator(),
        baseline_frames=3, gravity_comp_enabled=False,
    )
    source.capture_reference(None, sleep=0.0)
    assert source._comp.has_reference


def test_read_all_returns_plain_tare_when_gravity_comp_disabled():
    baseline = np.array([1.0, 2.0, -9.0, 0.1, 0.2, 0.3])
    reader = _FakeReader(baseline)
    source = ForceTorqueSource(
        reader, GravityCompensator(),
        baseline_frames=3, gravity_comp_enabled=False,
    )
    source.capture_reference(np.eye(3), sleep=0.0)

    reader._wrench = baseline + 4.0
    raw, tared, grav_comp = source.read_all(np.eye(3))
    np.testing.assert_allclose(raw, baseline + 4.0)
    np.testing.assert_allclose(tared, np.full(6, 4.0), atol=1e-9)
    # grav_comp disabled -> R_torso is withheld from compensate(), which then
    # falls back to the same plain tare.
    np.testing.assert_allclose(grav_comp, np.full(6, 4.0), atol=1e-9)


def test_read_all_orientation_reprojection_when_enabled():
    F0 = np.array([0.0, 0.0, -9.8])
    T0 = np.array([0.05, -0.05, 0.0])
    baseline = np.concatenate([F0, T0])
    reader = _FakeReader(baseline)
    source = ForceTorqueSource(
        reader, GravityCompensator(),
        baseline_frames=3, gravity_comp_enabled=True,
    )
    source.capture_reference(np.eye(3), sleep=0.0)

    R_cur = Rotation.from_euler("xyz", [0.4, -0.3, 0.9]).as_matrix()
    F_grav_now = R_cur.T @ F0
    reader._wrench = np.concatenate([F_grav_now, T0])   # pure gravity, no external pull

    raw, tared, grav_comp = source.read_all(R_cur)
    np.testing.assert_allclose(raw, reader._wrench)
    # tared is orientation-blind -- won't cancel to zero at the new pose.
    assert not np.allclose(tared[:3], np.zeros(3), atol=1e-6)
    # grav_comp IS orientation-aware -- cancels to ~zero (pure gravity, re-projected).
    np.testing.assert_allclose(grav_comp[:3], np.zeros(3), atol=1e-9)
