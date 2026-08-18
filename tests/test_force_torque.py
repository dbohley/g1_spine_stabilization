"""Tests for the ATI F/T sensor reader + gravity compensation.

Hardware-free: GravityCompensator is pure math; ForceTorqueSensor is
exercised over a localhost UDP loopback with a fake sensor thread (no real
ATI hardware).
"""
import socket
import struct
import threading
import time

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from g1_spine_stabilization.sensors.force_torque import (
    ForceTorqueSensor,
    GravityCompensator,
    parse_packet,
    build_command,
    rotation_from_rpy,
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
