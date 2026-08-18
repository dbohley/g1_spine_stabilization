"""ATI Axia80 force/torque sensor: RDT-over-UDP reader + gravity compensation.

Three pieces, used by both the robot recording path
(controllers/sdk_pose_tracking.py Phase 2, via run_pose_tracking_preview.py)
and the human-trial path (scripts/record_human_trial.py):

  ForceTorqueSensor   -- background-threaded UDP reader for the ATI RDT
                         protocol (port 49152). Lock-guarded latest wrench,
                         mirrors controllers/palm_force.py's PalmForceReader
                         pattern but owns its own recv thread (RDT is a raw
                         UDP stream, not DDS-callback driven). Degrades
                         gracefully (read_raw() -> None) if the sensor is
                         unreachable, so recording never crashes.
  GravityCompensator  -- pure-numpy gravity cancellation: orientation-aware
                         for the FORCE channels (rotate the startup gravity
                         reading by the torso/target marker's rotation),
                         tared for the TORQUE channels. See the class
                         docstring and docs/data_recording_conventions.md.
  ForceTorqueSource   -- thin glue wrapping the two, so both record loops
                         share one read()/capture_reference() contract.

Dependency-light (socket/struct/threading/time/numpy/scipy) -- no robot, no
IK, no g1_inspire_sdk. Calibration (cpf/cpt) and the sensor IP come from
configs/default.yaml's force_sensor: section. Networking setup for this
machine: docs/force_sensor_setup.md.
"""
import socket
import struct
import threading
import time
from typing import Optional, Tuple

import numpy as np
from scipy.spatial.transform import Rotation

# --- RDT protocol constants (ATI Ethernet Axia, manual 9610-05) ---
RDT_HEADER = 0x1234
CMD_START = 0x0002        # start high-speed streaming
CMD_STOP = 0x0000
RDT_PORT = 49152
COMMAND_FMT = "!HHI"      # header(u16), command(u16), sample_count(u32)
RESPONSE_FMT = "!IIIiiiiii"  # rdt_seq, ft_seq, status, Fx,Fy,Fz, Tx,Ty,Tz
RESPONSE_SIZE = 36

_SETUP_DOC = "docs/force_sensor_setup.md"


def build_command(command: int, sample_count: int = 0) -> bytes:
    """Build an RDT command packet (8 bytes). sample_count=0 = stream forever."""
    return struct.pack(COMMAND_FMT, RDT_HEADER, command, sample_count)


def parse_packet(data: bytes, cpf: float, cpt: float) -> Tuple[np.ndarray, int]:
    """Parse a 36-byte RDT response into a scaled wrench + status.

    Returns ((6,) [Fx,Fy,Fz,Mx,My,Mz] in N / N*m, status_int). Forces are
    raw counts / cpf; torques raw counts / cpt.
    """
    rdt_seq, ft_seq, status, fx, fy, fz, tx, ty, tz = struct.unpack(RESPONSE_FMT, data)
    wrench = np.array(
        [fx / cpf, fy / cpf, fz / cpf, tx / cpt, ty / cpt, tz / cpt], dtype=float
    )
    return wrench, status


def rotation_from_rpy(rpy) -> np.ndarray:
    """3x3 rotation from fixed-axis rpy (radians): R = Rz(yaw)Ry(pitch)Rx(roll)."""
    r, p, y = (float(v) for v in rpy)
    return Rotation.from_euler("xyz", [r, p, y]).as_matrix()


class ForceTorqueSensor:
    """Background-threaded RDT/UDP reader for the ATI Axia80.

    enabled=False -> no socket opened; read_raw() always returns None (so the
    same code path works with the sensor unplugged, and in tests).
    """

    def __init__(self, ip, port=RDT_PORT, cpf=1.0, cpt=1.0, *, enabled=True,
                 connect_timeout=3.0, stale_after=1.0):
        self.ip = ip
        self.port = int(port)
        self.cpf = float(cpf)
        self.cpt = float(cpt)
        self.enabled = bool(enabled)
        self.connect_timeout = float(connect_timeout)
        self.stale_after = float(stale_after)

        self._sock = None
        self._thread = None
        self._stop = threading.Event()
        self._lock = threading.Lock()
        self._latest: Optional[np.ndarray] = None
        self._latest_t = 0.0
        self._status = 0
        self._got_any = False
        self._status_warned = False

    def start(self) -> None:
        """Open the socket, request streaming, spawn the recv thread, and wait
        up to connect_timeout for the first packet (printing a diagnostic and
        continuing degraded if none arrives). No-op when disabled."""
        if not self.enabled:
            return
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.settimeout(1.0)
        try:
            self._sock.sendto(build_command(CMD_START), (self.ip, self.port))
        except OSError as exc:
            self._print_diagnostic(f"failed to send start command: {exc}")

        self._thread = threading.Thread(
            target=self._loop, name="ForceTorqueSensor", daemon=True
        )
        self._thread.start()

        deadline = time.monotonic() + self.connect_timeout
        while time.monotonic() < deadline:
            with self._lock:
                if self._got_any:
                    print(f"[ForceTorque] Connected to {self.ip}:{self.port} (streaming).")
                    return
            time.sleep(0.05)
        self._print_diagnostic(f"no data within {self.connect_timeout:.1f}s")

    def _loop(self) -> None:
        while not self._stop.is_set():
            try:
                data, _ = self._sock.recvfrom(1024)
            except socket.timeout:
                continue
            except OSError:
                break
            if len(data) < RESPONSE_SIZE:
                continue
            try:
                wrench, status = parse_packet(data[:RESPONSE_SIZE], self.cpf, self.cpt)
            except struct.error:
                continue
            with self._lock:
                self._latest = wrench
                self._latest_t = time.monotonic()
                self._status = status
                self._got_any = True
            if status != 0 and not self._status_warned:
                # Nonzero status = warning/error bit(s) (manual 4.8). Keep the
                # data (it's still a reading) but flag it once.
                print(f"[ForceTorque] WARNING: sensor status {status:#010x} "
                      f"(see manual section 4.8)")
                self._status_warned = True

    def read_raw(self) -> Optional[np.ndarray]:
        """Latest scaled wrench (6,), or None if disabled / no packet yet /
        the last packet is older than stale_after seconds."""
        if not self.enabled:
            return None
        with self._lock:
            if self._latest is None:
                return None
            if time.monotonic() - self._latest_t > self.stale_after:
                return None
            return self._latest.copy()

    def close(self) -> None:
        """Stop the thread, send stop-streaming, close the socket. Idempotent."""
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=1.0)
            self._thread = None
        if self._sock is not None:
            try:
                self._sock.sendto(build_command(CMD_STOP), (self.ip, self.port))
            except OSError:
                pass
            try:
                self._sock.close()
            except OSError:
                pass
            self._sock = None

    def _print_diagnostic(self, reason: str) -> None:
        print(f"[ForceTorque] Could not reach sensor at {self.ip}:{self.port} ({reason}).")
        print("[ForceTorque] Recording continues with NO force data (wrench fields = None).")
        print("[ForceTorque] Likely cause on this machine: the sensor NIC has no IP on the")
        print(f"[ForceTorque] sensor's subnet. Fix / alternatives: {_SETUP_DOC}")


class GravityCompensator:
    """Cancel the payload's weight from an F/T reading.

    FORCE channels: orientation-aware. The startup at-rest reading F0 is the
    gravity force in the sensor frame at the reference orientation. As the
    torso (carrying the target marker, rigidly tied to the sensor) rotates,
    the gravity force in the sensor frame is F_grav = M (R_cur.T R_ref) M.T F0,
    where R_ref/R_cur are the torso marker's rotations at reference/now and M
    is the fixed marker->sensor rotation (default identity = marker mounted
    axis-aligned with the sensor). The relative rotation R_cur.T R_ref is
    invariant to the fixed frame the marker rotation is expressed in, so
    optical- vs pelvis-frame rotations give the same result.

    TORQUE channels: tared -- T_comp = T_raw - T0. Fully orientation-aware
    torque would need the payload's center-of-mass offset r (not measured
    here), so torque cancellation is exact only near the reference pose.

    Fallbacks: if the torso rotation is unavailable (marker not tracked this
    sample, or none captured at reference), degrade to a plain tare on all six
    channels. Returns None before a reference is set or when raw is None.
    """

    def __init__(self, marker_to_sensor: Optional[np.ndarray] = None):
        self._M = np.eye(3) if marker_to_sensor is None else np.asarray(marker_to_sensor, float).reshape(3, 3)
        self._F0: Optional[np.ndarray] = None
        self._T0: Optional[np.ndarray] = None
        self._R_ref: Optional[np.ndarray] = None

    @property
    def has_reference(self) -> bool:
        return self._F0 is not None

    def set_reference(self, baseline_wrench, R_torso_ref) -> None:
        b = np.asarray(baseline_wrench, float).reshape(6)
        self._F0 = b[:3].copy()
        self._T0 = b[3:].copy()
        self._R_ref = None if R_torso_ref is None else np.asarray(R_torso_ref, float).reshape(3, 3).copy()

    def compensate(self, raw_wrench, R_torso_current) -> Optional[np.ndarray]:
        if raw_wrench is None or self._F0 is None:
            return None
        raw = np.asarray(raw_wrench, float).reshape(6)
        if R_torso_current is not None and self._R_ref is not None:
            R_cur = np.asarray(R_torso_current, float).reshape(3, 3)
            F_grav = self._M @ (R_cur.T @ self._R_ref) @ self._M.T @ self._F0
            F_comp = raw[:3] - F_grav
        else:
            # No orientation available -> plain tare on the force channels too.
            F_comp = raw[:3] - self._F0
        T_comp = raw[3:] - self._T0
        return np.concatenate([F_comp, T_comp])


class ForceTorqueSource:
    """Glue: a ForceTorqueSensor + GravityCompensator behind one contract, so
    both record loops share the same read()/capture_reference() calls."""

    def __init__(self, sensor: ForceTorqueSensor,
                 compensator: Optional[GravityCompensator] = None,
                 *, baseline_frames=50, gravity_comp_enabled=True):
        self._sensor = sensor
        self._comp = compensator
        self._baseline_frames = int(baseline_frames)
        self._gravity_comp_enabled = bool(gravity_comp_enabled) and compensator is not None

    @classmethod
    def from_config(cls, cfg, *, enabled_override: Optional[bool] = None) -> "ForceTorqueSource":
        fs = cfg.get("force_sensor", {})
        enabled = fs.get("enabled", False) if enabled_override is None else bool(enabled_override)
        sensor = ForceTorqueSensor(
            ip=fs.get("ip", "192.168.1.1"),
            port=fs.get("port", RDT_PORT),
            cpf=fs.get("cpf", 1.0),
            cpt=fs.get("cpt", 1.0),
            enabled=enabled,
            connect_timeout=fs.get("connect_timeout", 3.0),
        )
        gc = fs.get("gravity_comp", {})
        comp = GravityCompensator(
            marker_to_sensor=rotation_from_rpy(gc.get("marker_to_sensor_rpy", [0.0, 0.0, 0.0]))
        )
        return cls(sensor, comp,
                   baseline_frames=fs.get("baseline_frames", 50),
                   gravity_comp_enabled=gc.get("enabled", True))

    def start(self) -> None:
        self._sensor.start()

    def capture_reference(self, R_torso, *, sleep=0.02) -> None:
        """Average baseline_frames at-rest readings into the gravity reference,
        paired with the torso rotation now. Call once, with the neck device NOT
        pulling, at the start of the tracking phase. No-op if gravity comp is
        off or the sensor gave no data."""
        if not self._gravity_comp_enabled:
            return
        frames = []
        for _ in range(self._baseline_frames):
            w = self._sensor.read_raw()
            if w is not None:
                frames.append(w)
            time.sleep(sleep)
        if not frames:
            print("[ForceTorque] No sensor data for the gravity reference; "
                  "gravity-cancelled wrench will be unavailable this run.")
            return
        self._comp.set_reference(np.mean(frames, axis=0), R_torso)
        print(f"[ForceTorque] Gravity reference captured from {len(frames)} frames.")

    def read(self, R_torso) -> Tuple[Optional[np.ndarray], Optional[np.ndarray]]:
        """Return (raw_wrench|None, gravity_cancelled_wrench|None) for this sample."""
        raw = self._sensor.read_raw()
        comp = None
        if self._gravity_comp_enabled and self._comp.has_reference:
            comp = self._comp.compensate(raw, R_torso)
        return raw, comp

    def close(self) -> None:
        self._sensor.close()
