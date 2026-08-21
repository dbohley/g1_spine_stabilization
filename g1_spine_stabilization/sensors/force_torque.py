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
  GravityCompensator  -- pure-numpy gravity cancellation. tare() is a plain,
                         orientation-blind subtraction of the startup
                         reference (DC offset + reference-pose gravity),
                         always available once a reference is captured.
                         compensate() additionally re-projects the FORCE
                         channels' startup gravity reading by the torso/
                         target marker's rotation as it changes (TORQUE
                         channels stay tared either way). See the class
                         docstring and docs/data_recording_conventions.md.
  ForceTorqueSource   -- thin glue wrapping the two, so both record loops
                         share one read()/capture_reference() contract, plus
                         read_all() for callers that want raw/tared/
                         gravity-adjusted side by side (e.g.
                         scripts/gravity_comp_debug.py).

Dependency-light (socket/struct/threading/time/numpy/scipy) -- no robot, no
IK, no g1_inspire_sdk. Calibration (cpf/cpt) and the sensor IP come from
configs/default.yaml's force_sensor: section. Networking setup for this
machine: docs/force_sensor_setup.md.
"""
import math
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


def sin_model_gravity_residual(theta_rad: float, reference_force_n: float) -> float:
    """Experimental alternative to GravityCompensator's rotation-matrix
    reprojection -- debug/comparison use only, see
    scripts/gravity_comp_debug.py and configs/default.yaml's
    force_sensor.debug_sin_model. Estimates the orientation-dependent gravity
    residual left in an already-tared force reading as
    reference_force_n * sin(theta_rad): fit from a single observed data point
    (a torso rotation and its tared-force reading), where sin(0)=0 matches
    tared force being ~0 at the reference pose and sin(pi/2)=1 recovers
    reference_force_n at a 90-degree rotation. Not part of
    GravityCompensator -- not used by the real recording pipeline."""
    return reference_force_n * math.sin(theta_rad)


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

    def tare(self, raw_wrench) -> Optional[np.ndarray]:
        """Plain startup tare on all six channels: raw - [F0, T0]. No
        orientation tracking at all -- cancels whatever was present in the
        reference reading (DC/electrical offset + gravity at the reference
        pose) and stays fixed at that value for the rest of the run,
        regardless of how the torso later moves. Contrast with compensate(),
        which additionally re-projects the reference gravity force as the
        torso's orientation changes. Returns None before a reference is set
        or when raw_wrench is None."""
        if raw_wrench is None or self._F0 is None:
            return None
        raw = np.asarray(raw_wrench, float).reshape(6)
        return raw - np.concatenate([self._F0, self._T0])


class ForceTorqueSource:
    """Glue: a ForceTorqueSensor + GravityCompensator behind one contract, so
    both record loops share the same read()/capture_reference() calls.

    Two independent things happen here, not one:
      1. The startup tare -- ALWAYS captured (as long as the sensor produced
         data), regardless of gravity_comp.enabled. This cancels whatever is
         present in the raw reading at the reference instant: the sensor's
         own DC/electrical offset AND the payload's weight at the reference
         orientation (gravity). Neither is separable from a single static
         reading, so taring necessarily cancels both together.
      2. Orientation-aware reprojection -- an OPTIONAL upgrade, gated by
         gravity_comp.enabled, that re-projects the reference gravity force
         as the torso tilts (rather than assuming it stays at the reference
         orientation forever). This only affects wrench_grav_comp/compensate();
         it has no effect on the plain tare from (1).
    """

    def __init__(self, sensor: ForceTorqueSensor,
                 compensator: Optional[GravityCompensator] = None,
                 *, baseline_frames=50, gravity_comp_enabled=True):
        self._sensor = sensor
        self._comp = compensator
        self._baseline_frames = int(baseline_frames)
        # Gates ONLY orientation-aware reprojection in compensate() -- the
        # startup tare itself (capture_reference/read_all below) always runs
        # when a compensator is available. See class docstring.
        self._orientation_tracking_enabled = bool(gravity_comp_enabled) and compensator is not None

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
        """Average baseline_frames at-rest readings into the tare/gravity
        reference, paired with the torso rotation now (if available). Call
        once, with the neck device NOT pulling, at the start of the tracking
        phase. ALWAYS runs (regardless of gravity_comp.enabled) as long as a
        compensator exists and the sensor produced data -- this is the
        unconditional DC-offset + startup-gravity tare; whether it's later
        upgraded to orientation-aware reprojection is decided in read()/
        read_all() via gravity_comp.enabled. No-op if no compensator was
        provided or the sensor gave no data."""
        if self._comp is None:
            return
        frames = []
        for _ in range(self._baseline_frames):
            w = self._sensor.read_raw()
            if w is not None:
                frames.append(w)
            time.sleep(sleep)
        if not frames:
            print("[ForceTorque] No sensor data for the tare/gravity reference; "
                  "adjusted wrench will be unavailable this run.")
            return
        self._comp.set_reference(np.mean(frames, axis=0), R_torso)
        print(f"[ForceTorque] Tare/gravity reference captured from {len(frames)} frames.")

    def read(self, R_torso) -> Tuple[Optional[np.ndarray], Optional[np.ndarray]]:
        """Return (raw_wrench|None, gravity_cancelled_wrench|None) for this
        sample. gravity_cancelled_wrench is orientation-aware reprojection
        when gravity_comp.enabled, else the plain tare from capture_reference
        (see GravityCompensator.compensate()'s no-orientation fallback)."""
        raw = self._sensor.read_raw()
        comp = None
        if self._comp is not None and self._comp.has_reference:
            R_arg = R_torso if self._orientation_tracking_enabled else None
            comp = self._comp.compensate(raw, R_arg)
        return raw, comp

    def read_all(self, R_torso) -> Tuple[Optional[np.ndarray], Optional[np.ndarray], Optional[np.ndarray]]:
        """Return (raw_wrench|None, tared_wrench|None, gravity_adjusted_wrench|None)
        for this sample -- all three side by side, for by-hand comparison
        (e.g. scripts/gravity_comp_debug.py). tared_wrench is the plain
        startup tare (GravityCompensator.tare()), always populated once a
        reference exists, independent of gravity_comp.enabled.
        gravity_adjusted_wrench is the same orientation-aware value read()
        returns as its second element."""
        raw = self._sensor.read_raw()
        tared = self._comp.tare(raw) if self._comp is not None else None
        grav_comp = None
        if self._comp is not None and self._comp.has_reference:
            R_arg = R_torso if self._orientation_tracking_enabled else None
            grav_comp = self._comp.compensate(raw, R_arg)
        return raw, tared, grav_comp

    def close(self) -> None:
        self._sensor.close()
