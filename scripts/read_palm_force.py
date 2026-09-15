#!/usr/bin/env python3
"""Read out the Inspire hand PALM tactile sensor (left and right).

Standalone, hand-only, READ-ONLY diagnostic. It does NOT actuate the hands or
touch the arm -- it only reads the central-palm tactile register block and
prints a reduced summary (total / peak / active-cell count), optionally the
full 14x8 matrix as an ASCII heatmap.

Why this exists: the ``g1_inspire_sdk`` wrapper's ``robot.hand.get_state()``
only surfaces the 6 per-finger force scalars and discards the rich per-cell
tactile arrays. The central palm is the physically-correct grip signal for a
two-palm press grasp, so we read it directly here to characterize its
magnitudes before wiring it into force control (see
``docs/palm_force_control.md``).

Access method: direct Modbus/TCP read of the ``palm_touch`` register block
(register 4900, 112 signed shorts -> 14x8), mirroring the parse in
``inspire_sdkpy/inspire_sdk.py``'s ``read_and_parse_registers``. This avoids
the DDS init/publish side effects of instantiating the SDK's ModbusDataHandler
and keeps the tool self-contained. It does NOT go through ``G1InspireRobot``.

    Central palm register map (from inspire_hand_defaut.py data_sheet):
        ("掌心触觉数据", 4900, 224, (14, 8), "palm_touch")   # 224 bytes = 112 shorts

Usage:
    # Dry-run (safe, no hardware; synthetic data) -- verifies plumbing:
    python scripts/read_palm_force.py --dry-run --raw

    # Live hardware -- HUMAN-RUN ONLY (connects to the hands over Modbus).
    # Read-only, but still opens a network connection to the hand:
    python scripts/read_palm_force.py --no-dry-run --baseline --raw

    # One hand only:
    python scripts/read_palm_force.py --side left

    # Calibrate counts_per_newton, one hand at a time, load given as a mass:
    python scripts/read_palm_force.py --no-dry-run --side left --calibrate-kg 0.5
    #   keep the palm untouched for the baseline, then rest the known mass flat
    #   on the central palm pad (face-up, spread like the real object) at the
    #   prompt. Repeat with --side right.

    # Multi-point calibration (recommended): type a series of known weights
    # (kg), one per prompt, pressing Enter after each; blank line to finish
    # (need >= 2 points). Fits counts_per_newton via least squares AND
    # reports each point's own ratio + R^2/intercept, so you can see whether
    # summed-counts-proportional-to-force actually holds across the range,
    # not just trust it from a single point:
    python scripts/read_palm_force.py --no-dry-run --side left --calibrate-series
"""

from __future__ import annotations

import argparse
import struct
import sys
import time
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).parent.parent))

from g1_spine_stabilization.controllers.palm_force import fit_calibration_series

# --- Hardware constants (mirror g1_inspire_sdk/inspire_hand.py + data_sheet) ---
PALM_REGISTER = 4900          # holding-register start for palm_touch
PALM_N_SHORTS = 112           # 224 bytes / 2 = 112 signed shorts
PALM_SHAPE = (14, 8)          # 112 = 14 rows x 8 cols
MODBUS_PORT = 6000
DEVICE_ID = 1
DEFAULT_LEFT_IP = "192.168.11.210"
DEFAULT_RIGHT_IP = "192.168.123.210"
STANDARD_GRAVITY = 9.80665    # m/s^2, to convert a known mass (kg) -> force (N)

# ASCII heatmap ramp, low -> high pressure.
_RAMP = " .:-=+*#%@"


class PalmForceReader:
    """Reads the central-palm 14x8 tactile matrix for one hand.

    Live mode holds a ModbusTcpClient and reads ``PALM_REGISTER``. Dry-run
    synthesizes a moving contact blob so the readout is exercisable with no
    hardware. ``pymodbus`` is imported lazily so dry-run works without it.
    """

    def __init__(self, side: str, ip: str, *, dry_run: bool = True,
                 device_id: int = DEVICE_ID, port: int = MODBUS_PORT) -> None:
        self.side = side
        self.ip = ip
        self.dry_run = dry_run
        self.device_id = device_id
        self.port = port
        self._client = None
        self._t0 = time.monotonic()

    def connect(self) -> None:
        if self.dry_run:
            return
        from pymodbus.client import ModbusTcpClient  # noqa: PLC0415  (lazy hardware import)

        self._client = ModbusTcpClient(self.ip, port=self.port)
        if not self._client.connect():
            raise RuntimeError(
                f"[PalmForce] cannot connect to {self.side} hand at {self.ip}:{self.port}"
            )
        print(f"[PalmForce] {self.side} hand connected at {self.ip}:{self.port}")

    def read(self) -> np.ndarray:
        """Return the palm tactile matrix (14x8, float, raw int16 counts)."""
        if self.dry_run:
            return self._synthetic()

        resp = self._client.read_holding_registers(
            PALM_REGISTER, PALM_N_SHORTS, slave=self.device_id
        )
        if resp.isError():
            raise RuntimeError(f"[PalmForce] {self.side} modbus read error: {resp}")
        # Reinterpret the unsigned register words as signed 16-bit ints,
        # exactly as inspire_sdk.read_and_parse_registers does.
        packed = struct.pack(">" + "H" * PALM_N_SHORTS, *resp.registers)
        cells = struct.unpack(">" + "h" * PALM_N_SHORTS, packed)
        return np.asarray(cells, dtype=float).reshape(PALM_SHAPE)

    def close(self) -> None:
        if self._client is not None:
            self._client.close()
            self._client = None

    def _synthetic(self) -> np.ndarray:
        """A drifting Gaussian contact blob on a small noisy rest offset."""
        rng = np.random.default_rng()
        rest = 40.0 + rng.normal(0.0, 3.0, size=PALM_SHAPE)  # nonzero FSR rest offset
        t = time.monotonic() - self._t0
        rows, cols = np.mgrid[0:PALM_SHAPE[0], 0:PALM_SHAPE[1]]
        cr = 6.5 + 4.5 * np.sin(0.6 * t)                     # blob drifts down the palm
        cc = 3.5 + 2.5 * np.cos(0.4 * t)
        amp = 1800.0 * (0.5 + 0.5 * np.sin(0.3 * t)) ** 2    # contact rises and falls
        blob = amp * np.exp(-(((rows - cr) ** 2) / 8.0 + ((cols - cc) ** 2) / 4.0))
        return np.clip(rest + blob, 0.0, None)


def reduce_palm(palm: np.ndarray, baseline: np.ndarray, noise_threshold: float) -> dict:
    """Baseline-subtracted reduction. See docs/palm_force_control.md.

    total  = sum of positive contact  -> candidate grip control signal
    peak   = strongest single cell    -> crush-limit guard
    active = cells above noise floor  -> contact area
    """
    d = np.clip(palm - baseline, 0.0, None)
    return {
        "total": float(d.sum()),
        "peak": float(d.max()),
        "active": int((d > noise_threshold).sum()),
        "delta": d,
    }


def ascii_heatmap(delta: np.ndarray) -> str:
    """Render a baseline-subtracted matrix as an ASCII heatmap (peak-normalized)."""
    peak = delta.max()
    if peak <= 0.0:
        norm = np.zeros_like(delta)
    else:
        norm = delta / peak
    idx = np.clip((norm * (len(_RAMP) - 1)).astype(int), 0, len(_RAMP) - 1)
    lines = ["".join(_RAMP[i] for i in row) for row in idx]
    return "\n".join(f"    |{ln}|" for ln in lines)


def capture_baseline(readers: dict, n_frames: int, rate: float) -> dict:
    """Average n_frames of no-contact readings per side -> per-cell baseline."""
    print(f"[PalmForce] Capturing baseline ({n_frames} frames) -- keep palms UNTOUCHED...")
    accum = {side: np.zeros(PALM_SHAPE) for side in readers}
    period = 1.0 / rate
    for _ in range(n_frames):
        for side, reader in readers.items():
            accum[side] += reader.read()
        time.sleep(period)
    baseline = {side: accum[side] / max(n_frames, 1) for side in readers}
    for side in readers:
        print(f"[PalmForce]   {side} baseline mean={baseline[side].mean():.1f} "
              f"max={baseline[side].max():.1f} counts")
    return baseline


def run_calibration_series(reader: PalmForceReader, baseline: np.ndarray, side: str,
                            *, n_frames: int, rate: float, noise_threshold: float,
                            dry_run: bool) -> None:
    """Interactive multi-point calibration for ONE hand: repeatedly prompts for
    a known weight (kg), captures n_frames of baseline-subtracted summed
    counts under that load, and (once >= 2 points exist) fits
    counts_per_newton via least squares (fit_calibration_series). Blank
    input or Ctrl-C ends collection early with whatever points exist.
    """
    print(f"[PalmForce] Multi-point calibration for {side.upper()} hand.")
    print("[PalmForce] For each point: type the known weight in kg and press "
          "Enter (blank line to finish; need >= 2 points to fit a line).")
    period = 1.0 / rate
    forces_n: list[float] = []
    counts_totals: list[float] = []
    point = 1
    while True:
        try:
            raw = input(f"Point {point} weight (kg): ").strip()
        except (KeyboardInterrupt, EOFError):
            print("\n[PalmForce] Stopping calibration series.")
            break
        if raw == "":
            break
        try:
            weight_kg = float(raw)
            if weight_kg <= 0.0:
                raise ValueError
        except ValueError:
            print(f"[PalmForce]   '{raw}' is not a positive number -- try again.")
            continue

        force_n = weight_kg * STANDARD_GRAVITY
        if not dry_run:
            input(f"[PalmForce] Rest {weight_kg:.3f} kg ({force_n:.3f} N) flat on the "
                  f"palm pad, spread like the real object, then press Enter to capture...")
        total = 0.0
        for _ in range(n_frames):
            r = reduce_palm(reader.read(), baseline, noise_threshold)
            total += r["total"]
            time.sleep(period)
        mean_total = total / max(n_frames, 1)
        forces_n.append(force_n)
        counts_totals.append(mean_total)
        print(f"[PalmForce]   point {point}: {weight_kg:.3f} kg = {force_n:.3f} N -> "
              f"mean_total={mean_total:.1f} counts  (this point alone: "
              f"counts_per_newton={mean_total / force_n:.2f})")
        point += 1

    if len(forces_n) == 0:
        print("[PalmForce] No points collected -- nothing to fit.")
        return
    if len(forces_n) == 1:
        print(f"[PalmForce] Only 1 point collected -- can't fit a line (need >= 2). "
              f"Single-point ratio: counts_per_newton={counts_totals[0] / forces_n[0]:.2f}")
        return

    fit = fit_calibration_series(forces_n, counts_totals)
    print(f"[PalmForce] {side}: fit over {len(forces_n)} points -> "
          f"counts_per_newton={fit['slope']:.2f}  intercept={fit['intercept']:.1f}  "
          f"R^2={fit['r2']:.4f}")
    print("[PalmForce]   per-point ratios: "
          + ", ".join(f"{r:.1f}" for r in fit["per_point_ratio"]))
    if fit["r2"] < 0.98 or abs(fit["intercept"]) > 0.15 * max(counts_totals):
        print("[PalmForce]   NOTE: low R^2 or a large intercept means summed-counts-"
              "proportional-to-force may not hold well at these loads/spread -- see "
              "docs/palm_force_control.md §2 (\"Force, not pressure\").")
    print(f"[PalmForce] Put counts_per_newton={fit['slope']:.2f} into configs/default.yaml "
          f"palm_press.counts_per_newton for the {side} hand.")


def main() -> None:
    parser = argparse.ArgumentParser(description="Read Inspire palm tactile sensor")
    parser.add_argument("--side", choices=["both", "left", "right"], default="both",
                        help="Which hand(s) to read (default: both)")
    parser.add_argument("--dry-run", dest="dry_run", action="store_true", default=True,
                        help="Synthetic data, no hardware (default: True)")
    parser.add_argument("--no-dry-run", dest="dry_run", action="store_false",
                        help="Connect to live hands over Modbus (HUMAN-RUN ONLY)")
    parser.add_argument("--baseline", action="store_true", default=False,
                        help="Capture no-contact baseline frames at start and subtract them")
    parser.add_argument("--baseline-frames", type=int, default=20,
                        help="Number of baseline frames to average (default: 20)")
    parser.add_argument("--raw", action="store_true", default=False,
                        help="Also print the full 14x8 matrix as an ASCII heatmap")
    parser.add_argument("--noise-threshold", type=float, default=10.0,
                        help="Counts above baseline to count a cell as 'active' (default: 10)")
    parser.add_argument("--calibrate", type=float, default=None, metavar="NEWTONS",
                        help="Calibration mode: rest a known static load of NEWTONS on the palm, "
                             "average the baseline-subtracted total counts, and print "
                             "counts_per_newton = mean_total / NEWTONS, then exit. A baseline is "
                             "captured automatically. Calibrate ONE hand at a time (--side left / "
                             "--side right). The load must press NORMAL to the pad (palm face-up) "
                             "and SPREAD across the central palm like the real grasped object "
                             "(summing the 112 cells gives total force only in the linear regime).")
    parser.add_argument("--calibrate-kg", type=float, default=None, metavar="KG",
                        help="Same as --calibrate but the known load is given as a MASS in kg "
                             "(converted to force via g=9.80665). E.g. --calibrate-kg 0.5 -> 4.90 N. "
                             "Mutually exclusive with --calibrate.")
    parser.add_argument("--calibrate-series", action="store_true", default=False,
                        help="Multi-point calibration: interactively prompts for a series of "
                             "known weights (kg), one per Enter press (blank line to finish, "
                             "need >= 2 points), and fits counts_per_newton via least squares "
                             "-- also reports each point's own ratio + R^2/intercept so you can "
                             "see whether the linear (no-area-division) model actually holds "
                             "across the range. Requires a single --side (left or right). "
                             "Mutually exclusive with --calibrate/--calibrate-kg.")
    parser.add_argument("--rate", type=float, default=10.0,
                        help="Read rate in Hz (default: 10)")
    parser.add_argument("--duration", type=float, default=None,
                        help="Seconds to run (default: until Ctrl-C)")
    parser.add_argument("--left-ip", default=DEFAULT_LEFT_IP,
                        help=f"Left hand Modbus IP (default: {DEFAULT_LEFT_IP})")
    parser.add_argument("--right-ip", default=DEFAULT_RIGHT_IP,
                        help=f"Right hand Modbus IP (default: {DEFAULT_RIGHT_IP})")
    args = parser.parse_args()

    # Resolve the known calibration load in Newtons from either --calibrate (N)
    # or --calibrate-kg (mass * g). Mutually exclusive with each other AND
    # with --calibrate-series (single-point vs. multi-point modes).
    if args.calibrate is not None and args.calibrate_kg is not None:
        parser.error("pass only one of --calibrate / --calibrate-kg")
    if args.calibrate_series and (args.calibrate is not None or args.calibrate_kg is not None):
        parser.error("--calibrate-series is mutually exclusive with --calibrate/--calibrate-kg")
    if args.calibrate_series and args.side == "both":
        parser.error("--calibrate-series requires a single --side (left or right) -- "
                      "calibrate one hand at a time")
    force_n = None
    if args.calibrate is not None:
        if args.calibrate <= 0.0:
            parser.error("--calibrate NEWTONS must be positive")
        force_n = args.calibrate
    elif args.calibrate_kg is not None:
        if args.calibrate_kg <= 0.0:
            parser.error("--calibrate-kg KG must be positive")
        force_n = args.calibrate_kg * STANDARD_GRAVITY
    calibrating = force_n is not None or args.calibrate_series

    sides = ["left", "right"] if args.side == "both" else [args.side]
    ips = {"left": args.left_ip, "right": args.right_ip}

    if not args.dry_run:
        print("[PalmForce] WARNING: live mode selected.")
        print("[PalmForce]   - Reads ONLY (no actuation), but connects to the hands over Modbus")
        print("[PalmForce]   - Hands must be powered and reachable on the network")
        input("[PalmForce] Press Enter to continue (Ctrl-C to abort)...")

    readers = {
        side: PalmForceReader(side, ips[side], dry_run=args.dry_run)
        for side in sides
    }

    print(f"[PalmForce] Starting (dry_run={args.dry_run}, sides={sides}, "
          f"register={PALM_REGISTER}, shape={PALM_SHAPE[0]}x{PALM_SHAPE[1]})")

    for reader in readers.values():
        reader.connect()

    if args.baseline or calibrating:
        if calibrating and not args.baseline:
            print("[PalmForce] Calibration needs a baseline to be valid; capturing one now.")
        baseline = capture_baseline(readers, args.baseline_frames, args.rate)
    else:
        baseline = {side: np.zeros(PALM_SHAPE) for side in sides}
        print("[PalmForce] No baseline captured -- totals include the sensor's rest "
              "offset. Use --baseline to subtract it.")

    # --- Calibration mode: average total counts under a known load, print scale ---
    if calibrating:
        if args.calibrate_series:
            side = sides[0]
            run_calibration_series(
                readers[side], baseline[side], side,
                n_frames=args.baseline_frames, rate=args.rate,
                noise_threshold=args.noise_threshold, dry_run=args.dry_run,
            )
            for reader in readers.values():
                reader.close()
            return

        load_desc = f"{force_n:.3f} N"
        if args.calibrate_kg is not None:
            load_desc = f"{args.calibrate_kg:.3f} kg = {force_n:.3f} N"
        n = args.baseline_frames
        period = 1.0 / args.rate
        # Give the user time to place the load AFTER the untouched baseline. Skipped
        # in dry-run so headless/CI runs don't block (mirrors the live-warning gate).
        if not args.dry_run:
            input("[PalmForce] Rest the known load flat on the palm pad (face-up, spread "
                  "across the central palm like the grasped object), then press Enter to "
                  "capture...")
        print(f"[PalmForce] Calibrating against {load_desc}; hold it steady ({n} frames)...")
        totals = {side: 0.0 for side in sides}
        for _ in range(n):
            for side in sides:
                r = reduce_palm(readers[side].read(), baseline[side], args.noise_threshold)
                totals[side] += r["total"]
            time.sleep(period)
        for side in sides:
            mean_total = totals[side] / max(n, 1)
            cpn = mean_total / force_n
            print(f"[PalmForce] {side}: mean_total={mean_total:.1f} counts @ "
                  f"{load_desc}  ->  counts_per_newton = {cpn:.2f}")
        print("[PalmForce] Put counts_per_newton into configs/default.yaml "
              "palm_press.counts_per_newton (calibrate each hand separately). Done.")
        for reader in readers.values():
            reader.close()
        return

    period = 1.0 / args.rate
    t_start = time.monotonic()
    try:
        while True:
            if args.duration is not None and time.monotonic() - t_start >= args.duration:
                break
            parts = []
            heatmaps = []
            for side in sides:
                palm = readers[side].read()
                r = reduce_palm(palm, baseline[side], args.noise_threshold)
                parts.append(f"{side[0].upper()} total={r['total']:8.0f} "
                             f"peak={r['peak']:6.0f} active={r['active']:3d}/112")
                if args.raw:
                    heatmaps.append((side, ascii_heatmap(r["delta"])))
            print("[PalmForce] " + "   ".join(parts))
            for side, hm in heatmaps:
                print(f"  {side} palm (peak-normalized):")
                print(hm)
            time.sleep(period)
    except KeyboardInterrupt:
        print("\n[PalmForce] Ctrl-C -- stopping.")
    finally:
        for reader in readers.values():
            reader.close()

    print("[PalmForce] Done.")


if __name__ == "__main__":
    main()
