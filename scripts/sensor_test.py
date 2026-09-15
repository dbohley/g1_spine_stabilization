#!/usr/bin/env python3
"""Standalone ATI Axia80 F/T sensor smoke test.

Streams live readings and prints them, so you can verify connectivity and
calibration (CPF/CPT) before wiring the sensor into a recording run. Uses the
SAME ForceTorqueSensor reader as the recording scripts
(g1_spine_stabilization/sensors/force_torque.py) -- single source of truth for
the RDT/UDP protocol.

This is a passive network read (no robot actuation), so it's safe to run
directly. If the sensor is unreachable on this machine, the reader prints a
diagnostic pointing at docs/force_sensor_setup.md.

  python scripts/sensor_test.py                 # ip/cpf/cpt from configs/default.yaml
  python scripts/sensor_test.py --ip 192.168.1.1 --cpf 1000000 --cpt 1000000
"""
import argparse
import os
import sys
import time

_THIS_DIR = os.path.dirname(os.path.abspath(__file__))
_PROJECT_ROOT = os.path.abspath(os.path.join(_THIS_DIR, ".."))
if _PROJECT_ROOT not in sys.path:
    sys.path.insert(0, _PROJECT_ROOT)

from g1_spine_stabilization.sensors.force_torque import ForceTorqueSensor
from g1_spine_stabilization.utils.config import load_config


def main():
    cfg_fs = load_config().get("force_sensor", {})
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--ip", default=cfg_fs.get("ip", "192.168.1.1"),
                        help="Sensor IP (default from config).")
    parser.add_argument("--port", type=int, default=cfg_fs.get("port", 49152))
    parser.add_argument("--cpf", type=float, default=cfg_fs.get("cpf", 1000000.0),
                        help="Counts per force (from the sensor web UI).")
    parser.add_argument("--cpt", type=float, default=cfg_fs.get("cpt", 1000000.0),
                        help="Counts per torque.")
    parser.add_argument("--rate", type=float, default=10.0,
                        help="Print rate in Hz (default 10).")
    args = parser.parse_args()

    sensor = ForceTorqueSensor(args.ip, args.port, args.cpf, args.cpt)
    sensor.start()
    print("[sensor_test] Streaming... Ctrl+C to stop.")
    period = 1.0 / max(args.rate, 0.1)
    try:
        while True:
            w = sensor.read_raw()
            if w is None:
                print("[sensor_test] (no reading)")
            else:
                print("[sensor_test] F=[{:+.3f} {:+.3f} {:+.3f}] N  "
                      "T=[{:+.4f} {:+.4f} {:+.4f}] Nm".format(*w))
            time.sleep(period)
    except KeyboardInterrupt:
        print("\n[sensor_test] Stopping.")
    finally:
        sensor.close()


if __name__ == "__main__":
    main()
