#!/usr/bin/env python3
"""Plot a recorded trial's data (robot or human) over time.

Reads a .pkl written by RunRecorder (data_recording/recorder.py) -- from
either scripts/record_human_trial.py or run_pose_tracking_preview.py's
--record -- and saves one PNG per group of related fields: marker
positions, angle_diff_rpy, angular velocity magnitude, F/T wrench
magnitudes, the sin-model comparison (the basis for load_cell_wrench_
grav_comp -- see docs/gravity_comp_recording_plan.md), and reprojection
error.

Every field is None-safe: a field that's entirely None for this trial
(e.g. `theta` for a human trial, or the F/T fields when no sensor was
connected) is skipped with a printed note rather than producing an empty
or crashing plot -- so this script works unmodified on any schema-v3
pickle regardless of origin/source.

No robot, no camera, no actuation -- pure offline analysis of an already-
recorded file. Safe to run directly.
"""
import argparse
import glob
import os
import pickle
import sys

_THIS_DIR = os.path.dirname(os.path.abspath(__file__))
_PROJECT_ROOT = os.path.abspath(os.path.join(_THIS_DIR, ".."))
if _PROJECT_ROOT not in sys.path:
    sys.path.insert(0, _PROJECT_ROOT)

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

_DEFAULT_RUNS_DIR = os.path.join(_PROJECT_ROOT, "data", "runs")


def _newest_pickle(runs_dir):
    candidates = glob.glob(os.path.join(runs_dir, "*.pkl"))
    if not candidates:
        raise FileNotFoundError(f"No .pkl files found under {runs_dir}")
    return max(candidates, key=os.path.getmtime)


def _series(samples, attr, component=None):
    """(N,) float array for `attr` across samples, NaN where that sample's
    field was None -- so matplotlib draws a gap instead of crashing.
    `component` is an optional callable applied to a non-None value (e.g.
    `lambda pose: pose[0, 3]` to pull one coordinate out of a 4x4 pose)."""
    out = np.full(len(samples), np.nan, dtype=float)
    for i, s in enumerate(samples):
        val = getattr(s, attr)
        if val is None:
            continue
        out[i] = component(val) if component is not None else val
    return out


def _has_data(*series_list):
    return any(np.any(~np.isnan(s)) for s in series_list)


def _save(fig, output_dir, filename):
    path = os.path.join(output_dir, filename)
    fig.savefig(path, dpi=150, bbox_inches="tight")
    plt.close(fig)
    print(f"[plot_recorded_run] Wrote {path}")
    return path


def plot_positions(samples, t, output_dir, show):
    obj = [_series(samples, "object_pose", lambda p, i=i: p[i, 3]) for i in range(3)]
    tgt = [_series(samples, "target_pose", lambda p, i=i: p[i, 3]) for i in range(3)]
    if not _has_data(*obj, *tgt):
        print("[plot_recorded_run] positions: no data -- skipped")
        return None

    fig, axes = plt.subplots(3, 1, figsize=(9, 8), sharex=True)
    for ax, label, o, g in zip(axes, ["X (m)", "Y (m)", "Z (m)"], obj, tgt):
        ax.plot(t, o, label="object")
        ax.plot(t, g, label="target")
        ax.set_ylabel(label)
        ax.legend(loc="upper right")
        ax.grid(True, alpha=0.3)
    axes[-1].set_xlabel("t (s)")
    fig.suptitle("Marker position (pelvis frame)")
    path = _save(fig, output_dir, "positions.png")
    if show:
        plt.show()
    return path


def plot_angle_diff_rpy(samples, t, output_dir, show):
    rpy_deg = [_series(samples, "angle_diff_rpy", lambda a, i=i: np.degrees(a[i])) for i in range(3)]
    if not _has_data(*rpy_deg):
        print("[plot_recorded_run] angle_diff_rpy: no data -- skipped")
        return None

    fig, axes = plt.subplots(3, 1, figsize=(9, 8), sharex=True)
    for ax, label, series in zip(axes, ["roll (deg)", "pitch (deg)", "yaw (deg)"], rpy_deg):
        ax.plot(t, series, color="tab:purple")
        ax.set_ylabel(label)
        ax.grid(True, alpha=0.3)
    axes[-1].set_xlabel("t (s)")
    fig.suptitle("angle_diff_rpy -- object relative to target's own frame\n"
                 "(NOT a symmetric difference -- see data_recording_conventions.md)")
    path = _save(fig, output_dir, "angle_diff_rpy.png")
    if show:
        plt.show()
    return path


def plot_angular_velocity(samples, t, output_dir, show):
    obj_norm = _series(samples, "object_angular_velocity", lambda v: np.linalg.norm(v))
    tgt_norm = _series(samples, "target_angular_velocity", lambda v: np.linalg.norm(v))
    if not _has_data(obj_norm, tgt_norm):
        print("[plot_recorded_run] angular_velocity: no data -- skipped")
        return None

    fig, ax = plt.subplots(figsize=(9, 4))
    ax.plot(t, obj_norm, label="|object_angular_velocity|")
    ax.plot(t, tgt_norm, label="|target_angular_velocity|")
    ax.set_xlabel("t (s)")
    ax.set_ylabel("rad/s")
    ax.set_title("Angular velocity magnitude (camera optical frame -- magnitude only, see docs)")
    ax.legend(loc="upper right")
    ax.grid(True, alpha=0.3)
    path = _save(fig, output_dir, "angular_velocity.png")
    if show:
        plt.show()
    return path


def plot_load_cell_wrench(samples, t, output_dir, show):
    force = {
        "raw": _series(samples, "load_cell_wrench_raw", lambda w: np.linalg.norm(w[:3])),
        "grav_comp": _series(samples, "load_cell_wrench_grav_comp", lambda w: np.linalg.norm(w[:3])),
        "tared": _series(samples, "load_cell_wrench_tared", lambda w: np.linalg.norm(w[:3])),
    }
    torque = {
        "raw": _series(samples, "load_cell_wrench_raw", lambda w: np.linalg.norm(w[3:])),
        "grav_comp": _series(samples, "load_cell_wrench_grav_comp", lambda w: np.linalg.norm(w[3:])),
        "tared": _series(samples, "load_cell_wrench_tared", lambda w: np.linalg.norm(w[3:])),
    }
    if not _has_data(*force.values(), *torque.values()):
        print("[plot_recorded_run] load_cell_wrench: no data (F/T sensor absent/disabled) -- skipped")
        return None

    fig, axes = plt.subplots(2, 1, figsize=(9, 7), sharex=True)
    for name, series in force.items():
        axes[0].plot(t, series, label=name)
    axes[0].set_ylabel("Force magnitude (N)")
    axes[0].legend(loc="upper right")
    axes[0].grid(True, alpha=0.3)
    for name, series in torque.items():
        axes[1].plot(t, series, label=name)
    axes[1].set_ylabel("Torque magnitude (N*m)")
    axes[1].set_xlabel("t (s)")
    axes[1].legend(loc="upper right")
    axes[1].grid(True, alpha=0.3)
    fig.suptitle("Load cell wrench magnitude: raw vs. grav_comp vs. tared")
    path = _save(fig, output_dir, "load_cell_wrench.png")
    if show:
        plt.show()
    return path


def plot_sin_model(samples, t, output_dir, show):
    theta_deg = _series(samples, "sin_model_theta_rad", lambda v: np.degrees(v))
    sin_force = _series(samples, "sin_model_force_corrected")
    tared_force = _series(samples, "load_cell_wrench_tared", lambda w: np.linalg.norm(w[:3]))
    if not _has_data(theta_deg, sin_force):
        print("[plot_recorded_run] sin_model: no data -- skipped")
        return None

    fig, axes = plt.subplots(2, 1, figsize=(9, 7), sharex=True)
    axes[0].plot(t, theta_deg, color="tab:orange")
    axes[0].set_ylabel("theta (deg)")
    axes[0].grid(True, alpha=0.3)
    axes[1].plot(t, tared_force, label="tared force magnitude")
    axes[1].plot(t, sin_force, label="sin_model_force_corrected", linestyle="--")
    axes[1].set_ylabel("Force (N)")
    axes[1].set_xlabel("t (s)")
    axes[1].legend(loc="upper right")
    axes[1].grid(True, alpha=0.3)
    fig.suptitle("sin-model gravity-comp correction (primary basis for "
                 "load_cell_wrench_grav_comp -- see docs/gravity_comp_recording_plan.md)")
    path = _save(fig, output_dir, "sin_model.png")
    if show:
        plt.show()
    return path


def plot_reprojection_error(samples, t, output_dir, show):
    obj_err = _series(samples, "object_reprojection_error")
    tgt_err = _series(samples, "target_reprojection_error")
    if not _has_data(obj_err, tgt_err):
        print("[plot_recorded_run] reprojection_error: no data -- skipped")
        return None

    fig, ax = plt.subplots(figsize=(9, 4))
    ax.plot(t, obj_err, label="object")
    ax.plot(t, tgt_err, label="target")
    ax.set_xlabel("t (s)")
    ax.set_ylabel("reprojection error (px)")
    ax.set_title("AprilTag solvePnP reprojection error (data-quality diagnostic)")
    ax.legend(loc="upper right")
    ax.grid(True, alpha=0.3)
    path = _save(fig, output_dir, "reprojection_error.png")
    if show:
        plt.show()
    return path


def plot_theta(samples, t, output_dir, show):
    theta = _series(samples, "theta")
    if not _has_data(theta):
        print("[plot_recorded_run] theta: no data (robot-only field; None for a human "
              "trial or a non-marker-backed source) -- skipped")
        return None

    fig, ax = plt.subplots(figsize=(9, 4))
    ax.plot(t, np.degrees(theta), color="tab:green")
    ax.set_xlabel("t (s)")
    ax.set_ylabel("theta (deg)")
    ax.set_title("theta fed to compute_hand_targets (robot-only)")
    ax.grid(True, alpha=0.3)
    path = _save(fig, output_dir, "theta.png")
    if show:
        plt.show()
    return path


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--pickle", default=None, metavar="PATH",
        help="Path to a RunRecorder .pkl file. Default: the most recently "
             "modified .pkl under data/runs/.",
    )
    parser.add_argument(
        "--output-dir", default=None, metavar="DIR",
        help="Where to save the PNGs. Default: '<pickle stem>_plots/' next "
             "to the pickle file.",
    )
    parser.add_argument(
        "--show", action="store_true",
        help="Also display each figure interactively (in addition to saving).",
    )
    args = parser.parse_args()

    pickle_path = args.pickle or _newest_pickle(_DEFAULT_RUNS_DIR)
    print(f"[plot_recorded_run] Loading {pickle_path}")
    with open(pickle_path, "rb") as f:
        payload = pickle.load(f)

    samples = payload["samples"]
    print(f"[plot_recorded_run] schema_version={payload['schema_version']} "
          f"origin={payload['origin']} source_label={payload['source_label']} "
          f"tracking_mode={payload['tracking_mode']}  ({len(samples)} samples)")
    if not samples:
        print("[plot_recorded_run] No samples in this file -- nothing to plot.")
        return

    t = np.array([s.t for s in samples], dtype=float)

    if args.output_dir is not None:
        output_dir = args.output_dir
    else:
        stem = os.path.splitext(os.path.basename(pickle_path))[0]
        output_dir = os.path.join(os.path.dirname(os.path.abspath(pickle_path)), f"{stem}_plots")
    os.makedirs(output_dir, exist_ok=True)

    plotters = [
        plot_positions,
        plot_angle_diff_rpy,
        plot_angular_velocity,
        plot_load_cell_wrench,
        plot_sin_model,
        plot_reprojection_error,
        plot_theta,
    ]
    written = [p for p in (fn(samples, t, output_dir, args.show) for fn in plotters) if p is not None]
    print(f"[plot_recorded_run] Wrote {len(written)} plot(s) to {output_dir}")


if __name__ == "__main__":
    main()
