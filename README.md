# G1 Spine Stabilization

A control pipeline for the Unitree G1 humanoid that performs bimanual
rigid-body pose tracking: the robot holds an object in both hands and matches
its pose to a moving target reference. Development happens in CoppeliaSim
simulation today, with transfer to the real G1 as the longer-term goal. The
current sim baseline drives a single-axis (X) rotation with a sine wave as a
placeholder; ArUco-based perception is planned for an upcoming session.

## Prerequisites

- Python 3.10+
- [Conda](https://docs.conda.io/) (Pinocchio is installed from conda-forge,
  not pip)
- [CoppeliaSim 4.x](https://www.coppeliarobotics.com/) for the simulation
  scenes

## Setup

```bash
git clone <repo-url> g1_spine_stabilization
cd g1_spine_stabilization

# Pinocchio must come from conda-forge — the PyPI package named `pinocchio`
# is unrelated and will NOT work.
conda create -n g1 python=3.10
conda activate g1
conda install pinocchio -c conda-forge

pip install -r requirements.txt
pip install -e .
```

## Running the simulation

1. Open CoppeliaSim with an empty scene.
2. (First time only) import the URDF:
   ```bash
   python scripts/setup_scene.py
   ```
3. Run the controller:
   ```bash
   python scripts/run_pose_tracking_sim.py
   ```

   Pass `--config path/to/other.yaml` to use a non-default tuning.

## Verifying the kinematics standalone (no sim needed)

```bash
python scripts/verify_ik.py
```

Loads the URDF via Pinocchio, runs FK at the default arm config, solves IK
for that same target, and prints the position errors.

## Running tests

```bash
pytest tests/
```

The test suite is pure Pinocchio / NumPy / OpenCV — no CoppeliaSim or
RealSense hardware required. Perception tests synthesize AprilTag images
in-memory via `cv2.aruco.generateImageMarker`.

## Perception (AprilTag → pelvis-frame pose)

The `g1_spine_stabilization/perception/` module turns RGB frames from the
head-mounted RealSense D435 into 6-DoF marker poses expressed in the
robot's pelvis frame. It's built as three composable layers:

- **Frame sources** (`frame_sources.py`): `ImageFileSource`,
  `VideoFileSource`, `RealSenseSource`. `RealSenseSource` lazy-imports
  `pyrealsense2` so this package stays importable on machines without
  librealsense installed.
- **Detector** (`detector.py`): wraps `cv2.aruco.ArucoDetector` +
  `cv2.solvePnP` (planar `SOLVEPNP_IPPE_SQUARE`) to recover SE3 pose in
  the camera optical frame plus a per-detection reprojection error.
- **Tracker** (`tracker.py`): composes a source + detector + the
  camera→pelvis transform chain. `MockMarkerTracker` returns scripted
  observations for hardware-free testing.

### Visualizing detections against real hardware

```bash
python scripts/visualize_markers.py --source realsense
```

Streams color frames, draws axes on detected markers, and prints
pelvis-frame positions per detection. Press `q` to quit.

### Mock mode (no camera attached)

```bash
python scripts/visualize_markers.py --source mock
```

Emits three scripted observations and exits — useful for smoke-testing
imports and the config plumbing without any hardware.

### Important install notes

- `opencv-contrib-python` is in `requirements.txt` (`cv2.aruco` lives in
  the contrib build).
- **`pyrealsense2` is NOT in `requirements.txt`.** It must be installed
  separately from conda-forge:

  ```bash
  conda install -c conda-forge pyrealsense2
  ```

- The `T_optical_to_d435` transform in `configs/default.yaml` ships with
  the standard ROS optical-frame convention (`-90°` Z then `-90°` X) as
  a placeholder. **It needs calibration against the real robot the first
  time the perception pipeline is wired into the controller.**

## Folder structure

```
g1_spine_stabilization/
├── assets/                       # URDFs and meshes (do not edit)
├── configs/default.yaml          # tunable runtime parameters
├── g1_spine_stabilization/       # the package
│   ├── kinematics/               # G1HandIK (Pinocchio + CasADi)
│   ├── controllers/              # pose-tracking, grip-force (stub)
│   ├── trajectory/               # hand-target geometry + drivers
│   ├── perception/               # placeholder (ArUco — Session 2)
│   ├── sim/                      # CoppeliaSim helpers
│   └── utils/                    # SE3 math
├── scripts/                      # entry-point scripts
├── tests/                        # pytest suite
└── docs/                         # architecture + frame conventions
```

## For developers and AI-assisted work

The authoritative project context for Claude Code (and humans new to the
repo) lives in [`CLAUDE.md`](CLAUDE.md). Read it before making non-trivial
changes — it documents the hard rules, frame conventions, and known gotchas.

## Status

This is an internal research repo. The sim baseline is working; perception,
grip control, and real-robot transfer are upcoming sessions.
