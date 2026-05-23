"""One-time URDF import into a running CoppeliaSim instance."""

import os
import sys

_THIS_DIR = os.path.dirname(os.path.abspath(__file__))
_PROJECT_ROOT = os.path.abspath(os.path.join(_THIS_DIR, '..'))
if _PROJECT_ROOT not in sys.path:
    sys.path.insert(0, _PROJECT_ROOT)

from g1_spine_stabilization.sim.scene_setup import import_g1_urdf


if __name__ == '__main__':
    import_g1_urdf()
