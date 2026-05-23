"""
One-time URDF import into a running CoppeliaSim instance.

Run via `python scripts/setup_scene.py` with CoppeliaSim open.
"""

import os

from coppeliasim_zmqremoteapi_client import RemoteAPIClient


PROJECT_ROOT = os.path.abspath(
    os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..')
)
DEFAULT_URDF_PATH = os.path.join(PROJECT_ROOT, 'assets', 'g1_with_hands.urdf')


def import_g1_urdf(urdf_path=None):
    if urdf_path is None:
        urdf_path = DEFAULT_URDF_PATH
    urdf_path = os.path.abspath(urdf_path)

    client = RemoteAPIClient()
    sim = client.require('sim')
    simURDF = client.require('simURDF')

    print(f"Importing: {urdf_path}")
    result = simURDF.importFile(urdf_path)
    print("Done.")
    return result
