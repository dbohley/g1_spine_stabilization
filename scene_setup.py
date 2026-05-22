# Simplified scene_setup.py
from coppeliasim_zmqremoteapi_client import RemoteAPIClient
import os

client = RemoteAPIClient()
sim = client.require('sim')
simURDF = client.require('simURDF')

urdf_path = os.path.abspath("g1_with_hands.urdf")
print(f"Importing: {urdf_path}")
result = simURDF.importFile(urdf_path)
print(f"Done.")