"""Standalone IK self-test — FK at default arm config, IK round-trip."""

import os
import sys

import numpy as np

_THIS_DIR = os.path.dirname(os.path.abspath(__file__))
_PROJECT_ROOT = os.path.abspath(os.path.join(_THIS_DIR, '..'))
if _PROJECT_ROOT not in sys.path:
    sys.path.insert(0, _PROJECT_ROOT)

from g1_spine_stabilization.kinematics.ik_solver import (
    G1HandIK,
    L_SHOULDER_ROLL_MIN,
    R_SHOULDER_ROLL_MAX,
)
from g1_spine_stabilization.controllers.pose_tracking import ARM_DEFAULT_Q


def main():
    ik = G1HandIK()

    print("\n--- FK at default arm config ---")
    L_pose, R_pose = ik.fk(ARM_DEFAULT_Q)
    print(f"L_ee position: {np.round(L_pose.translation, 4)}")
    print(f"R_ee position: {np.round(R_pose.translation, 4)}")

    print("\n--- IK solve at default EE positions ---")
    L_test        = np.eye(4)
    L_test[:3, 3] = L_pose.translation
    L_test[:3,:3] = L_pose.rotation
    R_test        = np.eye(4)
    R_test[:3, 3] = R_pose.translation
    R_test[:3,:3] = R_pose.rotation

    sol_q, sol_tau = ik.solve_ik(L_test, R_test)
    print(f"sol_q: {np.round(sol_q, 4)}")
    print(f"  L_shoulder_roll: {sol_q[1]:+.4f}  (bound: >= {L_SHOULDER_ROLL_MIN})")
    print(f"  R_shoulder_roll: {sol_q[8]:+.4f}  (bound: <= {R_SHOULDER_ROLL_MAX})")

    L_actual, R_actual = ik.fk(sol_q)
    print(f"\nPosition errors:")
    print(f"  L: {np.round(L_actual.translation - L_test[:3,3], 4)}")
    print(f"  R: {np.round(R_actual.translation - R_test[:3,3], 4)}")


if __name__ == '__main__':
    main()
