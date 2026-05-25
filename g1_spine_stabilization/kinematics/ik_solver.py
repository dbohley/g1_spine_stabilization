"""
G1 Arm IK Solver using Pinocchio 4 + CasADi

EE frames are defined in g1_with_hands.urdf as fixed joints
attached to the hand base links.

Self-collision avoidance via direct shoulder_roll bounds:
  - Left shoulder roll cannot go below 0.15 rad
  - Right shoulder roll cannot go above -0.15 rad
This prevents the arms from collapsing inward into the torso.
"""

import os
import numpy as np
import pinocchio as pin
from pinocchio import casadi as cpin
import casadi


# -------------------------------------------------------
# Smoothing filter
# -------------------------------------------------------
class WeightedMovingFilter:
    """Weighted moving average filter for smoothing joint commands."""

    def __init__(self, weights, data_size=14):
        self.weights     = np.array(weights)
        self.window_size = len(weights)
        self.data_size   = data_size
        self.queue       = []
        self.filtered    = np.zeros(data_size)
        assert np.isclose(np.sum(self.weights), 1.0), \
            "Weights must sum to 1.0"

    def add_data(self, new_data):
        if len(self.queue) >= self.window_size:
            self.queue.pop(0)
        self.queue.append(np.array(new_data))
        if len(self.queue) < self.window_size:
            self.filtered = self.queue[-1].copy()
        else:
            self.filtered = sum(
                w * d for w, d in zip(self.weights, self.queue)
            )

    @property
    def filtered_data(self):
        return self.filtered


# -------------------------------------------------------
# Self-collision avoidance bounds
# Indices in reduced model:
#   0=L_shoulder_pitch, 1=L_shoulder_roll, 2=L_shoulder_yaw, 3=L_elbow
#   4=L_wrist_roll,     5=L_wrist_pitch,   6=L_wrist_yaw
#   7=R_shoulder_pitch, 8=R_shoulder_roll, 9=R_shoulder_yaw, 10=R_elbow
#   11=R_wrist_roll,    12=R_wrist_pitch,  13=R_wrist_yaw
# -------------------------------------------------------
L_SHOULDER_ROLL_MIN =  0.15   # left can't go below this (inward = bad)
R_SHOULDER_ROLL_MAX = -0.15   # right can't go above this (inward = bad)


# -------------------------------------------------------
# IK Solver
# -------------------------------------------------------
class G1HandIK:
    def __init__(self, urdf_path=None, visualization=False):
        np.set_printoptions(precision=5, suppress=True, linewidth=200)

        # -------------------------------------------------------
        # Resolve URDF path
        # -------------------------------------------------------
        # ik_solver.py lives at <project_root>/g1_spine_stabilization/kinematics/,
        # so the project root is two levels up.
        if urdf_path is None:
            project_root = os.path.abspath(
                os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..')
            )
            urdf_path    = os.path.join(project_root, 'assets', 'g1_with_hands.urdf')
        elif not os.path.isabs(urdf_path):
            project_root = os.path.abspath(
                os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..')
            )
            urdf_path    = os.path.join(project_root, urdf_path)

        urdf_dir = os.path.dirname(urdf_path)
        print(f"[G1HandIK] Loading URDF from: {urdf_path}")

        # -------------------------------------------------------
        # Load full robot model
        # -------------------------------------------------------
        self.robot = pin.RobotWrapper.BuildFromURDF(
            urdf_path,
            [urdf_dir],
        )
        print(f"[G1HandIK] Full model DOF: {self.robot.model.nq}")

        # -------------------------------------------------------
        # Lock everything except the 14 arm joints
        # -------------------------------------------------------
        joints_to_lock = [
            # Legs
            'left_hip_pitch_joint',    'left_hip_roll_joint',
            'left_hip_yaw_joint',      'left_knee_joint',
            'left_ankle_pitch_joint',  'left_ankle_roll_joint',
            'right_hip_pitch_joint',   'right_hip_roll_joint',
            'right_hip_yaw_joint',     'right_knee_joint',
            'right_ankle_pitch_joint', 'right_ankle_roll_joint',
            # Waist
            'waist_yaw_joint', 'waist_roll_joint', 'waist_pitch_joint',
            # Left hand fingers
            'L_hand_index_proximal_joint',
            'L_hand_index_intermediate_joint',
            'L_hand_middle_proximal_joint',
            'L_hand_middle_intermediate_joint',
            'L_hand_pinky_proximal_joint',
            'L_hand_pinky_intermediate_joint',
            'L_hand_ring_proximal_joint',
            'L_hand_ring_intermediate_joint',
            'L_hand_thumb_proximal_yaw_joint',
            'L_hand_thumb_proximal_pitch_joint',
            'L_hand_thumb_intermediate_joint',
            'L_hand_thumb_distal_joint',
            # Right hand fingers
            'R_hand_index_proximal_joint',
            'R_hand_index_intermediate_joint',
            'R_hand_middle_proximal_joint',
            'R_hand_middle_intermediate_joint',
            'R_hand_pinky_proximal_joint',
            'R_hand_pinky_intermediate_joint',
            'R_hand_ring_proximal_joint',
            'R_hand_ring_intermediate_joint',
            'R_hand_thumb_proximal_yaw_joint',
            'R_hand_thumb_proximal_pitch_joint',
            'R_hand_thumb_intermediate_joint',
            'R_hand_thumb_distal_joint',
        ]

        print("[G1HandIK] Building reduced model...")
        self.reduced = self.robot.buildReducedRobot(
            list_of_joints_to_lock=joints_to_lock,
            reference_configuration=np.zeros(self.robot.model.nq),
        )

        print(f"[G1HandIK] Reduced model DOF: {self.reduced.model.nq}")
        print("[G1HandIK] Active joints:")
        for i, name in enumerate(self.reduced.model.names):
            print(f"  {i}: {name}")

        # -------------------------------------------------------
        # Look up EE frames from URDF
        # -------------------------------------------------------
        self.L_ee_id = self.reduced.model.getFrameId('L_ee_link')
        self.R_ee_id = self.reduced.model.getFrameId('R_ee_link')

        n_frames = len(self.reduced.model.frames)
        if self.L_ee_id >= n_frames:
            raise RuntimeError("L_ee_link frame not found in reduced model.")
        if self.R_ee_id >= n_frames:
            raise RuntimeError("R_ee_link frame not found in reduced model.")

        print(f"[G1HandIK] L_ee_link frame id: {self.L_ee_id}")
        print(f"[G1HandIK] R_ee_link frame id: {self.R_ee_id}")

        # Print EE frame orientations at zero config
        q_zero = np.zeros(self.reduced.model.nq)
        data   = self.reduced.model.createData()
        pin.framesForwardKinematics(self.reduced.model, data, q_zero)

        print("\n[G1HandIK] EE frame orientations at zero config:")
        for name, fid in [('L_ee', self.L_ee_id), ('R_ee', self.R_ee_id)]:
            R   = data.oMf[fid].rotation
            pos = data.oMf[fid].translation
            print(f"  {name}:")
            print(f"    Position:       {np.round(pos, 4)}")
            print(f"    X (approach):   {np.round(R[:,0], 3)}")
            print(f"    Y (palm normal):{np.round(R[:,1], 3)}")
            print(f"    Z (fingers):    {np.round(R[:,2], 3)}")

        # -------------------------------------------------------
        # Build CasADi symbolic model
        # -------------------------------------------------------
        print("\n[G1HandIK] Building CasADi model...")
        self.cmodel = cpin.Model(self.reduced.model)
        self.cdata  = self.cmodel.createData()

        nq         = self.reduced.model.nq
        self.cq    = casadi.SX.sym('q',    nq, 1)
        self.cTf_l = casadi.SX.sym('tf_l', 4,  4)
        self.cTf_r = casadi.SX.sym('tf_r', 4,  4)

        cpin.framesForwardKinematics(self.cmodel, self.cdata, self.cq)

        self.translational_error = casadi.Function(
            'translational_error',
            [self.cq, self.cTf_l, self.cTf_r],
            [casadi.vertcat(
                self.cdata.oMf[self.L_ee_id].translation - self.cTf_l[:3, 3],
                self.cdata.oMf[self.R_ee_id].translation - self.cTf_r[:3, 3],
            )]
        )

        self.rotational_error = casadi.Function(
            'rotational_error',
            [self.cq, self.cTf_l, self.cTf_r],
            [casadi.vertcat(
                cpin.log3(
                    self.cdata.oMf[self.L_ee_id].rotation @ self.cTf_l[:3, :3].T
                ),
                cpin.log3(
                    self.cdata.oMf[self.R_ee_id].rotation @ self.cTf_r[:3, :3].T
                ),
            )]
        )

        # -------------------------------------------------------
        # IPOPT optimization problem
        # -------------------------------------------------------
        self.opti       = casadi.Opti()
        self.var_q      = self.opti.variable(nq)
        self.var_q_last = self.opti.parameter(nq)
        self.param_tf_l = self.opti.parameter(4, 4)
        self.param_tf_r = self.opti.parameter(4, 4)

        trans_cost  = casadi.sumsqr(
            self.translational_error(self.var_q, self.param_tf_l, self.param_tf_r)
        )
        rot_cost    = casadi.sumsqr(
            self.rotational_error(self.var_q, self.param_tf_l, self.param_tf_r)
        )
        reg_cost    = casadi.sumsqr(self.var_q)
        smooth_cost = casadi.sumsqr(self.var_q - self.var_q_last)

        # -------------------------------------------------------
        # Joint limits with tightened shoulder_roll bounds
        # for self-collision avoidance
        # -------------------------------------------------------
        lower = self.reduced.model.lowerPositionLimit.copy()
        upper = self.reduced.model.upperPositionLimit.copy()

        # Tighten shoulder roll bounds
        print(f"\n[G1HandIK] Shoulder roll bounds:")
        print(f"  Original L_shoulder_roll: [{lower[1]:.3f}, {upper[1]:.3f}]")
        print(f"  Original R_shoulder_roll: [{lower[8]:.3f}, {upper[8]:.3f}]")

        lower[1] = L_SHOULDER_ROLL_MIN  # left: don't go inward below this
        upper[8] = R_SHOULDER_ROLL_MAX  # right: don't go inward above this

        print(f"  Tightened L_shoulder_roll: [{lower[1]:.3f}, {upper[1]:.3f}]")
        print(f"  Tightened R_shoulder_roll: [{lower[8]:.3f}, {upper[8]:.3f}]")

        self.opti.subject_to(
            self.opti.bounded(lower, self.var_q, upper)
        )

        # Cost weights:
        #  50.0 * trans   — position accuracy
        #   5.0 * rot     — orientation
        #   0.02 * reg    — pull toward zero
        #   0.1 * smooth  — smoothness between timesteps
        self.opti.minimize(
            50.0  * trans_cost
          +  5.0  * rot_cost
          +  0.02 * reg_cost
          +  0.1  * smooth_cost
        )

        opts = {
            'expand':                True,
            'detect_simple_bounds':  True,
            'calc_lam_p':            False,
            'print_time':            False,
            'ipopt.sb':              'yes',
            'ipopt.print_level':     0,
            'ipopt.max_iter':        30,
            'ipopt.tol':             1e-4,
            'ipopt.acceptable_tol':  5e-4,
            'ipopt.acceptable_iter': 5,
            'ipopt.warm_start_init_point': 'yes',
        }
        self.opti.solver('ipopt', opts)

        self.init_data     = np.zeros(nq)
        self.smooth_filter = WeightedMovingFilter(
            np.array([0.4, 0.3, 0.2, 0.1]), 14
        )
        print("[G1HandIK] Ready.")

    def solve_ik(self, L_target, R_target, current_q=None, current_dq=None):
        """
        Solve IK for both arms simultaneously.

        Args:
            L_target:   4x4 SE3 left EE target in robot base/pelvis frame
            R_target:   4x4 SE3 right EE target in robot base/pelvis frame
            current_q:  current joint positions (14,) — used as warm start
            current_dq: current joint velocities (14,)

        Returns:
            sol_q:     solved joint angles (14,) in reduced model order:
                       [L_shoulder_pitch, L_shoulder_roll, L_shoulder_yaw,
                        L_elbow, L_wrist_roll, L_wrist_pitch, L_wrist_yaw,
                        R_shoulder_pitch, R_shoulder_roll, R_shoulder_yaw,
                        R_elbow, R_wrist_roll, R_wrist_pitch, R_wrist_yaw]
            sol_tauff: feedforward torques (14,)
        """
        if current_q is not None:
            self.init_data = current_q.copy()

        self.opti.set_initial(self.var_q,    self.init_data)
        self.opti.set_value(self.param_tf_l, L_target)
        self.opti.set_value(self.param_tf_r, R_target)
        self.opti.set_value(self.var_q_last, self.init_data)

        try:
            self.opti.solve()
            sol_q = self.opti.value(self.var_q)
        except Exception as e:
            print(f"[G1HandIK] IK warning: {e}")
            sol_q = self.opti.debug.value(self.var_q)

        self.smooth_filter.add_data(sol_q)
        sol_q          = self.smooth_filter.filtered_data.copy()
        self.init_data = sol_q.copy()

        v = np.zeros(self.reduced.model.nv) if current_dq is None \
            else current_dq * 0.0

        sol_tauff = pin.rnea(
            self.reduced.model,
            self.reduced.data,
            sol_q, v,
            np.zeros(self.reduced.model.nv),
        )

        return sol_q, sol_tauff

    def fk(self, q):
        """Forward kinematics — returns EE poses for a given joint config."""
        data = self.reduced.model.createData()
        pin.framesForwardKinematics(self.reduced.model, data, q)
        return data.oMf[self.L_ee_id], data.oMf[self.R_ee_id]

    def get_full_model_and_data(self):
        """Expose the un-reduced (53-DOF) Pinocchio model + a fresh data object.

        Needed by the perception module to compute FK of frames that the
        reduced model does not animate (e.g. d435_link via waist joints).
        """
        return self.robot.model, self.robot.model.createData()