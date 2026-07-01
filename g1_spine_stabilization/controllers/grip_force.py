"""Grip force controller for the Inspire RH56 bimanual hands.

Simple baseline controller: moves fingers to a configurable grasp pose and
holds with light force via TransportGraspController from g1_inspire_sdk.

For bimanual use create one instance per hand from the same config:
    ctrl_l = GraspController(cfg["grip_force"])
    ctrl_r = GraspController(cfg["grip_force"])

Extension point for SDK-5 Phase 2: pass
    tau_des = tau_baseline + k_a * np.linalg.norm(a_arm)
into step() to enable acceleration-coupled grip tightening.
"""

import numpy as np

# Inspire RH56 canonical finger order:
#   [index_mcp, middle_mcp, ring_mcp, little_mcp, thumb_rotate, thumb_flex]
# These values are the SDK's HAND_SOFT_LOWER_LIMITS — the safest open position.
HAND_OPEN_POSE = np.array([0.0, 0.0, 0.0, 0.0, 0.3, 0.14])


class GraspController:
    """Config-driven grip controller for one Inspire RH56 hand.

    Wraps TransportGraspController (close-only, force-feedback grip tightening).
    Call arm() once after fingers have settled at the grasp pose; subsequent
    step() calls run the force-feedback loop. Before arm() is called, step()
    returns close_pose as a static hold (safe at startup).

    Args:
        cfg: dict from config["grip_force"] with keys:
            tau_baseline  – target grip force per finger (N)
            kappa         – TransportGraspController coupling gain
            delta_q_max   – max additional closure beyond grasp pose (rad)
            grasp_pose    – 6-element list, soft-close joint angles (rad)
    """

    def __init__(self, cfg: dict):
        tau_des = float(cfg["tau_baseline"])
        kappa = float(cfg["kappa"])
        delta_q_max = float(cfg["delta_q_max"])
        self._close_pose = np.array(cfg["grasp_pose"], dtype=float)

        # Lazy import keeps the package importable without g1_inspire_sdk installed.
        from g1_inspire_sdk._ik.force_controller import TransportGraspController
        self._ctrl = TransportGraspController(
            n_joints=6,
            tau_des=tau_des,
            kappa=kappa,
            delta_q_max=delta_q_max,
        )
        self._armed = False

    @property
    def open_pose(self) -> np.ndarray:
        """Safe open position for the hand."""
        return HAND_OPEN_POSE.copy()

    @property
    def close_pose(self) -> np.ndarray:
        """Configured grasp pose (soft close)."""
        return self._close_pose.copy()

    def arm(self, q_measured: np.ndarray) -> None:
        """Snapshot the current measured pose as the force-control baseline.

        Call once after fingers have reached close_pose and settled (~0.5 s).
        Until arm() is called, step() returns close_pose as a static hold.
        """
        self._ctrl.set_grasp(q_measured.copy())
        self._armed = True

    def step(
        self,
        force_feedback: np.ndarray,
        q_measured: np.ndarray,
        dt: float,
        tau_des: float | None = None,
    ) -> np.ndarray:
        """Return q_cmd for one hand.

        Args:
            force_feedback: 6-element per-finger force from
                hand.get_state(side)["force"] (zeros in dry-run).
            q_measured:    6-element current finger position from
                hand.get_state(side)["q"].
            dt:            timestep (s).
            tau_des:       override the baseline force target (N per finger).
                           Pass tau_baseline + k_a*||a_arm|| for acceleration
                           coupling. If None, uses the value set at construction.

        Returns:
            6-element q_cmd array.
        """
        if not self._armed:
            return self._close_pose.copy()

        if tau_des is not None:
            self._ctrl.tau_des = tau_des

        return self._ctrl.step(force_feedback, self._close_pose, q_measured, dt)

    def reset(self) -> None:
        """Clear armed state and delta history. Call before re-grasping."""
        self._ctrl.reset()
        self._armed = False
