# Copyright (c) 2022-2025, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Gripper with pressure-based control for pneumatic actuation simulation."""

from __future__ import annotations

import torch
from typing import Sequence, TYPE_CHECKING

import isaaclab.sim as sim_utils
import isaaclab.utils.math as math_utils
from isaaclab.actuators import ImplicitActuatorCfg
from isaaclab.assets.articulation import Articulation, ArticulationCfg
from isaaclab.utils import configclass


if TYPE_CHECKING:
    from .gripper_cfg import GripperCfg



class Gripper(Articulation):
    """Gripper with pressure-based control.
    
    Extends Articulation to add pressure control that maps pressure values
    to gripper gap width using a PD controller for force compliance.
    """
    
    cfg: GripperCfg
    """Configuration for the gripper."""
    
    def __init__(self, cfg: GripperCfg):
        """Initialize the gripper.
        
        Args:
            cfg: Configuration for the gripper.
        """
        super().__init__(cfg)
        
        # Create buffer for target pressure [num_envs]
        self._target_pressure = None  # Will be initialized after parent init
    
    """
    Properties
    """
    
    @property
    def target_pressure(self) -> torch.Tensor:
        """Target pressure for each environment. Shape: (num_envs,)"""
        return self._target_pressure

    @property
    def gap(self) -> torch.Tensor:
        """Gap for each gripper finger (distance from root projected onto root y-axis). Shape: (num_envs, 2)"""
        return self._compute_gap()

    @property
    def gap_velocity(self) -> torch.Tensor:
        """Gap velocity magnitude for each gripper finger. Shape: (num_envs, 2)"""
        return self._compute_gap_velocity()
    
    """
    Operations - Setters
    """
    
    def set_pressure(self, pressure: torch.Tensor, env_ids: Sequence[int] | None = None):
        """Set target pressure for the gripper.
        
        This updates the internal pressure buffer. The actual forces are computed
        and applied in write_data_to_sim().
        
        Args:
            pressure: Target pressure values. Shape: (len(env_ids),) or (num_envs,)
            env_ids: Environment indices to set pressure for. If None, sets for all environments.
        """
        # Store target pressure
        if env_ids is None:
            self._target_pressure[:] = pressure
        else:
            self._target_pressure[env_ids] = pressure

    
    """
    Operations - Write to Simulation
    """
    
    def write_data_to_sim(self):
        """Write gripper forces to simulation based on pressure control.
        
        Overrides parent method to implement custom pressure-based force control:
        1. Compute current gap and gap velocity from finger positions/velocities (per-finger)
        2. Compute target gap from pressure: gap_target = (a + b * pressure) / 2 for each finger
        3. Compute control force per finger: tau = kp * (gap_target - gap) - kd * gap_dot
        4. Apply forces to both finger joints
        5. Write forces directly to PhysX
        """
        # 1. Compute current per-finger gaps and velocities [num_envs, 2]
        g = self._compute_gap()
        gdot = self._compute_gap_velocity()

        # 2. Compute target total gap from pressure [num_envs]
        gt_total = self._compute_gap_target(self._target_pressure)
        
        # 3. Split target equally between fingers [num_envs, 1]
        gt = (gt_total / 2.0).unsqueeze(-1)

        # 4. Compute per-finger control forces [num_envs, 2]
        tau = self._kp * (gt - g) - self._kd * gdot

        # 5. Apply forces to both finger joints using cached indices
        self._joint_effort_target_sim[:, self._finger_joint_indices[0]] = -tau[:, 0]
        self._joint_effort_target_sim[:, self._finger_joint_indices[1]] = tau[:, 1]

        # 6. Write forces directly to PhysX
        self.root_physx_view.set_dof_actuation_forces(self._joint_effort_target_sim, self._ALL_INDICES)
    
    """
    Internal Helpers
    """
    
    def _initialize_impl(self):
        """Initialize gripper-specific buffers after parent initialization."""
        # Call parent initialization first
        super()._initialize_impl()
        
        # Cache finger body and joint indices
        self._finger_body_indices, _ = self.find_bodies([".*ft0.*", ".*ft1.*"])
        self._finger_joint_indices, _ = self.find_joints([".*f0", ".*f1"])
        
        # Initialize pressure buffer
        self._target_pressure = torch.zeros(self.num_instances, device=self.device)
        self._kp = self.cfg.kp
        self._kd = self.cfg.kd
        self._pressure_to_gap_mapping = self.cfg.pressure_to_gap_mapping
        
        # Define y-axis in local frame (constant)
        self._projection_axis_local = torch.tensor([0.0, 1.0, 0.0], device=self.device)


    def _compute_gap_target(self, pressure: torch.Tensor) -> torch.Tensor:
        """Compute target gap width based on pressure.

        1) Fit the gap to pressure mapping to a linear function
        2) Use the linear function to compute the target gap
        
        Returns:
            Target gap width for each environment. Shape: (num_envs,)
        """
        # 1) Fit the map y = m*x + b using least squares on-the-fly
        # Mapping format: [[pressure, gap], ...]
        mapping_tensor = torch.as_tensor(
            self._pressure_to_gap_mapping,
            dtype=pressure.dtype,
            device=pressure.device,
        )

        # Ensure we have at least two points; if not, fall back to mean gap
        if mapping_tensor.ndim != 2 or mapping_tensor.shape[1] != 2 or mapping_tensor.shape[0] == 0:
            # Degenerate mapping, return zeros
            return torch.zeros_like(pressure)

        x = mapping_tensor[:, 0]
        y = mapping_tensor[:, 1]

        # Handle single-point mapping gracefully
        if x.numel() == 1:
            m = torch.zeros((), dtype=pressure.dtype, device=pressure.device)
            b = y[0]
        else:
            x_mean = x.mean()
            y_mean = y.mean()
            denom = torch.sum((x - x_mean) ** 2)
            if torch.abs(denom) < 1e-12:
                # Nearly vertical/constant x; default to zero slope
                m = torch.zeros((), dtype=pressure.dtype, device=pressure.device)
            else:
                m = torch.sum((x - x_mean) * (y - y_mean)) / denom
            b = y_mean - m * x_mean

        # 2) Use the linear function to compute the target gap
        return m * pressure + b
    
    def _compute_gap(self) -> torch.Tensor:
        """Compute gap for each finger as distance from root projected onto root's y-axis.
        
        Returns:
            Gap for each finger. Shape: (num_envs, 2)
        """
        # Get root data from parent class (already in world frame)
        root_pos = self.data.root_link_pos_w  # [num_envs, 3]
        root_quat = self.data.root_link_quat_w  # [num_envs, 4] (w,x,y,z)
        
        # Get finger tip positions using cached indices
        finger_pos = self.data.body_link_pos_w[:, self._finger_body_indices]  # [num_envs, 2, 3]
        
        # Transform local y-axis to world frame using parent's math utils
        projection_axis_world = math_utils.quat_apply(root_quat, self._projection_axis_local.unsqueeze(0))  # [num_envs, 3]
        
        # Compute vectors from root to each finger
        root_to_fingers = finger_pos - root_pos.unsqueeze(1)  # [num_envs, 2, 3]
        
        # Project onto y-axis: dot product
        gaps = torch.sum(root_to_fingers * projection_axis_world.unsqueeze(1), dim=-1)  # [num_envs, 2]
        
        return gaps
    
    def _compute_gap_velocity(self) -> torch.Tensor:
        """Compute gap velocity for each finger (magnitude of finger velocity).
        
        Returns:
            Gap velocity magnitude for each finger. Shape: (num_envs, 2)
        """
        # Get finger velocities using cached indices
        finger_lin_vel = self.data.body_link_vel_w[:, self._finger_body_indices, :3]  # [num_envs, 2, 3]
        
        # Compute magnitude of each finger's velocity
        gap_vel = torch.norm(finger_lin_vel, dim=-1)  # [num_envs, 2]
        
        return gap_vel
