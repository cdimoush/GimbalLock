# Copyright (c) 2022-2025, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Gripper with pressure-based control for pneumatic actuation simulation."""

from __future__ import annotations

import torch
from typing import Sequence, TYPE_CHECKING

import isaaclab.sim as sim_utils
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
        """Gap between the gripper fingers. Shape: (num_envs,)"""
        return self._compute_gap()

    @property
    def gap_velocity(self) -> torch.Tensor:
        """Gap velocity between the gripper fingers. Shape: (num_envs,)"""
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
        1. Compute current gap and gap velocity from finger positions/velocities
        2. Compute target gap from pressure: gap_target = a + b * pressure
        3. Compute control force: tau = kp * (gap_target - gap) - kd * gap_dot
        4. Apply symmetric forces to both finger joints
        5. Write forces directly to PhysX
        """
        # 1. Compute current gap and gap velocity
        g = self._compute_gap()
        gdot = self._compute_gap_velocity()

        # 2. Compute target gap from pressure
        gt = self._compute_gap_target(self._target_pressure)

        # 3. Compute control force: tau = kp * (gap_target - gap) - kd * gap_dot
        tau = self._kp * (gt - g) - self._kd * gdot

        # 4. Apply symmetric forces to both finger joints
        self._joint_effort_target_sim[:, 0] = -tau
        self._joint_effort_target_sim[:, 1] = tau

        # 5. Write forces directly to PhysX
        self.root_physx_view.set_dof_actuation_forces(self._joint_effort_target_sim, self._ALL_INDICES)
    
    """
    Internal Helpers
    """
    
    def _initialize_impl(self):
        """Initialize gripper-specific buffers after parent initialization."""
        # Call parent initialization first
        super()._initialize_impl()
        
        # Initialize pressure buffer
        self._target_pressure = torch.zeros(self.num_instances, device=self.device)
        self._kp = self.cfg.kp
        self._kd = self.cfg.kd
        self._pressure_to_gap_mapping = self.cfg.pressure_to_gap_mapping


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
        """Compute current gap width between gripper fingers by the norm of the difference in positions.
        
        Returns:
            Gap width for each environment. Shape: (num_envs,)
        """
        index, names = self.find_bodies([".*ft0.*", ".*ft1.*"])
        pos = self.data.body_pos_w[:, index]
        return torch.norm(pos[:, 0] - pos[:, 1], dim=1)
    
    def _compute_gap_velocity(self) -> torch.Tensor:
        """Compute current gap velocity (rate of change of gap width) by the average absolute projection of the velocity on the line between the fingers.
        
        Returns:
            Gap velocity for each environment. Shape: (num_envs,)
            Positive = opening, negative = closing
        """
        # Get finger body indices
        index, names = self.find_bodies([".*ft0.*", ".*ft1.*"])
        
        # Get positions and velocities in world frame [num_envs, 2, 3]
        pos = self.data.body_pos_w[:, index]  # [num_envs, 2, 3]
        vel = self.data.body_vel_w[:, index, :3]  # [num_envs, 2, 3] (linear velocity only)
        
        # Compute gap vector (from finger 1 to finger 0)
        gap_vector = pos[:, 0] - pos[:, 1]  # [num_envs, 3]
        gap_distance = torch.norm(gap_vector, dim=1, keepdim=True)  # [num_envs, 1]
        
        # Compute unit vector along gap direction
        gap_direction = gap_vector / (gap_distance + 1e-8)  # [num_envs, 3], add epsilon to avoid division by zero
        
        # Compute relative velocity (finger 0 velocity - finger 1 velocity)
        relative_velocity = vel[:, 0] - vel[:, 1]  # [num_envs, 3]
        
        # Project relative velocity onto gap direction
        gap_velocity = torch.sum(relative_velocity * gap_direction, dim=1)  # [num_envs]
        
        return gap_velocity
