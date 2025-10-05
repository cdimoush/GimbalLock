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
        
        # Simple three-state control (open/close/neutral)
        mag = torch.tensor([10.0, -10.0], device=self.device)
        index, _ = self.find_joints([".*f0", ".*f1"])
        
        # Compute forces using sign and broadcasting [num_envs, 2]
        sign = torch.sign(pressure).unsqueeze(1)  # [num_envs, 1]
        forces = sign * mag.unsqueeze(0)  # [num_envs, 2]
        self._joint_effort_target_sim[:, index] = forces
    
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
        # Step 5, only now
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
    
    def _compute_gap(self) -> torch.Tensor:
        """Compute current gap width between gripper fingers.
        
        Returns:
            Gap width for each environment. Shape: (num_envs,)
        """
        # TODO: Implement gap computation
        # Hint: Get finger body positions from self.data
        # Hint: Return scalar distance between fingertip bodies
        pass
    
    def _compute_gap_velocity(self) -> torch.Tensor:
        """Compute current gap velocity (rate of change of gap width).
        
        Returns:
            Gap velocity for each environment. Shape: (num_envs,)
        """
        # TODO: Implement gap velocity computation
        # Hint: Get finger body velocities from self.data
        # Hint: Return relative velocity (positive = opening, negative = closing)
        pass
