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
        # TODO: Implement pressure setting
        pass
    
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
        # TODO: Implement force computation and application
        # Hint: Use self._compute_gap() and self._compute_gap_velocity()
        # Hint: Apply forces with self.root_physx_view.set_dof_actuation_forces()
        pass
    
    """
    Internal Helpers
    """
    
    def _initialize_impl(self):
        """Initialize gripper-specific buffers after parent initialization."""
        # Call parent initialization first
        super()._initialize_impl()
        
        # TODO: Initialize pressure buffer
        # Hint: self._target_pressure = torch.zeros(self.num_instances, device=self.device)
        pass
    
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
