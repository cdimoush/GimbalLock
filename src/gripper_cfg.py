"""Gripper with pressure-based control for pneumatic actuation simulation."""

import torch
from typing import Sequence

import isaaclab.sim as sim_utils
from isaaclab.actuators import ImplicitActuatorCfg
from isaaclab.assets.articulation import Articulation, ArticulationCfg
from isaaclab.utils import configclass

from .gripper import Gripper

@configclass
class GripperCfg(ArticulationCfg):
    """Configuration for pressure-controlled gripper.
    
    Extends ArticulationCfg with parameters for pressure-to-gap mapping
    and PD control gains.
    """

    class_type: type = Gripper
    
    # PD controller gains for gap control
    kp: float = 100.0
    """Proportional gain for gap controller."""
    
    kd: float = 10.0
    """Derivative gain for gap controller."""
    
    max_effort: float = 1.0
    """Maximum effort (force) that can be applied to each finger joint (N or N·m)."""
    
    # Joint physical properties
    joint_armature: float = 0.01
    """Joint armature - added to joint-space inertia to improve stability."""
    
    joint_friction: float = 0.1
    """Joint static friction coefficient."""

    # Pressure to gap mapping: gap = a + b * pressure
    pressure_to_gap_mapping: list[list[float]] = [[-60.0, 0.0], [0.0, 0.006], [60.0, 0.015]] # [[pressure, gap], [pressure, gap]]
    """Mapping of pressure to gap."""
    

# Pre-configured gripper for easy use in simulations
GRIPPER_CFG = GripperCfg(
    spawn=sim_utils.UsdFileCfg(
        usd_path="/workspace/isaaclab/source/GimbalLock/models/gripper/usd/robot.usd",
        rigid_props=sim_utils.RigidBodyPropertiesCfg(
            disable_gravity=False,
            max_depenetration_velocity=5.0,
        ),
        articulation_props=sim_utils.ArticulationRootPropertiesCfg(
            enabled_self_collisions=False,
            solver_position_iteration_count=8,
            solver_velocity_iteration_count=0,
            fix_root_link=True,
        ),
    ),
    init_state=ArticulationCfg.InitialStateCfg(
        joint_pos={".*": 0.0},
        pos=(0.0, 0.0, 0.0),
    ),
    actuators={
        "joints": ImplicitActuatorCfg(
            joint_names_expr=[".*"],
            effort_limit_sim=100.0,
            velocity_limit_sim=100.0,
            stiffness=0.0,  # Zero stiffness - we control forces directly
            damping=0.0,    # Zero damping - we control forces directly
        )
    },
)