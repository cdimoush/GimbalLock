# Copyright (c) 2022-2025, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Joint data logger for articulated robots during simulation."""

import os
import torch
import numpy as np
import matplotlib.pyplot as plt
from isaaclab.assets.articulation import Articulation

plt.style.use('dark_background')


class JointLogger:
    """Logger for joint positions and velocities during simulation.
    
    This class efficiently stores joint data in PyTorch buffers during simulation
    and generates time-series plots at the end. Data is stored with shape
    [num_envs, num_timesteps, num_joints] to allow multi-environment logging.
    
    Example:
        >>> logger = JointLogger(robot, sim_dt=0.01, duration=10.0, device="cuda:0")
        >>> for step in range(num_steps):
        >>>     # ... run simulation ...
        >>>     logger.log(step)
        >>> logger.plot(output_dir="./output")
    """
    
    def __init__(self, robot: Articulation, sim_dt: float, duration: float, device: str = "cuda:0"):
        """Initialize the joint logger.
        
        Args:
            robot: Articulation instance to log data from
            sim_dt: Physics timestep in seconds
            duration: Total simulation duration in seconds
            device: PyTorch device for tensor storage (default: "cuda:0")
        """
        # Store metadata
        self.robot = robot
        self.sim_dt = sim_dt
        self.duration = duration
        self.device = device
        
        # Calculate dimensions from robot and simulation parameters
        self.num_envs = robot.num_instances
        self.num_joints = robot.num_joints
        self.num_steps = int(duration / sim_dt)
        
        # Get joint names for better plot labels
        self.joint_names = robot.joint_names
        
        # Initialize PyTorch buffers with zeros [num_envs, num_steps, num_joints]
        self.position_buffer = torch.zeros(
            (self.num_envs, self.num_steps, self.num_joints),
            device=device,
            dtype=torch.float32
        )
        self.velocity_buffer = torch.zeros(
            (self.num_envs, self.num_steps, self.num_joints),
            device=device,
            dtype=torch.float32
        )
        
        print(f"[INFO]: JointLogger initialized")
        print(f"[INFO]:   Environments: {self.num_envs}")
        print(f"[INFO]:   Joints: {self.num_joints} ({', '.join(self.joint_names)})")
        print(f"[INFO]:   Timesteps: {self.num_steps}")
        print(f"[INFO]:   Buffer shape: [{self.num_envs}, {self.num_steps}, {self.num_joints}]")
        print(f"[INFO]:   Memory per buffer: {self.position_buffer.numel() * 4 / 1024**2:.2f} MB")
    
    def log(self, time_index: int):
        """Log current joint data at the specified time index.
        
        Args:
            time_index: Index in the time dimension (0 to num_steps-1)
        """
        if time_index < 0 or time_index >= self.num_steps:
            print(f"[WARNING]: Time index {time_index} out of bounds [0, {self.num_steps})")
            return
        
        # Copy current joint data into buffers at the specified time index
        self.position_buffer[:, time_index, :] = self.robot.data.joint_pos.clone()
        self.velocity_buffer[:, time_index, :] = self.robot.data.joint_vel.clone()
    
    def plot(self, output_dir: str = "./output"):
        """Generate and save time-series plots of joint data.
        
        Creates a figure with 2 rows and num_joints columns:
        - Row 1: Joint positions vs time
        - Row 2: Joint velocities vs time
        
        Each subplot shows all environments as separate series.
        
        Args:
            output_dir: Directory to save the plot
        """
        # Create output directory if it doesn't exist
        os.makedirs(output_dir, exist_ok=True)
        
        # Reconstruct time array from timestep
        time_array = np.arange(self.num_steps) * self.sim_dt
        
        # Convert buffers to numpy (CPU) for plotting
        pos_data = self.position_buffer.cpu().numpy()  # [num_envs, num_steps, num_joints]
        vel_data = self.velocity_buffer.cpu().numpy()  # [num_envs, num_steps, num_joints]
        
        # Create figure with subplots: 2 rows x num_joints columns
        fig_width = max(5 * self.num_joints, 12)  # At least 12 inches wide
        fig, axes = plt.subplots(
            2, self.num_joints, 
            figsize=(fig_width, 8),
            squeeze=False  # Always return 2D array even if num_joints=1
        )
        
        # Plot positions (row 0) - all environments
        for joint_idx in range(self.num_joints):
            ax = axes[0, joint_idx]
            for env_idx in range(self.num_envs):
                ax.plot(
                    time_array, 
                    pos_data[env_idx, :, joint_idx], 
                    linewidth=1.5,
                    label=f'Env {env_idx}',
                    alpha=0.8
                )
            ax.set_title(f'{self.joint_names[joint_idx]}\nPosition', fontsize=10, fontweight='bold')
            ax.set_xlabel('Time (s)', fontsize=9)
            ax.set_ylabel('Position (rad)', fontsize=9)
            ax.grid(True, alpha=0.3)
            ax.tick_params(labelsize=8)
            if self.num_envs > 1:
                ax.legend(fontsize=8, loc='best')
        
        # Plot velocities (row 1) - all environments
        for joint_idx in range(self.num_joints):
            ax = axes[1, joint_idx]
            for env_idx in range(self.num_envs):
                ax.plot(
                    time_array, 
                    vel_data[env_idx, :, joint_idx], 
                    linewidth=1.5,
                    label=f'Env {env_idx}',
                    alpha=0.8
                )
            ax.set_title(f'{self.joint_names[joint_idx]}\nVelocity', fontsize=10, fontweight='bold')
            ax.set_xlabel('Time (s)', fontsize=9)
            ax.set_ylabel('Velocity (rad/s)', fontsize=9)
            ax.grid(True, alpha=0.3)
            ax.tick_params(labelsize=8)
            if self.num_envs > 1:
                ax.legend(fontsize=8, loc='best')
        
        # Add overall title
        fig.suptitle(
            f'Joint Data Logger - All Environments ({self.num_envs} envs)',
            fontsize=14,
            fontweight='bold',
            y=0.995
        )
        
        # Adjust layout to prevent overlap
        plt.tight_layout(rect=[0, 0, 1, 0.99])
        
        # Save figure
        output_path = os.path.join(output_dir, "joint_logs.png")
        plt.savefig(output_path, dpi=150, bbox_inches='tight')
        plt.close(fig)  # Close to free memory
        
        print(f"[INFO]: Joint logs plot saved to {output_path}")
        print(f"[INFO]:   Plotted {self.num_joints} joints over {self.duration}s")
        print(f"[INFO]:   Environments plotted: {self.num_envs}")
        print(f"[INFO]:   Data points per joint: {self.num_steps}")

