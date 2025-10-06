# Copyright (c) 2022-2025, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Gripper data logger for pressure-controlled grippers during simulation."""

import os
import torch
import numpy as np
import matplotlib.pyplot as plt
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from .gripper import Gripper

plt.style.use('dark_background')


class GripperLogger:
    """Logger for gripper pressure and gap metrics.
    
    This class efficiently stores gripper-specific data in PyTorch buffers during simulation
    and generates pressure-gap relationship plots at the end. Data is stored with shape
    [num_envs, num_timesteps] for pressure and [num_envs, num_timesteps, 2] for per-finger
    data to allow multi-environment logging.
    
    The main visualization is a pressure vs gap scatter plot that validates the
    pressure-to-gap mapping and reveals any hysteresis in the control system.
    
    Example:
        >>> logger = GripperLogger(gripper, sim_dt=0.01, duration=10.0, device="cuda:0")
        >>> for step in range(num_steps):
        >>>     # ... run simulation ...
        >>>     logger.log(step)
        >>> logger.plot(output_dir="./output")
    """
    
    def __init__(self, gripper, sim_dt: float, duration: float, device: str = "cuda:0"):
        """Initialize the gripper logger.
        
        Args:
            gripper: Gripper instance to log data from
            sim_dt: Physics timestep in seconds
            duration: Total simulation duration in seconds
            device: PyTorch device for tensor storage (default: "cuda:0")
        """
        # Store metadata
        self.gripper = gripper
        self.sim_dt = sim_dt
        self.duration = duration
        self.device = device
        
        # Calculate dimensions from gripper and simulation parameters
        self.num_envs = gripper.num_instances
        self.num_steps = int(duration / sim_dt)
        
        # Initialize PyTorch buffers
        # Pressure: [num_envs, num_steps]
        self.pressure_buffer = torch.zeros(
            (self.num_envs, self.num_steps),
            device=device,
            dtype=torch.float32
        )
        
        # Gap: [num_envs, num_steps, 2] - per-finger
        self.gap_buffer = torch.zeros(
            (self.num_envs, self.num_steps, 2),
            device=device,
            dtype=torch.float32
        )
        
        # Gap velocity: [num_envs, num_steps, 2] - per-finger
        self.gap_velocity_buffer = torch.zeros(
            (self.num_envs, self.num_steps, 2),
            device=device,
            dtype=torch.float32
        )
        
        print(f"[INFO]: GripperLogger initialized")
        print(f"[INFO]:   Environments: {self.num_envs}")
        print(f"[INFO]:   Timesteps: {self.num_steps}")
        print(f"[INFO]:   Buffer shapes:")
        print(f"[INFO]:     Pressure: {list(self.pressure_buffer.shape)}")
        print(f"[INFO]:     Gap: {list(self.gap_buffer.shape)}")
        print(f"[INFO]:     Gap velocity: {list(self.gap_velocity_buffer.shape)}")
        
        # Calculate total memory
        total_bytes = (
            self.pressure_buffer.numel() * 4 +
            self.gap_buffer.numel() * 4 +
            self.gap_velocity_buffer.numel() * 4
        )
        print(f"[INFO]:   Total memory: {total_bytes / 1024**2:.2f} MB")
    
    def log(self, time_index: int):
        """Log current gripper state at the specified time index.
        
        Args:
            time_index: Index in the time dimension (0 to num_steps-1)
        """
        if time_index < 0 or time_index >= self.num_steps:
            print(f"[WARNING]: Time index {time_index} out of bounds [0, {self.num_steps})")
            return
        
        # Copy current gripper data into buffers at the specified time index
        self.pressure_buffer[:, time_index] = self.gripper.pressure.clone()
        self.gap_buffer[:, time_index, :] = self.gripper.gap.clone()
        self.gap_velocity_buffer[:, time_index, :] = self.gripper.gap_velocity.clone()
    
    def plot(self, output_dir: str = "./output"):
        """Generate and save pressure-gap relationship plots.
        
        Creates two plots:
        1. Pressure vs Total Gap (scatter with connecting lines)
        2. Time-series: Pressure, Gap, and Velocity over time
        
        Args:
            output_dir: Directory to save the plots
        """
        # Create output directory if it doesn't exist
        os.makedirs(output_dir, exist_ok=True)
        
        # Convert buffers to numpy (CPU) for plotting
        pressure_data = self.pressure_buffer.cpu().numpy()  # [num_envs, num_steps]
        gap_data = self.gap_buffer.cpu().numpy()            # [num_envs, num_steps, 2]
        gap_vel_data = self.gap_velocity_buffer.cpu().numpy()  # [num_envs, num_steps, 2]
        
        # Compute total gap: sum of absolute values of both fingers
        total_gap = np.abs(gap_data[:, :, 0]) + np.abs(gap_data[:, :, 1])  # [num_envs, num_steps]
        
        # Print summary statistics
        print(f"[INFO]: Gripper logger captured {self.num_steps} timesteps")
        print(f"[INFO]:   Pressure range: [{pressure_data.min():.2f}, {pressure_data.max():.2f}] kPa")
        print(f"[INFO]:   Total gap range: [{total_gap.min():.6f}, {total_gap.max():.6f}] m")
        
        # Plot 1: Pressure vs Gap (main validation plot)
        self._plot_pressure_vs_gap(pressure_data, total_gap, output_dir)
        
        # Plot 2: Time-series (detailed dynamics)
        self._plot_time_series(pressure_data, total_gap, gap_vel_data, output_dir)
    
    def _plot_pressure_vs_gap(self, pressure_data: np.ndarray, total_gap: np.ndarray, output_dir: str):
        """Plot pressure vs total gap scatter plot.
        
        This plot validates the pressure-to-gap mapping and reveals hysteresis.
        
        Args:
            pressure_data: Pressure values [num_envs, num_steps]
            total_gap: Total gap values [num_envs, num_steps]
            output_dir: Directory to save the plot
        """
        fig, ax = plt.subplots(figsize=(10, 8))
        
        for env_idx in range(self.num_envs):
            # Scatter plot showing all data points
            ax.scatter(
                total_gap[env_idx, :], 
                pressure_data[env_idx, :],
                alpha=0.6,
                s=20,
                label=f'Env {env_idx}' if self.num_envs > 1 else 'Data'
            )
            
            # Connect sequential points to show temporal order and hysteresis
            ax.plot(
                total_gap[env_idx, :], 
                pressure_data[env_idx, :],
                alpha=0.3,
                linewidth=0.5
            )
        
        ax.set_xlabel('Total Gap (m)', fontsize=12, fontweight='bold')
        ax.set_ylabel('Pressure (kPa)', fontsize=12, fontweight='bold')
        ax.set_title('Pressure vs Gap Relationship', fontsize=14, fontweight='bold')
        ax.grid(True, alpha=0.3)
        
        if self.num_envs > 1:
            ax.legend(fontsize=10, loc='best')
        
        plt.tight_layout()
        
        # Save figure
        output_path = os.path.join(output_dir, "gripper_pressure_gap.png")
        plt.savefig(output_path, dpi=150, bbox_inches='tight')
        plt.close(fig)
        
        print(f"[INFO]: Pressure-gap plot saved to {output_path}")
    
    def _plot_time_series(self, pressure_data: np.ndarray, total_gap: np.ndarray, 
                         gap_vel_data: np.ndarray, output_dir: str):
        """Plot time-series data showing pressure, gap, and velocity over time.
        
        Args:
            pressure_data: Pressure values [num_envs, num_steps]
            total_gap: Total gap values [num_envs, num_steps]
            gap_vel_data: Gap velocity values [num_envs, num_steps, 2]
            output_dir: Directory to save the plot
        """
        # Reconstruct time array from timestep
        time_array = np.arange(self.num_steps) * self.sim_dt
        
        # Create figure with 3 subplots
        fig, axes = plt.subplots(3, 1, figsize=(12, 10), sharex=True)
        
        for env_idx in range(self.num_envs):
            env_label = f'Env {env_idx}' if self.num_envs > 1 else None
            
            # Subplot 1: Pressure over time
            axes[0].plot(
                time_array, 
                pressure_data[env_idx, :], 
                label=env_label,
                linewidth=1.5,
                alpha=0.8
            )
            axes[0].set_ylabel('Pressure (kPa)', fontsize=11, fontweight='bold')
            axes[0].set_title('Pressure Command', fontsize=12, fontweight='bold')
            axes[0].grid(True, alpha=0.3)
            axes[0].tick_params(labelsize=9)
            
            # Subplot 2: Total gap over time
            axes[1].plot(
                time_array, 
                total_gap[env_idx, :], 
                label=env_label,
                linewidth=1.5,
                alpha=0.8
            )
            axes[1].set_ylabel('Total Gap (m)', fontsize=11, fontweight='bold')
            axes[1].set_title('Gripper Gap Response', fontsize=12, fontweight='bold')
            axes[1].grid(True, alpha=0.3)
            axes[1].tick_params(labelsize=9)
            
            # Subplot 3: Gap velocity (both fingers) over time
            finger0_label = f'Env {env_idx} Finger 0' if self.num_envs > 1 else 'Finger 0'
            finger1_label = f'Env {env_idx} Finger 1' if self.num_envs > 1 else 'Finger 1'
            
            axes[2].plot(
                time_array, 
                gap_vel_data[env_idx, :, 0], 
                label=finger0_label,
                linewidth=1.5,
                alpha=0.7
            )
            axes[2].plot(
                time_array, 
                gap_vel_data[env_idx, :, 1], 
                label=finger1_label,
                linewidth=1.5,
                alpha=0.7,
                linestyle='--'
            )
            axes[2].set_ylabel('Gap Velocity (m/s)', fontsize=11, fontweight='bold')
            axes[2].set_xlabel('Time (s)', fontsize=11, fontweight='bold')
            axes[2].set_title('Finger Velocities', fontsize=12, fontweight='bold')
            axes[2].grid(True, alpha=0.3)
            axes[2].tick_params(labelsize=9)
        
        # Add legends if needed
        for ax in axes:
            if ax.get_legend_handles_labels()[0]:  # Check if there are any legend entries
                ax.legend(fontsize=9, loc='best')
        
        # Add overall title
        fig.suptitle(
            f'Gripper Metrics Over Time ({self.num_envs} env{"s" if self.num_envs > 1 else ""})',
            fontsize=14,
            fontweight='bold',
            y=0.995
        )
        
        # Adjust layout to prevent overlap
        plt.tight_layout(rect=[0, 0, 1, 0.99])
        
        # Save figure
        output_path = os.path.join(output_dir, "gripper_time_series.png")
        plt.savefig(output_path, dpi=150, bbox_inches='tight')
        plt.close(fig)
        
        print(f"[INFO]: Time-series plot saved to {output_path}")
        print(f"[INFO]:   Plotted {self.duration}s of data with {self.num_steps} timesteps")

