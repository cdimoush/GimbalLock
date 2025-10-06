# Copyright (c) 2022-2025, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Gripper simulation with pressure-based control."""

import argparse

from isaaclab.app import AppLauncher

# add argparse arguments
parser = argparse.ArgumentParser(description="Gripper with pressure control simulation.")
parser.add_argument("--num_envs", type=int, default=1, help="Number of environments to spawn.")
# append AppLauncher cli args
AppLauncher.add_app_launcher_args(parser)
# parse the arguments
args_cli = parser.parse_args()

# launch omniverse app
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

import torch
import numpy as np
import isaaclab.sim as sim_utils
from isaaclab.assets import AssetBaseCfg
from isaaclab.scene import InteractiveScene, InteractiveSceneCfg

# Import our custom gripper
from src.gripper import Gripper
from src.gripper_cfg import GRIPPER_CFG

# Import camera utilities
from src.camera import create_camera, setup_video_writer, record_frame

# Import gripper logger
from src.gripper_logger import GripperLogger

# Simulation parameters
SIM_DT = 0.01          # Physics timestep: 100 Hz (10ms steps)
DURATION = 10.0        # Total simulation duration in seconds
VIDEO_FPS = 30         # Video capture rate: 30 frames per second

# Pressure control parameters
PRESSURE_MIN = -60.0   # Minimum pressure (kPa)
PRESSURE_MAX = 60.0    # Maximum pressure (kPa)
PRESSURE_PERIOD = 2.5  # Period of sine wave (seconds)


class GripperSceneCfg(InteractiveSceneCfg):
    """Simple scene with pressure-controlled gripper."""
    
    # lights
    dome_light = AssetBaseCfg(
        prim_path="/World/Light", 
        spawn=sim_utils.DiskLightCfg(intensity=1000.0, color=(0.75, 0.75, 0.75))
    )
    
    # gripper robot with pressure control
    gripper = GRIPPER_CFG.replace(prim_path="{ENV_REGEX_NS}/Gripper")


def run_simulator(sim: sim_utils.SimulationContext, scene: InteractiveScene, camera=None, video_writer=None, gripper_logger=None):
    """Simulation loop with pressure control.
    
    Toggles between high and low pressure to demonstrate pressure-based gap control.
    
    Args:
        sim: Simulation context
        scene: Interactive scene
        camera: Optional camera instance for video recording
        video_writer: Optional video writer for MP4 recording
        gripper_logger: Optional gripper logger for data logging
    """
    # Get the gripper
    gripper: Gripper = scene["gripper"]
    
    # Print robot details
    print(f"Loaded the Gripper!")
    print(f"  Joint Names: {gripper.joint_names}")
    print(f"  Body Names: {gripper.body_names}")
    print(f"  Pressure control: Sine wave from {PRESSURE_MIN} to {PRESSURE_MAX} kPa, period {PRESSURE_PERIOD}s")
    
    # Calculate timing
    total_steps = int(DURATION / SIM_DT)
    next_frame_time = 0.0
    frames_captured = 0
    
    sim_time = 0.0
    time_index = 0
    
    print(f"[INFO]: Starting simulation")
    print(f"[INFO]:   Duration: {DURATION} seconds")
    print(f"[INFO]:   Physics DT: {SIM_DT} sec ({1/SIM_DT:.0f} Hz)")
    print(f"[INFO]:   Total physics steps: {total_steps}")
    if camera is not None:
        print(f"[INFO]:   Video FPS: {VIDEO_FPS}")
        print(f"[INFO]:   Total video frames: {int(DURATION * VIDEO_FPS)}")
    
    while simulation_app.is_running() and sim_time < DURATION:
        
        # Calculate pressure using sine wave
        # pressure = amplitude * sin(2π * time / period) + offset
        amplitude = (PRESSURE_MAX - PRESSURE_MIN) / 2.0
        offset = (PRESSURE_MAX + PRESSURE_MIN) / 2.0
        pressure_value = amplitude * np.sin(2 * np.pi * sim_time / PRESSURE_PERIOD) + offset
        current_pressure = torch.ones(gripper.num_instances, device=gripper.device) * pressure_value
        
        # Print status every 0.5 seconds
        if int(sim_time * 2) > int((sim_time - SIM_DT) * 2):
            print(f"[{sim_time:.2f}s] Pressure: {current_pressure[0].item():.1f} kPa, Gap: {gripper.gap[0].sum().item():.6f} m")

        # Set pressure (this updates internal target)
        gripper.set_pressure(current_pressure)
        
        # Write to simulation (this computes and applies forces)
        scene.write_data_to_sim()
        
        # Step simulation
        sim.step()
        
        # Update scene
        scene.update(SIM_DT)
        
        # Update camera if enabled
        if camera is not None:
            camera.update(dt=SIM_DT)
            
            # Check if it's time to capture a frame
            if sim_time >= next_frame_time:
                record_frame(camera, video_writer, camera_index=0)
                frames_captured += 1
                next_frame_time += 1.0 / VIDEO_FPS
        
        # Log gripper state
        if gripper_logger is not None:
            gripper_logger.log(time_index)
        
        sim_time += SIM_DT
        time_index += 1
    
    # Cleanup
    if video_writer is not None:
        video_writer.close()
        print(f"[INFO]: Video saved - {frames_captured} frames ({frames_captured/VIDEO_FPS:.2f}s)")
    
    # Generate gripper plots
    if gripper_logger is not None:
        gripper_logger.plot(output_dir="/workspace/isaaclab/source/GimbalLock/output")
    
    print("[INFO]: Simulation complete.")


def main():
    """Main function."""
    # Initialize the simulation context
    sim_cfg = sim_utils.SimulationCfg(device=args_cli.device, dt=SIM_DT)
    sim = sim_utils.SimulationContext(sim_cfg)
    sim.set_camera_view([0.1, 0.1, 0.1], [0.0, 0.0, 0.0])
    
    # Design scene
    scene_cfg = GripperSceneCfg(args_cli.num_envs, env_spacing=2.0)
    scene = InteractiveScene(scene_cfg)
    
    # Create camera (if enabled) - BEFORE sim.reset()
    camera = None
    video_writer = None
    
    if args_cli.enable_cameras:
        camera = create_camera(prim_path="/World/CameraOrigin", device=args_cli.device)
        print("[INFO]: Camera created")
    
    # Play the simulator
    sim.reset()
    
    # Set camera pose (if enabled) - AFTER sim.reset()
    if args_cli.enable_cameras:
        # Position camera to view gripper
        camera_position = torch.tensor([[0.15, 0.15, 0.05]], device=sim.device)
        camera_target = torch.tensor([[0.0, 0.0, 0.0]], device=sim.device)
        camera.set_world_poses_from_view(camera_position, camera_target)
        
        # Setup video writer
        video_writer = setup_video_writer(
            output_path="/workspace/isaaclab/source/GimbalLock/output/gripper_pressure_demo.mp4",
            fps=VIDEO_FPS,
            quality=8
        )
        
        print(f"[INFO]: Camera enabled - recording at {VIDEO_FPS} FPS")
    
    # Initialize gripper logger
    gripper = scene["gripper"]
    gripper_logger = GripperLogger(
        gripper=gripper,
        sim_dt=SIM_DT,
        duration=DURATION,
        device=args_cli.device
    )
    
    print("[INFO]: Gripper pressure control simulation ready...")
    
    # Run the simulator
    run_simulator(sim, scene, camera, video_writer, gripper_logger)


if __name__ == "__main__":
    main()
    simulation_app.close()
