# Copyright (c) 2022-2025, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

import argparse

from isaaclab.app import AppLauncher

# add argparse arguments
parser = argparse.ArgumentParser(description="Simple gyro robot simulation.")
parser.add_argument("--num_envs", type=int, default=1, help="Number of environments to spawn.")
# append AppLauncher cli args
AppLauncher.add_app_launcher_args(parser)
# parse the arguments
args_cli = parser.parse_args()

# launch omniverse app
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

import sys
import os
sys.path.append(os.path.join(os.path.dirname(__file__), ".."))

import torch
import isaaclab.sim as sim_utils
from isaaclab.assets import AssetBaseCfg
from isaaclab.assets.articulation import ArticulationCfg
from isaaclab.scene import InteractiveScene, InteractiveSceneCfg
from isaaclab.actuators import ImplicitActuatorCfg

from src.camera import create_camera, setup_camera_writer, take_picture, setup_video_writer, record_frame

# Simulation time controls
SIM_DT = 1/100        # Physics timestep in seconds (500 Hz)
DURATION = 30         # Total simulation duration in seconds
FPS = 10              # Video frame rate (frames per second)


# Gyro robot configuration
GYRO_CONFIG = ArticulationCfg(
    spawn=sim_utils.UsdFileCfg(
        usd_path="/workspace/isaaclab/source/GimbalLock/models/gyro/usd/robot.usd",
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
            velocity_limit_sim=1e9,
            stiffness=10000.0,
            damping=100.0,
        )
    },
)


class GyroSceneCfg(InteractiveSceneCfg):
    """Simple scene with just the gyro robot."""

    # Ground-plane
    # ground = AssetBaseCfg(prim_path="/World/defaultGroundPlane", spawn=sim_utils.GroundPlaneCfg())

    # lights
    distant_light = AssetBaseCfg(
        prim_path="/World/Light", spawn=sim_utils.DistantLightCfg(intensity=1000.0, color=(0.75, 0.75, 0.75))
    )

    # gyro robot
    gyro = GYRO_CONFIG.replace(prim_path="{ENV_REGEX_NS}/Gyro")


def run_simulator(sim: sim_utils.SimulationContext, scene: InteractiveScene, camera, rep_writer, video_writer=None):
    """Simple simulation loop - records video at specified FPS while running physics at SIM_DT.
    
    Args:
        sim: Simulation context
        scene: Interactive scene
        camera: Camera instance
        rep_writer: Replicator BasicWriter for images
        video_writer: Optional video writer for MP4 recording
    """
    import numpy as np
    
    gyro = scene["gyro"]
    
    # Set joint states directly to simulation (no control, just kinematics)
    # Joint 1 (index 1): position = pi/6
    # Joint 2 (index 2): velocity = 1e3 rad/s
    joint_pos = gyro.data.default_joint_pos.clone()
    joint_pos[:, 1] = np.pi / 4  # Set joint1 to pi/6 radians
    
    # Write directly to simulation
    gyro.write_joint_position_to_sim(joint_pos)
    
    # Calculate timing parameters
    total_physics_steps = int(DURATION / SIM_DT)
    total_video_frames = int(DURATION * FPS)
    frame_capture_interval = SIM_DT * FPS  # Time between video frames
    
    print(f"[INFO]: Starting simulation")
    print(f"[INFO]:   Duration: {DURATION} seconds")
    print(f"[INFO]:   Physics DT: {SIM_DT} sec ({1/SIM_DT:.0f} Hz)")
    print(f"[INFO]:   Total physics steps: {total_physics_steps}")
    print(f"[INFO]:   Video FPS: {FPS}")
    print(f"[INFO]:   Total video frames: {total_video_frames}")
    print(f"[INFO]:   Capturing every {1/SIM_DT/FPS:.0f} physics steps")
    
    # Simulation state
    physics_step = 0
    sim_time = 0.0
    next_frame_time = 0.0
    frames_captured = 0
    
    while simulation_app.is_running() and sim_time < DURATION:
        # Set joint velocity every step
        joint_vel = gyro.data.joint_vel.clone()
        joint_vel[:, 2] = 1e3
        gyro.write_joint_velocity_to_sim(joint_vel)
        
        # Step simulation
        sim.step()
        physics_step += 1
        sim_time += SIM_DT
        
        # Update scene and camera
        gyro.update(SIM_DT)
        scene.update(SIM_DT)
        camera.update(dt=SIM_DT)
        scene.write_data_to_sim()
        
        # Check if we should capture a video frame (at FPS intervals, not every physics step)
        if sim_time >= next_frame_time and video_writer is not None:
            record_frame(camera, video_writer, camera_index=0)
            frames_captured += 1
            next_frame_time += 1.0 / FPS
            
            # Debug: Print joint2 position for captured frames
            joint2_pos_rad = gyro.data.joint_pos[0, 2].item()
            joint2_pos_deg = np.degrees(joint2_pos_rad)
            print(f"[DEBUG] Frame {frames_captured}/{total_video_frames} @ t={sim_time:.3f}s: "
                  f"Joint2 = {joint2_pos_rad:.6f} rad ({joint2_pos_deg:.0f}°)")
        
        # Take a single snapshot at 1 second
        if 1.0 <= sim_time < 1.0 + SIM_DT and physics_step > 1:
            take_picture(camera, rep_writer, camera_index=0)
            print(f"[INFO]: Snapshot saved at t={sim_time:.3f}s")
    
    # Close video writer when done
    if video_writer is not None:
        video_writer.close()
        print(f"[INFO]: Video recording complete!")
        print(f"[INFO]:   Physics steps executed: {physics_step}")
        print(f"[INFO]:   Simulation time: {sim_time:.3f} seconds")
        print(f"[INFO]:   Video frames captured: {frames_captured}")
        print(f"[INFO]:   Expected video duration: {frames_captured/FPS:.3f} seconds")
    
    print("[INFO]: Simulation complete. Exiting...")
    simulation_app.close()


def main():
    """Main function."""
    # Initialize the simulation context
    sim_cfg = sim_utils.SimulationCfg(device=args_cli.device, dt=SIM_DT)
    sim = sim_utils.SimulationContext(sim_cfg)
    sim.set_camera_view([2.0, 0.0, 2.0], [0.0, 0.0, 0.5])
    
    # Design scene
    scene_cfg = GyroSceneCfg(args_cli.num_envs, env_spacing=2.0)
    scene = InteractiveScene(scene_cfg)
    
    # Create camera BEFORE sim.reset() (following tutorial)
    camera = create_camera(prim_path="/World/CameraOrigin", device=args_cli.device)
    
    # Play the simulator
    sim.reset()
    
    # Set camera pose AFTER sim.reset() (following tutorial)
    camera_position = torch.tensor([[2.0, 2.0, 2.0]], device=sim.device)
    camera_target = torch.tensor([[0.0, 0.0, 0.5]], device=sim.device)
    camera.set_world_poses_from_view(camera_position, camera_target)
    
    # Setup camera writer for images
    rep_writer = setup_camera_writer(camera, output_dir="/workspace/isaaclab/source/GimbalLock/output")
    
    # Setup video writer (optional - set to None to disable video recording)
    video_writer = setup_video_writer(
        output_path="/workspace/isaaclab/source/GimbalLock/output/gyro_robot.mp4",
        fps=FPS,
        quality=8
    )
    
    print("[INFO]: Gyro robot simulation ready...")
    
    # Run the simulator with time-controlled loop
    run_simulator(sim, scene, camera, rep_writer, video_writer)


if __name__ == "__main__":
    try:
        main()
    finally:
        # Ensure clean shutdown
        if simulation_app.is_running():
            simulation_app.close()
