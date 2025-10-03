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
            velocity_limit_sim=100.0,
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


def run_simulator(sim: sim_utils.SimulationContext, scene: InteractiveScene, camera, rep_writer, video_writer=None, num_frames=100):
    """Simple simulation loop - records video and optionally takes a picture.
    
    Args:
        sim: Simulation context
        scene: Interactive scene
        camera: Camera instance
        rep_writer: Replicator BasicWriter for images
        video_writer: Optional video writer for MP4 recording
        num_frames: Number of frames to record (default: 100)
    """
    import numpy as np
    
    count = 0
    gyro = scene["gyro"]
    
    # Set joint states directly to simulation (no control, just kinematics)
    # Joint 1 (index 1): position = pi/6
    # Joint 2 (index 2): velocity = 1e3 rad/s
    joint_pos = gyro.data.default_joint_pos.clone()
    joint_pos[:, 1] = np.pi / 6  # Set joint1 to pi/6 radians
    
    joint_vel = torch.zeros_like(gyro.data.default_joint_vel)
    joint_vel[:, 2] = 1e6  # Set joint2 velocity to 1000 rad/s
    
    # Write initial position to simulation
    gyro.write_joint_position_to_sim(joint_pos)
    
    print(f"[INFO]: Initial joint states set - Joint1: {np.pi/6:.3f} rad")
        
    while simulation_app.is_running() and count < num_frames:
        # Maintain constant velocity for joint2 by writing it every frame
        gyro.write_joint_velocity_to_sim(joint_vel)
        
        # Step simulation
        sim.step()
        count += 1
        
        # Update scene and camera
        scene.update(sim.get_physics_dt())
        camera.update(dt=sim.get_physics_dt())
        
        # Write scene data (for other assets, not controlling gyro)
        scene.write_data_to_sim()
        
        # Debug: Print joint2 position every frame
        joint2_pos_rad = gyro.data.joint_pos[0, 2].item()
        joint2_pos_deg = np.degrees(joint2_pos_rad)
        joint2_vel = gyro.data.joint_vel[0, 2].item()
        print(f"[DEBUG] Frame {count}: Joint2 position = {joint2_pos_rad:.6f} rad ({joint2_pos_deg:.0f}°), Joint2 velocity = {joint2_vel:.6f} rad/s")
        
        # Record frame to video (every frame)
        if video_writer is not None:
            record_frame(camera, video_writer, camera_index=0)
        
        # Take a single picture at frame 10 (optional)
        if count == 10:
            take_picture(camera, rep_writer, camera_index=0)
            print(f"[INFO]: Picture saved at frame {count}")
            print(f"[INFO]: Current joint positions: {gyro.data.joint_pos[0].cpu().numpy()}")
            print(f"[INFO]: Current joint velocities: {gyro.data.joint_vel[0].cpu().numpy()}")
    
    # Close video writer when done
    if video_writer is not None:
        video_writer.close()
        print(f"[INFO]: Video recording complete! {count} frames recorded.")
    
    print("[INFO]: Simulation complete. Exiting...")
    simulation_app.close()


def main():
    """Main function."""
    # Initialize the simulation context
    sim_cfg = sim_utils.SimulationCfg(device=args_cli.device)
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
        fps=10,
        quality=8
    )
    
    print("[INFO]: Gyro robot simulation ready...")
    
    # Run the simulator (records 100 frames by default)
    run_simulator(sim, scene, camera, rep_writer, video_writer, num_frames=100)


if __name__ == "__main__":
    try:
        main()
    finally:
        # Ensure clean shutdown
        if simulation_app.is_running():
            simulation_app.close()
