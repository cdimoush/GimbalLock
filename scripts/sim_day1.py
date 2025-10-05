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
import isaaclab.sim as sim_utils
from isaaclab.assets import AssetBaseCfg
from isaaclab.scene import InteractiveScene, InteractiveSceneCfg

# Import our custom gripper
from src.gripper import Gripper
from src.gripper_cfg import GRIPPER_CFG


class GripperSceneCfg(InteractiveSceneCfg):
    """Simple scene with pressure-controlled gripper."""
    
    # lights
    dome_light = AssetBaseCfg(
        prim_path="/World/Light", 
        spawn=sim_utils.DiskLightCfg(intensity=1000.0, color=(0.75, 0.75, 0.75))
    )
    
    # gripper robot with pressure control
    gripper = GRIPPER_CFG.replace(prim_path="{ENV_REGEX_NS}/Gripper")


def run_simulator(sim: sim_utils.SimulationContext, scene: InteractiveScene):
    """Simulation loop with pressure control.
    
    Toggles between high and low pressure to demonstrate pressure-based gap control.
    """
    # Get the gripper
    gripper: Gripper = scene["gripper"]
    
    # Print robot details
    print(f"Loaded the Gripper!")
    print(f"  Joint Names: {gripper.joint_names}")
    print(f"  Body Names: {gripper.body_names}")

    # Define pressure targets
    pressure_low = torch.ones(gripper.num_instances, device=gripper.device) * 0.0
    pressure_high = torch.ones(gripper.num_instances, device=gripper.device) * 60.0
    
    current_pressure = pressure_low.clone()
    
    count = 0
    while simulation_app.is_running():
        
        # Toggle pressure every 100 frames
        if count % 100 == 0:
            if torch.equal(current_pressure, pressure_low):
                current_pressure = pressure_high.clone()
                print(f"Pressure: {current_pressure[0].item()}")
            else:
                current_pressure = pressure_low.clone() 
                print(f"Pressure: {current_pressure[0].item()}")

        if count % 10 == 0:
            print(f"Gap: {gripper.gap[0].tolist()}")
            print(f"Gap Target: {gripper._compute_gap_target(current_pressure)[0].item()}")
            print(f"Joint Effort Target: {gripper._joint_effort_target_sim[0].tolist()}")
            print(f"--------------------------------")



        # Set pressure (this updates internal target)
        gripper.set_pressure(current_pressure)
        
        # Write to simulation (this computes and applies forces)
        scene.write_data_to_sim()
        
        # Step simulation
        sim.step()
        
        # Update scene
        scene.update(sim.get_physics_dt())
        
        # TODO: Add debug printing of gap, target gap, and forces
        
        count += 1


def main():
    """Main function."""
    # Initialize the simulation context
    sim_cfg = sim_utils.SimulationCfg(device=args_cli.device)
    sim = sim_utils.SimulationContext(sim_cfg)
    sim.set_camera_view([0.1, 0.1, 0.1], [0.0, 0.0, 0.0])
    
    # Design scene
    scene_cfg = GripperSceneCfg(args_cli.num_envs, env_spacing=2.0)
    scene = InteractiveScene(scene_cfg)
    
    # Play the simulator
    sim.reset()
    print("[INFO]: Gripper pressure control simulation ready...")
    
    # Run the simulator
    run_simulator(sim, scene)


if __name__ == "__main__":
    main()
    simulation_app.close()
