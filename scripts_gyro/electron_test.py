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

import torch
import isaaclab.sim as sim_utils
from isaaclab.assets import AssetBaseCfg
from isaaclab.assets.articulation import Articulation, ArticulationCfg
from isaaclab.scene import InteractiveScene, InteractiveSceneCfg
from isaaclab.actuators import ImplicitActuatorCfg

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
            stiffness=1000.0,
            damping=100.0,
        )
    },
)


class GyroSceneCfg(InteractiveSceneCfg):
    """Simple scene with just the gyro robot."""
    # lights
    dome_light = AssetBaseCfg(
        prim_path="/World/Light", spawn=sim_utils.DistantLightCfg(intensity=1000.0, color=(0.75, 0.75, 0.75))
    )

    # gyro robot
    gyro = GYRO_CONFIG.replace(prim_path="{ENV_REGEX_NS}/Gyro")


def run_simulator(sim: sim_utils.SimulationContext, scene: InteractiveScene):
    """Simple simulation loop
    
    1) Position control
    
    
    
    """
    # Simulation Params
    dummy_position = 10.0

    # Simulation Loop
    robot: Articulation = scene["gyro"]
    joint_limits = robot.data.joint_limits.clone()
    print(f"Shape of joint_limits: {joint_limits}")
    jp0 = torch.zeros_like(robot.data.joint_pos)
    jp0[:, 0] = joint_limits[0, 0, 0]
    jp0[:, 1] = joint_limits[0, 1, 1]
    jp1 = torch.zeros_like(robot.data.joint_pos)
    jp1[:, 0] = joint_limits[0, 0, 1]
    jp1[:, 1] = joint_limits[0, 1, 0]

    target = jp0




    count = 0
    while simulation_app.is_running():
        # No actions - robot just sits there
        if count % 100 == 0:
            if torch.equal(target, jp0):
                target = jp1
            else:
                target = jp0

        robot.set_joint_position_target(target)
        scene.write_data_to_sim()
        sim.step()
        scene.update(sim.get_physics_dt())
        count += 1

def main():
    """Main function."""
    # Initialize the simulation context
    sim_cfg = sim_utils.SimulationCfg(device=args_cli.device)
    sim = sim_utils.SimulationContext(sim_cfg)
    sim.set_camera_view([0.1, 0.1, 0.1], [0.0, 0.0, 0.0])
    
    # Design scene
    scene_cfg = GyroSceneCfg(args_cli.num_envs, env_spacing=2.0)
    scene = InteractiveScene(scene_cfg)
    
    # Play the simulator
    sim.reset()
    print("[INFO]: Gyro robot simulation ready...")
    
    # Run the simulator
    run_simulator(sim, scene)


if __name__ == "__main__":
    main()
    simulation_app.close()