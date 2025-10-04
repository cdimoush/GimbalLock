# Copyright (c) 2022-2025, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""
Utility to convert robot URDF into USD format.

Usage: python scripts/usd.py <robot_name>

Arguments:
    robot_name: Name of the robot (must match the directory in models/)
"""

"""Launch Isaac Sim Simulator first."""

import argparse
import sys
from pathlib import Path
from isaaclab.app import AppLauncher

# Path to this file
SCRIPT_PATH = Path(__file__).parent

# Default configuration parameters
MERGE_JOINTS = True
FIX_BASE = False
JOINT_STIFFNESS = 0.0
JOINT_DAMPING = 0.0
JOINT_TARGET_TYPE = "none"

# Create parser for robot name and AppLauncher
parser = argparse.ArgumentParser(
    description="Convert robot URDF to USD format.",
    formatter_class=argparse.RawDescriptionHelpFormatter,
    epilog="""
Example:
  python scripts/usd.py gyro
  python scripts/usd.py my_robot
"""
)
parser.add_argument("robot_name", type=str, help="Name of the robot (must match directory in models/)")
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

# Force headless mode
args_cli.headless = True

# launch omniverse app
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

"""Rest everything follows."""

import contextlib
import os
import re

import carb
import isaacsim.core.utils.stage as stage_utils
import omni.kit.app

from isaaclab.sim.converters import UrdfConverter, UrdfConverterCfg
from isaaclab.utils.assets import check_file_path
from isaaclab.utils.dict import print_dict


def main():
    # Get and validate robot name
    robot_name = args_cli.robot_name.strip()
    
    if not robot_name or not re.match(r'^[a-zA-Z0-9_-]+$', robot_name):
        print(f"ERROR: Invalid robot name '{robot_name}'")
        print("Robot name must contain only alphanumeric characters, hyphens, or underscores")
        sys.exit(1)
    
    print("=" * 80)
    print(f"Converting robot '{robot_name}' URDF to USD")
    print("=" * 80)
    
    # Construct paths based on robot name
    models_dir = SCRIPT_PATH.parent / "models" / robot_name
    urdf_path = models_dir / "urdf" / "robot.urdf"
    usd_dir = models_dir / "usd"
    usd_path = usd_dir / "robot.usd"
    
    # Check if model directory exists
    if not models_dir.exists():
        print(f"ERROR: Model directory not found: {models_dir}")
        print(f"Please ensure the robot '{robot_name}' exists in the models/ directory")
        sys.exit(1)
    
    # Check if URDF file exists
    if not urdf_path.exists():
        print(f"ERROR: URDF file not found: {urdf_path}")
        print(f"Please run model.py first to generate the URDF for '{robot_name}'")
        sys.exit(1)
    
    # Create USD output directory
    usd_dir.mkdir(parents=True, exist_ok=True)
    
    # Convert to absolute paths
    urdf_path_abs = urdf_path.resolve()
    usd_path_abs = usd_path.resolve()
    
    # Validate URDF file
    if not check_file_path(str(urdf_path_abs)):
        raise ValueError(f"Invalid URDF file path: {urdf_path_abs}")
    
    print(f"Input URDF: {urdf_path_abs}")
    print(f"Output USD: {usd_path_abs}")
    print("-" * 80)
    
    # Create Urdf converter config
    urdf_converter_cfg = UrdfConverterCfg(
        asset_path=str(urdf_path_abs),
        usd_dir=str(usd_path_abs.parent),
        usd_file_name=usd_path_abs.name,
        fix_base=FIX_BASE,
        merge_fixed_joints=MERGE_JOINTS,
        force_usd_conversion=True,
        joint_drive=UrdfConverterCfg.JointDriveCfg(
            gains=UrdfConverterCfg.JointDriveCfg.PDGainsCfg(
                stiffness=JOINT_STIFFNESS,
                damping=JOINT_DAMPING,
            ),
            target_type=JOINT_TARGET_TYPE,
        ),
    )

    # Print configuration
    print("URDF Converter Configuration:")
    print_dict(urdf_converter_cfg.to_dict(), nesting=0)
    print("-" * 80)

    # Create Urdf converter and import the file
    print("Converting URDF to USD...")
    urdf_converter = UrdfConverter(urdf_converter_cfg)
    
    # Print output
    print("-" * 80)
    print("Conversion successful!")
    print(f"Generated USD file: {urdf_converter.usd_path}")
    print("-" * 80)

    # Determine if there is a GUI to update:
    # acquire settings interface
    carb_settings_iface = carb.settings.get_settings()
    # read flag for whether a local GUI is enabled
    local_gui = carb_settings_iface.get("/app/window/enabled")
    # read flag for whether livestreaming GUI is enabled
    livestream_gui = carb_settings_iface.get("/app/livestream/enabled")

    # Simulate scene (if not headless)
    if local_gui or livestream_gui:
        # Open the stage with USD
        stage_utils.open_stage(urdf_converter.usd_path)
        # Reinitialize the simulation
        app = omni.kit.app.get_app_interface()
        # Run simulation
        with contextlib.suppress(KeyboardInterrupt):
            while app.is_running():
                # perform step
                app.update()


if __name__ == "__main__":
    # run the main function
    main()
    # close sim app
    simulation_app.close()
