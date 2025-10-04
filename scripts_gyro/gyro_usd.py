# Copyright (c) 2022-2025, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""
Utility to convert the gyro URDF into USD format.

This script converts the gyro robot URDF to USD format with hardcoded paths and parameters.
"""

"""Launch Isaac Sim Simulator first."""

import argparse

from isaaclab.app import AppLauncher

# Hard-coded configuration
URDF_PATH = "/workspace/isaaclab/source/GimbalLock/models/gyro/urdf/robot.urdf"
USD_PATH = "/workspace/isaaclab/source/GimbalLock/models/gyro/usd/robot.usd"
MERGE_JOINTS = True
FIX_BASE = False
JOINT_STIFFNESS = 0.0
JOINT_DAMPING = 0.0
JOINT_TARGET_TYPE = "none"

# Create minimal parser for AppLauncher
parser = argparse.ArgumentParser(description="Convert gyro URDF to USD format.")
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
import tempfile
import xml.etree.ElementTree as ET

import carb
import isaacsim.core.utils.stage as stage_utils
import omni.kit.app

from isaaclab.sim.converters import UrdfConverter, UrdfConverterCfg
from isaaclab.utils.assets import check_file_path
from isaaclab.utils.dict import print_dict


def remove_joint_limits_from_urdf(urdf_path: str) -> str:
    """
    Remove all joint limit tags from a URDF file and save to a temporary file.
    
    Args:
        urdf_path: Path to the original URDF file
        
    Returns:
        Path to the temporary URDF file with joint limits removed
    """
    # Parse the URDF XML
    tree = ET.parse(urdf_path)
    root = tree.getroot()
    
    # Find all joint elements and remove their limit children
    for joint in root.findall('joint'):
        limit = joint.find('limit')
        if limit is not None:
            joint.remove(limit)
            print(f"Removed limit from joint: {joint.get('name')}")
    
    # Create a temporary file in the same directory as the original URDF
    # This preserves relative mesh paths
    urdf_dir = os.path.dirname(urdf_path)
    urdf_basename = os.path.basename(urdf_path)
    urdf_name_without_ext = os.path.splitext(urdf_basename)[0]
    temp_path = os.path.join(urdf_dir, f"{urdf_name_without_ext}_temp.urdf")
    
    # Write the modified URDF to the temporary file
    tree.write(temp_path, encoding='utf-8', xml_declaration=True)
    
    print(f"Created temporary URDF without joint limits: {temp_path}")
    return temp_path


def main():
    # check valid file path
    original_urdf_path = URDF_PATH
    if not os.path.isabs(original_urdf_path):
        original_urdf_path = os.path.abspath(original_urdf_path)
    if not check_file_path(original_urdf_path):
        raise ValueError(f"Invalid file path: {original_urdf_path}")
    
    # Remove joint limits and create temporary URDF
    print("-" * 80)
    print("Removing joint limits from URDF...")
    temp_urdf_path = remove_joint_limits_from_urdf(original_urdf_path)
    print("-" * 80)
    
    # create destination path
    dest_path = USD_PATH
    if not os.path.isabs(dest_path):
        dest_path = os.path.abspath(dest_path)

    try:
        # Create Urdf converter config using the temporary URDF
        urdf_converter_cfg = UrdfConverterCfg(
            asset_path=temp_urdf_path,
            usd_dir=os.path.dirname(dest_path),
            usd_file_name=os.path.basename(dest_path),
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

        # Print info
        print("-" * 80)
        print("-" * 80)
        print(f"Original URDF file: {original_urdf_path}")
        print(f"Temporary URDF file (no limits): {temp_urdf_path}")
        print("URDF importer config:")
        print_dict(urdf_converter_cfg.to_dict(), nesting=0)
        print("-" * 80)
        print("-" * 80)

        # Create Urdf converter and import the file
        urdf_converter = UrdfConverter(urdf_converter_cfg)
        # print output
        print("URDF importer output:")
        print(f"Generated USD file: {urdf_converter.usd_path}")
        print("-" * 80)
        print("-" * 80)
    finally:
        # Clean up temporary URDF file
        if os.path.exists(temp_urdf_path):
            os.remove(temp_urdf_path)
            print(f"Cleaned up temporary URDF file: {temp_urdf_path}")
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
