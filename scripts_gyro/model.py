#!/usr/bin/env python3
"""
Generate robot models (URDF and MuJoCo) from Onshape assembly URL.
Usage: python scripts/onshape_to_robot.py <onshape_url>
"""

import json
import os
import re
import sys
from pathlib import Path


def parse_onshape_url(url):
    """
    Parse Onshape URL to extract document_id, workspace_id/version_id, and element_id.
    Pattern: https://{domain}/{type}/d/{document_id}/{w|v}/{workspace_id|version_id}/e/{element_id}
    """
    pattern = r"https://(.*)/(.*)/([wv])/(.*)/e/(.*)"
    match = re.match(pattern, url)

    if not match:
        raise ValueError(f"Invalid Onshape URL: {url}")

    groups = match.groups()
    document_id = groups[1]
    element_id = groups[4]

    # Determine if it's workspace or version
    if groups[2] == "w":
        workspace_id = groups[3]
        version_id = None
    else:
        workspace_id = None
        version_id = groups[3]

    return document_id, workspace_id, version_id, element_id


def create_temp_config(document_id, workspace_id, version_id, element_id, output_dir, output_format):
    """
    Create a temporary config.json file for onshape-to-robot compatibility.
    """
    config = {
        "document_id": document_id,
        "element_id": element_id,
        "robot_name": "gyro",
        "output_format": output_format,
        "output_filename": "robot",
        "assets_directory": "assets",
        "configuration": "default",
        "ignore_limits": False,
        "no_dynamics": False,
        "draw_frames": False,
        "joint_properties": {},
        "ignore": {},
        "post_import_commands": []
    }

    # Add workspace_id or version_id
    if workspace_id:
        config["workspace_id"] = workspace_id
    if version_id:
        config["version_id"] = version_id

    # Write config to temp file in output directory
    config_path = os.path.join(output_dir, "config.json")
    with open(config_path, "w") as f:
        json.dump(config, f, indent=2)

    return config_path

def cleanup():
    """
    Remove all files with .part extension in assets/
    """
    assets_dir = Path(__file__).parent.parent / "models" / "gyro" / "assets"
    for file in assets_dir.glob("*.part"):
        os.remove(file)

def main():
    if len(sys.argv) < 2:
        print("ERROR: Missing Onshape URL")
        print("Usage: python scripts/onshape_to_robot.py <onshape_url>")
        sys.exit(1)

    url = sys.argv[1]

    # Parse URL
    print("Parsing Onshape URL...")
    try:
        document_id, workspace_id, version_id, element_id = parse_onshape_url(url)
        print(f"  Document ID: {document_id}")
        print(f"  Element ID: {element_id}")
        if workspace_id:
            print(f"  Workspace ID: {workspace_id}")
        if version_id:
            print(f"  Version ID: {version_id}")
    except ValueError as e:
        print(f"ERROR: {e}")
        sys.exit(1)

    # Create output directories
    base_dir = Path(__file__).parent.parent / "models" / "gyro"
    urdf_dir = base_dir / "urdf"
    mjcf_dir = base_dir / "mjcf"
    assets_dir = base_dir / "assets"

    print("\nCreating output directories...")
    urdf_dir.mkdir(parents=True, exist_ok=True)
    mjcf_dir.mkdir(parents=True, exist_ok=True)
    assets_dir.mkdir(parents=True, exist_ok=True)

    # Import onshape-to-robot modules
    # from onshape_to_robot import processors
    from onshape_to_robot.config import Config
    from onshape_to_robot.exporter_mujoco import ExporterMuJoCo
    from onshape_to_robot.exporter_urdf import ExporterURDF
    from onshape_to_robot.robot_builder import RobotBuilder

    # Build robot model once (using URDF config initially)
    print("\nBuilding robot model from Onshape...")
    temp_config_path = create_temp_config(
        document_id, workspace_id, version_id, element_id,
        str(base_dir), "urdf"
    )

    try:
        config = Config(str(base_dir))
        robot_builder = RobotBuilder(config)
        robot = robot_builder.robot

        print(f"  Robot '{robot.name}' built successfully")
        print(f"  Links: {len(robot.links)}")
        print(f"  Joints: {len(robot.joints)}")

        # Apply default processors
        print("\nApplying processors...")
        for processor in config.processors:
            print(f"  {processor.__class__.__name__}")
            processor.process(robot)

        # Export to URDF
        print("\nExporting to URDF...")
        urdf_exporter = ExporterURDF(config)
        urdf_path = str(urdf_dir / "robot.urdf")
        urdf_exporter.write_xml(robot, urdf_path)
        print(f"  ✓ {urdf_path}")

        # Export to MuJoCo (update config format)
        print("\nExporting to MuJoCo...")
        config.output_format = "mujoco"
        mjcf_exporter = ExporterMuJoCo(config)
        mjcf_path = str(mjcf_dir / "robot.xml")
        mjcf_exporter.write_xml(robot, mjcf_path)
        print(f"  ✓ {mjcf_path}")

        cleanup()

        # Summary
        print("\n{'='*60}")
        print("Successfully generated robot models:")
        print(f"  URDF: {urdf_path}")
        print(f"  MuJoCo: {mjcf_path}")
        print(f"  Assets: {assets_dir}")
        print(f"{'='*60}")

    except Exception as e:
        print(f"\nERROR: {e}")
        raise
    finally:
        # Clean up temp config
        if os.path.exists(temp_config_path):
            os.remove(temp_config_path)


if __name__ == "__main__":
    main()
