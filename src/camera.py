# Copyright (c) 2022-2025, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Simple camera module for taking pictures during simulation."""

import os
import omni.replicator.core as rep
from isaaclab.sensors.camera import Camera, CameraCfg
from isaaclab.utils import convert_dict_to_backend
import isaaclab.sim as sim_utils


def create_camera(prim_path: str = "/World/CameraView", position: tuple = (2.0, 2.0, 2.0), 
                  target: tuple = (0.0, 0.0, 0.5)) -> Camera:
    """Create a simple RGB camera.
    
    Args:
        prim_path: Path where camera will be spawned in the scene
        position: Camera position (x, y, z) in world frame
        target: Point the camera looks at (x, y, z)
        
    Returns:
        Camera instance ready to capture images
    """
    camera_cfg = CameraCfg(
        prim_path=prim_path,
        update_period=0,  # Update every step
        height=480,
        width=640,
        data_types=["rgb"],  # Only RGB, no depth or segmentation
        spawn=sim_utils.PinholeCameraCfg(
            focal_length=24.0,
            focus_distance=400.0,
            horizontal_aperture=20.955,
            clipping_range=(0.1, 1000.0),
        ),
    )
    
    # Create camera
    camera = Camera(camera_cfg)
    
    return camera


def take_picture(camera: Camera, output_dir: str = "./output", filename: str = "snapshot"):
    """Take a single picture and save it.
    
    Args:
        camera: Camera instance to capture from
        output_dir: Directory to save the image
        filename: Base filename (will add .png extension automatically)
    """
    # Create output directory if it doesn't exist
    os.makedirs(output_dir, exist_ok=True)
    
    # Initialize BasicWriter for saving
    rep_writer = rep.BasicWriter(
        output_dir=output_dir,
        frame_padding=0,
    )
    
    # Get camera data (assumes single camera, index 0)
    camera_index = 0
    
    # Convert torch tensors to numpy
    single_cam_data = convert_dict_to_backend(
        {k: v[camera_index] for k, v in camera.data.output.items()}, 
        backend="numpy"
    )
    
    # Extract camera info
    single_cam_info = camera.data.info[camera_index]
    
    # Pack data in replicator format
    rep_output = {"annotators": {}}
    for key, data, info in zip(single_cam_data.keys(), single_cam_data.values(), single_cam_info.values()):
        if info is not None:
            rep_output["annotators"][key] = {"render_product": {"data": data, **info}}
        else:
            rep_output["annotators"][key] = {"render_product": {"data": data}}
    
    # Add trigger output
    rep_output["trigger_outputs"] = {"on_time": camera.frame[camera_index]}
    
    # Save the image
    rep_writer.write(rep_output)
    
    print(f"[INFO]: Picture saved to {output_dir}/")

