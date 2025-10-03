# Copyright (c) 2022-2025, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Simple camera module for taking pictures and recording videos during simulation."""

import os
import torch
import imageio
import omni.replicator.core as rep
import isaacsim.core.utils.prims as prim_utils
from isaaclab.sensors.camera import Camera, CameraCfg
from isaaclab.utils import convert_dict_to_backend
import isaaclab.sim as sim_utils


def create_camera(prim_path: str = "/World/CameraOrigin", device: str = "cuda:0") -> Camera:
    """Create a simple RGB camera.
    
    Args:
        prim_path: Path where camera origin will be created (camera will be at prim_path/CameraSensor)
        device: Device to use for camera tensors
        
    Returns:
        Camera instance ready to capture images
    """
    # Create Xform prim to hold the camera (required by Isaac Lab)
    prim_utils.create_prim(prim_path, "Xform")
    
    # Configure camera
    camera_cfg = CameraCfg(
        prim_path=f"{prim_path}/CameraSensor",
        update_period=0,  # Update every step
        height=480,
        width=640,
        data_types=["rgb"],  # Only RGB, no depth or segmentation
        spawn=sim_utils.PinholeCameraCfg(
            focal_length=24.0,
            focus_distance=400.0,
            horizontal_aperture=20.955,
            clipping_range=(0.1, 1.0e5),
        ),
    )
    
    # Create camera
    camera = Camera(cfg=camera_cfg)
    
    return camera


def setup_camera_writer(camera: Camera, output_dir: str = "./output") -> rep.BasicWriter:
    """Create a BasicWriter for saving camera images.
    
    Args:
        camera: Camera instance
        output_dir: Directory to save images
        
    Returns:
        BasicWriter instance
    """
    # Create output directory if it doesn't exist
    os.makedirs(output_dir, exist_ok=True)
    
    # Initialize BasicWriter for saving (following tutorial exactly)
    rep_writer = rep.BasicWriter(
        output_dir=output_dir,
        frame_padding=0,
        colorize_instance_id_segmentation=camera.cfg.colorize_instance_id_segmentation,
        colorize_instance_segmentation=camera.cfg.colorize_instance_segmentation,
        colorize_semantic_segmentation=camera.cfg.colorize_semantic_segmentation,
    )
    
    return rep_writer


def setup_video_writer(output_path: str, fps: int = 30, quality: int = 8):
    """Setup efficient video writer using imageio + ffmpeg.
    
    Args:
        output_path: Path to output MP4 file
        fps: Frames per second
        quality: Quality level 0-10 (higher is better)
        
    Returns:
        imageio writer instance or None if imageio not available
    """
    # Create output directory if needed
    os.makedirs(os.path.dirname(output_path), exist_ok=True)
    
    writer = imageio.get_writer(
        output_path,
        fps=fps,
        codec='libx264',  # H.264 codec - widely compatible
        quality=quality,  # 0-10, higher is better
        pixelformat='yuv420p',  # Compatible with most players
        macro_block_size=None
    )
    
    print(f"[INFO]: Video writer initialized: {output_path}")
    return writer


def record_frame(camera: Camera, video_writer, camera_index: int = 0):
    """Record a single frame to the video.
    
    Args:
        camera: Camera instance to capture from
        video_writer: imageio video writer instance
        camera_index: Index of camera to use (default 0)
    """
    if video_writer is None:
        return
    
    # Get RGB frame and convert to numpy (CPU)
    rgb_frame = camera.data.output["rgb"][camera_index].cpu().numpy()
    
    # Write frame to video (encodes in real-time)
    video_writer.append_data(rgb_frame)


def take_picture(camera: Camera, rep_writer: rep.BasicWriter, camera_index: int = 0):
    """Take a single picture and save it.
    
    Args:
        camera: Camera instance to capture from
        rep_writer: BasicWriter instance for saving
        camera_index: Index of camera to use (default 0)
    """
    # Save images from camera at camera_index
    # note: BasicWriter only supports saving data in numpy format, so we need to convert the data to numpy.
    single_cam_data = convert_dict_to_backend(
        {k: v[camera_index] for k, v in camera.data.output.items()}, backend="numpy"
    )

    # Extract the other information
    single_cam_info = camera.data.info[camera_index]

    # Pack data back into replicator format to save them using its writer
    rep_output = {"annotators": {}}
    for key, data, info in zip(single_cam_data.keys(), single_cam_data.values(), single_cam_info.values()):
        if info is not None:
            rep_output["annotators"][key] = {"render_product": {"data": data, **info}}
        else:
            rep_output["annotators"][key] = {"render_product": {"data": data}}
    # Save images
    # Note: We need to provide On-time data for Replicator to save the images.
    rep_output["trigger_outputs"] = {"on_time": camera.frame[camera_index]}
    rep_writer.write(rep_output)
    
    print(f"[INFO]: Picture saved!")

