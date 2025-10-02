# GimbalLock

A gyroscope modeling and simulation project that bridges Onshape CAD and physics simulator(s) to explore a conceptual system identification problem.

## Purpose

The goal of **GimbalLock** is to model a gyroscope in Onshape, convert it into a URDF using *onshape-to-robot*, generate simulation assets for multiple simulators, and perform system identification of the gyroscope across those environments.

**Currently only using Isaac Lab**

## Project Structure

```
GimbalLock/
├── .devcontainer/  # VS Code dev container configuration
├── src/            # Source Code 
|    ├── sys_id/    # System Identification methods 
|    ├── modeling/  # Code for creating robot models  
│    ├── isaac/     # Isaac integration
│    └── mujoco/    # MuJoCo integration (Aspiration)
├── scripts/        # Excutables  
├── models/         # Robot descriptions (URDF/USD/Onshape conversions)
├── docs/           # Diagrams, notes, writeups
├── requirements.txt # Python dependencies
└── README.md       # Project overview
```

## Development Environment

### VS Code Dev Container (Recommended)

This project includes a VS Code dev container for a seamless development experience.

**Prerequisites:**
- VS Code with [Dev Containers extension](https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.remote-containers)
- Docker Desktop

TODO add dev container stuff later. 

## Usage

### Generate Robot Models

Generate URDF and MuJoCo models from an Onshape assembly:

```bash
python scripts/onshape_to_robot.py <onshape_url>
```

The script:
- Parses the Onshape URL to extract document/workspace/element IDs
- Builds the robot model using onshape-to-robot
- Exports to both URDF (`models/gyro/urdf/`) and MuJoCo MJCF (`models/gyro/mjcf/`)
- Downloads mesh assets to `models/gyro/assets/`


# EC2 Workflow


## DCV Linux Server 

### Setup
https://docs.aws.amazon.com/dcv/latest/adminguide/setting-up-installing-linux-prereq.html

https://docs.aws.amazon.com/dcv/latest/adminguide/setting-up-installing-linux-server.html

### Using
https://docs.aws.amazon.com/dcv/latest/adminguide/manage-start.html

$ sudo dcv create-session my-console-session --type=console --owner ubuntu
$ dcv list-sessions


Once a session is running, go to https://ip_address:port and virtual ubuntu machine can be seen through browser. 