# GimbalLock

A gyroscope simulation project for physics-based modeling and system identification using Isaac Lab.

## Purpose

**GimbalLock** bridges CAD design and physics simulation by converting an Onshape gyroscope model to Isaac Lab (USD) format. The project provides tools to simulate gyroscope dynamics, log joint data, and capture video—laying groundwork for system identification experiments.

## Project Structure

```
GimbalLock/
├── src/                # Source code modules
│   ├── camera.py       # Camera utilities for simulation video/image capture
│   ├── joint_logger.py # Joint data logging and plotting
│   ├── sim/            # Simulator integrations (isaac, mujoco)
│   └── sys_id/         # System identification methods (future)
├── scripts/            # Executable scripts
│   ├── model.py        # Generate URDF/MJCF from Onshape URL
│   ├── gyro_usd.py     # Convert URDF to USD format
│   ├── gyro_sim.py     # Main Isaac Lab simulation
│   └── test_sim.py     # Test simulation configurations
├── models/gyro/        # Robot model assets (URDF, USD, MJCF, meshes)
├── output/             # Simulation outputs (videos, plots)
├── docs/               # Documentation and notes
└── requirements.txt    # Python dependencies
```

## Workflow

### 1. Generate Robot Models from Onshape

Convert an Onshape assembly to URDF and MJCF formats:

```bash
python scripts/model.py <onshape_url>
```

This generates:
- URDF: `models/gyro/urdf/robot.urdf`
- MJCF: `models/gyro/mjcf/robot.xml`
- Mesh assets: `models/gyro/assets/`

### 2. Convert URDF to USD

Prepare the model for Isaac Lab simulation:

```bash
python scripts/gyro_usd.py
```

Outputs USD file to `models/gyro/usd/robot.usd`

### 3. Run Simulation

Launch the gyroscope simulation in Isaac Lab:

```bash
# Headless mode with cameras for video recording
python scripts/gyro_sim.py --headless --enable_cameras

# Interactive mode with GUI (requires display)
python scripts/gyro_sim.py --num_envs 1
```

Simulation outputs (videos, joint plots) are saved to `output/`

## EC2 Workflow

For running simulations on AWS EC2 with DCV (NICE DCV remote desktop):

### Setup
- [Installing Linux Prerequisites](https://docs.aws.amazon.com/dcv/latest/adminguide/setting-up-installing-linux-prereq.html)
- [Installing DCV Server on Linux](https://docs.aws.amazon.com/dcv/latest/adminguide/setting-up-installing-linux-server.html)

### Start DCV Session

```bash
sudo dcv create-session my-console-session --type=console --owner ubuntu
dcv list-sessions
```

Connect via browser at `https://<ec2-ip>:<port>` to access the virtual Ubuntu desktop.

## Development Environment

This project runs inside Isaac Lab's devcontainer, which provides a complete Isaac Sim + Isaac Lab environment with all dependencies pre-configured (PyTorch, CUDA, USD libraries, and robotics tools).

**Prerequisites:**
- Docker Desktop
- VS Code with [Dev Containers extension](https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.remote-containers)
- NVIDIA GPU with updated drivers (for physics simulation)

Clone this project into Isaac Lab's `source/` directory and use the Isaac Lab devcontainer for seamless development. The container handles all complex dependencies—just open and code.
