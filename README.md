# GimbalLock

A gyroscope modeling and simulation project that bridges Onshape CAD to multiple physics simulators for system identification.

## Purpose

The goal of **GimbalLock** is to model a gyroscope in Onshape, convert it into a URDF using *onshape-to-robot*, generate simulation assets for multiple simulators, and perform system identification of the gyroscope across those environments.

## Project Structure

```
GimbalLock/
├── .devcontainer/  # VS Code dev container configuration
├── sim/            # Simulator-specific integrations
│    ├── isaac/     # Isaac Automator workflows + robot loader
│    └── mujoco/    # MuJoCo integration
├── models/         # Robot descriptions (URDF/USD/Onshape conversions)
├── analysis/       # System identification, PyTorch workflows
├── tests/          # Test files and validation scripts
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

**To get started:**
1. Open the project in VS Code
2. Click "Reopen in Container" when prompted (or use Command Palette > "Dev Containers: Reopen in Container")
3. VS Code will build the container and set up the environment automatically

The dev container includes:
- Python 3.11 with PyTorch
- Pre-configured Python tools (black, ruff, pytest)
- VS Code extensions for Python development
- All project dependencies from `requirements.txt`

## Goals

1. Model gyroscope in Onshape
2. Convert to URDF using onshape-to-robot
3. Generate simulation assets for multiple environments
4. Perform system identification across simulators
5. Compare results between simulation environments