# GimbalLock

A gyroscope modeling and simulation project that bridges Onshape CAD to multiple physics simulators for system identification.

## Purpose

The goal of **GimbalLock** is to model a gyroscope in Onshape, convert it into a URDF using *onshape-to-robot*, generate simulation assets for multiple simulators, and perform system identification of the gyroscope across those environments.

## Project Structure

```
GimbalLock/
├── sim/            # Simulator-specific integrations
│    ├── isaac/     # Isaac Automator workflows + robot loader
│    └── mujoco/    # MuJoCo integration
├── models/         # Robot descriptions (URDF/USD/Onshape conversions)
├── analysis/       # System identification, PyTorch workflows
├── docker/         # Dockerfiles for reproducible environments
├── docs/           # Diagrams, notes, writeups
└── README.md       # Project overview
```

## Development Environment

### Docker Setup

Build the development container:

```bash
docker build -t gimbal-dev -f docker/Dockerfile.dev .
```

Run the container:

```bash
docker run -it -v ~/dev/GimbalLock:/workspace gimbal-dev bash
```

## Goals

1. Model gyroscope in Onshape
2. Convert to URDF using onshape-to-robot
3. Generate simulation assets for multiple environments
4. Perform system identification across simulators
5. Compare results between simulation environments