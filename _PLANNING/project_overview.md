# Electron Pneumatic Gripper Demo - Project Overview

## Summary

The GimbalLock project has been repurposed as a demonstration of an **Electron Pneumatic Gripper** with pressure-based control. The demo showcases how to extend Isaac Lab's `Articulation` class to create custom robot controllers that map high-level commands (pressure) to low-level physics forces.

## Architecture

### Scripts Directory

Four main scripts manage the workflow from CAD to simulation:

1. **`model.py`** - Onshape to Robot Models
   - Parses Onshape URL to extract document/element IDs
   - Uses `onshape-to-robot` library to download CAD and generate URDF/MJCF
   - Outputs robot models with mesh assets to `models/gripper/`

2. **`export_usd.py`** - URDF to USD Conversion
   - Converts URDF to Isaac Sim's USD format using `UrdfConverter`
   - Configures joint properties (stiffness, damping, drive modes)
   - Outputs USD file ready for Isaac Sim physics engine

3. **`sim_day0.py`** - Basic Position Control Demo
   - Simple demo using raw `Articulation` class
   - Position control: toggles between joint limits every 100 frames
   - Shows baseline behavior without custom controller

4. **`sim_day1.py`** - Pressure Control Demo
   - Uses custom `Gripper` class with pressure-based control
   - Toggles between 0 kPa and 60 kPa pressure
   - Logs gap width, target gap, and applied forces for debugging

### Source Directory

Custom gripper implementation extending Isaac Lab's articulation framework:

#### `gripper.py` - Core Gripper Class

The `Gripper` class **extends** `Articulation` to add pressure control:

**Key Inheritance Pattern:**
- Inherits from `Articulation` (Isaac Lab's joint-based robot class)
- Calls `super().__init__(cfg)` to leverage parent's initialization
- Accesses parent data via `self.data.joint_pos`, `self.data.body_link_pos_w`, etc.
- Uses parent methods: `find_bodies()`, `find_joints()`, `write_joint_*_to_sim()`

**Core Override:**
- **`write_data_to_sim()`** - Replaces parent's actuator model with custom pressure control:
  1. Computes current gap (finger positions projected onto gripper's y-axis)
  2. Maps target pressure → target gap using linear fit
  3. Computes PD control forces: `tau = kp*(gap_target - gap) - kd*gap_dot`
  4. Clamps forces to effort limits
  5. Writes forces directly to PhysX via `root_physx_view.set_dof_actuation_forces()`

**Custom Properties:**
- `gap` - Current finger separation (computed from body positions)
- `gap_velocity` - Rate of change of gap (computed from body velocities)
- `target_pressure` - User-set pressure command

**Custom Methods:**
- `set_pressure()` - User-facing API to command gripper pressure
- `_compute_gap()` - Projects finger positions onto gripper's y-axis
- `_compute_gap_velocity()` - Projects finger velocities onto y-axis
- `_compute_gap_target()` - Fits linear model to pressure-gap mapping

#### `gripper_cfg.py` - Configuration

Extends `ArticulationCfg` with gripper-specific parameters:
- **PD gains**: `kp`, `kd` for gap controller
- **Physical properties**: `joint_armature`, `joint_friction`
- **Pressure-gap mapping**: List of `[pressure, gap]` pairs for linear interpolation
- **Pre-configured instance**: `GRIPPER_CFG` with USD path and actuator settings

#### Supporting Utilities

- **`camera.py`** - Video/image recording during simulation
  - `create_camera()` - Sets up RGB camera with Isaac Lab's `Camera` class
  - `setup_video_writer()` - Configures imageio+ffmpeg for MP4 encoding
  - `record_frame()` - Captures and encodes frames in real-time

- **`joint_logger.py`** - Joint data logging and visualization
  - Stores joint pos/vel in PyTorch tensors during simulation
  - Generates matplotlib plots (2 rows × N joints) showing time-series data
  - Supports multi-environment logging

## How Gripper Wraps Articulation

The key design pattern is **composition through inheritance**:

```
Articulation (Isaac Lab base class)
    ├── Manages PhysX articulation view
    ├── Handles actuator models (implicit/explicit)
    ├── Provides joint/body state access
    └── write_data_to_sim() - applies actuator forces

Gripper (Custom extension)
    ├── Inherits all Articulation functionality
    ├── Adds pressure control layer
    ├── Overrides write_data_to_sim() for custom control law
    └── Maps pressure → forces via PD control on gap
```

**Data Flow:**
1. User calls `gripper.set_pressure(pressure)` → stores target
2. Scene calls `gripper.write_data_to_sim()` → computes forces from pressure
3. Forces written to PhysX → simulation steps
4. Scene calls `gripper.update(dt)` → parent reads new state from PhysX

**Key Insight:** The gripper bypasses Isaac Lab's standard actuator models (implicit PD, explicit models) by directly computing and writing forces in `write_data_to_sim()`. This allows full control over the control law while still leveraging the parent class for state management and PhysX interfacing.

## Technical Highlights

- **Zero-stiffness actuators**: Actuator config sets `stiffness=0.0`, `damping=0.0` to disable built-in PD control
- **Geometric gap computation**: Uses quaternion rotations (`quat_apply`) to project 3D positions onto 1D gap metric
- **Linear pressure mapping**: Fits `gap = m*pressure + b` on-the-fly from user-provided data points
- **Per-finger control**: Computes independent forces for each finger with symmetric gap targets

## Use Cases

This pattern enables:
- Custom control laws (impedance, admittance, model-based)
- Non-standard actuators (pneumatic, hydraulic, soft robotics)
- Contact-rich manipulation with force compliance
- System identification (the original project goal)

The pressure-controlled gripper serves as a reference implementation for extending Isaac Lab's articulation framework with domain-specific control strategies.

