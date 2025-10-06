# sim_day1.py Camera Upgrade Plan

## Overview

Add video recording capability to the pressure control demo using the existing `camera.py` module. Camera rendering will be decoupled from physics timestep to allow efficient video capture at standard framerates (e.g., 30 FPS) while physics runs at higher rates (e.g., 1000 Hz).

## Requirements

### 1. Command-Line Interface

Add new argument to control camera functionality:

```python
parser.add_argument("--enable_cameras", action="store_true", default=False, 
                    help="Enable camera rendering and video recording.")
```

**Note:** Isaac Lab's `AppLauncher` already has `--enable_cameras` flag that affects backend rendering. We'll piggyback on this by checking `args_cli.enable_cameras` to conditionally set up our camera.

### 2. Configuration Parameters

Add simulation timing constants at top of file:

```python
# Simulation parameters
SIM_DT = 0.01          # Physics timestep: 100 Hz (10ms steps)
DURATION = 10.0        # Total simulation duration in seconds
VIDEO_FPS = 30         # Video capture rate: 30 frames per second
```

**Rationale:**
- Physics at 100 Hz is sufficient for gripper control stability
- 30 FPS video is standard and sufficient for visualizing gripper motion
- 10 second duration captures multiple pressure cycles (currently toggling every 100 frames)

### 3. Camera Setup (Conditional)

Add camera initialization between scene creation and `sim.reset()`:

```python
# Create camera (if enabled)
camera = None
video_writer = None

if args_cli.enable_cameras:
    camera = create_camera(prim_path="/World/CameraOrigin", device=args_cli.device)
```

**After** `sim.reset()`, set camera pose:

```python
if args_cli.enable_cameras:
    # Position camera to view gripper
    camera_position = torch.tensor([[0.15, 0.15, 0.15]], device=sim.device)
    camera_target = torch.tensor([[0.0, 0.0, 0.0]], device=sim.device)
    camera.set_world_poses_from_view(camera_position, camera_target)
    
    # Setup video writer
    video_writer = setup_video_writer(
        output_path="/workspace/isaaclab/source/GimbalLock/output/gripper_pressure_demo.mp4",
        fps=VIDEO_FPS,
        quality=8
    )
    
    print(f"[INFO]: Camera enabled - recording at {VIDEO_FPS} FPS")
```

### 4. Simulation Loop Modifications

Update `run_simulator()` signature:

```python
def run_simulator(sim: sim_utils.SimulationContext, scene: InteractiveScene, 
                  camera=None, video_writer=None):
```

**Frame capture timing logic:**

```python
# Calculate timing
total_steps = int(DURATION / SIM_DT)
next_frame_time = 0.0
frames_captured = 0

count = 0
sim_time = 0.0

while simulation_app.is_running() and sim_time < DURATION:
    # [Existing pressure control logic]
    
    # Write to simulation
    scene.write_data_to_sim()
    sim.step()
    scene.update(sim.get_physics_dt())
    
    # Update camera if enabled
    if camera is not None:
        camera.update(dt=SIM_DT)
        
        # Check if it's time to capture a frame
        if sim_time >= next_frame_time:
            record_frame(camera, video_writer, camera_index=0)
            frames_captured += 1
            next_frame_time += 1.0 / VIDEO_FPS
    
    sim_time += SIM_DT
    count += 1

# Cleanup
if video_writer is not None:
    video_writer.close()
    print(f"[INFO]: Video saved - {frames_captured} frames ({frames_captured/VIDEO_FPS:.2f}s)")
```

### 5. Pressure Toggle Adjustment

Current toggle happens every 100 frames. Update to use time-based toggling:

```python
PRESSURE_TOGGLE_INTERVAL = 1.0  # Toggle every 1 second

# In loop:
if int(sim_time / PRESSURE_TOGGLE_INTERVAL) % 2 == 0:
    current_pressure = pressure_low.clone()
else:
    current_pressure = pressure_high.clone()
```

This makes behavior independent of simulation timestep.

### 6. Debug Printing Cleanup

Current script prints debug info every 10 steps. Update to time-based or reduce verbosity:

```python
# Option 1: Time-based (every 0.5 seconds)
if int(sim_time * 2) > int((sim_time - SIM_DT) * 2):
    print(f"[{sim_time:.2f}s] Gap: {gripper.gap[0].tolist()}")
    
# Option 2: Remove entirely for cleaner output when recording video
```

## Implementation Checklist

- [ ] Add `SIM_DT`, `DURATION`, `VIDEO_FPS` constants
- [ ] Add `--enable_cameras` argument (or just use Isaac Lab's built-in)
- [ ] Import camera functions: `create_camera`, `setup_video_writer`, `record_frame`
- [ ] Add camera setup block (before `sim.reset()`)
- [ ] Add camera pose setting (after `sim.reset()`)
- [ ] Update `run_simulator()` signature to accept camera parameters
- [ ] Add frame capture timing logic in simulation loop
- [ ] Add `camera.update()` call in loop
- [ ] Add video writer cleanup at end
- [ ] Convert pressure toggle to time-based
- [ ] Update or remove debug printing
- [ ] Update `SimulationCfg` to use `dt=SIM_DT`

## Simplifications from Example Script

1. **No joint logger** - Focus on video only (can add later if needed)
2. **No image writer** - Only need video, not individual frames
3. **Simpler timing** - Use `sim_time` directly instead of tracking `physics_step` separately
4. **Remove redundant prints** - Keep output minimal during recording

## Expected Output

When run with `--enable_cameras`:
```bash
python scripts/sim_day1.py --num_envs 1 --enable_cameras
```

Output:
- Console: Initialization messages, periodic status updates
- File: `output/gripper_pressure_demo.mp4` (30 FPS, ~10 seconds)
- Behavior: Gripper opening/closing with pressure changes, clearly visible at 30 FPS

When run without flag:
```bash
python scripts/sim_day1.py --num_envs 1
```

Output:
- Console only (existing behavior)
- No video file created
- Potentially faster execution (no rendering overhead)

## Technical Notes

### Camera Update Frequency
- Physics: 100 Hz (every 0.01s)
- Camera render: 30 Hz (every 0.033s)
- Frame capture uses `sim_time >= next_frame_time` check to avoid drift

### Camera Pose
- Position: `[0.15, 0.15, 0.15]` - isometric-ish view
- Target: `[0.0, 0.0, 0.0]` - gripper root
- Adjust if gripper is clipped or too far

### Video Settings
- Codec: H.264 (libx264) - universal compatibility
- Quality: 8/10 - good balance of size and clarity
- Pixel format: yuv420p - standard for playback

## Future Enhancements

- [ ] Add `--video_fps` argument for user control
- [ ] Add multiple camera angles (side, top, front)
- [ ] Add joint logger integration (separate video from data plots)
- [ ] Add overlaid text showing pressure/gap values in video
- [ ] Support headless rendering for EC2 deployment

---

# Part 2: Gripper Logger

## Overview

Create a custom logger for gripper-specific metrics that captures the pressure-to-gap relationship during simulation. Unlike `JointLogger` which produces time-series plots, `GripperLogger` will visualize the pressure-gap mapping to validate the control system.

## GripperLogger Design

### Data to Log

Capture gripper-specific properties at each timestep:

1. **Pressure** - Command pressure (scalar per environment)
2. **Gap** - Per-finger gaps (2 values per environment) 
3. **Gap Velocity** - Per-finger velocities (2 values per environment)

### Storage Structure

```python
# PyTorch buffers [num_envs, num_steps]
self.pressure_buffer = torch.zeros((num_envs, num_steps), device=device)

# Per-finger data [num_envs, num_steps, 2]
self.gap_buffer = torch.zeros((num_envs, num_steps, 2), device=device)
self.gap_velocity_buffer = torch.zeros((num_envs, num_steps, 2), device=device)
```

### Plot Design

**Pressure vs Total Gap Scatter Plot:**

- **X-axis**: Total gap = `sum(abs(gap[:, 0]) + abs(gap[:, 1]))` 
  - Sums absolute values of both finger gaps
  - Represents total gripper opening
- **Y-axis**: Pressure (kPa)
- **Plot type**: Scatter plot with optional line connecting sequential points
- **Purpose**: Visualize hysteresis and validate pressure-to-gap mapping

**Additional Time-Series Subplots (optional):**
- Pressure vs time
- Total gap vs time  
- Gap velocity vs time (per finger or combined)

### Class Structure

```python
class GripperLogger:
    """Logger for gripper pressure and gap metrics.
    
    Captures gripper-specific data and generates pressure-gap plots
    to visualize the control system behavior and validate the 
    pressure-to-gap mapping.
    """
    
    def __init__(self, gripper: Gripper, sim_dt: float, duration: float, device: str = "cuda:0"):
        """Initialize the gripper logger.
        
        Args:
            gripper: Gripper instance to log data from
            sim_dt: Physics timestep in seconds
            duration: Total simulation duration in seconds
            device: PyTorch device for tensor storage
        """
        self.gripper = gripper
        self.sim_dt = sim_dt
        self.duration = duration
        self.device = device
        
        self.num_envs = gripper.num_instances
        self.num_steps = int(duration / sim_dt)
        
        # Initialize buffers
        self.pressure_buffer = torch.zeros(
            (self.num_envs, self.num_steps), device=device, dtype=torch.float32
        )
        self.gap_buffer = torch.zeros(
            (self.num_envs, self.num_steps, 2), device=device, dtype=torch.float32
        )
        self.gap_velocity_buffer = torch.zeros(
            (self.num_envs, self.num_steps, 2), device=device, dtype=torch.float32
        )
    
    def log(self, time_index: int):
        """Log current gripper state at specified time index.
        
        Args:
            time_index: Index in time dimension (0 to num_steps-1)
        """
        if time_index < 0 or time_index >= self.num_steps:
            return
        
        # Log pressure, gap, and gap velocity from gripper properties
        self.pressure_buffer[:, time_index] = self.gripper.pressure.clone()
        self.gap_buffer[:, time_index, :] = self.gripper.gap.clone()
        self.gap_velocity_buffer[:, time_index, :] = self.gripper.gap_velocity.clone()
    
    def plot(self, output_dir: str = "./output"):
        """Generate and save pressure-gap relationship plot.
        
        Creates two plots:
        1. Main: Pressure vs Total Gap (scatter with connecting lines)
        2. Time-series: Pressure, Gap, and Velocity over time
        
        Args:
            output_dir: Directory to save plots
        """
        # Implementation details below...
```

## Implementation Details

### Total Gap Computation

```python
# In plot() method:
# Convert to numpy
pressure_data = self.pressure_buffer.cpu().numpy()  # [num_envs, num_steps]
gap_data = self.gap_buffer.cpu().numpy()            # [num_envs, num_steps, 2]
gap_vel_data = self.gap_velocity_buffer.cpu().numpy()  # [num_envs, num_steps, 2]

# Compute total gap: sum of absolute values
total_gap = np.abs(gap_data[:, :, 0]) + np.abs(gap_data[:, :, 1])  # [num_envs, num_steps]
```

### Plot 1: Pressure vs Gap

```python
fig, ax = plt.subplots(figsize=(10, 8))

for env_idx in range(self.num_envs):
    # Scatter plot showing all data points
    ax.scatter(
        total_gap[env_idx, :], 
        pressure_data[env_idx, :],
        alpha=0.6,
        s=20,
        label=f'Env {env_idx}'
    )
    
    # Optional: Connect sequential points to show temporal order
    ax.plot(
        total_gap[env_idx, :], 
        pressure_data[env_idx, :],
        alpha=0.3,
        linewidth=0.5
    )

ax.set_xlabel('Total Gap (m)', fontsize=12)
ax.set_ylabel('Pressure (kPa)', fontsize=12)
ax.set_title('Pressure vs Gap Relationship', fontsize=14, fontweight='bold')
ax.grid(True, alpha=0.3)
ax.legend()

plt.savefig(os.path.join(output_dir, "gripper_pressure_gap.png"), dpi=150)
```

### Plot 2: Time-Series (Optional)

```python
fig, axes = plt.subplots(3, 1, figsize=(12, 10), sharex=True)

time_array = np.arange(self.num_steps) * self.sim_dt

for env_idx in range(self.num_envs):
    # Pressure over time
    axes[0].plot(time_array, pressure_data[env_idx, :], label=f'Env {env_idx}')
    axes[0].set_ylabel('Pressure (kPa)')
    axes[0].grid(True, alpha=0.3)
    
    # Total gap over time
    axes[1].plot(time_array, total_gap[env_idx, :], label=f'Env {env_idx}')
    axes[1].set_ylabel('Total Gap (m)')
    axes[1].grid(True, alpha=0.3)
    
    # Gap velocity (both fingers) over time
    axes[2].plot(time_array, gap_vel_data[env_idx, :, 0], 
                 label=f'Env {env_idx} Finger 0', alpha=0.7)
    axes[2].plot(time_array, gap_vel_data[env_idx, :, 1], 
                 label=f'Env {env_idx} Finger 1', alpha=0.7, linestyle='--')
    axes[2].set_ylabel('Gap Velocity (m/s)')
    axes[2].set_xlabel('Time (s)')
    axes[2].grid(True, alpha=0.3)

for ax in axes:
    ax.legend(fontsize=8)

fig.suptitle('Gripper Metrics Over Time', fontsize=14, fontweight='bold')
plt.tight_layout()

plt.savefig(os.path.join(output_dir, "gripper_time_series.png"), dpi=150)
```

## Integration into sim_day1.py

### 1. Import GripperLogger

```python
from src.gripper_logger import GripperLogger
```

### 2. Initialize in main()

```python
# After sim.reset() and gripper initialization
gripper = scene["gripper"]

# Initialize gripper logger
gripper_logger = GripperLogger(
    gripper=gripper,
    sim_dt=SIM_DT,
    duration=DURATION,
    device=args_cli.device
)

print("[INFO]: Gripper logger initialized")
```

### 3. Log in Simulation Loop

```python
def run_simulator(sim, scene, camera=None, video_writer=None, gripper_logger=None):
    # ... existing setup ...
    
    time_index = 0
    
    while simulation_app.is_running() and sim_time < DURATION:
        # ... pressure control logic ...
        
        # Update scene
        scene.update(SIM_DT)
        
        # Log gripper state
        if gripper_logger is not None:
            gripper_logger.log(time_index)
        
        time_index += 1
        sim_time += SIM_DT
    
    # Generate plots at end
    if gripper_logger is not None:
        gripper_logger.plot(output_dir="/workspace/isaaclab/source/GimbalLock/output")
```

### 4. Pass to run_simulator

```python
# In main()
run_simulator(sim, scene, camera, video_writer, gripper_logger)
```

## Expected Outputs

After simulation completes, two plots are saved:

1. **`gripper_pressure_gap.png`** - Scatter plot showing pressure-gap relationship
   - Should show roughly linear relationship based on configured mapping
   - Hysteresis loops visible if control dynamics introduce lag
   - Validates `pressure_to_gap_mapping` configuration

2. **`gripper_time_series.png`** - Three subplots showing:
   - Pressure command over time (square wave toggling)
   - Total gap response over time (smooth transitions)
   - Per-finger gap velocities (transient spikes during transitions)

## File Creation

Create new file: `src/gripper_logger.py`

Structure similar to `joint_logger.py`:
- Same style/conventions
- Copyright header
- Dark background matplotlib style
- Comprehensive docstrings
- Error handling for out-of-bounds indices

## Implementation Checklist

- [ ] Create `src/gripper_logger.py` with `GripperLogger` class
- [ ] Implement `__init__()` with buffer initialization
- [ ] Implement `log()` method to capture gripper state
- [ ] Implement `plot()` method with pressure-gap scatter plot
- [ ] Add optional time-series subplots
- [ ] Import `GripperLogger` in `sim_day1.py`
- [ ] Initialize logger in `main()`
- [ ] Add `gripper_logger` parameter to `run_simulator()`
- [ ] Call `logger.log(time_index)` in simulation loop
- [ ] Call `logger.plot()` at simulation end
- [ ] Test with single environment
- [ ] Test with multiple environments (verify per-env plotting)

## Technical Considerations

### Memory Usage

For 10s at 100 Hz = 1000 steps, single environment:
```
pressure: 1000 * 4 bytes = 4 KB
gap: 1000 * 2 * 4 bytes = 8 KB  
gap_velocity: 1000 * 2 * 4 bytes = 8 KB
Total per env: ~20 KB (negligible)
```

### Validation Use Cases

The pressure-gap plot validates:
1. **Linearity**: Does observed gap match configured mapping?
2. **Hysteresis**: Is there lag between pressure commands and gap response?
3. **Stability**: Does the control system reach steady-state?
4. **Symmetry**: Do both fingers behave identically?

### Debug Information

Add optional console output:
```python
print(f"[INFO]: Gripper logger captured {self.num_steps} timesteps")
print(f"[INFO]:   Pressure range: [{pressure_data.min():.2f}, {pressure_data.max():.2f}] kPa")
print(f"[INFO]:   Gap range: [{total_gap.min():.6f}, {total_gap.max():.6f}] m")
```

