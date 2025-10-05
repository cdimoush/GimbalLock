# Gripper Demo - Coding Spec

## Overview
Simple gripper simulation with pressure-based control. This is a **hackathon demo** - keep it simple!

## Goal
Map air pressure → gripper gap width, with PD tuning for force/velocity response.

---

## Part 1: Custom Gripper Class (`src/gripper.py`)

### Extends `Articulation`
Subclass Isaac Lab's `Articulation` to add pressure control.

### Key Components

#### 1. Configuration Class: `GripperCfg`
```
Inherits from: ArticulationCfg
Additional fields:
  - kp: float (proportional gain for gap controller)
  - kd: float (derivative gain for gap controller)  
  - pressure_to_gap_a: float (gap = a + b*pressure)
  - pressure_to_gap_b: float
```

#### 2. Main Class: `Gripper`
```
Inherits from: Articulation
Additional state:
  - _target_pressure: torch.Tensor [num_envs] - desired pressure per env
```

#### 3. Core Methods

**`set_pressure(pressure: torch.Tensor, env_ids=None)`**
- Input: pressure values [num_envs] or subset
- Updates internal `_target_pressure` buffer
- Does NOT apply to sim (that happens in write_data_to_sim)

**`write_data_to_sim()` [OVERRIDE]**
- Calculate current gap `g` from finger positions
- Calculate gap velocity `g_dot` from finger velocities  
- Calculate target gap `g_target = a + b * pressure`
- Compute force: `tau = kp * (g_target - g) - kd * g_dot`
- Apply symmetric forces to both finger joints
- Call `root_physx_view.set_dof_actuation_forces(forces, indices)`

**Helper: `_compute_gap() -> torch.Tensor`**
- Get finger body positions in world frame
- Return distance between fingertip bodies [num_envs]

**Helper: `_compute_gap_velocity() -> torch.Tensor`**  
- Get finger body velocities
- Return relative velocity (closing speed) [num_envs]

#### 4. Pre-configured Gripper Config: `GRIPPER_CFG`

Export a ready-to-use configuration constant so sim files can just import and use it.

**`GRIPPER_CFG: GripperCfg`**
- Based on existing gripper USD file path (`/workspace/isaaclab/source/GimbalLock/models/gripper/usd/robot.usd`)
- Includes all spawn/rigid body/articulation settings from `sim_day0.py`
- Sets PD gains to starting values (kp=50, kd=10)  
- Sets pressure mapping (a=0.0, b=0.05)
- Actuators config with zero stiffness/damping (force control only)

**Usage**: 
```python
from src.gripper import Gripper, GRIPPER_CFG

# In scene config
gripper = GRIPPER_CFG.replace(prim_path="{ENV_REGEX_NS}/Gripper")
```

---

## Part 2: Simulation Script (`scripts/sim_day1.py`)

### Structure
Copy from `sim_day0.py`, modify to use `Gripper`.

### Changes from Day 0
1. **Import**: `from src.gripper import Gripper, GRIPPER_CFG`
2. **Config**: Use `GRIPPER_CFG` (no need to define inline!)
3. **Scene Config**: Replace gripper with `GRIPPER_CFG.replace(prim_path=...)`
4. **Control Loop**: Replace position commands with pressure commands

### Simulation Loop
```
Every N frames:
  - Toggle between high/low pressure
  - Call gripper.set_pressure(pressure_value)
  
Every frame:
  - scene.write_data_to_sim() [calls our override]
  - sim.step()
  - scene.update()
```

### Print Debug Info
- Current gap width
- Target gap width  
- Applied forces

---

## Part 3: Keyboard Control (Optional - Time Permitting)

Add keyboard callback:
- `+` / `=` : increase pressure
- `-` / `_` : decrease pressure
- `0` : reset to zero pressure

---

## Implementation Notes

### Simplifications (Keep It Simple!)
1. **Assume 2 DOF gripper**: 1 joint per finger, symmetric motion
2. **Hardcode finger body names**: No fancy string matching - just use indices [0, 1]
3. **Gap = single scalar distance**: Not full 3D position tracking
4. **Linear pressure mapping**: No fancy curves, just `y = a + b*x`
5. **Same force both joints**: Symmetric, equal magnitude forces
6. **No clamping/safety**: Trust the physics engine (for now)

### Tuning Parameters (Starting Values)
- `kp = 50.0` - stiff enough to reach target quickly
- `kd = 10.0` - damping to prevent oscillation
- `pressure_to_gap_a = 0.0` - closed at zero pressure
- `pressure_to_gap_b = 0.05` - 5cm per unit pressure

### Expected Behavior
1. Zero pressure → fingers close/touch
2. High pressure → fingers open to gap width
3. PD controller smooths motion, provides force compliance

---

## Success Criteria

### Minimal (Must Have)
- ✅ `Gripper` class loads and doesn't crash
- ✅ `set_pressure()` updates internal state
- ✅ `write_data_to_sim()` applies forces to joints
- ✅ Simulation runs with pressure commands

### Good (Should Have)  
- ✅ Gap actually correlates with pressure
- ✅ PD control prevents wild oscillations
- ✅ Can toggle between open/closed states

### Stretch (Nice to Have)
- ✅ Keyboard control works
- ✅ Can place object between fingers and grip it
- ✅ Debug visualization of forces/gaps

---

## File Structure
```
GimbalLock/
├── src/
│   └── gripper.py          [NEW: Gripper + GripperCfg + GRIPPER_CFG]
├── scripts/
│   ├── sim_day0.py         [DONE: proof of concept]
│   └── sim_day1.py         [NEW: pressure control demo - imports from src]
└── _PLANNING/
    ├── start_of_day.md     [EXISTING: initial thoughts]
    └── initial_spec.md     [THIS FILE]
```

---

## Key Architectural Decision

**Why override `write_data_to_sim()`?**
- Isaac Lab's actuator framework is designed for motor models (position/velocity/effort targets)
- We want **direct force control** based on a custom state (pressure)
- Cleanest approach: bypass actuator layer, write forces directly to PhysX
- See `articulation.py:184-221` for parent implementation

**Trade-off**: We lose actuator model features (effort limits, etc.) but gain simplicity for demo.

---

## Anti-Patterns to Avoid

❌ **Don't**: Create complex state machines  
✅ **Do**: Simple pressure value, PD control

❌ **Don't**: Make it configurable for N-finger grippers  
✅ **Do**: Hardcode for 2-finger case

❌ **Don't**: Add safety limits, clamping, error handling (yet)  
✅ **Do**: Trust physics, add guards only if things break

❌ **Don't**: Optimize performance  
✅ **Do**: Make it work first, optimize never (it's a demo!)

❌ **Don't**: Write unit tests  
✅ **Do**: Visual confirmation in sim is enough

---

## Timeline Estimate
- **30 min**: Write `Gripper` class skeleton
- **30 min**: Implement gap calculation and force application  
- **20 min**: Update `sim_day1.py` to use new class
- **10 min**: Debug and tune PD gains
- **10 min**: Add keyboard control (if time)

**Total: ~2 hours** for core functionality
