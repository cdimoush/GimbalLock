# Fatal Flaw

## Problem

The idea of gap control is great, but if instability gets introduced, controlling in gap space (relative to the two finger tips) becomes wildly unstable.

**Why?** When we control the distance between two fingertips, we create a **coupled system**:
- Both fingers respond to the same error signal
- If finger 0 overshoots, the controller tries to correct both fingers
- This creates oscillations that feed back between the two fingers
- The system can diverge because there's no independent reference frame

## Solution

**Use independent per-finger control** where each finger is controlled relative to a stable reference: the gripper root.

Instead of controlling:
- `gap_total = distance(finger0, finger1)` → scalar per environment

We control:
- `gap[0] = projection(finger0 - root, root_x_axis)` → per-finger value
- `gap[1] = projection(finger1 - root, root_x_axis)` → per-finger value

This gives us **decoupled control** where each finger has its own setpoint and error signal.

## Implementation Details

### Shape Changes
- **Before**: `gap` returns shape `[num_envs]` 
- **After**: `gap` returns shape `[num_envs, 2]` where index 0 = finger0, index 1 = finger1

### Available Infrastructure from Parent Classes

From `Articulation` and `ArticulationData`, we have access to:
- `self.data.root_link_pos_w` → Root position [num_envs, 3]
- `self.data.root_link_quat_w` → Root quaternion (w,x,y,z) [num_envs, 4]
- `self.data.root_link_vel_w` → Root velocity [lin, ang] [num_envs, 6]
- `self.data.body_link_pos_w` → All body positions [num_envs, num_bodies, 3]
- `self.data.body_link_vel_w` → All body velocities [num_envs, num_bodies, 6]
- `self.find_bodies()` → Get body indices by name pattern
- `math_utils.quat_apply()` → Rotate a vector by a quaternion
- `math_utils.quat_rotate_inverse()` → Rotate a vector by inverse quaternion

### 1. Computing Gap (`_compute_gap`)

**Concept**: For each finger, compute its distance from the gripper root along the root's x-axis.

**Simplified Strategy using Parent Class Data**:

```python
def _compute_gap(self) -> torch.Tensor:
    # 1. Get data from parent class (already in world frame)
    root_pos = self.data.root_link_pos_w  # [num_envs, 3]
    root_quat = self.data.root_link_quat_w  # [num_envs, 4] (w,x,y,z)
    
    # 2. Get finger tip positions using parent's find_bodies
    finger_indices, _ = self.find_bodies([".*ft0.*", ".*ft1.*"])
    finger_pos = self.data.body_link_pos_w[:, finger_indices]  # [num_envs, 2, 3]
    
    # 3. Define x-axis in local frame (this is what we want to project onto)
    x_axis_local = torch.tensor([1, 0, 0], device=self.device)
    
    # 4. Transform local x-axis to world frame using parent's math utils
    x_axis_world = math_utils.quat_apply(root_quat, x_axis_local.unsqueeze(0))  # [num_envs, 3]
    
    # 5. Compute vectors from root to each finger
    root_to_fingers = finger_pos - root_pos.unsqueeze(1)  # [num_envs, 2, 3]
    
    # 6. Project onto x-axis: dot product
    gaps = torch.sum(root_to_fingers * x_axis_world.unsqueeze(1), dim=-1)  # [num_envs, 2]
    
    return gaps
```

**Result**: `gaps` shape `[num_envs, 2]`

### 2. Computing Gap Velocity (`_compute_gap_velocity`)

**Concept**: Project each finger's velocity onto the root's x-axis independently.

**Simplified Strategy - Just magnitude along x-axis**:

```python
def _compute_gap_velocity(self) -> torch.Tensor:
    # 1. Get data from parent class
    root_quat = self.data.root_link_quat_w  # [num_envs, 4]
    
    # 2. Get finger velocities
    finger_lin_vel = self.data.body_link_vel_w[:, self._finger_body_indices, :3]  # [num_envs, 2, 3]
    
    # 3. Transform local x-axis to world frame
    x_axis_world = math_utils.quat_apply(root_quat, self._x_axis_local.unsqueeze(0))  # [num_envs, 3]
    
    # 4. Project finger velocities onto x-axis
    gap_vel = torch.sum(finger_lin_vel * x_axis_world.unsqueeze(1), dim=-1)  # [num_envs, 2]
    
    return gap_vel
```

**Result**: `gap_vel` shape `[num_envs, 2]`

**Why this works**:
- Each finger's velocity component along the root's x-axis is all we need for damping
- Positive = finger moving away from root along x-axis
- Negative = finger moving toward root along x-axis
- No need for complex relative velocities or rotating frame terms
- Simple and numerically stable

**Key Simplifications**:
- No manual quaternion-to-rotation-matrix conversion
- Use `math_utils.quat_apply()` for vector rotation
- Directly access `self.data.body_link_vel_w` properties
- Let parent class handle world-frame transformations

### 3. Writing Forces (`write_data_to_sim`)

**Concept**: Apply independent PD control to each finger.

**Simplified Strategy**:

```python
def write_data_to_sim(self):
    # 1. Compute current per-finger gaps and velocities [num_envs, 2]
    g = self._compute_gap()
    gdot = self._compute_gap_velocity()
    
    # 2. Compute target total gap from pressure [num_envs]
    gt_total = self._compute_gap_target(self._target_pressure)
    
    # 3. Split target equally between fingers [num_envs, 1]
    gt = (gt_total / 2.0).unsqueeze(-1)
    
    # 4. Compute per-finger control forces [num_envs, 2]
    tau = self._kp * (gt - g) - self._kd * gdot
    
    # 5. Apply forces to joints using parent's infrastructure
    joint_indices, _ = self.find_joints([".*f0", ".*f1"])
    self._joint_effort_target_sim[:, joint_indices[0]] = -tau[:, 0]
    self._joint_effort_target_sim[:, joint_indices[1]] = tau[:, 1]
    
    # 6. Write to simulation using parent's PhysX view
    self.root_physx_view.set_dof_actuation_forces(
        self._joint_effort_target_sim, self._ALL_INDICES
    )
```

**Why opposite signs?** 
- Positive projection on x-axis for finger0 means it's extended forward
- Positive projection for finger1 means it's extended backward (opposite direction)
- To close: apply negative force to f0, positive to f1
- To open: apply positive force to f0, negative to f1

### 4. Optimization: Cache Indices

**Problem**: Calling `find_bodies()` and `find_joints()` every timestep is inefficient.

**Solution**: Cache indices during initialization:

```python
def _initialize_impl(self):
    # Call parent initialization first
    super()._initialize_impl()
    
    # Cache finger body and joint indices
    self._finger_body_indices, _ = self.find_bodies([".*ft0.*", ".*ft1.*"])
    self._finger_joint_indices, _ = self.find_joints([".*f0", ".*f1"])
    
    # Initialize other buffers
    self._target_pressure = torch.zeros(self.num_instances, device=self.device)
    self._kp = self.cfg.kp
    self._kd = self.cfg.kd
    self._pressure_to_gap_mapping = self.cfg.pressure_to_gap_mapping
    
    # Define x-axis in local frame (constant)
    self._x_axis_local = torch.tensor([1.0, 0.0, 0.0], device=self.device)
```

Then use cached indices in compute methods:
```python
finger_pos = self.data.body_link_pos_w[:, self._finger_body_indices]
```

### 5. Required Imports

Add to top of `gripper.py`:
```python
import isaaclab.utils.math as math_utils
```

## Important Considerations

### Sign Convention
The gripper geometry determines which finger gets positive/negative projections:
- If fingers are symmetric about root, both may project positively in opposite world directions
- The key is that they move in **opposite directions along the local x-axis**
- Verify sign convention by testing: increasing pressure should increase gap

### Edge Cases
1. **Root rotation**: The implementation correctly handles root rotation via `quat_apply()`
2. **Zero velocity**: When gripper is stationary, angular velocity terms vanish correctly
3. **Numerical stability**: The cross product and dot products are numerically stable for unit vectors

### Tensor Broadcasting
- Be careful with unsqueeze operations to ensure correct broadcasting
- Shape annotations in comments help catch dimension mismatches
- Use `.expand()` instead of `.repeat()` when possible (no memory copy)

### Performance Notes
- `quat_apply()` is optimized in `math_utils` - don't reimplement
- Caching indices and constants saves repeated lookups
- Parent class properties use lazy evaluation with timestamps - accessing them is efficient

## Summary

By switching from coupled (finger-to-finger) control to independent (finger-to-root) control, we:
1. **Eliminate coupling** between finger errors
2. **Provide stable reference** (root frame moves with gripper base)
3. **Enable independent correction** of each finger's position
4. **Maintain symmetric behavior** through equal setpoints (total_gap / 2)
5. **Leverage parent infrastructure** (ArticulationData properties, math_utils, find_bodies/joints)
6. **Optimize performance** (cache indices, use parent's lazy evaluation)