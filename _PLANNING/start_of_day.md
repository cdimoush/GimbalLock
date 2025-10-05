# Start of Day

To day I need to have pressure to position mapping to control then pneumatic gripper. This also needs to include parameters for tuning force.

Ideally this maping it done in the `gap_space` and not in joint space. This will allow for the designing and testing of the control to be around the data sheet. 

## Gap Space Controller.
f(p, s) = tau

p: pressure
s: robot state
tau: torque to apply to joints.


## Custom gripper articulation

I want my own custom write data to sim method that will write to the dof actuation forces directly for me. This is a bet so I need to try this first