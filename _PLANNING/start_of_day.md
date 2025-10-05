# Start of Day

To day I need to have pressure to position mapping to control then pneumatic gripper. This also needs to include parameters for tuning force.

Ideally this maping it done in the `gap_space` and not in joint space. This will allow for the designing and testing of the control to be around the data sheet. 

## Gap Space Controller.
f(p, s) = tau

p: pressure
s: robot state
tau: torque to apply to joints.


def calc_g(s):
    pos = s.finger_positions # Shape [envs, fingers, 3]
    return abs(pos[:, 0, 0] - pos[:, 0, 1])

def calc_gdot(s):
    pos = s.finger_vel # Shape [envs, fingers, 3]
    return abs(pos[:, 0, 0] - pos[:, 0, 1])

def calc_gs(p):
    return a + p*b

def f(p, s):
    g = calc_g(s)
    gs = calc_gs(p)
    tau = kp(g-gs) + kd*gdot


## Custom gripper articulation

I want my own custom write data to sim method that will write to the dof actuation forces directly for me. This is a bet so I need to try this first


    def write_data_to_sim(self):
        """Ripping out all of parent write_data_to_sim and 
        """

        self.root_physx_view.set_dof_actuation_forces(self._joint_effort_target_sim, self._ALL_INDICES)


    def set_pressure(self, pressure: Tensor):
        """This is the custom method to to take presure and then update the joint_effort_target_sim"""


## Keyboard settings

Do this, I need it now!