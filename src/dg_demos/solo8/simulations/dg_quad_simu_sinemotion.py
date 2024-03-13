## This code is a simulation for a sinemotion using the impedance controller

## Author: Avadesh Meduri
## Date: 12/02./2019


import dg_blmc_robots
from dg_blmc_robots.solo.solo_bullet import get_robot
from dg_tools.traj_generators import *

import pinocchio as se3
from pinocchio.utils import zero

from dg_tools.leg_impedance_control.utils import *
from dg_tools.leg_impedance_control.quad_leg_impedance_controller import (
    QuadrupedLegImpedanceController,
)

##########################################################################################
# Get the robot corresponding to the quadruped.
robot = get_robot()

# Define the desired position.
q = zero(robot.pin_robot.nq)
dq = zero(robot.pin_robot.nv)

q[0] = 0.2
q[1] = 0.0
q[2] = 0.4
q[6] = 1.0
for i in range(4):
    q[7 + 2 * i] = 0.8
    q[8 + 2 * i] = -1.6

# Update the initial state of the robot.
robot.reset_state(q, dq)

#########################################################################################

des_pos, des_vel = sine_generator(0.08, 3.5, 0.0, -0.2, "hopper")

pos_des = constVector([0.0, 0.0, -0.25], "pos_des")
# For making gain input dynamic through terminal
kp = Add_of_double("Kd")
kp.sin1.value = 0
### Change this value for different gains
kp.sin2.value = 100.0
Kp = kp.sout

# For making gain input dynamic through terminal
kd = Add_of_double("Kd")
kd.sin1.value = 0
### Change this value for different gains
kd.sin2.value = 10.0
Kd = kp.sout


control_torques = QuadrupedLegImpedanceController(
    robot, des_pos, des_pos, des_pos, des_pos, Kp
)

plug(control_torques, robot.device.ctrl_joint_torques)


#########################################################################################

robot.run(10000, 1.0 / 60.0)
