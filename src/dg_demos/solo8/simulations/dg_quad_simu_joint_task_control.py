## This code attempts to read a trajectory from a file and track it with the compliance CONTROLLER

## Author: Avadesh Meduri
## Date: 21/02/2019

from dg_tools.utils import *

# from leg_impedance_control.quad_leg_impedance_controller import quad_leg_impedance_controller
from dg_tools.leg_impedance_control.quad_leg_impedance_controller import (
    QuadrupedLegImpedanceController,
    QuadrupedComControl,
)
from dg_tools.traj_generators import mul_double_vec_2

from dynamic_graph.sot.core.reader import Reader

import time

# import py_dg_blmc_robots
# from py_dg_blmc_robots.quadruped import get_quadruped_robot
from dg_blmc_robots.solo.solo_bullet import get_robot
from os.path import join

###### robot init #######################################################
# Get the robot corresponding to the quadruped.

import pinocchio as se3
from pinocchio.utils import zero

robot = get_robot(
    use_fixed_base=False,
    record_video=False,
    init_sliders_pose=[0.5, 0.5, 0.0, 0.0],
)

# Define the desired position.
q = zero(robot.pin_robot.nq)
dq = zero(robot.pin_robot.nv)

q[0] = 0.2
q[1] = 0.0
q[2] = 0.22
q[6] = 1.0
q[7] = 0.82
q[8] = -1.48
q[9] = 0.82
q[10] = -1.48
q[11] = -0.82
q[12] = 1.48
q[13] = -0.82
q[14] = 1.48

# Gravity compensation
des_fff = constVector(
    [
        0.0,
        0.0,
        (2.2 * 9.8) / 4.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        (2.2 * 9.8) / 4.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        (2.2 * 9.8) / 4.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        (2.2 * 9.8) / 4.0,
        0.0,
        0.0,
        0.0,
    ],
    "fff",
)

# Update the initial state of the robot.
robot.reset_state(q, dq)


def file_exists(filename):
    if os.path.isfile(filename):
        print("The file %s exists" % filename)
    else:
        print("The file %s does not exist" % filename)


###############################################################################

### reading createData
reader_pos = Reader("PositionReader")
reader_vel = Reader("VelocityReader")
reader_joint_pos = Reader("JointPositionReader")
reader_joint_vel = Reader("JointVelocityReader")
reader_forces = Reader("forces")

filename_pos = join(
    rospkg.RosPack().get_path("momentumopt"),
    "nodes",
    "quadruped_positions_eff.dat",
)
filename_vel = join(
    rospkg.RosPack().get_path("momentumopt"),
    "nodes",
    "quadruped_velocities_eff.dat",
)

filename_joint_pos = join(
    rospkg.RosPack().get_path("momentumopt"),
    "nodes",
    "quadruped_positions.dat",
)
filename_joint_vel = join(
    rospkg.RosPack().get_path("momentumopt"),
    "nodes",
    "quadruped_velocities.dat",
)


file_exists(filename_pos)
file_exists(filename_vel)
file_exists(filename_joint_pos)
file_exists(filename_joint_vel)


print("Loading data files:")
reader_pos.load(filename_pos)
reader_vel.load(filename_vel)
reader_joint_pos.load(filename_joint_pos)
reader_joint_vel.load(filename_joint_vel)

# Specify which of the columns to select.
# NOTE: This is selecting the columns in reverse order - the last number is the first column in the file
reader_pos.selec.value = "111111111111111111111111"
reader_vel.selec.value = "111111111111111111111111"
reader_joint_pos.selec.value = "111111110"
reader_joint_vel.selec.value = "111111110"

des_pos = reader_pos.vector
des_vel = reader_vel.vector
des_joint_pos = reader_joint_pos.vector
des_joint_vel = reader_joint_vel.vector
# des_joint_pos =constVector([0.5, -1., 0.5, -1., 0.5, -1., 0.5, -1.], "des_joint_pos")
# des_joint_vel =constVector([.0, .0, .0, .0, .0, .0, .0, .0], "des_joint_vel")

###############################################################################

kp = constVector([0.0, 0.0, 0.0, 0.0, 0.0, 0.0], "kp_split")
kd = constVector([0.0, 0.0, 0.0, 0.0, 0.0, 0.0], "kd_split")

add_kpj = Add_of_double("kpj")
add_kpj.sin1.value = 0
### Change this value for different gains
add_kpj.sin2.value = 15.0
kpj = add_kpj.sout


kp_joint = mul_double_vec_2(
    kpj,
    constVector([1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0], "unit"),
    "kpj_split",
)
kd_joint = constVector([0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1], "kdj_split")

# kp_joint = constVector([0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1], "kpj_split")
# kd_joint = constVector([0., 0., 0., 0., 0., 0., 0., 0.], "kdj_split")

add_kf = Add_of_double("kf")
add_kf.sin1.value = 0
### Change this value for different gains
add_kf.sin2.value = 1.0
kf = add_kf.sout

quad_imp_ctrl = QuadrupedLegImpedanceController(robot)
joint_ctrl_torques = quad_imp_ctrl.return_joint_ctrl_torques(
    kp_joint, des_joint_pos, kd_joint, des_joint_vel
)
task_control_torques = quad_imp_ctrl.return_control_torques(
    kp, des_pos, kd, des_vel, kf, des_fff
)
control_torques = add_vec_vec(
    joint_ctrl_torques, task_control_torques, "control_torques"
)

plug(control_torques, robot.device.ctrl_joint_torques)


##############################################################################

robot.run(19000, 1.0 / 60.0)
