## This code attempts to read a trajectory from a file and track it with the compliance CONTROLLER

## Author: Avadesh Meduri
## Date: 21/02/2019

from dg_tools.utils import *
from dg_tools.leg_impedance_control.quad_leg_impedance_controller import (
    QuadrupedLegImpedanceController,
    QuadrupedComControl,
)

from dynamic_graph.sot.core.reader import Reader

import time
import dg_blmc_robots
from dg_blmc_robots.solo.solo_bullet import get_robot
from dg_blmc_robots.vicon_client_bullet import (
    ViconClientEntityBullet as ViconClientEntity,
)

###### robot init #######################################################
# Get the robot corresponding to the quadruped.

import pinocchio as se3
from pinocchio.utils import zero

robot = get_robot(record_video=False)

# Define the desired position.
q = zero(robot.pin_robot.nq)
dq = zero(robot.pin_robot.nv)

q[0] = 0.0
q[1] = 0.0
q[2] = 0.22
q[6] = 1.0
q[7] = 0.8
q[8] = -1.6
q[9] = 0.8
q[10] = -1.6
q[11] = -0.8
q[12] = 1.6
q[13] = -0.8
q[14] = 1.6

# Update the initial state of the robot.
robot.reset_state(q, dq)

#############################################################################


def file_exists(filename):
    if os.path.isfile(filename):
        print("The file %s exists" % filename)
    else:
        print("The file %s does not exist" % filename)
        assert False


#############################################################################
### reading createData
reader_pos = Reader("PositionReader")
reader_vel = Reader("VelocityReader")
reader_com = Reader("ComReader")
reader_lmom = Reader("Lmom")
reader_amom = Reader("Amom")
reader_forces = Reader("forces")
reader_lqr1 = Reader("lqr1")
reader_lqr2 = Reader("lqr2")
reader_lqr3 = Reader("lqr3")

filename_pos = "./../../../../../control/kino-dynamic-opt/momentumopt/demos/quadruped_positions_eff.dat"
filename_vel = "./../../../../../control/kino-dynamic-opt/momentumopt/demos/quadruped_velocities_eff.dat"
filename_com = "./../../../../../control/kino-dynamic-opt/momentumopt/demos/quadruped_com.dat"
filename_lmom = "./../../../../../control/kino-dynamic-opt/momentumopt/demos/quadruped_lmom.dat"
filename_amom = "./../../../../../control/kino-dynamic-opt/momentumopt/demos/quadruped_amom.dat"
filename_forces = "./../../../../../control/kino-dynamic-opt/momentumopt/demos/quadruped_forces.dat"
filename_lqr1 = "./../../../../../control/kino-dynamic-opt/momentumopt/demos/quadruped_lqr1.dat"
filename_lqr2 = "./../../../../../control/kino-dynamic-opt/momentumopt/demos/quadruped_lqr2.dat"
filename_lqr3 = "./../../../../../control/kino-dynamic-opt/momentumopt/demos/quadruped_lqr3.dat"

# tmp = np.loadtxt(filename_lqr1)


file_exists(filename_pos)
file_exists(filename_vel)
file_exists(filename_com)
file_exists(filename_lmom)
file_exists(filename_amom)
file_exists(filename_forces)
file_exists(filename_lqr1)
file_exists(filename_lqr2)
file_exists(filename_lqr3)

print("Loading data files:")
reader_pos.load(filename_pos)
reader_vel.load(filename_vel)
reader_com.load(filename_com)
reader_lmom.load(filename_lmom)
reader_amom.load(filename_amom)
reader_forces.load(filename_forces)
reader_lqr1.load(filename_lqr1)
reader_lqr2.load(filename_lqr2)
reader_lqr3.load(filename_lqr3)


# Specify which of the columns to select.
# NOTE: This is selecting the columns in reverse order - the last number is the first column in the file
reader_pos.selec.value = "111111111111111111111111"
reader_vel.selec.value = "111111111111111111111111"
reader_com.selec.value = "1110"
reader_lmom.selec.value = "1110"
reader_amom.selec.value = "1110"
reader_forces.selec.value = "1111111111110"
reader_lqr1.selec.value = 36 * "1"
reader_lqr2.selec.value = 36 * "1"
reader_lqr3.selec.value = 36 * "1"

len(reader_lqr1.selec.value)

des_pos = reader_pos.vector
des_vel = reader_vel.vector
des_com = reader_com.vector
des_lmom = reader_com.vector
des_amom = reader_amom.vector
des_forces = reader_forces.vector
### sot_reader can read a vector of max size 40.
des_lqr1 = reader_lqr1.vector
des_lqr2 = reader_lqr2.vector
des_lqr3 = reader_lqr3.vector

# des_lqr = stack_two_vectors(des_lqr1, des_lqr2, 36, 36)
# des_lqr = stack_two_vectors(des_lqr, des_lqr3, 72, 36)

des_lqr = zero_vec(108, "zero_lqr")

###############################################################################

quad_com_ctrl = QuadrupedComControl(robot, ViconClientEntity)
f_lqr = quad_com_ctrl.return_lqr_tau(
    des_com, des_lmom, des_amom, des_forces, des_lqr
)

# ################################################################################
des_fff = f_lqr

###############################################################################

kp = constVector([250.0, 0.0, 250.0, 0.0, 0.0, 0.0], "kp_split")

kd = constVector([2.0, 0.0, 2.0, 0.0, 0.0, 0.0], "kd_split")

add_kf = Add_of_double("kf")
add_kf.sin1.value = 0
### Change this value for different gains
add_kf.sin2.value = 1.0
kf = add_kf.sout

quad_imp_ctrl = QuadrupedLegImpedanceController(robot)
control_torques = quad_imp_ctrl.return_control_torques(
    kp, des_pos, kd, des_vel, kf, des_fff
)

plug(control_torques, robot.device.ctrl_joint_torques)


##############################################################################

robot.run(5000, 1.0 / 60.0, plot=True)
