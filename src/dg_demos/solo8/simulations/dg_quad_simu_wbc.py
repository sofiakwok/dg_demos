##
## Author: Avadesh Meduri
## Date: 1/03/2019

from dg_tools.leg_impedance_control.utils import *
from dg_tools.leg_impedance_control.quad_leg_impedance_controller import (
    QuadrupedComControl,
    QuadrupedLegImpedanceController,
)
from dg_tools.traj_generators import mul_double_vec_2
from dg_blmc_robots.vicon_client_bullet import (
    ViconClientEntityBullet as ViconClientEntity,
)

###### robot init #######################################################
# Get the robot corresponding to the quadruped.

import time
import dg_blmc_robots
from dg_blmc_robots.solo.solo_bullet import get_robot

import pinocchio as se3
from pinocchio.utils import zero

robot = get_robot(record_video=False)

# Define the desired position.
q = zero(robot.pin_robot.nq)
dq = zero(robot.pin_robot.nv)

q[0] = 0.0
q[1] = 0.0
q[2] = 0.22
q[3] = 0.0
q[4] = 0.0
q[5] = 0.0
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

###############################################################################
kp_com = constVector([100.0, 0.0, 100.0], "kp_com")
kd_com = constVector([5.0, 0.0, 5.0], "kd_com")
kp_ang_com = constVector([100.0, 100.0, 0.0], "kp_ang_com")
kd_ang_com = constVector([2.0, 2.0, 0.0], "kd_ang_com")
des_pos_com = constVector([0.0, 0.0, 0.25], "des_pos_com")
des_vel_com = constVector([0.0, 0.0, 0.0], "des_vel_com")
des_fff_com = constVector([0.0, 0.0, 2.32 * 9.8], "des_fff_com")
des_ori_com = constVector([0.0, 0.0, 0.0, 1.0], "des_com_ori")
des_ang_vel_com = constVector([0.0, 0.0, 0.0], "des_com_omega")
des_fft_com = constVector([0.0, 0.0, 0.0], "des_fft_com")
des_abs_vel = constVector(
    [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], "des_abs_vel"
)


pos_des = constVector(
    [
        0.0,
        0.0,
        -0.22,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        -0.22,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        -0.22,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        -0.22,
        0.0,
        0.0,
        0.0,
    ],
    "des_pos",
)

vel_des = zero_vec(24, "des_vel")

###############################################################################


def gen_r_matrix(rx, ry, rz):
    R = np.matrix([[0, -rz, ry], [rz, 0, -rx], [-ry, rx, 0]])
    return R


def gen_v_matrix(vx, vy, vz):
    V = np.matrix([[vx, 0, 0], [0, vy, 0], [0, 0, vz]])
    return V


###############################################################################


I = np.matrix([[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]])

r_fl = gen_r_matrix(0.2, 0.15, 0.0)
r_fr = gen_r_matrix(0.2, -0.15, 0.0)
r_hl = gen_r_matrix(-0.2, 0.15, 0.0)
r_hr = gen_r_matrix(-0.2, -0.15, 0.0)

v_fl = gen_v_matrix(0.0, 0.0, 0.0)
v_fr = gen_v_matrix(0.0, 0.0, 0.0)
v_hl = gen_v_matrix(0.0, 0.0, 0.0)
v_hr = gen_v_matrix(0.0, 0.0, 0.0)

zm = np.zeros((3, 3))

py_ce = np.block(
    [
        [I, I, I, I, -I, zm, zm, zm, zm, zm],
        [r_fl, r_fr, r_hl, r_hr, zm, -I, zm, zm, zm, zm],
        [v_fl, zm, zm, zm, zm, zm, -I, zm, zm, zm],
        [zm, v_fr, zm, zm, zm, zm, zm, -I, zm, zm],
        [zm, zm, v_hl, zm, zm, zm, zm, zm, -I, zm],
        [zm, zm, zm, v_hr, zm, zm, zm, zm, zm, -I],
    ]
)

py_ce = np.asarray(py_ce)

w1 = 1.0
w2 = 1.0
py_hess = np.zeros((30, 30))
np.fill_diagonal(py_hess, w1)

py_reg = np.zeros((30, 30))
np.fill_diagonal(py_reg, 0.0001)

hess = constMatrix(py_hess, "hess")
reg = constMatrix(py_reg, "regularizer")
g0 = zero_vec(30, "g0")
ce = constMatrix(py_ce, "ce")
ce0 = zero_vec(18, "ce0")
ci = constMatrix(np.zeros((18, 30)), "ci")
ci0 = zero_vec(18, "ci0")
###############################################################################

quad_com_ctrl = QuadrupedComControl(robot, ViconClientEntity, "solo")
lctrl = quad_com_ctrl.compute_torques(
    kp_com, des_pos_com, kd_com, des_vel_com, des_fff_com
)
actrl = quad_com_ctrl.compute_ang_control_torques(
    kp_ang_com, des_ori_com, kd_ang_com, des_ang_vel_com, des_fft_com
)
# lctrl = zero_vec(3, "ltau")

com_torques = quad_com_ctrl.return_com_torques(
    lctrl, actrl, des_abs_vel, hess, g0, ce, ci, ci0, reg
)

############################################################################


###############################################################################
kp = constVector([0.0, 0.0, 0.0, 0.0, 0.0, 0.0], "kp_split")
kd = constVector([0.0, 0.0, 0.0, 0.0, 0.0, 0.0], "kd_split")

## setting desired position
des_pos = constVector(
    [
        0.0,
        0.0,
        -0.25,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        -0.25,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        -0.25,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        -0.25,
        0.0,
        0.0,
        0.0,
    ],
    "pos_des",
)

des_vel = constVector(
    [
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
    ],
    "vel_des",
)

des_fff = com_torques

add_kf = Add_of_double("kf")
add_kf.sin1.value = 0.0
### Change this value for different gains
add_kf.sin2.value = 1.0
kf = add_kf.sout

quad_imp_ctrl = QuadrupedLegImpedanceController(robot)
control_torques = quad_imp_ctrl.return_control_torques(
    kp, des_pos, kd, des_vel, kf, des_fff
)

plug(control_torques, robot.device.ctrl_joint_torques)

######## robot simulation ################################################
robot.run(5000, 1.0 / 60.0, plot=False)
