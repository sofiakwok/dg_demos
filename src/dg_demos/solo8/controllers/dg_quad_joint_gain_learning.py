## This is code to track desired trajectories from the planner using the whole body controller
## The gains have to be tuned with the sliders
## and leg impedance

##Author : Avadesh Meduri
# Date : 25/03/19

from dynamic_graph.sot.core.reader import Reader
from dg_tools.utils import *

# from leg_impedance_control.quad_leg_impedance_controller import quad_leg_impedance_controller
from dg_tools.leg_impedance_control.quad_leg_impedance_controller import (
    QuadrupedLegImpedanceController,
    QuadrupedComControl,
)
from dg_tools.traj_generators import mul_double_vec_2, scale_values
from dynamic_graph_manager.vicon_sdk import ViconClientEntity

from dynamic_graph.sot.core.switch import SwitchVector
from os.path import join

#############################################################################

# ## filter slider_value
from dynamic_graph.sot.core.fir_filter import FIRFilter_Vector_double

slider_filtered = FIRFilter_Vector_double("slider_fir_filter")
filter_size = 400
slider_filtered.setSize(filter_size)
for i in range(filter_size):
    slider_filtered.setElement(i, 1.0 / float(filter_size))
# we plug the centered sliders output to the input of the filter.
plug(robot.device.slider_positions, slider_filtered.sin)

slider_1_op = Component_of_vector("slider_1")
slider_1_op.setIndex(0)
plug(slider_filtered.sout, slider_1_op.sin)
slider_1 = slider_1_op.sout

slider_2_op = Component_of_vector("slider_2")
slider_2_op.setIndex(1)
plug(slider_filtered.sout, slider_2_op.sin)
slider_2 = slider_2_op.sout

slider_3_op = Component_of_vector("slider_3")
slider_3_op.setIndex(2)
plug(slider_filtered.sout, slider_3_op.sin)
slider_3 = slider_3_op.sout

slider_4_op = Component_of_vector("slider_4")
slider_4_op.setIndex(3)
plug(slider_filtered.sout, slider_4_op.sin)
slider_4 = slider_4_op.sout


kp_com = scale_values(slider_1, 1.0, "scale_kp_com")
kd_com = scale_values(slider_2, 1.0, "scale_kd_com")

kp_ang_com = scale_values(slider_3, 1.0, "scale_kp_ang_com")
kd_ang_com = scale_values(slider_4, 1.0, "scale_kd_ang_com")


unit_vec_101 = constVector([1.0, 0.0, 1.0], "unit_vec_101")
unit_vec_110 = constVector([1.0, 1.0, 0.0], "unit_vec_110")


kp_com = mul_double_vec_2(kp_com, unit_vec_101, "kp_com")
kd_com = mul_double_vec_2(kd_com, unit_vec_101, "kd_com")
kp_ang_com = mul_double_vec_2(kp_ang_com, unit_vec_110, "kp_ang_com")
kd_ang_com = mul_double_vec_2(kd_ang_com, unit_vec_110, "kd_ang_com")

################################################################################


def file_exists(filename):
    if os.path.isfile(filename):
        print("The file %s exists" % filename)
    else:
        print("The file %s does not exist" % filename)
        assert False


from dynamic_graph.sot.core.reader import Reader

reader_pos = Reader("PositionReader")
reader_vel = Reader("VelocityReader")
reader_abs_vel = Reader(
    "AbsVelReader"
)  # absolute velocity of the end_effector
reader_pos_com = Reader("PositionComReader")
reader_vel_com = Reader("VelocityComReader")
reader_fff_com = Reader("FeedForwardForceComReader")
reader_ori_com = Reader("OrientationComReader")
reader_ang_vel_com = Reader("AngVelComReader")
reader_fft_com = Reader("FeedForwardMomentsComReader")
reader_cnt_plan = Reader("CntPlan")
reader_joint_pos = Reader("JointPositionReader")
reader_joint_vel = Reader("JointVelocityReader")
reader_joint_pos_init = Reader("JointPositionInitReader")
reader_gain_ratio = Reader("GainRatioReader")

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
filename_abs_vel = join(
    rospkg.RosPack().get_path("momentumopt"),
    "nodes",
    "quadruped_velocities_abs.dat",
)
filename_cnt_plan = join(
    rospkg.RosPack().get_path("momentumopt"),
    "nodes",
    "quadruped_contact_activation.dat",
)
filename_pos_com = join(
    rospkg.RosPack().get_path("momentumopt"), "nodes", "quadruped_com.dat"
)
filename_vel_com = join(
    rospkg.RosPack().get_path("momentumopt"), "nodes", "quadruped_com_vel.dat"
)
filename_fff_com = join(
    rospkg.RosPack().get_path("momentumopt"),
    "nodes",
    "quadruped_centroidal_forces.dat",
)
filename_ori_com = join(
    rospkg.RosPack().get_path("momentumopt"),
    "nodes",
    "quadruped_quaternion.dat",
)
filename_ang_vel_com = join(
    rospkg.RosPack().get_path("momentumopt"),
    "nodes",
    "quadruped_base_ang_velocities.dat",
)
filename_fft_com = join(
    rospkg.RosPack().get_path("momentumopt"),
    "nodes",
    "quadruped_centroidal_moments.dat",
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
filename_joint_pos_init = join(
    rospkg.RosPack().get_path("momentumopt"),
    "nodes",
    "quadruped_positions_init.dat",
)
filename_gain_ratio = join(
    rospkg.RosPack().get_path("momentumopt"),
    "nodes",
    "quadruped_gain_ratio.dat",
)

file_exists(filename_pos)
file_exists(filename_vel)
file_exists(filename_abs_vel)
file_exists(filename_cnt_plan)
file_exists(filename_pos_com)
file_exists(filename_vel_com)
file_exists(filename_fff_com)
file_exists(filename_ori_com)
file_exists(filename_ang_vel_com)
file_exists(filename_fft_com)
file_exists(filename_joint_pos)
file_exists(filename_joint_vel)
file_exists(filename_joint_pos_init)
file_exists(filename_gain_ratio)

print("Loading data files:")
reader_pos.load(filename_pos)
reader_vel.load(filename_vel)
reader_abs_vel.load(filename_abs_vel)
reader_cnt_plan.load(filename_cnt_plan)
reader_pos_com.load(filename_pos_com)
reader_vel_com.load(filename_vel_com)
reader_fff_com.load(filename_fff_com)
reader_ori_com.load(filename_ori_com)
reader_ang_vel_com.load(filename_ang_vel_com)
reader_fft_com.load(filename_fft_com)
reader_joint_pos.load(filename_joint_pos)
reader_joint_vel.load(filename_joint_vel)
reader_joint_pos_init.load(filename_joint_pos_init)
reader_gain_ratio.load(filename_gain_ratio)

# Specify which of the columns to select.
# NOTE: This is selecting the columns in reverse order - the last number is the first column in the file
reader_pos.selec.value = "111111111111111111111111"
reader_vel.selec.value = "111111111111111111111111"
reader_abs_vel.selec.value = "1111111111110"
reader_cnt_plan.selec.value = "11110"
reader_pos_com.selec.value = "1110"
reader_vel_com.selec.value = "1110"
reader_ori_com.selec.value = "11110"
reader_fff_com.selec.value = "1110"
reader_ang_vel_com.selec.value = "1110"
reader_fft_com.selec.value = "1110"
reader_joint_pos.selec.value = "111111110"
reader_joint_vel.selec.value = "111111110"
reader_joint_pos_init.selec.value = "111111110"
reader_gain_ratio.selec.value = "111111110"

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

w1 = 10.0
w2 = 160000.0
w3 = 100.0
py_hess = np.zeros((30, 30))
np.fill_diagonal(py_hess, w1)


py_hess[12][12] = w3
py_hess[13][13] = w3
py_hess[14][14] = w3
py_hess[15][15] = w3
py_hess[16][16] = w3
py_hess[17][17] = w3

py_hess[18][18] = w2
py_hess[19][19] = w2
py_hess[20][20] = w2
py_hess[21][21] = w2
py_hess[22][22] = w2
py_hess[23][23] = w2
py_hess[24][24] = w2
py_hess[25][25] = w2
py_hess[26][26] = w2
py_hess[27][27] = w2
py_hess[28][28] = w2
py_hess[29][29] = w2


py_reg = np.zeros((30, 30))
np.fill_diagonal(py_reg, 0.0001)

hess = constMatrix(py_hess, "hess")
reg = constMatrix(py_reg, "regularizer")
g0 = zero_vec(30, "g0")
ce = constMatrix(py_ce, "ce")
ci = constMatrix(np.zeros((18, 30)), "ci")
ci0 = zero_vec(18, "ci0")

###############################################################################
com_pos_switch = SwitchVector("com_pos_switch")
com_pos_switch.setSignalNumber(2)
plug(reader_pos_com.vector, com_pos_switch.sin0)
plug(constVector([0.0, 0.0, 0.2], "tmp"), com_pos_switch.sin1)
com_pos_switch.selection.value = 1

###############################################################################
# This switch brings the robot posture the initial point of the trajectory file
joint_switch = SwitchVector("joint_switch")
joint_switch.setSignalNumber(2)
plug(reader_joint_pos.vector, joint_switch.sin0)
plug(reader_joint_pos_init.vector, joint_switch.sin1)
joint_switch.selection.value = 0

###############################################################################

des_pos_com = com_pos_switch.sout
des_joint_pos = joint_switch.sout
des_vel_com = reader_vel_com.vector
des_abs_vel = reader_abs_vel.vector
des_cnt_plan = reader_cnt_plan.vector
des_fff_com = reader_fff_com.vector
des_ori_com = reader_ori_com.vector
des_ang_vel_com = reader_ang_vel_com.vector
des_fft_com = reader_fft_com.vector
des_pos = reader_pos.vector
des_vel = reader_vel.vector
# des_joint_pos = reader_joint_pos.vector
des_joint_vel = reader_joint_vel.vector
gain_ratio = reader_gain_ratio.vector
###############################################################################

des_ori_com = constVector([0.0, 0.0, 0.0, 1.0], "des_com_ori")

quad_com_ctrl = QuadrupedComControl(robot, ViconClientEntity)
lctrl = quad_com_ctrl.compute_torques(
    kp_com, des_pos_com, kd_com, des_vel_com, des_fff_com
)
actrl = quad_com_ctrl.compute_ang_control_torques(
    kp_ang_com, des_ori_com, kd_ang_com, des_ang_vel_com, des_fft_com
)
# lctrl = zero_vec(3, "ltau")

com_torques = quad_com_ctrl.return_com_torques(
    lctrl, actrl, des_abs_vel, hess, g0, ce, ci, ci0, reg, des_cnt_plan
)
des_fff = com_torques
############################################################################


def start_traj():
    reader_pos.rewind()
    reader_pos.vector.recompute(0)
    reader_vel.rewind()
    reader_vel.vector.recompute(0)
    reader_abs_vel.rewind()
    reader_abs_vel.vector.recompute(0)
    reader_pos_com.rewind()
    reader_pos_com.vector.recompute(0)
    reader_vel_com.rewind()
    reader_vel_com.vector.recompute(0)
    reader_fff_com.rewind()
    reader_fff_com.vector.recompute(0)
    reader_ori_com.rewind()
    reader_ori_com.vector.recompute(0)
    reader_ang_vel_com.rewind()
    reader_ang_vel_com.vector.recompute(0)
    reader_fft_com.rewind()
    reader_fft_com.vector.recompute(0)
    reader_joint_pos.rewind()
    reader_joint_pos.vector.recompute(0)
    reader_joint_vel.rewind()
    reader_joint_vel.vector.recompute(0)
    reader_gain_ratio.rewind()
    reader_gain_ratio.vector.recompute(0)

    com_pos_switch.selection.value = 0
    joint_switch.selection.value = 0


def go_zero():
    reader_joint_pos_init.rewind()
    reader_joint_pos_init.vector.recompute(0)
    reader_gain_ratio.rewind()
    reader_gain_ratio.vector.recompute(0)
    joint_switch.selection.value = 1


###############################################################################
# For knee inversion
# kp = constVector([50.0, 0.0, 20.0, 0.0, 0.0, 0.0], "kp_split")
# kd = constVector([0.5, 0.0, 0.2, 0.0, 0.0, 0.0], "kd_split")
# For step on box
kp = constVector([0.0, 0.0, 0.0, 0.0, 0.0, 0.0], "kp_split")
kd = constVector([0.0, 0.0, 0.0, 0.0, 0.0, 0.0], "kd_split")


kp_joint_min = constVector(4 * [2.0, 1.0], "kpj_min")
kp_joint_max = constVector(4 * [0.0, 0.0], "kpj_max")

kd_joint_min = constVector(4 * [0.0, 0.0], "kdj_min")
kd_joint_max = constVector(4 * [0.12, 0.12], "kdj_max")

one_vector = constVector(8 * [1.0])
gain_ratio_complement = subtract_vec_vec(one_vector, gain_ratio)

# interpolation between kp_min and kp_max
mul_kp_gain_ratio_complement = Multiply_of_vector("kp_var")
plug(kp_joint_max, mul_kp_gain_ratio_complement.sin0)
plug(gain_ratio_complement, mul_kp_gain_ratio_complement.sin1)
mul_kp_comp = mul_kp_gain_ratio_complement.sout

mul_kp_gain_ratio = Multiply_of_vector("kp_comp_var")
plug(kp_joint_min, mul_kp_gain_ratio.sin0)
plug(gain_ratio, mul_kp_gain_ratio.sin1)
mul_kp_gain = mul_kp_gain_ratio.sout

kp_joint_var = add_vec_vec(mul_kp_gain, mul_kp_comp, "variable_stiffness")

# interpolation between kd_min and kd_max
mul_kd_gain_ratio_complement = Multiply_of_vector("kd_var")
plug(kd_joint_min, mul_kd_gain_ratio_complement.sin0)
plug(gain_ratio_complement, mul_kd_gain_ratio_complement.sin1)
mul_kd_comp = mul_kd_gain_ratio_complement.sout

mul_kd_gain_ratio = Multiply_of_vector("kd_comp_var")
plug(kd_joint_max, mul_kd_gain_ratio.sin0)
plug(gain_ratio, mul_kd_gain_ratio.sin1)
mul_kd_gain = mul_kd_gain_ratio.sout

kd_joint_var = add_vec_vec(mul_kd_gain, mul_kd_comp, "variable_damping")

add_kf = Add_of_double("kf")
add_kf.sin1.value = 0
### Change this value for different gains
add_kf.sin2.value = 0.0
kf = add_kf.sout

quad_imp_ctrl = QuadrupedLegImpedanceController(robot)
joint_ctrl_torques = quad_imp_ctrl.return_joint_ctrl_torques(
    kp_joint_var, des_joint_pos, kd_joint_var, des_joint_vel
)
task_control_torques = quad_imp_ctrl.return_control_torques(
    kp, des_pos, kd, des_vel, kf, des_fff
)
control_torques = add_vec_vec(
    joint_ctrl_torques, task_control_torques, "control_torques"
)

plug(control_torques, robot.device.ctrl_joint_torques)

###############################################################################
# quad_com_ctrl.record_data()
# robot.add_trace("variable_gain", "sout")
# robot.add_trace("PositionReader", "vector")
# robot.add_ros_and_trace("PositionReader", "vector")
#
# robot.add_trace("VelocityReader", "vector")
# robot.add_ros_and_trace("VelocityReader", "vector")
#
# robot.add_trace("PositionComReader", "vector")
# robot.add_ros_and_trace("PositionComReader", "vector")
#
# robot.add_trace("VelocityComReader", "vector")
# robot.add_ros_and_trace("VelocityComReader", "vector")
#
# robot.add_trace("FeedForwardForceComReader", "vector")
# robot.add_ros_and_trace("FeedForwardForceComReader", "vector")
#
# robot.add_trace("OrientationComReader", "vector")
# robot.add_ros_and_trace("OrientationComReader", "vector")
#
# robot.add_trace("AngVelComReader", "vector")
# robot.add_ros_and_trace("AngVelComReader", "vector")
#
# robot.add_trace("FeedForwardMomentsComReader", "vector")
# robot.add_ros_and_trace("FeedForwardMomentsComReader", "vector")
#
# robot.add_trace("AbsVelReader", "vector")
# robot.add_ros_and_trace("AbsVelReader", "vector")
