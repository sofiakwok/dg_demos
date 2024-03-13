"""
@brief Similar to dg_quad_joint_variable_gain.py but uses the force as planned by
planner directly instead of passing them through the centroidal controller.

@author Julian Viereck
@date March 04, 2020
"""

from dynamic_graph.sot.core.reader import Reader
from dg_tools.utils import *

# from leg_impedance_control.quad_leg_impedance_controller import quad_leg_impedance_controller
from dg_tools.leg_impedance_control.quad_leg_impedance_controller import (
    QuadrupedLegImpedanceController,
    QuadrupedComControl,
)
from dg_tools.traj_generators import mul_double_vec_2, scale_values
from dynamic_graph_manager.vicon_sdk import ViconClientEntity
from dg_demos.solo.controllers.pd_controller import PDController

from dynamic_graph.sot.core.switch import SwitchVector
from os.path import join


def file_exists(filename):
    if os.path.isfile(filename):
        print("The file %s exists" % filename)
    else:
        print("The file %s does not exist" % filename)


class SoloJointVariableGainForce(object):
    def __init__(self, prefix="solo_"):
        self.prefix = prefix

    def plug(self, robot):
        ###############################################################################

        ### reading createData
        self.reader_pos = Reader("PositionReader")
        self.reader_vel = Reader("VelocityReader")
        self.reader_joint_pos = Reader("JointPositionReader")
        self.reader_joint_vel = Reader("JointVelocityReader")
        self.reader_forces = Reader("forces")
        self.reader_gain_ratio = Reader("GainRatioReader")

        folder = join(rospkg.RosPack().get_path("momentumopt"), "nodes")
        # folder = '/home/jviereck/dev/solo8_micro/workspace/src/catkin/dg_control/dg_demos/python/dg_demos/solo/data/jump'

        filename_pos = join(folder, "quadruped_positions_eff.dat")
        filename_vel = join(folder, "quadruped_velocities_eff.dat")
        # filename_abs_vel = join(folder, "quadruped_velocities_abs.dat")
        # filename_cnt_plan = join(folder, "quadruped_contact_activation.dat")
        # filename_pos_com = join(folder, "quadruped_com.dat")
        # filename_vel_com = join(folder, "quadruped_com_vel.dat")
        # filename_fff_com = join(folder, "quadruped_centroidal_forces.dat")
        # filename_ori_com = join(folder, "quadruped_quaternion.dat")
        # filename_ang_vel_com = join(folder, "quadruped_base_ang_velocities.dat")
        # filename_fft_com = join(folder, "quadruped_centroidal_moments.dat")
        filename_eff_force = join(folder, "quadruped_forces.dat")
        filename_joint_pos = join(folder, "quadruped_positions.dat")
        filename_joint_vel = join(folder, "quadruped_velocities.dat")
        filename_gain_ratio = join(folder, "quadruped_gain_ratio.dat")

        file_exists(filename_pos)
        file_exists(filename_vel)
        file_exists(filename_eff_force)
        file_exists(filename_joint_pos)
        file_exists(filename_joint_vel)
        file_exists(filename_gain_ratio)

        print("Loading data files:")
        self.reader_pos.load(filename_pos)
        self.reader_vel.load(filename_vel)
        self.reader_forces.load(filename_eff_force)
        self.reader_joint_pos.load(filename_joint_pos)
        self.reader_joint_vel.load(filename_joint_vel)
        self.reader_gain_ratio.load(filename_gain_ratio)

        # Specify which of the columns to select.
        # NOTE: This is selecting the columns in reverse order - the last number is the first column in the file
        self.reader_pos.selec.value = "111111111111111111111111"
        self.reader_vel.selec.value = "111111111111111111111111"
        self.reader_forces.selec.value = "1111111111110"
        self.reader_joint_pos.selec.value = "111111110"
        self.reader_joint_vel.selec.value = "111111110"
        self.reader_gain_ratio.selec.value = "111111110"

        des_pos = self.reader_pos.vector
        des_vel = self.reader_vel.vector
        des_joint_pos = self.reader_joint_pos.vector
        des_joint_vel = self.reader_joint_vel.vector
        gain_ratio = self.reader_gain_ratio.vector

        des_eff_ff = VectorSignal(self.reader_forces.vector, 12)
        des_eff_ff = (
            des_eff_ff[0:3]
            .concat(VectorSignal([0.0, 0.0, 0.0]))
            .concat(des_eff_ff[3:6].concat(VectorSignal([0.0, 0.0, 0.0])))
            .concat(des_eff_ff[6:9].concat(VectorSignal([0.0, 0.0, 0.0])))
            .concat(des_eff_ff[9:12].concat(VectorSignal([0.0, 0.0, 0.0])))
        )

        ###############################################################################

        kp = constVector([0.0, 0.0, 0.0, 0.0, 0.0, 0.0], "kp_split")
        kd = constVector([0.0, 0.0, 0.0, 0.0, 0.0, 0.0], "kd_split")

        add_kpj = Add_of_double("kpj")
        add_kpj.sin1.value = 0
        ### Change this value for different gains
        add_kpj.sin2.value = 5.0
        kpj = add_kpj.sout

        kp_joint = mul_double_vec_2(
            kpj,
            constVector([1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0], "unit"),
            "kpj_split",
        )
        kd_joint = constVector(
            [0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1], "kdj_split"
        )
        kp_joint_min = constVector([2.0, 1.0, 2.0, 1.0, 2.0, 1.0, 2.0, 1.0])
        one_vector = constVector([1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0])
        gain_ratio_complement = subtract_vec_vec(one_vector, gain_ratio)

        mul_gain_ratio_complement = Multiply_of_vector("kp_var")
        plug(kp_joint, mul_gain_ratio_complement.sin0)
        plug(gain_ratio_complement, mul_gain_ratio_complement.sin1)
        mul_gain_comp = mul_gain_ratio_complement.sout

        mul_gain_ratio = Multiply_of_vector("kp_comp_var")
        plug(kp_joint_min, mul_gain_ratio.sin0)
        plug(gain_ratio, mul_gain_ratio.sin1)
        mul_gain = mul_gain_ratio.sout

        multiplied_gain = add_vec_vec(mul_gain, mul_gain_comp, "variable_gain")

        add_kf = Add_of_double("kf")
        add_kf.sin1.value = 0
        ### Change this value for different gains
        add_kf.sin2.value = 1.0
        kf = add_kf.sout

        quad_imp_ctrl = QuadrupedLegImpedanceController(robot)
        joint_ctrl_torques = quad_imp_ctrl.return_joint_ctrl_torques(
            multiplied_gain, des_joint_pos, kd_joint, des_joint_vel
        )
        # task_control_torques = quad_imp_ctrl.return_control_torques(kp, des_pos, kd, des_vel, kf, des_fff)
        task_control_torques = quad_imp_ctrl.return_control_torques(
            kp, des_pos, kd, des_vel, kf, des_eff_ff
        )
        control_torques = add_vec_vec(
            joint_ctrl_torques, task_control_torques, "control_torques"
        )

        plug(control_torques, robot.device.ctrl_joint_torques)

    def rewind_traj(self):
        self.reader_pos.rewind()
        self.reader_pos.vector.recompute(0)
        self.reader_vel.rewind()
        self.reader_vel.vector.recompute(0)
        self.reader_joint_pos.rewind()
        self.reader_joint_pos.vector.recompute(0)
        self.reader_joint_vel.rewind()
        self.reader_joint_vel.vector.recompute(0)
        self.reader_gain_ratio.rewind()
        self.reader_gain_ratio.vector.recompute(0)
        self.reader_forces.rewind()
        self.reader_forces.vector.recompute(0)


def get_controller():
    return SoloJointVariableGainForce("solo8")


if "robot" in globals():
    ctrl = SoloJointVariableGainForce("solo8")
    ctrl_pd = PDController("solo8")
    ctrl_pd.follow_slider()

    def go_pd():
        ctrl_pd.plug(robot)

    def go_traj():
        ctrl.plug(robot)

    def restart_traj():
        ctrl.rewind_traj()
