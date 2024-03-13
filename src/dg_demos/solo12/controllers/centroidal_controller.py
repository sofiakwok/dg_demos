# Implementation for plugable centroidal controller for solo12.
# Author: Julian Viereck
# Date : 16 January 2020
#
# Difference to the solo8 centroidal controller:
# - The endeffectors are expressed in global frames.
# - The tracking is on the com and vcom, not the base position and velocity


from dg_tools.utils import *
from dg_tools.traj_generators import mul_double_vec_2, scale_values
from dg_tools.leg_impedance_control.solo12_impedance_controller import (
    Solo12ComController,
    Solo12ImpedanceController,
)

from dg_tools.filter import ButterWorthFilter

from robot_properties_solo.config import Solo12Config

from dg_demos.solo12.controllers import pd_controller

from dg_tools.sliders import Sliders

#############################################################################


class Solo12ComWbc(object):
    def __init__(self, prefix):
        self.prefix = prefix + "_Solo12ComWbc_"
        unit_vec_111 = constVector([1.0, 1.0, 1.0], "unit_vec_111")

        self.sliders = Sliders(4, self.prefix, filter_size=100)
        self.sliders.set_scale_values([200.0, 50.0, 50.0, 25.0])

        self.kp_com_val, self.kp_com_val_entity = constDouble(
            100, self.prefix + "_kp_com_val", True
        )
        self.kd_com_val, self.kd_com_val_entity = constDouble(
            15, self.prefix + "_kd_com_val", True
        )
        self.kp_ang_com_val, self.kp_ang_com_val_entity = constDouble(
            25.0, self.prefix + "_kp_ang_com_val", True
        )
        self.kd_ang_com_val, self.kd_ang_com_val_entity = constDouble(
            22.5, self.prefix + "_kd_ang_com_val", True
        )
        # self.kp_com_val = constDouble(100)
        # self.kd_com_val = constDouble(10)
        # self.kp_ang_com_val = constDouble(50)
        # self.kd_ang_com_val = constDouble(5)

        self.kp_com = mul_double_vec_2(self.kp_com_val, unit_vec_111, "kp_com")
        self.kd_com = mul_double_vec_2(self.kd_com_val, unit_vec_111, "kd_com")
        self.kp_ang_com = mul_double_vec_2(
            self.kp_ang_com_val, unit_vec_111, "kp_ang_com"
        )
        self.kd_ang_com = mul_double_vec_2(
            self.kd_ang_com_val, unit_vec_111, "kd_ang_com"
        )

        self.des_pos_com_sin, self.des_pos_com = constVectorOp(
            [0.0, 0.0, 0.20], "des_pos_com"
        )
        self.des_vel_com_sin, self.des_vel_com = constVectorOp(
            [0.0, 0.0, 0.0], "des_vel_com"
        )
        self.des_fff_com_sin, self.des_fff_com = constVectorOp(
            [0.0, 0.0, Solo12Config.mass * 9.81], "des_fff_com"
        )
        self.des_ori_com_sin, self.des_ori_com = constVectorOp(
            [0.0, 0.0, 0.0, 1.0], "des_com_ori"
        )
        self.des_ang_vel_com_sin, self.des_ang_vel_com = constVectorOp(
            [0.0, 0.0, 0.0], "des_com_ang_vel"
        )
        self.des_fft_com_sin, self.des_fft_com = constVectorOp(
            [0.0, 0.0, 0.0], "des_fft_com"
        )

        self.contact_plan_sin, self.contact_plan = constVectorOp(
            [1, 1, 1, 1], "des_cnt_plan"
        )

        self.des_pos_eff_sin, self.des_pos_eff = constVectorOp(
            [
                0.195,
                0.147,
                0.015,
                0.195,
                -0.147,
                0.015,
                -0.195,
                0.147,
                0.015,
                -0.195,
                -0.147,
                0.015,
            ],
            "pos_des",
        )

        self.des_vel_eff_sin, self.des_vel_eff = constVectorOp(
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
            ],
            "vel_des",
        )

        self.kp_eff_val = DoubleSignal(50.0)
        self.kd_eff_val = DoubleSignal(0.7)
        self.kp_eff_x_val = DoubleSignal(25.0)
        self.kp_eff_y_val = DoubleSignal(25.0)
        self.kd_eff_plane_val = DoubleSignal(0.7)
        self.kp_eff = (
            self.kp_eff_val * VectorSignal(4 * [0.0, 0.0, 1.0])
            + self.kp_eff_x_val * VectorSignal(4 * [1.0, 0.0, 0.0])
            + self.kp_eff_y_val * VectorSignal(4 * [0.0, 1.0, 0.0])
        )
        self.kd_eff = self.kd_eff_val * VectorSignal(
            4 * [0.0, 0.0, 1.0]
        ) + self.kd_eff_plane_val * VectorSignal(4 * [1.0, 1.0, 0.0])
        # self.kp_eff = mul_double_vec_2(self.kp_eff_val, constVector(12*[1., ], "unit"), "kp_split")
        # self.kd_eff = mul_double_vec_2(self.kd_eff_val, constVector(12*[1.,], "kd_split"), "kd_split_mult")

        ## setting desired position.
        # Here, we are working in global coordinate frame.
        self.kf_eff = constDouble(0.0)

        self.mu = 0.6

    def set_des_com_translation(self, des_pos_com, des_vel_com):
        """Provide signals for desired com posture.

        Args:
          des_pos_com: VectorSignal(size=3) Desired com velocity(3) desired com position.
          des_vel_com: VectorSignal(size=3) Desired com velocity.
        """
        plug(des_pos_com, self.des_pos_com_sin)
        plug(des_vel_com, self.des_vel_com_sin)

    def set_des_contact_plan(self, contact_plan):
        """Specifies the desired contact behavior of the endeffectors.

        Args:
          contact_plan: VectorSignal(size=4) If one, endeffector is assumed to
              be in contact with the ground.
        """
        plug(contact_plan, self.contact_plan_sin)

    def set_des_pos_eff(self, des_pos_eff):
        plug(des_pos_eff, self.des_pos_eff_sin)

    def set_des_vel_eff(self, des_vel_eff):
        plug(des_vel_eff, self.des_vel_eff_sin)

    def set_gains_eff(self, eff_kp, eff_kd):
        self.kp_eff = eff_kp
        self.kd_eff = eff_kd

    def init_vicon(self, robot, ViconClientEntity):
        """ Init the vicon entity and return the base positon and velocity."""
        self.robot_vicon_name = "solo12"
        self.vicon_client = ViconClientEntity(self.prefix + "_vicon_client")
        # self.vicon_client.connect_to_vicon('10.32.3.16:801')
        self.vicon_client.connect_to_vicon("10.32.27.53:801")
        self.vicon_client.add_object_to_track(
            "{}/{}".format(self.robot_vicon_name, self.robot_vicon_name)
        )

        try:
            self.vicon_client.robot_wrapper(robot, self.robot_vicon_name)
        except:
            pass

        base_position = self.vicon_client.signal(
            self.robot_vicon_name + "_position"
        )
        base_velocity = self.vicon_client.signal(
            self.robot_vicon_name + "_velocity_body"
        )

        # # Filter the base position and velocity.
        # self.filter_base_position = ButterWorthFilter('fltr_base_position')
        # self.filter_base_position.init(7, 0.001, 0.9999, 2)
        # plug(base_position, self.filter_base_position.sin)

        # self.filter_base_velocity = ButterWorthFilter('fltr_base_velocity')
        # self.filter_base_velocity.init(6, 0.001, 0.9999, 2)
        # plug(base_velocity, self.filter_base_velocity.sin)

        # robot.add_trace('fltr_base_position', 'x')
        # robot.add_trace('fltr_base_position', 'x_filtered')

        # robot.add_trace('fltr_base_velocity', 'x')
        # robot.add_trace('fltr_base_velocity', 'x_filtered')

        # return self.filter_base_position.sout, base_velocity
        return base_position, base_velocity

    def tune_vicon_filter(self, filter_percentage):
        self.filter_base_position.update(filter_percentage)
        self.filter_base_velocity.update(filter_percentage)

    def init(self, robot, ViconClientEntity):
        self.robot = robot
        vel_des = zero_vec(24, "des_vel")

        robot.add_trace("des_cnt_plan", "sout")

        ###############################################################################

        I = np.eye(3)
        zm = np.zeros((3, 3))

        # The r_xx vectors are specified via the `abs_end_eff_pos`
        r_fl = np.zeros((3, 3))
        r_fr = np.zeros((3, 3))
        r_hl = np.zeros((3, 3))
        r_hr = np.zeros((3, 3))

        v_fl = np.zeros((3, 3))
        v_fr = np.zeros((3, 3))
        v_hl = np.zeros((3, 3))
        v_hr = np.zeros((3, 3))

        py_ce = np.block(
            [[I, I, I, I, -I, zm], [r_fl, r_fr, r_hl, r_hr, zm, -I]]
        )

        py_ce = np.asarray(py_ce)
        py_hess = np.diag(
            12
            * [
                2,
            ]
            + 3
            * [
                5e4,
            ]
            + 3
            * [
                1e6,
            ]
        )

        py_reg = np.zeros((18, 18))
        np.fill_diagonal(py_reg, 0.0001)

        self.hess = constMatrix(py_hess, "hess")
        self.reg = constMatrix(py_reg, "regularizer")
        self.g0 = zero_vec(18, "g0")
        self.ce = constMatrix(py_ce, "ce")
        self.ce0 = zero_vec(6, "ce0")

        # Setup friction cone and unilateral forces constrain.
        py_ci = np.zeros((20, 18))
        for j in range(4):
            py_ci[5 * j + 0, 3 * j + 0] = -1  # mu Fz - Fx >= 0
            py_ci[5 * j + 0, 3 * j + 2] = self.mu
            py_ci[5 * j + 1, 3 * j + 0] = 1  # mu Fz + Fx >= 0
            py_ci[5 * j + 1, 3 * j + 2] = self.mu
            py_ci[5 * j + 2, 3 * j + 1] = -1  # mu Fz - Fy >= 0
            py_ci[5 * j + 2, 3 * j + 2] = self.mu
            py_ci[5 * j + 3, 3 * j + 1] = 1  # mu Fz + Fy >= 0
            py_ci[5 * j + 3, 3 * j + 2] = self.mu
            py_ci[5 * j + 4, 3 * j + 2] = 1  # Fz >= 0

        self.ci = constMatrix(py_ci, "ci")
        self.ci0 = zero_vec(20, "ci0")

        ###############################################################################

        self.base_position, self.base_velocity = self.init_vicon(
            robot, ViconClientEntity
        )

        ###############################################################################

        des_abs_vel = constVector(
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            "des_abs_vel",
        )

        ###############################################################################

        # Init the centroidal controller using the filtered base.
        self.quad_com_ctrl = Solo12ComController(
            robot,
            base_position=self.base_position,
            base_velocity=self.base_velocity,
        )
        self.lctrl = self.quad_com_ctrl.compute_torques(
            self.kp_com,
            self.des_pos_com,
            self.kd_com,
            self.des_vel_com,
            self.des_fff_com,
        )
        self.actrl = self.quad_com_ctrl.compute_ang_control_torques(
            self.kp_ang_com,
            self.des_ori_com,
            self.kd_ang_com,
            self.des_ang_vel_com,
            self.des_fft_com,
        )

        # Track the actual com and vcom position instead of the base position.
        # self.quad_com_ctrl.track_com()

        self.com_torques = self.quad_com_ctrl.return_com_torques(
            self.lctrl,
            self.actrl,
            des_abs_vel,
            self.hess,
            self.g0,
            self.ce,
            self.ci,
            self.ci0,
            self.reg,
            self.contact_plan,
        )

        ###############################################################################
        ### Change this value for different gains

        # Final impedance controller mapping desired forces to endeffectors.
        self.quad_imp_ctrl = Solo12ImpedanceController(robot)
        self.control_torques = self.quad_imp_ctrl.compute_control_torques(
            self.kp_eff,
            self.des_pos_eff,
            self.kd_eff,
            self.des_vel_eff,
            self.kf_eff,
            self.com_torques,
            base_position=self.quad_com_ctrl.get_biased_base_position(),
            base_velocity=self.quad_com_ctrl.get_biased_base_velocity(),
            pos_global=True,
        )

        # Set the endeffector position for the centroidal force qp.
        self.quad_com_ctrl.set_abs_end_eff_pos(
            self.quad_imp_ctrl.compute_com_end_eff_pos()
        )

    def plug(self):
        """ Use the sliders values for the com gains. """
        self.sliders.plug_slider_signal(self.robot.device.slider_positions)

        # Plug the main controller.
        plug(self.control_torques, self.robot.device.ctrl_joint_torques)

        # Set the bias right away.
        self.quad_com_ctrl.set_bias()

    def slider_tune_com_pd(self):
        # Offset the values here to make sure we offset according to the sliders
        # values used at the time when calling `slider_tune_com_pd`.
        print("zero_sliders", self.robot.device.slider_positions.value)
        self.sliders.set_offset_values(
            (-np.array(self.robot.device.slider_positions.value)).tolist()
        )

        # HACK: Plug the scaled slider values to get offset from current gains.
        dg.plug(self.sliders.slider_A.double.sout, self.kp_com_val_entity.sin2)
        dg.plug(self.sliders.slider_B.double.sout, self.kd_com_val_entity.sin2)
        dg.plug(
            self.sliders.slider_C.double.sout, self.kp_ang_com_val_entity.sin2
        )
        dg.plug(
            self.sliders.slider_D.double.sout, self.kd_ang_com_val_entity.sin2
        )

        self.robot.add_ros_and_trace(
            self.kp_com_val_entity.name, "sout", topic_type="double"
        )
        self.robot.add_ros_and_trace(
            self.kd_com_val_entity.name, "sout", topic_type="double"
        )
        self.robot.add_ros_and_trace(
            self.kp_ang_com_val_entity.name, "sout", topic_type="double"
        )
        self.robot.add_ros_and_trace(
            self.kd_ang_com_val_entity.name, "sout", topic_type="double"
        )

    def record_data(self):
        # Setup data recording.
        self.quad_com_ctrl.record_data()
        self.quad_imp_ctrl.record_data()


def get_controller(prefix="solo12"):
    return Solo12ComWbc(prefix)


if ("robot" in globals()) or ("robot" in locals()):
    ctrl_pd = pd_controller.get_controller()

    from dynamic_graph_manager.vicon_sdk import ViconClientEntity

    ctrl = Solo12ComWbc("solo12")
    ctrl.init(robot, ViconClientEntity)
    # Offsetting the vicon base to make sure ground aligns with actual
    # ground of the robot.
    ctrl.quad_com_ctrl.vicon_offset.value = [0.0, 0.0, 0.0]

    def go_pd():
        ctrl_pd.plug(robot)

    def go_imp():
        ctrl.plug()
