"""
@package dg_demos
@file impedance_controller.py
@author Julian Viereck
@license License BSD-3-Clause
@copyright Copyright (c) 2019, New York University and Max Planck Gesellschaft.
@date 2019-11-27
@brief Solo12 impedance controller.
"""

import numpy as np
import dynamic_graph as dg
from dynamic_graph.sot.core.math_small_entities import (
    Multiply_double_vector,
    Add_of_vector,
    Stack_of_vector,
)
from dynamic_graph.sot.core.switch import SwitchVector
from dynamic_graph.sot.core.vector_constant import VectorConstant
from robot_properties_solo.config import Solo12Config
from dg_tools.sliders import Sliders
from mim_control.dynamic_graph.go_to import GoTo
from mim_control.dynamic_graph.wbc import ImpedanceController


class Solo12ImpedanceController(object):
    """
    This class takes a solo robot and build an impedance controller for it.
    """

    def __init__(self, prefix=""):

        # Copy the argument internally.
        self._prefix = prefix
        self._robot_config = Solo12Config()
        self._robot_mass = self._robot_config.mass

        # define the sliders behavior
        self.sliders = Sliders(4, self._prefix)
        self.sliders.set_scale_values([0.3, 0.3, 0.3, 0.3])

        #
        # define the desired end-effector locations
        #

        # use the slider to move up and down.
        self._minus_z = VectorConstant(self._prefix + "_minus_z")
        self._minus_z.set(np.array([0.0, 0.0, -1.0, 0.0, 0.0, 0.0]))
        self._slider_z = Multiply_double_vector(self._prefix + "_slider_z")
        dg.plug(self.sliders.slider_A.double.sout, self._slider_z.sin1)
        dg.plug(self._minus_z.sout, self._slider_z.sin2)

        # Move the legs a bit outside.
        y_pos = 0.05

        # Set the default pose from the hips.
        self._fl_hip_to_ee = VectorConstant(self._prefix + "_fl_hip_to_ee")
        self._fr_hip_to_ee = VectorConstant(self._prefix + "_fr_hip_to_ee")
        self._hl_hip_to_ee = VectorConstant(self._prefix + "_hl_hip_to_ee")
        self._hr_hip_to_ee = VectorConstant(self._prefix + "_hr_hip_to_ee")
        self._fl_hip_to_ee.set(np.array([0.0, y_pos, -0.2, 0.0, 0.0, 0.0]))
        self._fr_hip_to_ee.set(np.array([0.0, -y_pos, -0.2, 0.0, 0.0, 0.0]))
        self._hl_hip_to_ee.set(np.array([0.0, y_pos, -0.2, 0.0, 0.0, 0.0]))
        self._hr_hip_to_ee.set(np.array([0.0, -y_pos, -0.2, 0.0, 0.0, 0.0]))

        # Set the default pose from the base.
        self._fl_base_to_ee = VectorConstant(self._prefix + "_fl_base_to_ee")
        self._fr_base_to_ee = VectorConstant(self._prefix + "_fr_base_to_ee")
        self._hl_base_to_ee = VectorConstant(self._prefix + "_hl_base_to_ee")
        self._hr_base_to_ee = VectorConstant(self._prefix + "_hr_base_to_ee")
        self._fl_base_to_ee.set(
            np.array([0.195, 0.147 + y_pos, -0.2, 0.0, 0.0, 0.0])
        )
        self._fr_base_to_ee.set(
            np.array([0.195, -(0.147 + y_pos), -0.2, 0.0, 0.0, 0.0])
        )
        self._hl_base_to_ee.set(
            np.array([-0.195, 0.147 + y_pos, -0.2, 0.0, 0.0, 0.0])
        )
        self._hr_base_to_ee.set(
            np.array([-0.195, -(0.147 + y_pos), -0.2, 0.0, 0.0, 0.0])
        )

        # We sum up the self._slider_z with the self._hip_to_ee
        self._fl_ee_des_pos = Add_of_vector(self._prefix + "_fl_ee_des_pos")
        self._fr_ee_des_pos = Add_of_vector(self._prefix + "_fr_ee_des_pos")
        self._hl_ee_des_pos = Add_of_vector(self._prefix + "_hl_ee_des_pos")
        self._hr_ee_des_pos = Add_of_vector(self._prefix + "_hr_ee_des_pos")

        # Plug the end-effectors reference position in the hip frames.
        dg.plug(self._fl_base_to_ee.sout, self._fl_ee_des_pos.sin(0))
        dg.plug(self._slider_z.sout, self._fl_ee_des_pos.sin(1))
        #
        dg.plug(self._fr_base_to_ee.sout, self._fr_ee_des_pos.sin(0))
        dg.plug(self._slider_z.sout, self._fr_ee_des_pos.sin(1))
        #
        dg.plug(self._hl_base_to_ee.sout, self._hl_ee_des_pos.sin(0))
        dg.plug(self._slider_z.sout, self._hl_ee_des_pos.sin(1))
        #
        dg.plug(self._hr_base_to_ee.sout, self._hr_ee_des_pos.sin(0))
        dg.plug(self._slider_z.sout, self._hr_ee_des_pos.sin(1))

        #
        # We set the desired velocity.
        #
        self._ee_des_vel = VectorConstant(self._prefix + "_ee_des_vel")
        self._ee_des_vel.set(np.array(6 * [0.0]))

        #
        # Desired wrench at the end-effectors.
        #
        self._fff = VectorConstant(self._prefix + "_ee_fff")
        self._fff.set(
            np.array(
                [0.0, 0.0, (self._robot_mass * 9.81) / 4.0, 0.0, 0.0, 0.0]
            )
        )

        #
        # Robot configuration and velocity
        #
        self._q = Stack_of_vector("_robot_configuration")
        self._q.selec1(0, 7)
        self._q.selec2(0, self._robot_config.nb_joints)
        self._q.sin1.value = np.array(7 * [0.0])
        #
        self._dq = Stack_of_vector("_robot_velocity")
        self._dq.selec1(0, 6)
        self._dq.selec2(0, self._robot_config.nb_joints)
        self._dq.sin1.value = np.array(6 * [0.0])

        #
        # Impedance controllers and plug
        #
        self._fl_imp_ctrl = ImpedanceController(self._prefix + "_fl_imp_ctrl")
        self._fr_imp_ctrl = ImpedanceController(self._prefix + "_fr_imp_ctrl")
        self._hl_imp_ctrl = ImpedanceController(self._prefix + "_hl_imp_ctrl")
        self._hr_imp_ctrl = ImpedanceController(self._prefix + "_hr_imp_ctrl")
        # Initialize the controllers.
        model = self._robot_config.pin_robot.model
        self._fl_imp_ctrl.initialize(model, "base_link", "FL_FOOT")
        self._fr_imp_ctrl.initialize(model, "base_link", "FR_FOOT")
        self._hl_imp_ctrl.initialize(model, "base_link", "HL_FOOT")
        self._hr_imp_ctrl.initialize(model, "base_link", "HR_FOOT")
        # Impedance gains.
        self._kp_val = [50.0, 50.0, 50.0, 0.0, 0.0, 0.0]
        self._kd_val = [0.7, 0.7, 0.7, 0.0, 0.0, 0.0]
        self._kf_val = 0.0
        self.set_gains(self._kp_val, self._kd_val, self._kf_val)
        # plug the feed forward force as input.
        dg.plug(self._fff.sout, self._fl_imp_ctrl.feed_forward_force_sin)
        dg.plug(self._fff.sout, self._fr_imp_ctrl.feed_forward_force_sin)
        dg.plug(self._fff.sout, self._hl_imp_ctrl.feed_forward_force_sin)
        dg.plug(self._fff.sout, self._hr_imp_ctrl.feed_forward_force_sin)
        # plug the robot configuration to the impedance controller.
        dg.plug(self._q.sout, self._fl_imp_ctrl.robot_configuration_sin)
        dg.plug(self._q.sout, self._fr_imp_ctrl.robot_configuration_sin)
        dg.plug(self._q.sout, self._hl_imp_ctrl.robot_configuration_sin)
        dg.plug(self._q.sout, self._hr_imp_ctrl.robot_configuration_sin)
        # plug the robot velocity to the impedance controller.
        dg.plug(self._dq.sout, self._fl_imp_ctrl.robot_velocity_sin)
        dg.plug(self._dq.sout, self._fr_imp_ctrl.robot_velocity_sin)
        dg.plug(self._dq.sout, self._hl_imp_ctrl.robot_velocity_sin)
        dg.plug(self._dq.sout, self._hr_imp_ctrl.robot_velocity_sin)
        # plug the end-effectors reference signal
        dg.plug(
            self._fl_ee_des_pos.sout,
            self._fl_imp_ctrl.desired_end_frame_placement_sin,
        )
        dg.plug(
            self._fr_ee_des_pos.sout,
            self._fr_imp_ctrl.desired_end_frame_placement_sin,
        )
        dg.plug(
            self._hl_ee_des_pos.sout,
            self._hl_imp_ctrl.desired_end_frame_placement_sin,
        )
        dg.plug(
            self._hr_ee_des_pos.sout,
            self._hr_imp_ctrl.desired_end_frame_placement_sin,
        )
        #
        dg.plug(
            self._ee_des_vel.sout,
            self._fl_imp_ctrl.desired_end_frame_velocity_sin,
        )
        dg.plug(
            self._ee_des_vel.sout,
            self._fr_imp_ctrl.desired_end_frame_velocity_sin,
        )
        dg.plug(
            self._ee_des_vel.sout,
            self._hl_imp_ctrl.desired_end_frame_velocity_sin,
        )
        dg.plug(
            self._ee_des_vel.sout,
            self._hr_imp_ctrl.desired_end_frame_velocity_sin,
        )

        #
        # Define the output control as the sum of all impedance controllers.
        #
        self._sum_joint_torques = Add_of_vector(
            self._prefix + "_sum_joint_torques"
        )
        self._sum_joint_torques.n_sin = 4
        dg.plug(
            self._fl_imp_ctrl.joint_torque_sout, self._sum_joint_torques.sin(0)
        )
        dg.plug(
            self._fr_imp_ctrl.joint_torque_sout, self._sum_joint_torques.sin(1)
        )
        dg.plug(
            self._hl_imp_ctrl.joint_torque_sout, self._sum_joint_torques.sin(2)
        )
        dg.plug(
            self._hr_imp_ctrl.joint_torque_sout, self._sum_joint_torques.sin(3)
        )

        #
        # Define the interface to the graph
        #
        self.joint_positions_sin = self._q.sin2
        self.joint_velocities_sin = self._dq.sin2
        self.control_torques_sout = self._sum_joint_torques.sout

    def set_gains(self, kp, kd, kf):
        self._kp_val = kp  # [50.0, 50.0, 50.0, 0.0, 0.0, 0.0]
        self._kd_val = kd  # [0.7, 0.7, 0.7, 0.0, 0.0, 0.0]
        self._kf_val = kf  # 0.0
        #
        self._fl_imp_ctrl.gain_proportional_sin.value = np.array(kp)
        self._fr_imp_ctrl.gain_proportional_sin.value = np.array(kp)
        self._hl_imp_ctrl.gain_proportional_sin.value = np.array(kp)
        self._hr_imp_ctrl.gain_proportional_sin.value = np.array(kp)
        #
        self._fl_imp_ctrl.gain_derivative_sin.value = np.array(kd)
        self._fr_imp_ctrl.gain_derivative_sin.value = np.array(kd)
        self._hl_imp_ctrl.gain_derivative_sin.value = np.array(kd)
        self._hr_imp_ctrl.gain_derivative_sin.value = np.array(kd)
        #
        self._fl_imp_ctrl.gain_feed_forward_force_sin.value = kf
        self._fr_imp_ctrl.gain_feed_forward_force_sin.value = kf
        self._hl_imp_ctrl.gain_feed_forward_force_sin.value = kf
        self._hr_imp_ctrl.gain_feed_forward_force_sin.value = kf

    def plug(
        self,
        joint_positions_sout,
        joint_velocities_sout,
        slider_signal_sout,
        joint_torques_sin,
    ):
        # Plug the sensors
        dg.plug(joint_positions_sout, self.joint_positions_sin)
        dg.plug(joint_velocities_sout, self.joint_velocities_sin)
        # Plug the sliders.
        self.sliders.plug_slider_signal(slider_signal_sout)
        # Plug the output
        dg.plug(self.control_torques_sout, joint_torques_sin)


def get_controller():
    """
    Common interface for controllers.

    This allow to have a generic simulation file
    """
    return Solo12ImpedanceController(prefix="solo12_impedance_controller")


class ImpedanceDemo(object):
    def __init__(self):
        self._config = Solo12Config()
        # we create 2 controllers
        self._ctrl_go_to = GoTo(self._config.nb_joints, prefix="solo12_")
        self._ctrl_go_to.set_pd_gains(3.0, 0.05)
        self._ctrl_imp = get_controller()
        # we create a master switch
        self._master_switch = SwitchVector("master_switch")
        self._master_switch.n_sin = 3
        self._master_switch.sin(0).value = np.array(12 * [0.0])
        self._master_switch.selection.value = 0

    def plug(self, robot):
        # Plug the graph
        self._ctrl_go_to.plug(
            robot.device.joint_positions,
            robot.device.joint_velocities,
            self._master_switch.sin(1),
        )
        self._ctrl_imp.plug(
            robot.device.joint_positions,
            robot.device.joint_velocities,
            robot.device.slider_positions,
            self._master_switch.sin(2),
        )
        dg.plug(self._master_switch.sout, robot.device.ctrl_joint_torques)

    def go_idle(self):
        self._master_switch.selection.value = 0

    def go_init(self):
        self._ctrl_go_to.go_to(np.array(config.initial_configuration), 2000)
        self._master_switch.selection.value = 1

    def go_imp(self, robot):
        # Offset the sliders by the current value.
        self._ctrl_imp.sliders.set_offset_values(
            -np.array(robot.device.slider_positions.value)
        )
        self._master_switch.selection.value = 2


if "robot" in globals():

    ctrl = ImpedanceDemo()
    ctrl.plug(robot)

    # Some renaming.
    def go_idle():
        ctrl.go_idle()

    def go_init():
        ctrl.go_init()

    def go_imp():
        ctrl.go_imp(robot)

    print(
        "#####\n\n",
        "Please execute `go_init()` and ",
        "`go_imp` function to start the controller.\n\n" "#####\n\n",
    )
