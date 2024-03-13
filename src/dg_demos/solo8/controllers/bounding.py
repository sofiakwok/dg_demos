"""
@package dg_demos
@file bounding.py
@author Avadesh Meduri
@license License BSD-3-Clause
@copyright Copyright (c) 2019, New York University and Max Planck Gesellschaft.
@date 2019-03-01
@brief This code for creating a stationary bound
"""

import numpy as np
import dynamic_graph as dg

from dg_tools.math_small_entities import (
    ConstantDouble,
    ConstantVector,
    MultiplyDoubleVector,
    Add2Vectors,
    Stack2Vectors,
)
from dg_tools.traj_generators import (
    CircularCartesianTrajectoryGenerator,
)
from dg_tools.sliders import Sliders
from dg_tools.leg_impedance_control.quad_leg_impedance_controller import (
    QuadrupedLegImpedanceController,
    # QuadrupedComControl,
)


class SoloBounding(object):
    """
    This class takes a solo robot and build an impedance controller for it.
    """

    def __init__(self, prefix=""):
        """
        Initialize the controller
        """
        self.prefix = prefix

        #
        # Manage the sliders
        #

        # filter slider_value
        self.sliders = Sliders(4, self.prefix)
        # scale the sliders
        self.sliders.set_scale_values([300.0, 300.0, 0.04, 0.1])
        # Affect the sliders to the different gains
        self.p_gain_x = self.sliders.slider_A.double
        self.p_gain_z = self.sliders.slider_B.double
        self.magnitude_z = self.sliders.slider_C.double
        self.magnitude_x = self.sliders.slider_D.double

        #
        # Create some cartesian oscillation
        #

        # create the paramters: (magnitude comes from slider C)
        self.magnitude_y = ConstantDouble(0.0, self.prefix + "magnitude_y")
        self.omega = ConstantDouble(3.0 * np.pi, self.prefix + "omega")
        self.phase = ConstantDouble(0.0, self.prefix + "phase")
        self.bias_xy = ConstantDouble(0.0, self.prefix + "bias_xy")
        self.bias_z = ConstantDouble(-0.22, self.prefix + "bias_z")

        # create 3 pointers to the same object
        self.magnitude_vec = [
            self.magnitude_x,
            self.magnitude_y,
            self.magnitude_z,
        ]
        self.omega_vec = 3 * [self.omega]
        self.phase_vec = 3 * [self.phase]
        self.bias_vec = [self.bias_xy, self.bias_xy, self.bias_z]

        # Generate the oscillations
        self.fl_osc = CircularCartesianTrajectoryGenerator(
            prefix=self.prefix + "fl_osc_"
        )
        self.fl_osc.plug(
            self.magnitude_vec, self.omega_vec, self.phase_vec, self.bias_vec
        )

        self.fr_osc = CircularCartesianTrajectoryGenerator(
            prefix=self.prefix + "fr_osc_"
        )
        self.fr_osc.plug(
            self.magnitude_vec, self.omega_vec, self.phase_vec, self.bias_vec
        )

        self.hl_osc = CircularCartesianTrajectoryGenerator(
            prefix=self.prefix + "hl_osc_"
        )
        self.hl_osc.plug(
            self.magnitude_vec, self.omega_vec, self.phase_vec, self.bias_vec
        )

        self.hr_osc = CircularCartesianTrajectoryGenerator(
            prefix=self.prefix + "hr_osc_"
        )
        self.hr_osc.plug(
            self.magnitude_vec, self.omega_vec, self.phase_vec, self.bias_vec
        )

        #
        # Stack up the reference
        #

        # position
        self.des_pos_fl_fr = Stack2Vectors(
            self.fl_osc.des_pos,
            self.fr_osc.des_pos,
            6,
            6,
            self.prefix + "des_pos_fl_fr",
        )

        self.des_pos_hl_hr = Stack2Vectors(
            self.hl_osc.des_pos,
            self.hr_osc.des_pos,
            6,
            6,
            self.prefix + "des_pos_hl_hr",
        )

        self.des_pos_stack = Stack2Vectors(
            self.des_pos_fl_fr.sout,
            self.des_pos_hl_hr.sout,
            12,
            12,
            self.prefix + "des_pos",
        )
        self.des_pos = self.des_pos_stack.sout

        # velocity
        self.des_vel_fl_fr = Stack2Vectors(
            self.fl_osc.des_vel,
            self.fr_osc.des_vel,
            6,
            6,
            self.prefix + "des_vel_fl_fr",
        )

        self.des_vel_hl_hr = Stack2Vectors(
            self.hl_osc.des_vel,
            self.hr_osc.des_vel,
            6,
            6,
            self.prefix + "des_vel_hl_hr",
        )

        self.des_vel_stack = Stack2Vectors(
            self.des_vel_fl_fr.sout,
            self.des_vel_hl_hr.sout,
            12,
            12,
            self.prefix + "des_vel",
        )
        self.des_vel = self.des_vel_stack.sout

        #
        # Desired Contact Forces for [fl fr hl hr]
        #
        self.des_fff = ConstantVector(
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
            self.prefix + "des_fff",
        )

        #
        # P_gain
        #
        self.unit_vector_x = ConstantVector(
            [1.0, 0.0, 0.0, 0.0, 0.0, 0.0], self.prefix + "unit_kp_x"
        )
        self.unit_vector_z = ConstantVector(
            [0.0, 0.0, 1.0, 0.0, 0.0, 0.0], self.prefix + "unit_kp_z"
        )

        self.p_gain_x_6d = MultiplyDoubleVector(
            self.p_gain_x.sout,
            self.unit_vector_x.sout,
            self.prefix + "p_gain_x_6d",
        )
        self.p_gain_z_6d = MultiplyDoubleVector(
            self.p_gain_z.sout,
            self.unit_vector_z.sout,
            self.prefix + "p_gain_z_6d",
        )

        self.p_gain_split = Add2Vectors(
            self.p_gain_x_6d.sout,
            self.p_gain_z_6d.sout,
            self.prefix + "p_gain_split",
        )

        # self.kd_split = ConstantVector([0.5, 0.0, 0.5, 0.0, 0.0, 0.0],
        #                             self.prefix + "kd_split")
        self.kd_split = ConstantVector(
            [0.5, 0.0, 0.5, 0.0, 0.0, 0.0], self.prefix + "kd_split"
        )

        # For making gain input dynamic through terminal
        self.add_kf = ConstantDouble(0.0, "add_kf")

    def plug(self, robot):
        dg.plug(robot.device.slider_positions, self.sliders.sin)

        self.quad_imp_ctrl = QuadrupedLegImpedanceController(robot)

        self.control_torques = self.quad_imp_ctrl.return_control_torques(
            self.p_gain_split.sout,
            self.des_pos,
            self.kd_split.sout,
            self.des_vel,
            self.add_kf.sout,
            self.des_fff.sout,
        )

        # self.quad_com_ctrl = QuadrupedComControl(robot)
        dg.plug(self.control_torques, robot.device.ctrl_joint_torques)

        self.trace(robot)

    def trace(self, robot):
        robot.add_trace(self.prefix + "des_pos", "sout")
        robot.add_trace(self.prefix + "des_vel", "sout")
        self.quad_imp_ctrl.record_data()
        self.fl_osc.trace(robot)
        self.fr_osc.trace(robot)
        self.hl_osc.trace(robot)
        self.hr_osc.trace(robot)

    def add_to_ros(self, robot):
        robot.add_to_ros(self.prefix + "des_pos", "sout")
        robot.add_to_ros(self.prefix + "des_vel", "sout")


def get_controller():
    """
    Common interface for controllers.

    This allow to have a generic simulation file
    """
    return SoloBounding(prefix="solo_")


if ("robot" in globals()) or ("robot" in locals()):
    ctrl = get_controller()
    ctrl.plug(robot)
