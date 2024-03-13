"""
@package dg_demos
@file teststand/controllers/foot_circle.py
@author Elham Daneshmand
@license License BSD-3-Clause
@copyright Copyright (c) 2019, New York University and Max Planck Gesellschaft.
@date 2019-10-06
@brief Make the foot of the Teststand move in circle
"""

import numpy as np
from dynamic_graph import plug

from dg_tools.math_small_entities import (
    StackZero,
    ConstantVector,
    ConstantDouble,
)
from dg_tools.sliders import Sliders
from dg_tools.leg_impedance_control.leg_impedance_controller import (
    LegImpedanceController,
)
from dg_tools.traj_generators import CircularCartesianTrajectoryGenerator


class TeststandFootCircle(object):
    """
    This class integrate a leg impedance controller for the Teststand robot
    """

    def __init__(self, prefix=""):
        """
        Initialize the controller
        """
        self.prefix = prefix

        # managing the sliders
        self.sliders = Sliders(2, self.prefix)
        # scale the sliders
        self.sliders.set_scale_values([0.1, 2 * np.pi])

        #
        # Define a circular trajectory depending on the silders on the XZ plane
        #

        # Magnitude = [Slider_A, 0, Slider_A]
        self.magnitude_xz = self.sliders.slider_A.double
        self.magnitude_y = ConstantDouble(0.0, self.prefix + "radius_y")
        # Omega = 3 * [slider_B,]
        self.omega_xyz = self.sliders.slider_B.double
        # Phase = [0, 0, 0]
        self.phase_yz = ConstantDouble(0.0, self.prefix + "phase_yz")
        self.phase_x = ConstantDouble(np.pi / 2.0, self.prefix + "phase_x")
        # Bias = [0, 0, -]
        self.bias_xy = ConstantDouble(0.0, self.prefix + "bias_xy")
        self.bias_z = ConstantDouble(-0.2, self.prefix + "bias_z")
        # Circular trajectory entity
        self.circle = CircularCartesianTrajectoryGenerator(
            self.prefix + "3D_circle"
        )
        self.circle.plug(
            [self.magnitude_xz, self.magnitude_y, self.magnitude_xz],
            [self.omega_xyz, self.omega_xyz, self.omega_xyz],
            [self.phase_x, self.phase_yz, self.phase_yz],
            [self.bias_xy, self.bias_xy, self.bias_z],
        )

        #
        # Define the impedance controller
        #
        self.kp_split = ConstantVector(
            [100.0, 0.0, 100.0, 0.0, 0.0, 0.0], "kp_split"
        )
        self.kd_split = ConstantVector(
            [0.5, 0.0, 0.5, 0.0, 0.0, 0.0], "kd_split"
        )
        self.des_pos = self.circle.des_pos
        self.des_vel = self.circle.des_vel

        self.leg_imp_ctrl = LegImpedanceController(self.prefix[:-1])

        self.joint_positions = StackZero(
            1, 2, prefix=self.prefix + "joint_positions_"
        )
        plug(self.joint_positions.sout, self.leg_imp_ctrl.robot_dg.position)

        self.joint_velocities = StackZero(
            1, 2, prefix=self.prefix + "joint_velocities_"
        )
        plug(self.joint_velocities.sout, self.leg_imp_ctrl.robot_dg.velocity)

        self.ctrl_sout = self.leg_imp_ctrl.return_control_torques(
            self.kp_split.sout,
            self.des_pos.sout,
            self.kd_split.sout,
            self.des_vel.sout,
        )

    def plug_signals(
        self,
        sliders_sout,
        joint_positions_sout,
        joint_velocities_sout,
        ctrl_joint_torques_sin,
    ):
        plug(sliders_sout, self.sliders.sin)
        plug(joint_positions_sout, self.joint_positions.sin)
        plug(joint_velocities_sout, self.joint_velocities.sin)
        plug(self.ctrl_sout, ctrl_joint_torques_sin)

    def plug(self, robot):
        self.plug_signals(
            robot.device.slider_positions,
            robot.device.joint_positions,
            robot.device.joint_velocities,
            robot.device.ctrl_joint_torques,
        )

        self.trace(robot)

    def trace(self, robot):
        self.leg_imp_ctrl.record_data(robot)
        self.circle.trace(robot)
        self.sliders.trace(robot)


def get_controller():
    """
    Common interface for controllers.

    This allow to have a generic simulation file
    """
    return TeststandFootCircle(prefix="teststand_")


if ("robot" in globals()) or ("robot" in locals()):
    ctrl = get_controller()
    ctrl.plug(robot)
