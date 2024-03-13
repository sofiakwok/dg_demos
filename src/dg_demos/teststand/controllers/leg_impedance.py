"""
@package dg_demos
@file teststand/controllers/leg_impedance.py
@author Avadesh Meduri
@license License BSD-3-Clause
@copyright Copyright (c) 2019, New York University and Max Planck Gesellschaft.
@date 2019-03-19
@brief Simple impedance controller implementation on the Teststand robot
"""


from dynamic_graph import plug
from dg_tools.math_small_entities import StackZero, ConstantVector
from dg_tools.leg_impedance_control.leg_impedance_controller import (
    LegImpedanceController,
)


class LegImpedance(object):
    """
    This class integrate a leg impedance controller for the Teststand robot
    """

    def __init__(self, prefix=""):
        """
        Initialize the controller
        """
        self.name = prefix + "LegImpedance_"

        kp = 20.0
        kd = 0.5
        foot_rel_height = -0.2

        self.kp_split = ConstantVector(
            [kp, 0.0, kp, 0.0, 0.0, 0.0], self.name + "kp_split"
        )
        self.kd_split = ConstantVector(
            [kd, 0.0, kd, 0.0, 0.0, 0.0], self.name + "kd_split"
        )
        self.des_pos = ConstantVector(
            [0.0, 0.0, foot_rel_height, 0.0, 0.0, 0.0], self.name + "pos_des"
        )
        self.des_vel = ConstantVector(
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0], self.name + "vel_des"
        )

        self.leg_imp_ctrl = LegImpedanceController(self.name[:-1])

        self.joint_positions = StackZero(
            1, 2, prefix=self.name + "joint_positions_"
        )
        plug(self.joint_positions.sout, self.leg_imp_ctrl.robot_dg.position)

        self.joint_velocities = StackZero(
            1, 2, prefix=self.name + "joint_velocities_"
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
        joint_positions_sout,
        joint_velocities_sout,
        ctrl_joint_torques_sin,
    ):
        plug(joint_positions_sout, self.joint_positions.sin)
        plug(joint_velocities_sout, self.joint_velocities.sin)
        plug(self.ctrl_sout, ctrl_joint_torques_sin)

    def plug(self, robot):
        self.plug_signals(
            robot.device.joint_positions,
            robot.device.joint_velocities,
            robot.device.ctrl_joint_torques,
        )

        self.trace(robot)

    def set_gains(self, kp, kd):
        self.kp_split.sout.value = kp
        self.kd_split.sout.value = kd

    def trace(self, robot):
        self.leg_imp_ctrl.record_data(robot)

    def add_to_ros(self, robot):
        self.leg_imp_ctrl.add_to_ros(robot)
        robot.add_to_ros(self.leg_imp_ctrl.robot_dg.name, "position")
        robot.add_to_ros(self.leg_imp_ctrl.robot_dg.name, "velocity")
        robot.add_to_ros(self.name + "pos_des", "sout")
        robot.add_to_ros(self.name + "vel_des", "sout")


def get_controller():
    """
    Common interface for controllers.

    This allow to have a generic simulation file
    """
    return LegImpedance(prefix="teststand_")


if ("robot" in globals()) or ("robot" in locals()):
    ctrl = get_controller()
    ctrl.plug(robot)
