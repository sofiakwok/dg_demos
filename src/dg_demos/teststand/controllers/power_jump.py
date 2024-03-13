"""
@package dg_demos
@file teststand/controllers/jump.py
@author Avadesh Meduri
@license License BSD-3-Clause
@copyright Copyright (c) 2019, New York University and Max Planck Gesellschaft.
@date 2019-04-14
@brief Controller for power jump with teststand
"""

from dynamic_graph import plug
from dg_tools.math_small_entities import (
    StackZero,
    ConstantVector,
    hom2pos,
    subtract_vec_vec,
    stack_two_vectors,
    Add_of_double,
    constVector,
)
from dg_tools.leg_impedance_control.leg_impedance_controller import (
    LegImpedanceController,
)
from dynamic_graph_manager.dg_tools import power_jump_control


class PowerJump:
    """
    TODO this controller needs debugging!!!
    """

    def __init__(self, prefix=""):
        self.name = prefix + "PowerJump"

        self.des_weight_fff = ConstantVector(
            [0.0, 0.0, 1.8 * 9.81, 0.0, 0.0, 0.0],
            self.name + "_des_weight_fff",
        )
        self.des_fff = ConstantVector(
            [0.0, 0.0, 4.0 * 9.81, 0.0, 0.0, 0.0], self.name + "_des_fff"
        )
        self.des_pos_trigger = ConstantVector(
            [0.0, 0.0, -0.18, 0.0, 0.0, 0.0], self.name + "_pos_des_trigger"
        )
        self.des_pos_air = ConstantVector(
            [0.0, 0.0, -0.25, 0.0, 0.0, 0.0], self.name + "_pos_des_air"
        )
        self.des_vel = ConstantVector(
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0], self.name + "_vel_des"
        )

        self.des_kp_ground = ConstantVector(
            [100.0, 0.0, 250.0, 0.0, 0.0, 0.0], self.name + "_des_kp_ground"
        )
        self.des_kp_air = ConstantVector(
            [50.0, 0.0, 150.0, 0.0, 0.0, 0.0], self.name + "_des_kp_air"
        )
        self.des_kd = ConstantVector(
            [1.0, 0.0, 2.5, 0.0, 0.0, 0.0], self.name + "_des_kd_air"
        )

        self.leg_imp_ctrl = LegImpedanceController(self.name + "_leg_imp_ctrl")

        self.joint_positions = StackZero(
            1, 2, prefix=self.name + "joint_positions_"
        )
        plug(self.joint_positions.sout, self.leg_imp_ctrl.robot_dg.position)

        self.joint_velocities = StackZero(
            1, 2, prefix=self.name + "joint_velocities_"
        )
        plug(self.joint_velocities.sout, self.leg_imp_ctrl.robot_dg.velocity)

        self.leg_imp_ctrl.xyzpos_hip = hom2pos(
            self.leg_imp_ctrl.robot_dg.signal(
                "pos_hip_" + self.leg_imp_ctrl.leg_name
            ),
            "xyzpos_hip_" + self.leg_imp_ctrl.leg_name,
        )

        self.leg_imp_ctrl.xyzpos_foot = hom2pos(
            self.leg_imp_ctrl.robot_dg.signal(
                "pos_foot_" + self.leg_imp_ctrl.leg_name
            ),
            "xyzpos_foot_" + self.leg_imp_ctrl.leg_name,
        )

        self.leg_imp_ctrl.rel_pos_foot = subtract_vec_vec(
            self.leg_imp_ctrl.xyzpos_foot,
            self.leg_imp_ctrl.xyzpos_hip,
            "rel_pos_foot_" + self.leg_imp_ctrl.leg_name,
        )

        self.leg_imp_ctrl.rel_pos_foot = stack_two_vectors(
            self.leg_imp_ctrl.rel_pos_foot,
            constVector(
                [0.0, 0.0, 0.0],
                "stack_to_wrench_" + self.leg_imp_ctrl.leg_name,
            ),
            3,
            3,
        )

        self.power_jump_ctrl = power_jump_control("power_jump_ctrl")

        plug(self.leg_imp_ctrl.rel_pos_foot, self.power_jump_ctrl.leg_length)
        plug(
            self.des_pos_trigger.sout, self.power_jump_ctrl.leg_length_trigger
        )
        plug(self.des_pos_air.sout, self.power_jump_ctrl.leg_length_air)
        plug(self.des_fff.sout, self.power_jump_ctrl.des_fff)
        plug(self.des_weight_fff.sout, self.power_jump_ctrl.des_weight_fff)
        plug(self.des_kp_ground.sout, self.power_jump_ctrl.kp_ground)
        plug(self.des_kp_air.sout, self.power_jump_ctrl.kp_air)

        self.des_pos = self.power_jump_ctrl.des_pos
        self.des_force = self.power_jump_ctrl.des_force
        self.des_kp = self.power_jump_ctrl.des_kp

        # For making gain input dynamic through terminal
        self.add_kf = Add_of_double("kf")
        self.add_kf.sin1.value = 0
        # Change this value for different gains
        self.add_kf.sin2.value = 1.0
        self.des_kf = self.add_kf.sout

        self.control_torques = self.leg_imp_ctrl.return_control_torques(
            self.des_kp,
            self.des_pos,
            self.des_kd.sout,
            self.des_vel.sout,
            self.des_kf,
            self.des_force,
        )

    def plug(self, robot):
        plug(robot.device.joint_positions, self.joint_positions.sin)
        plug(robot.device.joint_velocities, self.joint_velocities.sin)
        plug(robot.device.contact_sensors, self.power_jump_ctrl.cnt_sensor)
        plug(self.control_torques, robot.device.ctrl_joint_torques)

    def trace(self, robot):
        self.leg_imp_ctrl.record_data(robot)
        robot.add_trace("power_jump_ctrl", "des_pos")
        robot.add_trace("power_jump_ctrl", "des_force")
        robot.add_trace("power_jump_ctrl", "des_kp")


def get_controller():
    """
    Common interface for controllers.

    This allow to have a generic simulation file
    """
    return PowerJump(prefix="teststand_")


if ("robot" in globals()) or ("robot" in locals()):
    ctrl = get_controller()
    ctrl.plug(robot)
