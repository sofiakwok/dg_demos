"""
@package dg_demos
@file teststand/controllers/stiffness_measurement.py
@author Maximilien Naveau
@license License BSD-3-Clause
@copyright Copyright (c) 2019, New York University and Max Planck Gesellschaft.
@date 2019-02-06
@brief Impedance controller easily Tunable online
"""

from dynamic_graph import plug
from dg_tools.math_small_entities import (
    StackZero,
    ConstantDouble,
    ConstantVector,
    Multiply_double_vector,
)
from dg_tools.leg_impedance_control.leg_impedance_controller import (
    LegImpedanceController,
)
from dynamic_graph.sot.core.fir_filter import FIRFilter_Vector_double


class StiffnessMeasurement:
    def __init__(self, prefix=""):
        #
        # Get the arguments
        #
        self.name = prefix + "StiffnessMeasurement_"

        #
        # Filter the input signals
        #

        # we define a filter for the height sensor, the output is our Height
        # sensor data
        self.height_sensor_filtered = FIRFilter_Vector_double(
            self.name + "_height_sensor"
        )
        self.force_sensor_filtered = FIRFilter_Vector_double(
            self.name + "_force_sensor"
        )

        # initilialize the filters
        filter_size = 100
        self.height_sensor_filtered.setSize(filter_size)
        for i in range(filter_size):
            self.height_sensor_filtered.setElement(i, 1.0 / float(filter_size))
        self.force_sensor_filtered.setSize(filter_size)
        for i in range(filter_size):
            self.force_sensor_filtered.setElement(i, 1.0 / float(filter_size))

        #
        # Defines the impedance controller parameters
        #
        self.kp_z = ConstantDouble(20.0, self.name + "_kp_z")
        self.foot_rel_height = -0.27

        # Along axis-x the gains are multiplied by 0.4,
        # along axis-z gains are unmodified
        self.kp_const_vec = ConstantVector(
            [0.4, 0.0, 1.0, 0.0, 0.0, 0.0], self.name + "kp_const_vec"
        )
        self.kp_split = Multiply_double_vector(self.name + "kp_split")
        plug(self.kp_z.sout, self.kp_split.sin1)
        plug(self.kp_const_vec.sout, self.kp_split.sin2)

        # No damping by default!!!
        self.kd_split = ConstantVector(
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0], "kd_split"
        )

        # The foot pose relative to the base
        self.des_pos = ConstantVector(
            [0.0, 0.0, self.foot_rel_height, 0.0, 0.0, 0.0],
            self.name + "_pos_des",
        )
        self.des_vel = ConstantVector(
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0], self.name + "_vel_des"
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

    def set_vertical_stiffness(self, kp_z):
        self.kp_z.set_value(kp_z)

    def plug_signals(
        self,
        height_sensors_sout,
        ati_force_sout,
        joint_positions_sout,
        joint_velocities_sout,
        ctrl_joint_torques_sin,
    ):
        """
        Optional method in order to plus other stuff to this.
        """
        plug(height_sensors_sout, self.height_sensor_filtered.sin)
        plug(ati_force_sout, self.force_sensor_filtered.sin)
        plug(joint_positions_sout, self.joint_positions.sin)
        plug(joint_velocities_sout, self.joint_velocities.sin)
        plug(self.ctrl_sout, ctrl_joint_torques_sin)

    def plug(self, robot):
        """
        Mandtory method. Used in the tests and simulation scripts.
        """
        self.plug_signals(
            robot.device.height_sensors,
            robot.device.ati_force,
            robot.device.joint_positions,
            robot.device.joint_velocities,
            robot.device.ctrl_joint_torques,
        )

        self.trace(robot)

    def trace(self, robot):
        self.leg_imp_ctrl.record_data(robot)

        robot.add_trace(self.name + "_pos_des", "sout")
        robot.add_trace(self.name + "_vel_des", "sout")
        robot.add_trace(self.name + "_height_sensor", "sout")
        robot.add_trace(self.name + "_force_sensor", "sout")


def get_controller():
    """
    Common interface for controllers.

    This allow to have a generic simulation file
    """
    return StiffnessMeasurement(prefix="teststand_")


if ("robot" in globals()) or ("robot" in locals()):
    ctrl = get_controller()
    ctrl.plug(robot)
