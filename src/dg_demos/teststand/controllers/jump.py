"""
@package dg_demos
@file teststand/controllers/jump.py
@author Maximilien Naveau
@license License BSD-3-Clause
@copyright Copyright (c) 2019, New York University and Max Planck Gesellschaft.
@date 2019-02-06
@brief Impedance controller easily Tunable online
"""

from dynamic_graph import plug
import numpy as np
from dg_tools.math_small_entities import (
    StackZero,
    ConstantDouble,
    ConstantVector,
    Multiply_double_vector,
)
from dg_tools.sliders import Sliders
from dg_tools.leg_impedance_control.leg_impedance_controller import (
    LegImpedanceController,
)
from dynamic_graph.sot.core.fir_filter import FIRFilter_Vector_double
from dg_tools.traj_generators import CircularCartesianTrajectoryGenerator


class Jump:
    def __init__(self, prefix=""):
        #
        # Get the arguments
        #
        self.name = prefix + "Jump_"

        #
        # Controller parameters
        #

        self.kp_scale = 500.0
        # amplitude of foot oscillation on axis-z
        self.magnitude_z_scale = 0.08
        self.leg_length = -0.22

        #
        # Filter the input signals
        #

        # managing the sliders
        self.sliders = Sliders(2, self.name)
        # scale the sliders
        self.sliders.set_scale_values([self.kp_scale, self.magnitude_z_scale])

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
        # Define a circular trajectory depending on the silders on the axis-z
        #

        # Magnitude = [0, 0, Slider_B]
        self.magnitude_z = self.sliders.slider_B.double
        self.magnitude_xy = ConstantDouble(0.0, self.name + "radius_y")
        # Omega = 3 * [slider_B,]
        self.omega_xyz = ConstantDouble(4.0 * np.pi, self.name + "omega_xyz")
        # Phase = [0, 0, 0]
        self.phase_xyz = ConstantDouble(0.0, self.name + "phase_yz")
        # Bias = [0, 0, -]
        self.bias_xy = ConstantDouble(0.0, self.name + "bias_xy")
        self.bias_z = ConstantDouble(self.leg_length, self.name + "bias_z")
        # Circular trajectory entity
        self.z_osc = CircularCartesianTrajectoryGenerator(self.name + "z_osc")
        self.z_osc.plug(
            [self.magnitude_xy, self.magnitude_xy, self.magnitude_z],
            [self.omega_xyz, self.omega_xyz, self.omega_xyz],
            [self.phase_xyz, self.phase_xyz, self.phase_xyz],
            [self.bias_xy, self.bias_xy, self.bias_z],
        )

        #
        # Defines the impedance controller parameters
        #

        # Along axis-x the gains are multiplied by 0.4,
        # along axis-z gains are unmodified
        self.kp_const_vec = ConstantVector(
            [0.4, 0.0, 1.0, 0.0, 0.0, 0.0], self.name + "kp_const_vec"
        )
        self.kp_split = Multiply_double_vector(self.name + "kp_split")
        self.kp = self.sliders.slider_A.double
        plug(self.kp.sout, self.kp_split.sin1)
        plug(self.kp_const_vec.sout, self.kp_split.sin2)

        # Some minimal damping
        self.kd_split = ConstantVector(
            [0.8, 0.0, 2.0, 0.0, 0.0, 0.0], "kd_split"
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
            self.z_osc.des_pos.sout,
            self.kd_split.sout,
            self.z_osc.des_vel.sout,
        )

    def plug_signals(
        self,
        sliders_sout,
        height_sensors_sout,
        ati_force_sout,
        joint_positions_sout,
        joint_velocities_sout,
        ctrl_joint_torques_sin,
    ):
        """
        Optional method in order to plus other stuff to this.
        """
        plug(sliders_sout, self.sliders.sin)
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
            robot.device.slider_positions,
            robot.device.height_sensors,
            robot.device.ati_force,
            robot.device.joint_positions,
            robot.device.joint_velocities,
            robot.device.ctrl_joint_torques,
        )

        self.trace(robot)

    def trace(self, robot):
        self.leg_imp_ctrl.record_data(robot)
        self.z_osc.trace(robot)

        robot.add_trace(self.name + "_height_sensor", "sout")
        robot.add_trace(self.name + "_force_sensor", "sout")


def get_controller():
    """
    Common interface for controllers.

    This allow to have a generic simulation file
    """
    return Jump(prefix="teststand_")


if ("robot" in globals()) or ("robot" in locals()):
    ctrl = get_controller()
    ctrl.plug(robot)
