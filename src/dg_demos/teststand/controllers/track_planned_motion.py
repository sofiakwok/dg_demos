"""
@package dg_demos
@file teststand/controllers/track_planned_motion.py
@author Elham Daneshmand
@license License BSD-3-Clause
@copyright Copyright (c) 2019, New York University and Max Planck Gesellschaft.
@date 2019-02-06
@brief Controller that track the kino-dyn-opt output
"""

import time
from os.path import join, isfile
import rospkg
from dynamic_graph import plug
from dynamic_graph.sot.core.reader import Reader

from dg_tools.math_small_entities import (
    constVector,
    mul_double_vec_2,
    stack_zero,
    stack_two_vectors,
    mul_double_vec,
    Component_of_vector,
    Add_of_double,
)
from dg_tools.leg_impedance_control.leg_impedance_controller import (
    LegImpedanceController,
)
from dg_tools.sliders import Sliders


class PlannedMotion(object):
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
        self.sliders.set_scale_values([200.0, 10.0 / 8])

        self.kp = self.sliders.slider_A.double
        self.kd = self.sliders.slider_B.double

        self.unit_vec_101 = constVector(
            [1.0, 0.0, 1.0, 0.0, 0.0, 0.0], "unit_vec_101"
        )
        self.unit_vec_102 = constVector(
            [0.8, 0.0, 0.2, 0.0, 0.0, 0.0], "unit_vec_102"
        )

        self.kp = mul_double_vec_2(self.kp.sout, self.unit_vec_101, "kp")
        self.kd = mul_double_vec_2(self.kd.sout, self.unit_vec_102, "kd")

        self.reader_pos = Reader("PositionReader")
        self.reader_vel = Reader("VelocityReader")
        self.reader_fff = Reader("FeedForwardForceReader")
        self.reader_acc = Reader("AccelerationReader")

        # By default we look into the rospkg path
        self.load_trajectories()

        # Specify which of the columns to select.
        # NOTE: This is selecting the columns in reverse order - the last
        # number is the first column in the file
        self.reader_pos.selec.value = "000000000000000000111111"
        self.reader_vel.selec.value = "000000000000000000111111"
        self.reader_fff.selec.value = "0000000001110"
        self.reader_acc.selec.value = "11"

        self.des_pos = self.reader_pos.vector
        self.des_vel = self.reader_vel.vector
        self.des_fff = stack_two_vectors(
            self.reader_fff.vector, constVector([0.0, 0.0, 0.0], "zero"), 3, 3
        )
        self.des_acc = mul_double_vec(
            81 * 0.00000447, self.reader_acc.vector, "acc"
        )

        ###############################################################################
        self.ati_force = Component_of_vector("ati_force")

        self.height = Component_of_vector("height")

        ###############################################################################

        self.add_kf = Add_of_double("kf")
        self.add_kf.sin1.value = 0
        # Change this value for different gains
        self.add_kf.sin2.value = 1.0
        self.kf = self.add_kf.sout

        self.leg_imp_ctrl = LegImpedanceController("hopper")

        control_torques = self.leg_imp_ctrl.return_control_torques(
            self.kp,
            self.des_pos,
            self.kd,
            self.des_vel,
            self.kf,
            self.des_fff,
            self.des_acc,
        )

        plug(control_torques, robot.device.ctrl_joint_torques)

    def load_trajectories(self, file_dir=None):
        def file_exists(filename):
            if isfile(filename):
                print("The file %s exists" % filename)
            else:
                print("The file %s does not exist" % filename)
                assert False

        if file_dir is None:
            self.filename_pos = join(
                rospkg.RosPack().get_path("momentumopt"),
                "demos",
                "quadruped_positions_eff.dat",
            )
            self.filename_vel = join(
                rospkg.RosPack().get_path("momentumopt"),
                "demos",
                "quadruped_velocities_eff.dat",
            )
            self.filename_fff = join(
                rospkg.RosPack().get_path("momentumopt"),
                "demos",
                "quadruped_forces.dat",
            )
            self.filename_acc = join(
                rospkg.RosPack().get_path("momentumopt"),
                "demos",
                "quadruped_generalized_acceleration2.dat",
            )
        else:
            self.filename_pos = join(file_dir, "quadruped_positions_eff.dat")
            self.filename_vel = join(file_dir, "quadruped_velocities_eff.dat")
            self.filename_fff = join(file_dir, "quadruped_forces.dat")
            self.filename_acc = join(
                file_dir, "quadruped_generalized_acceleration2.dat"
            )

        file_exists(self.filename_pos)
        file_exists(self.filename_vel)
        file_exists(self.filename_fff)
        file_exists(self.filename_acc)

        print("Loading data files:")
        self.reader_pos.load(self.filename_pos)
        self.reader_vel.load(self.filename_vel)
        self.reader_fff.load(self.filename_fff)
        self.reader_acc.load(self.filename_acc)

    def start_traj(self):
        self.reader_pos.rewind()
        self.reader_pos.vector.recompute(0)
        self.reader_vel.rewind()
        self.reader_vel.vector.recompute(0)
        self.reader_fff.rewind()
        self.reader_fff.vector.recompute(0)

    def log_traj(self, robot):
        robot.start_tracer()
        self.start_traj()
        time.sleep(3)
        robot.stop_tracer()

    def plug(self, robot):
        """
        Mandtory method. Used in the tests and simulation scripts.
        """
        plug(robot.device.ati_force, self.ati_force.sin)
        plug(
            stack_zero(
                robot.device.joint_positions, "add_base_joint_position"
            ),
            self.leg_imp_ctrl.robot_dg.position,
        )
        plug(
            stack_zero(
                robot.device.joint_velocities, "add_base_joint_velocity"
            ),
            self.leg_imp_ctrl.robot_dg.velocity,
        )
        plug(robot.device.height_sensors, self.height.sin)

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
    return PlannedMotion(prefix="teststand_")


if ("robot" in globals()) or ("robot" in locals()):
    ctrl = get_controller()
    ctrl.plug(robot)
