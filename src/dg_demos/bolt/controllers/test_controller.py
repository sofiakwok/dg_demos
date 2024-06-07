"""This file is a demo for using the DG whole body controller.

License BSD-3-Clause
Copyright (c) 2021, New York University and Max Planck Gesellschaft.

Author: Julian Viereck, Elham Daneshmand
Date:   Oct 26, 2021
"""
import numpy as np

#np.set_printoptions(suppress=True, precision=3)
#import pinocchio as pin
import dynamic_graph as dg
from dynamic_graph.sot.core.control_pd import ControlPD

from lqr_control.dynamic_graph.test_graph import testController
from robot_properties_bolt.config import BoltConfig

from dg_tools.utils import (
    constVector,
    stack_two_vectors,
    selec_vector,
    add_vec_vec,
    subtract_vec_vec,
    multiply_mat_vec,
    Stack_of_vector,
    zero_vec,
    mul_double_vec,
    mul_vec_vec,
    constDouble,
)
from dynamic_graph.sot.core.math_small_entities import Component_of_vector

from dynamic_graph.sot.core.math_small_entities import Add_of_double

from dg_tools.dynamic_graph.dg_tools_entities import (
    CreateWorldFrame,
    PoseRPYToPoseQuaternion,
)

class BoltLQRStepper:
    def __init__(self, prefix, is_real_robot):
        pin_robot = BoltConfig.buildRobotWrapper()
        end_effector_names = BoltConfig.end_effector_names

        self.is_real_robot = is_real_robot

        # Create the whole body controller.
        self.lqr = lqr = testController(
            prefix + "_test",
            pin_robot,
            end_effector_names,
        )

        if self.is_real_robot:
            des_com_pos = np.array(
                [0.0, 0.0, 0.38487417 - 0.05]
            )  # self.com_height
        else:
            des_com_pos = np.array(
                [0.0, 0.0, 0.40795507 - 0.045]
            )
        des_com_vel = np.zeros(3)
        des_ori_vel = np.zeros(3)

        des_quat = np.array([0, 0, 0, 1])

        print("Bolt testStepper init done")

    def plug(self, robot, base_position, base_velocity):
        self.base_position = base_position
        self.robot = robot
        #self.sliders.plug_slider_signal(robot.device.slider_positions)
        self.lqr.plug(robot, base_position, base_velocity)
        #self.plug_swing_foot_forces()

    def plug_base_as_com(self, base_position, base_velocity_world):
        self.lqr.plug_base_as_com(base_position, base_velocity_world)
        self.lqr.des_com_pos_sin.value = np.array(
            [0.0, 0.0, self.com_height + self.base_com_offset]
        )
    
    def trace(self):
        print("robot for controller trace: " + str(self.robot))
        self.lqr.trace(self.robot)

        # self.robot.add_trace(self.stepper.stepper.name, 'swing_foot_forces_sout')
        # self.robot.add_trace(
        #     self.stepper.stepper.name, "next_support_foot_position_sout"
        # )
        # self.robot.add_trace(
        #     self.stepper.stepper.name, "left_foot_position_sout"
        # )
        # self.robot.add_trace(
        #     self.stepper.stepper.name, "right_foot_position_sout"
        # )
        self.robot.add_trace("des_pos_l", "sout")
        self.robot.add_trace("des_pos_r", "sout")
        self.robot.add_trace("imp0", "sout")
        self.robot.add_trace("imp1", "sout")
        self.robot.add_trace("mulp0", "sout")
        self.robot.add_trace("muld0", "sout")
        self.robot.add_trace("mulp1", "sout")
        self.robot.add_trace("muld1", "sout")

        # self.robot.add_trace("optitrack_entity", "1049_position")
        # self.robot.add_trace("optitrack_entity", "1049_velocity_world")
        # self.robot.add_trace("optitrack_entity", "1049_velocity_body")
        # self.robot.add_trace("des", "sout")


def get_controller(prefix="biped_lqr_stepper", is_real_robot=False):
    return BoltLQRStepper(prefix, is_real_robot)
