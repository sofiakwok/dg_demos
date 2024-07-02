import numpy as np

import dynamic_graph as dg
from mim_control.dynamic_graph.bolt_pd_graph import BoltPD
from robot_properties_bolt.config import BoltConfig

from dynamic_graph.sot.tools import Oscillator

from dynamic_graph.sot.core.math_small_entities import (
    Multiply_double_vector,
    Add_of_double,
)

from dg_tools.utils import add_vec_vec

np.set_printoptions(suppress=True)


class BoltPDController(object):
    def __init__(self, prefix, is_real_robot):
        pin_robot = BoltConfig.buildRobotWrapper()
        end_effector_names = BoltConfig.end_effector_names

        self.is_real_robot = is_real_robot

        # Create the whole body controller.
        self.pd = pd = BoltPD(
            prefix + "_pd",
            pin_robot,
            end_effector_names,
        )

        if self.is_real_robot:
            des_com_pos = np.array(
                [0.0, 0.0, 0.38487417 - 0.05]
            )  # self.com_height
        else:
            des_com_pos = np.array(
                [0.0, 0.0, 0.35487417]
            )

        des_quat = np.array([0, 0, 0, 1])

        des_joint_pos = np.array([-0.3, 0.78539816, -1.57079633, 0.3, 0.78539816, -1.57079633])

        pd.des_robot_configuration_sin.value = np.concatenate((des_com_pos, des_quat, des_joint_pos), axis=None)
        pd.des_robot_velocity_sin.value = np.zeros(12)

        print("BoltPDStepper init done")

    def set_desired_position(self, joint_pos):
        ctrl.pd.desired_position.value = joint_pos
            
    def bend_legs(self):
        # go from straight legs to bent legs
        timescale = 50000 
        # go to bent knee position
        for i in range(timescale):
            # make a linear function from starting joint position to desired joint position
            joint_pos = ctrl.final_pos*(i/timescale)
            # Specify the desired joint positions.
            ctrl.pd.desired_position.value = joint_pos
        print(joint_pos)

    def jump_from_bend(self):

        timescale = 20000 #works without triggering driver error
        # go to straight legs
        for i in range(timescale):
            # make a linear function from starting joint position to desired joint position
            joint_pos = ctrl.final_pos - ctrl.final_pos*(i/timescale)
            # Specify the desired joint positions.
            ctrl.pd.desired_position.value = joint_pos
        print(joint_pos)

    def plug(self, robot, base_position, base_velocity):
        self.base_position = base_position
        self.robot = robot
        #self.sliders.plug_slider_signal(robot.device.slider_positions)
        self.pd.plug(robot, base_position, base_velocity)
        #self.plug_swing_foot_forces()

    def trace(self):
        print("robot for controller trace: " + str(self.robot))
        self.pd.trace(self.robot)


def get_controller(prefix="biped_pd_stepper", is_real_robot=False):
    return BoltPDController(prefix, is_real_robot)


if "robot" in globals():
    ctrl = get_controller()

    def go():
        ctrl.plug_to_robot(robot)
        print("plugged robot")

    def bend():
        ctrl.bend_legs()

    def jump():
        ctrl.jump_from_bend()

    print("#####")
    print("")
    print("Please execute `go()` function to start the controller.")
    print("")
    print("#####")