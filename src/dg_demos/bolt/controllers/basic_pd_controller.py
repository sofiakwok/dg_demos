import numpy as np

import dynamic_graph as dg
from dynamic_graph.sot.core.control_pd import ControlPD
from robot_properties_bolt.config import BoltConfig

from dynamic_graph.sot.tools import Oscillator

from dynamic_graph.sot.core.math_small_entities import (
    Multiply_double_vector,
    Add_of_double,
)

from dg_tools.utils import add_vec_vec

np.set_printoptions(suppress=True)


class BoltPDController(object):
    def __init__(self, prefix=""):
        self.prefix = prefix
        # self.sliders = Sliders(4, self.prefix)
        # self.sliders.set_scale_values([1.5, 2.0, 1.0, 1.0])

        self.bolt_config = BoltConfig()

        # Setup the control graph to track the desired joint positions.
        self.pd = pd = ControlPD("PDController")

        # setup the gains
        pd.Kp.value = np.array(6 * [3.0])
        pd.Kd.value = np.array(6 * [0.05])

        pd.desired_velocity.value = np.array(
            self.bolt_config.initial_velocity[6:]
        )

        #self.joint_pos = Multiply_double_vector(prefix + "joint_angles")
        #dg.plug(self.height_add.sout, self.desired_joint_angles.sin1)
        #self.joint_pos.sin2.value = np.array(self.bolt_config.initial_configuration[7:])
        self.starting_pos = np.array(6 * [0.0])
        bend_angle = 0.2 #20 * np.pi/180
        self.final_pos = np.array([0.0, bend_angle, -2*bend_angle, 0.0, bend_angle, -2*bend_angle])
        #np.array(self.bolt_config.initial_configuration[7:])
        # Specify the desired joint positions.
        pd.desired_position.value = self.starting_pos
        #dg.plug(self.joint_pos.sout, self.pd.desired_position)

        #self.slider_values = self.sliders

        print("done initializing pd controller")

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

    def plug_to_robot(self, robot):
        self.plug(
            robot.device.joint_positions,
            robot.device.joint_velocities,
            robot.device.ctrl_joint_torques,
        )

    def plug(
        self,
        joint_positions,
        joint_velocities,
        ctrl_joint_torques,
    ):
        print(joint_positions)
        # plug the desired quantity signals in the pd controller.
        dg.plug(joint_positions, self.pd.position)
        dg.plug(joint_velocities, self.pd.velocity)
        dg.plug(self.pd.control, ctrl_joint_torques)
        # self.set_desired_position()
        # self.jump_from_bent_legs()

    def record_data(self, robot):
        # Adding logging traces.
        robot.add_trace(self.pd.name, "desired_position")


def get_controller():
    return BoltPDController(prefix="Bolt_")


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