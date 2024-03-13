import numpy as np

import dynamic_graph as dg
from dynamic_graph.sot.core.control_pd import ControlPD
from robot_properties_solo.config import Solo12Config

from dg_tools.sliders import Sliders
from dynamic_graph.sot.tools import Oscillator

from dynamic_graph.sot.core.math_small_entities import (
    Multiply_double_vector,
    Add_of_double,
)

from dg_tools.utils import add_vec_vec

np.set_printoptions(suppress=True)


class Solo12PDController(object):
    def __init__(self, prefix=""):
        self.prefix = prefix
        self.sliders = Sliders(4, self.prefix)
        self.sliders.set_scale_values([1.5, np.pi / 2 - np.pi / 9, 1.0, 1.0])

        self.solo_config = Solo12Config()

        # Setup the control graph to track the desired joint positions.
        self.pd = pd = ControlPD("PDController")

        # setup the gains
        pd.Kp.value = np.array(12 * [3.0])
        pd.Kd.value = np.array(12 * [0.05])

        pd.desired_velocity.value = np.array(
            self.solo_config.initial_velocity[6:]
        )

        # HAA angles.
        self.desired_haa_pos_op = Multiply_double_vector(prefix + "haa_pos_op")
        dg.plug(self.sliders.A, self.desired_haa_pos_op.sin1)
        self.desired_haa_pos_op.sin2.value = np.array(
            [1.0, 0.0, 0.0, -1.0, 0.0, 0.0, 1.0, 0.0, 0.0, -1, 0.0, 0.0]
        )

        # HFE and KFE angles
        self.osc_height = Oscillator(prefix + "height_oscillator")
        self.osc_height.setTimePeriod(0.001)
        self.osc_height.omega.value = 2.0 * np.pi / 2.0
        self.osc_height.magnitude.value = 0.0
        self.osc_height.bias.value = 0.0

        self.height_add = Add_of_double(prefix + "add_slider_oscillator")
        dg.plug(self.sliders.B, self.height_add.sin(0))
        dg.plug(self.osc_height.sout, self.height_add.sin(1))

        self.height_add.sin(1).value = np.pi / 4

        self.desired_fe_pos_op = Multiply_double_vector(prefix + "fe_pos_op")
        dg.plug(self.height_add.sout, self.desired_fe_pos_op.sin1)
        self.desired_fe_pos_op.sin2.value = np.array(
            [0.0, 1.0, -2.0, 0.0, 1.0, -2.0, 0, -1.0, 2.0, 0, -1.0, 2.0]
        )

        self.desired_joint_pos = add_vec_vec(
            self.desired_haa_pos_op.sout, self.desired_fe_pos_op.sout
        )

        # Specify the desired joint positions.
        dg.plug(self.desired_joint_pos, self.pd.desired_position)

    def set_sliders_desired_position(self):
        # Specify the desired joint positions.
        dg.plug(self.desired_joint_pos, self.pd.desired_position)

    def set_desired_position(self, desired_pos):
        self.pd.desired_position.value = desired_pos

    def plug_to_robot(self, robot):
        self.plug(
            robot.device.slider_positions,
            robot.device.joint_positions,
            robot.device.joint_velocities,
            robot.device.ctrl_joint_torques,
        )

    def plug(
        self,
        slider_positions,
        joint_positions,
        joint_velocities,
        ctrl_joint_torques,
    ):
        # Plug the sliders.
        self.sliders.plug_slider_signal(slider_positions)
        # Offset the sliders by the current value.
        self.sliders.zero_slider()
        # plug the desired quantity signals in the pd controller.
        dg.plug(joint_positions, self.pd.position)
        dg.plug(joint_velocities, self.pd.velocity)
        dg.plug(self.pd.control, ctrl_joint_torques)

    def simple_plug(
        self,
        slider_positions,
        joint_positions,
        joint_velocities,
        ctrl_joint_torques,
    ):
        # Plug the sliders.
        self.sliders.plug_slider_signal(slider_positions)
        # plug the desired quantity signals in the pd controller.
        dg.plug(joint_positions, self.pd.position)
        dg.plug(joint_velocities, self.pd.velocity)
        dg.plug(self.pd.control, ctrl_joint_torques)

    def zero_slider(self):
        self.sliders.zero_slider()

    def record_data(self, robot):
        # Adding logging traces.
        robot.add_trace(self.pd.name, "desired_position")


def get_controller(prefix="solo12_"):
    return Solo12PDController(prefix=prefix)


if "robot" in globals():
    ctrl = get_controller()

    def go():
        ctrl.plug_to_robot(robot)

    def go_vicon():
        """
        Adds tracking via vicon and adds the vicon signal to the tracer.
        """
        from dg_vicon_sdk.dynamic_graph.entities import ViconClientEntity

        vicon = ViconClientEntity("vicon_entity")
        vicon.connect_to_vicon("172.24.117.119:801")  # NYU MIM vicon.
        vicon.add_object_to_track("solo12/solo12")

        robot.add_ros_and_trace("vicon_entity", "solo12_position")
        robot.add_ros_and_trace("vicon_entity", "solo12_velocity_body")
        robot.add_ros_and_trace("vicon_entity", "solo12_velocity_world")

    def angles():
        print(np.array(robot.device.joint_positions.value).reshape(4, 3))

    print("#####")
    print("")
    print("Please execute `go()` function to start the controller.")
    print("")
    print("#####")
