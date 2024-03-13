import numpy as np
from pinocchio.utils import zero

from dynamic_graph import plug
from dynamic_graph.sot.core.control_pd import ControlPD
from robot_properties_solo.config import Solo8Config
from dg_tools.sliders import Sliders
from dynamic_graph.sot.core.math_small_entities import (
    Multiply_double_vector,
)


class PDController(object):
    def __init__(self, prefix):
        self.prefix = prefix

        # Setup the control graph to track the desired joint positions.
        self.pd = pd = ControlPD(self.prefix + "_PDController")

        # define the desired position and set the desired velocity 0.
        solo_config = Solo8Config()
        pd.desired_position.value = np.zeros(8)
        pd.desired_velocity.value = np.zeros(8)

        self.sliders = Sliders(4, self.prefix)
        self.sliders.set_scale_values([1.0, 1.0, 1.0, 1.0])

    def follow_slider(self):
        self.desired_pos_op = Multiply_double_vector(
            self.prefix + "_desired_pos_op"
        )
        plug(self.sliders.A, self.desired_pos_op.sin1)
        self.desired_pos_op.sin2.value = np.array(
            [1.0, -2.0, 1.0, -2.0, -1.0, 2.0, -1.0, 2.0]
        )

        # Specify the desired joint positions.
        plug(self.desired_pos_op.sout, self.pd.desired_position)

    def plug(self, robot):
        # Plug the sliders.
        self.sliders.plug_slider_signal(robot.device.slider_positions)

        # Offset the sliders by the current value.
        self.sliders.set_offset_values(
            (-np.array(robot.device.slider_positions.value)).tolist()
        )

        # setup the gains
        pd = self.pd
        pd.Kp.value = np.array(8 * (2.0,))
        pd.Kd.value = np.array(8 * (0.05,))

        # plug the desired quantity signals in the pd controller.
        plug(robot.device.joint_positions, pd.position)
        plug(robot.device.joint_velocities, pd.velocity)
        # plug the ouput of the pd controller to the robot motor torques
        plug(pd.control, robot.device.ctrl_joint_torques)


def solo_pd_control(robot):
    ctrl = PDController("solo8_pd_")
    ctrl.plug(robot)
    return ctrl


if "robot" in globals():
    print("Hello world from PD")
    ctrl = solo_pd_control(robot)
