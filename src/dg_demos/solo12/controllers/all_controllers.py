"""master_demo

License BSD-3-Clause
Copyright (c) 2019, New York University and Max Planck Gesellschaft.

Collects all available demos and allows a selection among them.
"""

import numpy as np
from robot_properties_solo.config import Solo12Config
from dynamic_graph.sot.core.switch import SwitchVector
from mim_control.dynamic_graph.go_to import GoTo
from dg_demos.solo12.controllers.pd_controller import (
    get_controller as pd_osc_ctrl,
)
from dg_demos.solo12.controllers.impedance_controller import (
    get_controller as imp_ctrl,
)


class CtrlMode(object):
    """Control modes of the graph below."""

    (
        IDLE,
        GO_TO,
        PD_OSC,
        IMPEDANCE,
        NUMBER,
    ) = range(5)


class AllControllers(object):
    """LTS integration of the controllers implemented in dg_demos. """

    def __init__(self):

        # Robot configuration.
        config = Solo12Config()

        # Main switch. It is plugged to the robot control vector.
        self.main_switch = SwitchVector("main_switch")
        self.master_switch.setSignalNumber(CtrlMode.NUMBER)
        self.master_switch.selection.value = CtrlMode.IDLE  # (default)

        # Idle mode:
        self.master_switch.sin(CtrlMode.IDLE).value = np.array(
            self.nb_joints * [0.0]
        )

        # Go to mode
        self._go_to = GoTo(config.nb_joints)

        # pd_controller + slider mode
        self._pd_osc_ctrl = pd_osc_ctrl()

        # Impedance controller
        self._imp_ctrl = imp_ctrl()

    def plug(self, robot):
        self._joint_positions = robot.device.joint_positions
        self._joint_velocities = robot.device.joint_velocities
        self._slider_positions = robot.device.slider_positions

        self._go_to.plug(
            self._joint_positions,
            self._joint_velocities,
            self.master_switch.sin(CtrlMode.GO_TO),
        )

        self._pd_osc_ctrl.plug(
            self._slider_positions,
            self._joint_positions,
            self._joint_velocities,
            self.master_switch.sin(CtrlMode.PD_OSC),
        )

        self._imp_ctrl.plug(
            self._joint_positions,
            self._joint_velocities,
            self._slider_positions,
            self.master_switch.sin(CtrlMode.IMPEDANCE),
        )

    #
    # changing mode
    #
    def is_switch_mode_safe(self):
        """Make sure the switch is safe."""
        pass

    def set_idle_mode(self):
        """Use idle mode (zero torques to all joints). """
        self.master_switch.selection.value = CtrlMode.IDLE

    def set_go_to_mode(self):
        """Use the go_to controller, freeze the robot upon activation. """
        # freeze the robot.
        self.go_to.freeze()
        # Activate the controller
        self.master_switch.selection.value = CtrlMode.GO_TO

    def set_pd_osc_mode(self):
        """Use PD controller with oscillators input from the sliders. """
        # Offset the sliders by the current value.
        self._pd_osc_ctrl.sliders.set_offset_values(
            -self._slider_positions.value
        )
        # (re-)plug the desired position from the sliders.
        self._pd_osc_ctrl.set_sliders_desired_position()
        # Activate the controller
        self.master_switch.selection.value = CtrlMode.PD_OSC

    #
    # Utilities
    #
    def freeze(self):
        """ Freeze the robot at the current joint position. """
        self.set_go_to_mode()

    def go_to(self, desired_joint_position_rad, nb_iteration):
        self._go_to.go_to(desired_joint_position_rad, nb_iteration)
