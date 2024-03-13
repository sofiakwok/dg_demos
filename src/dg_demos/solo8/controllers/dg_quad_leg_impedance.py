## This code is a simulation for the impedance controller

## Author: Avadesh Meduri
## Date: 1/03/2019


from dg_tools.utils import *
from dg_tools.leg_impedance_control.quad_leg_impedance_controller import (
    QuadrupedLegImpedanceController,
    QuadrupedComControl,
)
from dg_tools.sliders import Sliders

#########################################################################################


class ImpedanceController(object):
    def __init__(self, prefix):
        self.prefix = prefix
        self.sliders = Sliders(4, self.prefix, filter_size=100)
        self.sliders.set_scale_values([1.0, 1.0, 1.0, 1.0])

        ## setting desired position
        self.des_pos_ff = VectorSignal(
            [
                0.0,
                0.0,
                -0.25,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                -0.25,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                -0.25,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                -0.25,
                0.0,
                0.0,
                0.0,
            ]
        ).name_entity(prefix + "_pos_des")

        self.pos_slider_offset = VectorSignal(
            4 * [0.0, 0.0, 0.25, 0.0, 0.0, 0.0]
        )
        self.des_pos = (
            self.des_pos_ff + self.sliders.A * self.pos_slider_offset
        )

        self.des_vel = VectorSignal(
            [
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
            ]
        ).name_entity(prefix + "_vel_des")

        self.des_fff = VectorSignal(
            [
                0.0,
                0.0,
                (2.2 * 9.8) / 4.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                (2.2 * 9.8) / 4.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                (2.2 * 9.8) / 4.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                (2.2 * 9.8) / 4.0,
                0.0,
                0.0,
                0.0,
            ]
        ).name_entity(prefix + "_ff")

        self.kp = VectorSignal([50.0, 0.0, 50.0, 0.0, 0.0, 0.0]).name_entity(
            prefix + "_kp_split"
        )
        self.kd = VectorSignal([0.5, 0.0, 0.5, 0.0, 0.0, 0.0]).name_entity(
            prefix + "_kd_split"
        )
        self.kf = DoubleSignal(0.0)

    def plug(self, robot):
        self.sliders.plug_slider_signal(robot.device.slider_positions)

        self.quad_imp_ctrl = QuadrupedLegImpedanceController(robot)
        self.control_torques = self.quad_imp_ctrl.return_control_torques(
            self.kp, self.des_pos, self.kd, self.des_vel, self.kf, self.des_fff
        )

        plug(self.control_torques, robot.device.ctrl_joint_torques)


def get_controller():
    return ImpedanceController(prefix="solo8")


if "robot" in globals():
    ctrl = ImpedanceController("solo8")

    def go():
        ctrl.plug(robot)
