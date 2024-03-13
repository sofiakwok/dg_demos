# ReactiveStepper for solo12.
# Author: Julian Viereck
# Date : 28 January 2020
#
# Used to test the endeffector generation for solo12.

from dg_tools.utils import *
from dg_tools.traj_generators import mul_double_vec_2, scale_values
from dg_tools.leg_impedance_control.solo12_impedance_controller import (
    Solo12ComController,
    Solo12ImpedanceController,
)

from robot_properties_solo.config import Solo12Config
from dg_demos.solo12.controllers import pd_controller, centroidal_controller

from dynamic_graph_manager.dg_reactive_planners import (
    StepperHead,
    DcmVrpPlanner,
)
from dynamic_graph.sot.core.switch import SwitchVector

from dynamic_graph_manager.dg_tools import VectorIntegrator
import dynamic_graph.sot.dynamics_pinocchio as dp

from dg_tools.sliders import Sliders

#############################################################################


def switch_vector(sigs, entity_name=""):
    op = SwitchVector(entity_name)
    op.setSignalNumber(len(sigs))
    for i, sig in enumerate(sigs):
        dg.plug(sig, op.signal("sin" + str(i)))

    op.selection.value = 0
    return op.selection, op.sout


class Solo12EffectorMotion(object):
    def __init__(self, prefix):
        self.prefix = prefix

        # Uses a centroidal controller as part of the control structure.
        self.centroidal_ = centroidal_controller.Solo12ComWbc(
            self.prefix + "_Solo12Centroidal"
        )
        self.sh = StepperHead(self.prefix + "stepper_head")

        self.x0_ = VectorSignal(
            [
                0.195,
                0.147,
                0.015,
                0.195,
                -0.147,
                0.015,
                -0.195,
                0.147,
                0.015,
                -0.195,
                -0.147,
                0.015,
            ]
        )

        self.sliders = Sliders(4, self.prefix)
        self.sliders.set_scale_values([50, 10.0, 50.0, 0.0])

        self.eff_kp = vectorIdentity(
            self.sliders.A
            * VectorSignal(
                [1.0, 1.0, 1.0]
                + 9
                * [
                    0.0,
                ]
            )
            + VectorSignal(
                12
                * [
                    50.0,
                ]
            ),
            12,
            self.prefix + "_eff_kp",
        )
        self.eff_kd = vectorIdentity(
            self.sliders.B
            * VectorSignal(
                [1.0, 1.0, 1.0]
                + 9
                * [
                    0.0,
                ]
            )
            + VectorSignal(
                12
                * [
                    3.0,
                ]
            ),
            12,
            self.prefix + "_eff_kd",
        )

    def init_stepper(self, robot):
        # Connecting the dcm_planner to stepper_head.
        self.sh.duration_before_step_landing_sin.value = 0.2
        self.sh.next_step_location_sin.value = [0.0, 0.0, 0.0]

        # Simple trajectory generation.
        z_max = DoubleSignal(0.1)
        self.t = DoubleSignal(self.sh.time_from_last_step_touchdown_sout)
        self.t_end = DoubleSignal(0.2)

        self.switch_u_sel, self.u = switch_vector(
            [
                VectorSignal([0.05, 0.05, 0.1]),
                VectorSignal([-0.05, 0.05, 0.1]),
                VectorSignal([0.00, 0.00, 0.0]),
            ]
        )
        self.switch_u_old_sel, self.u_old = switch_vector(
            [
                VectorSignal([-0.05, -0.05, 0.0]),
                VectorSignal([+0.05, +0.05, 0.0]),
                VectorSignal([0.00, +0.00, 0.0]),
            ]
        )
        self.u = VectorSignal(self.u, 3)
        self.u_old = VectorSignal(self.u_old, 3)

        self.switch_u_sel.value = 2
        self.switch_u_old_sel.value = 2

        t_quot = self.t / (self.t_end)
        x_foot_des_air_xy = (self.u_old + t_quot * (self.u - self.u_old))[0:2]
        x_foot_des_air_z = self.u[2] * DoubleSignal(sin_doub(t_quot * np.pi))

        # Compute the velocity as well. HACK: cos(x) = sin(x + pi/2)
        # TODO: Add vec / double support.
        x_foot_vel_air_x = (self.u[0] - self.u_old[0]) / self.t_end
        x_foot_vel_air_y = (self.u[1] - self.u_old[1]) / self.t_end
        x_foot_vel_air_z = (self.u[2] * np.pi / self.t_end) * DoubleSignal(
            sin_doub(t_quot * np.pi + (np.pi / 2.0))
        )

        self.x_foot_des_air = x_foot_des_air_xy.concat(
            x_foot_des_air_z.vector()
        )
        self.x_des = (
            self.x_foot_des_air.concat(VectorSignal(9 * [0.0])) + self.x0_
        )
        self.v_des = (
            x_foot_vel_air_x.vector()
            .concat(x_foot_vel_air_y.vector())
            .concat(x_foot_vel_air_z.vector())
            .concat(
                VectorSignal(
                    9
                    * [
                        0.0,
                    ]
                )
            )
        )

        self.switch_cnt_array = SwitchVector("control_switch_cnt_array")
        self.switch_cnt_array.setSignalNumber(
            3
        )  # we want to switch between 2 signals
        dg.plug(VectorSignal([0, 1, 1, 1]), self.switch_cnt_array.sin0)
        dg.plug(VectorSignal([0, 1, 1, 1]), self.switch_cnt_array.sin1)
        dg.plug(
            VectorSignal([1.0, 1.0, 1.0, 1.0]), self.switch_cnt_array.sin2
        )  # Default
        self.switch_cnt_array.selection.value = 2

        self.cnt_array_stepper = self.switch_cnt_array.sout

    def init(self, robot, ViconClientEntity):
        self.robot = robot
        self.init_stepper(robot)

        # self.centroidal_.set_des_contact_plan(self.cnt_array_stepper)
        self.centroidal_.set_des_pos_eff(self.x_des)
        self.centroidal_.set_des_vel_eff(self.v_des)
        self.centroidal_.set_gains_eff(self.eff_kp, self.eff_kd)

        self.centroidal_.init(robot, ViconClientEntity)

    def start(self):
        self.tune_gains()

        dg.plug(self.sh.is_left_leg_in_contact_sout, self.switch_u_sel)
        dg.plug(self.sh.is_left_leg_in_contact_sout, self.switch_u_old_sel)
        dg.plug(
            self.sh.is_left_leg_in_contact_sout,
            self.switch_cnt_array.selection,
        )

    def tune_gains(self):
        self.sliders.plug_slider_signal(self.robot.device.slider_positions)

    def record_data(self):
        self.centroidal_.record_data()

    def plug(self):
        self.centroidal_.plug()

        # Plug a zero-signal for the sliders here. The gain tuning should
        # only start after callling "start".
        self.sliders.plug_slider_signal(
            VectorSignal(
                4
                * [
                    0.0,
                ]
            )
        )


def get_controller():
    return Solo12EffectorMotion(prefix="solo12_")


if ("robot" in globals()) or ("robot" in locals()):
    ctrl_pd = pd_controller.get_controller()

    from dynamic_graph_manager.vicon_sdk import ViconClientEntity

    ctrl = Solo12EffectorMotion("solo12")
    ctrl.init(robot, ViconClientEntity)

    def go_pd():
        ctrl_pd.plug(robot)

    def go_stepper():
        ctrl.plug()
