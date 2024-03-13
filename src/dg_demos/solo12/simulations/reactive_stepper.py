"""
Simulation demo for solo12 reactive stepper.

License BSD-3-Clause
Copyright (c) 2021, New York University and Max Planck Gesellschaft.

Author: Julian Viereck
Date : 02 March, 2021
"""

import numpy as np
import pybullet as p

import dynamic_graph as dg

from robot_properties_solo.config import Solo12Config
from solo.solo12_bullet import get_solo12_robot

from dg_demos.solo12.controllers.reactive_stepper import get_controller


def simulate(with_gui=True):

    robot = get_solo12_robot(hide_gui=not with_gui)

    p.resetDebugVisualizerCamera(1.2, 50, -35, (0.0, 0.0, 0.0))
    p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)

    q0 = Solo12Config.q0.copy()
    q0[0] = 0.0
    q0[1] = 0.0
    q0[2] = 0.237

    robot.reset_state(q0, Solo12Config.v0)

    ctrl = get_controller()

    ctrl.plug(robot, *robot.base_signals())

    print("Balance for 1s")

    robot.run(3000)

    print("start stepping")
    ctrl.start()
    robot.run(1000)

    print("Move forward.")
    ctrl.set_velocity([1.0, 0.0, 0.0])
    robot.run(2000)

    print("Move sideway.")
    ctrl.set_velocity([0.0, 1.0, 0.0])
    robot.run(2000)

    print("Turn around.")
    ctrl.set_velocity([0.0, 0.0, 0.1])
    robot.run(2000)

    print("Finish normally!")


if __name__ == "__main__":
    simulate()
