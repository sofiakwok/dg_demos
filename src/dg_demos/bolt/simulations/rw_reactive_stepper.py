"""
Simulation demo for bolt reactive stepper with reaction wheel.

License BSD-3-Clause
Copyright (c) 2021, New York University and Max Planck Gesellschaft.

Author: Julian Viereck, Elham Daneshmand
Date:   Feb 26, 2021
"""

import numpy as np
import pybullet as p

import dynamic_graph as dg
from dg_demos.bolt.controllers.rw_reactive_stepper import get_controller

# import the simulated robot
from bolt.dg_bolt_rw_bullet import get_bolt_robot, BoltRWConfig


def simulate(with_gui=True):
    #
    # setup and run simulation
    #
    robot = get_bolt_robot(use_fixed_base=True)
    p.resetDebugVisualizerCamera(1.3, 60, -35, (0.0, 0.0, 0.0))
    bolt_config = BoltRWConfig()

    # Update the initial state of the robot.
    q0 = bolt_config.q0.copy()
    q0[0] = 0.5
    # q0[1] = 0.0
    # q0[2] = 0.35487417
    # q0[6] = 1.0
    robot.reset_state(q0, bolt_config.v0)
    ctrl = get_controller()

    ctrl.plug(robot, *robot.base_signals())

    ctrl.trace()
    #robot.start_tracer()

    # robot.run(1000)

    #robot.run(100,0.01)
    ctrl.set_kf(5)
    ctrl.start()
    robot.run(3000, 0.01)
    # print("after start")
    # from dynamic_graph import writeGraph
    # writeGraph("/tmp/my_graph.dot")
    # robot.run(1000,0.01)
    print("Finished normally!")
    #robot.stop_tracer()


if __name__ == "__main__":
    simulate()
