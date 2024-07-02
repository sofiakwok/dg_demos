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
    time = 0
    sim_freq = 10000  # Hz
    ctrl_freq = 1000
    plan_freq = 1000 

    robot = get_bolt_robot(use_fixed_base=False)
    p.resetDebugVisualizerCamera(1.3, 60, -35, (0.0, 0.0, 0.0))
    p.setTimeStep(1.0 / sim_freq)
    p.setRealTimeSimulation(0)

    # Update the initial state of the robot.
    q0 = np.matrix(BoltRWConfig.initial_configuration).T
    qdot = np.matrix(BoltRWConfig.initial_velocity).T
    # q0[0] = -0.1
    # q0[1] = 0.0
    # q0[2] = 0.4
    # print(q0[2])
    # q0[6] = 1.0
    robot.reset_state(q0, qdot)
    ctrl = get_controller(is_real_robot=False)

    ctrl.plug(robot, *robot.base_signals())

    ctrl.trace()
    robot.start_tracer()

    # robot.run(1000)

    # robot.run(100,0.01)
    ctrl.set_kf(3)
    ctrl.start()

    robot.run(10000, 0.01)
    # print("after start")

    # robot.run(1000,0.01)
    print("Finished normally!")
    ctrl.stop()
    robot.stop_tracer()


if __name__ == "__main__":
    simulate()
