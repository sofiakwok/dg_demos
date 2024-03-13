"""
Simulation demo for solo12 reactive stepper.

License BSD-3-Clause
Copyright (c) 2021, New York University and Max Planck Gesellschaft.
"""

import subprocess
import numpy as np
import pybullet as p
import pinocchio
import random
from datetime import datetime

from robot_properties_solo.config import Solo12Config
from solo.solo12_bullet import get_solo12_robot
from dg_demos.solo12.controllers.reactive_stepper_gamepad import (
    ReactiveStepperDemo,
)


def signed_random():
    random.seed(datetime.now())
    ret = random.random()
    if int(random.random() * 1000) % 2 == 0:
        ret *= -1.0
    return ret


def simulate(with_gui=True):
    # Start the gamepad controller
    bashCommand = "ros2 run joy joy_node"
    subprocess.Popen(bashCommand.split(), stdout=subprocess.PIPE)
    bashCommand = "ros2 run dg_demos joy_forwarder.py"
    subprocess.Popen(bashCommand.split(), stdout=subprocess.PIPE)

    # init simu
    robot = get_solo12_robot(
        hide_gui=not with_gui,
        init_sliders_pose=[0, 0.5, 0, 0],
    )
    p.resetDebugVisualizerCamera(1.2, 50, -35, (0.0, 0.0, 0.0))
    # p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
    q0 = Solo12Config.q0.copy()
    x = signed_random()
    y = signed_random()
    yaw = signed_random()
    print(x, y, yaw)

    q0[0] = x
    q0[1] = y
    q0[2] = 0.237
    q0[3:7] = pinocchio.Quaternion(
        pinocchio.rpy.rpyToMatrix(np.array([0, 0, yaw]))
    ).coeffs()
    robot.reset_state(q0, Solo12Config.v0)

    # Create the demo and initialize it.
    base_pos_sout, base_vel_sout = robot.base_signals()
    base_vel_world_sout = robot.base_vel_world_signal()
    ctrl = ReactiveStepperDemo()
    ctrl.plug(robot, base_pos_sout, base_vel_sout, base_vel_world_sout)
    ctrl.trace(robot)
    robot.start_tracer()
    robot.run(1000)

    # go slider.
    ctrl.go_pd_init()
    robot.run(3000)
    ctrl.go_pd()
    robot.run(5000000)

    # Init the demo
    ctrl.go_init()
    robot.run(3000)

    ctrl.go_stepper()
    robot.run(500)

    # Run the controller
    ctrl.start()  # starting the stepping.
    robot.run(5000)
    ctrl.stop()  # starting the stepping.
    robot.run(2000)
    ctrl.start()  # starting the stepping.
    robot.run(5000)
    ctrl.stop()  # starting the stepping.
    robot.run(2000)

    # set the velocity manually:
    # ctrl.use_user_velocity(0.0, 0.0, 0.5) # setting the velocity manually
    # robot.run(5000)

    robot.stop_tracer()

    # Stop the gamepad controller
    bashCommand = "killall joy_node"
    subprocess.Popen(bashCommand.split(), stdout=subprocess.PIPE)
    bashCommand = "killall joy_forwarder.py"
    subprocess.Popen(bashCommand.split(), stdout=subprocess.PIPE)


if __name__ == "__main__":
    simulate()
