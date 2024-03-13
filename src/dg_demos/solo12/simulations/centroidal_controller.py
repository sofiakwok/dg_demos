# Simulation demo for solo12 centroidal controller.
# Author: Julian Viereck
# Date : 16 January 2020

import numpy as np
from robot_properties_solo.config import Solo12Config
from dg_blmc_robots.solo.solo12_bullet import get_solo12_robot

from dg_blmc_robots.vicon_client_bullet import ViconClientEntityBullet

from dg_demos.solo12.controllers.centroidal_controller import get_controller

if __name__ == "__main__":
    ctrl = get_controller()

    robot = get_solo12_robot()
    ctrl.init(robot, ViconClientEntityBullet)
    ctrl.plug()
    robot.q0[0] = 0.0
    robot.reset_state(robot.q0, robot.dq0)

    # Use the current position to zero the robot position.
    ctrl.kf_eff.value = 0.1

    robot.run(10000)
