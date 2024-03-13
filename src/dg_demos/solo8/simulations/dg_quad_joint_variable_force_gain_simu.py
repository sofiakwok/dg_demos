# Simulation demo for dg_quad_joint_variable_gain controller.
# Author: Julian Viereck
# Date: March 04, 2020

import numpy as np
from pinocchio.utils import zero
from dg_blmc_robots.solo.solo_bullet import get_robot

from dg_demos.solo.controllers.dg_quad_joint_variable_gain_force import (
    get_controller,
)

if __name__ == "__main__":
    ctrl = get_controller()

    robot = get_robot(
        use_fixed_base=False,
        record_video=False,
        init_sliders_pose=[0.5, 0.5, 0.0, 0.0],
    )
    # Define the desired position.
    q = zero(robot.pin_robot.nq)
    dq = zero(robot.pin_robot.nv)

    q[0] = 0.2
    q[1] = 0.0
    q[2] = 0.22
    q[6] = 1.0
    q[7] = 0.82
    q[8] = -1.48
    q[9] = 0.82
    q[10] = -1.48
    q[11] = -0.82
    q[12] = 1.48
    q[13] = -0.82
    q[14] = 1.48

    # Update the initial state of the robot.
    robot.reset_state(q, dq)

    ctrl.plug(robot)

    robot.run(4000, 1.0 / 60.0)
