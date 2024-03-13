#
# Author: Julian Viereck
# Date: Demo for using centroidal controller class

import dynamic_graph as dg

from dg_blmc_robots.vicon_client_bullet import ViconClientEntityBullet
from dg_blmc_robots.solo.solo_bullet import get_robot

# Change this line to include your own controller
from dg_demos.solo.controllers.dg_quad_centroidal import Solo8ComWbc

if __name__ == "__main__":
    #
    # Simu parameter
    #
    init_sliders_pose = [1.0, 0.2, 0.2, 0.2]
    simu_duration = 10000  # millisec
    simu_sampling_period = 1.0 / 60.0

    #
    # robot init
    #

    # Get the robot corresponding to the quadruped.
    robot = get_robot(init_sliders_pose=init_sliders_pose, with_gui=True)

    # Define the desired position.
    q = robot.config.q0.copy()
    dq = robot.config.v0.copy()

    # Update the initial state of the robot.
    robot.reset_state(q, dq)

    #
    # load controller
    #
    # ctrl = get_controller()
    # ctrl.plug(robot)
    ctrl = Solo8ComWbc()
    ctrl.plug(robot, ViconClientEntityBullet)
    ctrl.quad_com_ctrl.set_bias()

    #
    # run simulation
    #
    robot.run(simu_duration, simu_sampling_period)
