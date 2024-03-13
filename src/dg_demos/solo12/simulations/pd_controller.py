import numpy as np

# import the controller
from dg_demos.solo12.controllers.pd_controller import get_controller

# import the simulated robot
from robot_properties_solo.config import Solo12Config
from dg_blmc_robots.solo.solo12_bullet import get_solo12_robot

if __name__ == "__main__":
    #
    # setup and run simulation
    #
    robot = get_solo12_robot(init_sliders_pose=4 * [0.0])

    # Update the initial state of the robot.
    # robot.reset_state(
    #     np.matrix([0., 0., 0.5, 0., 0., 0., 1.] + 12 * [0.]).T,
    #     robot.config.v0)

    # Run for one step to sync the configuration with
    robot.run(1)

    # load controller
    ctrl = get_controller()
    ctrl.plug(robot)

    # run the Simulation
    robot.run(5000)
