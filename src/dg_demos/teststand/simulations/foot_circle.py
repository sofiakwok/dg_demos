"""
@package dg_demos
@file teststand/simulations/foot_circle.py
@author Elham Daneshmand
@license License BSD-3-Clause
@copyright Copyright (c) 2019, New York University and Max Planck Gesellschaft.
@date 2019-02-06
@brief This code for creating a bounding motion for Solo
"""

from dg_blmc_robots.teststand.teststand_bullet import get_robot
from dg_demos.simulation_tools import write_current_control_graph

# Change this line to include your own controller
from dg_demos.teststand.controllers.foot_circle import get_controller


def simulate(with_gui=True):
    """
    This method is re-used in the unnittest so do not modify its name
    """
    #
    # Simu parameter
    #
    init_sliders_pose = [0.5, 0.5]
    simu_duration = 10000  # millisec
    simu_sampling_period = 1.0 / 60.0

    #
    # robot init
    #

    # Get the robot corresponding to the quadruped.
    robot = get_robot(
        use_fixed_base=True,
        init_sliders_pose=init_sliders_pose,
        with_gui=with_gui,
    )

    # Define the desired position.
    q = robot.config.q0.copy()
    dq = robot.config.v0.copy()

    # Update the initial state of the robot.
    robot.reset_state(q, dq)

    #
    # load controller
    #
    ctrl = get_controller()
    ctrl.plug(robot)

    #
    # Write the graph for debugging
    #
    write_current_control_graph(robot.config.robot_name, __file__)

    #
    # run simulation
    #
    robot.start_tracer()
    robot.run(simu_duration, simu_sampling_period)
    robot.stop_tracer()


if __name__ == "__main__":
    simulate()
