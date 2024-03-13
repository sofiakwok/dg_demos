"""
License BSD-3-Clause

Copyright (c) 2019, New York University and Max Planck Gesellschaft.

This code for creating a succession of smooth go_to motions for Solo
"""

import numpy as np
import dynamic_graph as dg
from dg_blmc_robots.solo.solo12_bullet import get_solo12_robot
from dg_demos.simulation_tools import write_current_control_graph

# Change this line to include your own controller
from dg_demos.solo12.controllers.impedance_controller import ImpedanceDemo


def simulate(with_gui=True):
    """
    This method is re-used in the unnittest so do not modify its name
    """
    #
    # Simu parameter
    #
    init_sliders_pose = [0.5, 0.5, 0.5, 0.5]
    simu_sampling_period = 1.0 / 1000.0
    simu_duration = 3.0 / simu_sampling_period  # nb iteration: milli-sec

    #
    # robot init
    #

    # Get the robot corresponding to the quadruped.
    robot = get_solo12_robot(
        use_fixed_base=False,
        record_video=False,
        init_sliders_pose=init_sliders_pose,
        hide_gui=(not with_gui),
    )

    # Define the desired position.
    q = robot.config.q0.copy()
    q[0] = 0.0
    q[1] = 0.0
    q[2] = 0.5
    dq = robot.config.v0.copy()

    # Update the initial state of the robot.
    robot.reset_state(q, dq)

    #
    # load controller
    #
    ctrl = ImpedanceDemo()
    ctrl.plug(robot)
    ctrl.go_imp(robot)

    #
    # Write the graph for debugging
    #
    write_current_control_graph(robot.config.robot_name, __file__)

    #
    # run simulation
    #
    robot.start_video_recording("/tmp/solo12_go_to.mp4")
    robot.start_tracer()

    # Run the simulation.
    robot.run(int(simu_duration))

    robot.stop_tracer()
    robot.stop_video_recording()
    print("Finish normally!")


if __name__ == "__main__":
    simulate()
