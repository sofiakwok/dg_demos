"""
License BSD-3-Clause

Copyright (c) 2019, New York University and Max Planck Gesellschaft.

This code for creating a succession of smooth go_to motions for Solo
"""

from time import sleep
import os.path
import rospkg
import pinocchio
from dg_blmc_robots.solo.solo12_bullet import get_solo12_robot
from dg_demos.simulation_tools import write_current_control_graph

# Change this line to include your own controller
from dg_demos.solo12.controllers.go_to import get_controller


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
        use_fixed_base=True,
        record_video=False,
        init_sliders_pose=init_sliders_pose,
        hide_gui=(not with_gui),
    )

    # Define the desired position.
    q = robot.config.q0.copy()
    q[2] = 0.5
    dq = robot.config.v0.copy()

    # Define a random position
    scale = 0.1
    q[7:] = pinocchio.randomConfiguration(robot.config.robot_model)[7:] * scale

    # Update the initial state of the robot.
    robot.reset_state(q, dq)

    #
    # load controller
    #
    ctrl = get_controller()
    ctrl.set_pd_gains(Kp=7.0, Kd=0.1)
    ctrl.record_data(robot)
    ctrl.plug(
        robot.device.joint_positions,
        robot.device.joint_velocities,
        robot.device.ctrl_joint_torques,
    )

    #
    # Write the graph for debugging
    #
    write_current_control_graph(robot.config.robot_name, __file__)

    #
    # run simulation
    #
    robot.start_video_recording("/tmp/solo12_go_to.mp4")

    # Move smoothly into different positions
    for _ in range(3):

        print("Define a random position.")

        q_goal = (
            pinocchio.randomConfiguration(robot.config.robot_model)[7:] * scale
        )
        print(q_goal)

        robot.start_tracer()
        ctrl.go_to(q_goal, int((simu_duration)))
        robot.run(int(simu_duration))
        robot.stop_tracer()

    robot.stop_video_recording()
    print("Finish normally!")


if __name__ == "__main__":
    simulate()
