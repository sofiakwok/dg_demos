"""
@package dg_demos
@file teststand/simulations/jump.py
@author Elham Daneshmand
@license License BSD-3-Clause
@copyright Copyright (c) 2019, New York University and Max Planck Gesellschaft.
@date 2019-02-06
@brief This code for creating a bounding motion for Solo
"""

import os.path
import rospkg
from dg_blmc_robots.solo.solo12_bullet import get_solo12_robot
from dg_demos.simulation_tools import write_current_control_graph

# Change this line to include your own controller
from dg_demos.solo12.controllers.trajectory_replay import get_controller


def simulate(with_gui=True):
    """
    This method is re-used in the unnittest so do not modify its name
    """
    #
    # Simu parameter
    #
    init_sliders_pose = [0.5, 0.5, 0.5, 0.5]
    simu_sampling_period = 1.0 / 1000.0
    simu_duration = 1.5 * 60.0 / simu_sampling_period
    print("simu_duration = ", simu_duration)

    #
    # robot init
    #

    # Get the robot corresponding to the quadruped.
    robot = get_solo12_robot(
        use_fixed_base=False,
        record_video=True,
        init_sliders_pose=init_sliders_pose,
        hide_gui=(not with_gui),
    )

    # Define the desired position.
    q = robot.config.q0.copy()
    dq = robot.config.v0.copy()
    # q[0] = 0.0
    # q[1] = 0.0
    q[2] = 0.25

    # Update the initial state of the robot.
    robot.reset_state(q, dq)

    #
    # load controller
    #
    ctrl = get_controller()
    ctrl.set_gains(Kp=5.0, Kd=0.05, use_feed_forward_torques=False)
    ctrl.plug(
        robot.device.joint_positions,
        robot.device.joint_velocities,
        robot.device.ctrl_joint_torques,
    )

    # load the trajectories
    file_paths = os.path.join(
        rospkg.RosPack().get_path("dg_demos"),
        "python",
        "dg_demos",
        "solo12",
        "simulations",
    )
    file_paths = os.path.join(file_paths, "com_oscillation")
    # file_paths = os.path.join(file_paths, "stamping")
    ctrl.set_desired_trajectory_files(
        os.path.join(file_paths, "joint_positions.dat"),
        os.path.join(file_paths, "joint_velocities.dat"),
        os.path.join(file_paths, "joint_torques.dat"),
    )

    # Record the data
    ctrl.record_data(robot)

    #
    # Write the graph for debugging
    #
    write_current_control_graph(robot.config.robot_name, __file__)

    #
    # Reset the pose of the robot to the init pose.
    #
    q[7:] = ctrl.init_position
    robot.reset_state(q, dq)

    #
    # run simulation
    #
    # robot.start_video_recording()
    robot.start_tracer()

    ctrl.go_zero()
    ctrl.run_trajectory()
    robot.run(int(simu_duration), simu_sampling_period)

    robot.stop_tracer()
    # robot.stop_video_recording()


if __name__ == "__main__":
    simulate()
