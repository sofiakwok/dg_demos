"""
License BSD-3-Clause

Copyright (c) 2019, New York University and Max Planck Gesellschaft.

This code for creating a succession of smooth go_to motions for Solo
"""

import numpy as np
import pinocchio
from time import sleep
from dg_blmc_robots.solo.solo12_bullet import get_solo12_robot
from dg_demos.simulation_tools import write_current_control_graph
from dg_demos.solo12.controllers.go_to import Solo12GoTo

# Change this line to include your own controller
from dg_demos.solo12.controllers.full_state_feedback_controller import (
    get_controller,
)


def simulate(with_gui=True):
    """
    This method is re-used in the unittest so do not modify its name
    """
    #
    # Simu parameter
    #
    init_sliders_pose = [0.5, 0.5, 0.5, 0.5]
    simu_sampling_period = 1.0 / 1000.0
    fall_simu_duration = int(
        1.0 / simu_sampling_period
    )  # nb iteration: milli-sec
    goto_simu_duration = int(
        2.5 / simu_sampling_period
    )  # nb iteration: milli-sec
    simu_duration = int(5.0 / simu_sampling_period)  # nb iteration: milli-sec
    simu_duration_risk = int(
        10.0 / simu_sampling_period
    )  # nb iteration: milli-sec

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
    random_yaw = np.random.rand(3, 1)
    random_yaw[0] = 0
    random_yaw[1] = 0
    random_yaw[2] *= 10
    q[:7] = pinocchio.SE3ToXYZQUAT(
        pinocchio.SE3(
            pinocchio.rpy.rpyToMatrix(random_yaw), np.random.rand(3, 1)
        )
    )
    q[2] = 0.5
    dq = robot.config.v0.copy()

    # Update the initial state of the robot.
    robot.reset_state(q, dq)

    #
    # load controller
    #
    traj_path = "/home/mnaveau/Documents/bilal_trajectories"
    # traj_path = "/home/bhammoud/Downloads"

    print("loading ", traj_path)

    ctrl = get_controller()
    ctrl.record_data(robot)
    ctrl.initialize(traj_path)
    ctrl.plug(
        robot.signal_base_pos_.sout,
        robot.device.joint_positions,
        robot.signal_base_vel_world_.sout,
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
    robot.start_video_recording()
    robot.start_tracer()

    print("fall down")
    robot.run(fall_simu_duration, simu_sampling_period)

    print("go init ddp")
    ctrl.go_init_ddp_plan()
    robot.run(goto_simu_duration, simu_sampling_period)

    print("run ddp plan")
    ctrl.run_ddp_plan()
    robot.run(simu_duration, simu_sampling_period)

    print("go idle")
    ctrl.go_idle()
    robot.reset_state(q, dq)
    robot.run(fall_simu_duration, simu_sampling_period)

    print("go init risk")
    ctrl.go_init_risk_plan()
    robot.run(goto_simu_duration, simu_sampling_period)

    print("run risk plan")
    ctrl.run_risk_plan()
    robot.run(simu_duration_risk, simu_sampling_period)

    robot.stop_tracer()
    robot.stop_video_recording()
    print("Finish normally!")


if __name__ == "__main__":
    simulate()
