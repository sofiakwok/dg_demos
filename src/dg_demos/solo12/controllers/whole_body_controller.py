"""This file is a demo for using the DG whole body controller.

License BSD-3-Clause
Copyright (c) 2021, New York University and Max Planck Gesellschaft.

Author: Julian Viereck
Date:   Feb 16, 2021
"""

import numpy as np

np.set_printoptions(suppress=True, precision=3)

import dynamic_graph as dg

import pinocchio as pin

from robot_properties_solo.config import Solo12Config

from dg_tools.utils import (
    stack_two_vectors,
    selec_vector,
    subtract_vec_vec,
    add_vec_vec,
    zero_vec,
    mul_double_vec,
    constVector,
    component_of_vector,
)
from dg_tools.dynamic_graph.dg_tools_entities import (
    CreateWorldFrame,
    PoseRPYToPoseQuaternion,
)

from mim_control.dynamic_graph.wbc_graph import WholeBodyController


def get_controller():
    pin_robot = Solo12Config.buildRobotWrapper()

    qp_penalty_weights = np.array([5e5, 5e5, 5e5, 1e6, 1e6, 1e6])

    ###
    # Create the whole body controller.
    wbc = WholeBodyController(
        "wbc",
        pin_robot,
        Solo12Config.end_effector_names,
        0.6,
        qp_penalty_weights,
    )

    ###
    # Specify gains for the controller.
    x_des = np.array(
        [
            0.195,
            0.147,
            0.015,
            0.195,
            -0.147,
            0.015,
            -0.195,
            0.147,
            0.015,
            -0.195,
            -0.147,
            0.015,
        ]
    ).reshape(4, 3)

    # For the centroidal controllers.
    wbc.kc_sin.value = np.array([100.0, 100.0, 100.0])
    wbc.dc_sin.value = np.array([10.0, 10.0, 10.0])
    wbc.kb_sin.value = np.array([25.0, 25.0, 25.0])
    wbc.db_sin.value = np.array([10.0, 10.0, 10.0])

    wbc.des_com_pos_sin.value = np.array([0.0, 0.0, 0.20])
    wbc.des_com_vel_sin.value = np.zeros(3)
    wbc.des_ori_pos_sin.value = np.array([0.0, 0.0, 0.0, 1.0])
    wbc.des_ori_vel_sin.value = np.zeros(3)

    wbc.cnt_array_sin.value = np.array([1.0, 1.0, 1.0, 1.0])

    # Impedance controllers.
    for i, imp in enumerate(wbc.imps):
        imp.gain_proportional_sin.value = np.array(
            [50.0, 50.0, 50.0, 0.0, 0.0, 0.0]
        )
        imp.gain_derivative_sin.value = np.array(
            [0.7, 0.7, 0.7, 0.0, 0.0, 0.0]
        )
        imp.desired_end_frame_placement_sin.value = np.hstack(
            [x_des[i], np.zeros(4)]
        )
        imp.desired_end_frame_velocity_sin.value = np.zeros(6)
        imp.gain_feed_forward_force_sin.value = 1.0

    wbc.w_com_ff_sin.value = np.array([0.0, 0.0, 9.81 * 2.5, 0.0, 0.0, 0.0])

    return wbc


if "robot" in globals():
    from dg_demos.solo12.controllers.pd_controller import (
        get_controller as get_pd_controller,
    )

    from dg_vicon_sdk.dynamic_graph.entities import ViconClientEntity

    # Init vicon.
    vicon = ViconClientEntity("vicon_entity")
    vicon.connect_to_vicon("172.24.117.119:801")  # NYU MIM vicon.
    vicon.add_object_to_track("solo12/solo12")

    robot.add_ros_and_trace("vicon_entity", "solo12_position")
    robot.add_ros_and_trace("vicon_entity", "solo12_velocity_body")
    robot.add_ros_and_trace("vicon_entity", "solo12_velocity_world")

    # Create a PD controller to setup the robot at the beginning.
    pd_ctrl = get_pd_controller()

    # Create the main whole body controller.
    ctrl = get_controller()

    # Zero the initial position from the vicon signal.
    base_posture_sin = vicon.signal("solo12_position")

    wf = CreateWorldFrame("wf")
    dg.plug(base_posture_sin, wf.frame_sin)
    wf.set_which_dofs(np.array([1.0, 1.0, 0.0, 0.0, 0.0, 0.0]))

    base_posture_local_sin = stack_two_vectors(
        selec_vector(
            subtract_vec_vec(base_posture_sin, wf.world_frame_sout), 0, 3
        ),
        selec_vector(base_posture_sin, 3, 7),
        3,
        4,
    )

    # Create the base velocity using the IMU.
    base_velocity_sin = stack_two_vectors(
        selec_vector(vicon.signal("solo12_velocity_body"), 0, 3),
        robot.device.imu_gyroscope,
        3,
        3,
    )

    # Setup the gamepad subscriber.
    robot.ros.ros_subscribe.add("vector", "gamepad_axes", "/gamepad_axes")
    gamepad_axes = robot.ros.ros_subscribe.signal("gamepad_axes")
    gamepad_axes.value = np.zeros(8)  # Initital value

    def go_pd():
        pd_ctrl.plug(robot)

    def go_wbc():
        wf.update()
        ctrl.plug(robot, base_posture_local_sin, base_velocity_sin)

    def go_gamepad():
        # Move the robot a bit more up.
        default_pos = constVector(np.array([0.0, 0.0, 0.225]))

        # Right trigger controlls the height.
        dg.plug(
            add_vec_vec(
                default_pos,
                mul_double_vec(
                    component_of_vector(gamepad_axes, 7, "comp_right_trigger"),
                    constVector(np.array([0.0, 0.0, -0.08])),
                ),
            ),
            ctrl.des_com_pos_sin,
        )

        # Right stick controlls the rp-angle.
        op_rpy2quat = PoseRPYToPoseQuaternion("")
        dg.plug(
            stack_two_vectors(
                zero_vec(3),
                stack_two_vectors(
                    stack_two_vectors(
                        mul_double_vec(0.25, selec_vector(gamepad_axes, 2, 3)),
                        mul_double_vec(0.25, selec_vector(gamepad_axes, 3, 4)),
                        1,
                        1,
                    ),
                    zero_vec(1),
                    2,
                    1,
                ),
                3,
                3,
            ),
            op_rpy2quat.sin,
        )
        dg.plug(selec_vector(op_rpy2quat.sout, 3, 7), ctrl.des_ori_pos_sin)
