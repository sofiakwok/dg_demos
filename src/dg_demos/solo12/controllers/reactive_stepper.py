"""This file is a demo for using the DG whole body controller.

License BSD-3-Clause
Copyright (c) 2021, New York University and Max Planck Gesellschaft.

Author: Julian Viereck
Date:   Feb 26, 2021
"""

import numpy as np

np.set_printoptions(suppress=True, precision=3)
import pinocchio as pin
import dynamic_graph as dg

from robot_properties_solo.config import Solo12Config
from mim_control.dynamic_graph.wbc_graph import WholeBodyController
from reactive_planners.dynamic_graph.quadruped_stepper import QuadrupedStepper

from dg_tools.sliders import Sliders
from dg_tools.utils import (
    constVector,
    stack_two_vectors,
    selec_vector,
    subtract_vec_vec,
    multiply_mat_vec,
    Stack_of_vector,
    zero_vec,
    mul_double_vec,
)
from dg_tools.dynamic_graph.dg_tools_entities import (
    CreateWorldFrame,
    PoseRPYToPoseQuaternion,
    PoseQuaternionToPoseRPY,
    RPYToRotationMatrix,
    VectorIntegrator,
)


class Solo12WBCStepper:
    def __init__(self, prefix, friction_coeff):
        pin_robot = Solo12Config.buildRobotWrapper()
        end_effector_names = Solo12Config.end_effector_names

        # sliders to tune p and d gains.
        self.sliders = Sliders(4, prefix + "_sliders")
        self.sliders.set_scale_values([0.0, 0.0, 2 * 25.0, 2.0 * 10.0])

        ###
        # Create the whole body controller.
        qp_penalty_weights = np.array([1e0, 1e0, 1e6, 1e6, 1e6, 1e6])
        self.wbc = wbc = WholeBodyController(
            prefix + "_wbc",
            pin_robot,
            end_effector_names,
            friction_coeff,
            qp_penalty_weights,
        )

        ###
        # Specify gains for the controller.

        # For the centroidal controllers com PD.
        wbc.kc_sin.value = np.array([0.0, 0.0, 200.0])
        wbc.dc_sin.value = np.array([15.0, 15.0, 15.0])
        wbc.kb_sin.value = np.array([10.0, 25.0, 25.0])
        wbc.db_sin.value = np.array([2.0, 10.0, 10.0])

        # Debug, plug som silders to the gains if needed.
        # For the centroidal controllers base orientation (roll, pitch, yaw) PD.
        # dg.plug(
        #     stack_two_vectors(
        #         self.sliders.C_vec,
        #         constVector(np.array([25.0, 25.0])),
        #         1,
        #         2
        #     ),
        #     wbc.kb_sin
        # )
        # dg.plug(
        #     stack_two_vectors(
        #         self.sliders.D_vec,
        #         constVector(np.array([10.0, 10.0])),
        #         1,
        #         2
        #     ),
        #     wbc.db_sin
        # )

        wbc.des_com_pos_sin.value = np.array([0.0, 0.0, 0.25])
        wbc.des_com_vel_sin.value = np.zeros(3)
        wbc.des_ori_vel_sin.value = np.zeros(3)

        # Make it possible to specify desired orientation as RPY.
        op = Stack_of_vector(prefix + "_stack_pose_rpy")
        op.selec1(0, 3)
        op.selec2(0, 3)
        op.sin1.value = np.array([0.0, 0.0, 0.0])

        op_rpy2quat = PoseRPYToPoseQuaternion("")
        dg.plug(op.sout, op_rpy2quat.sin)
        dg.plug(selec_vector(op_rpy2quat.sout, 3, 7), wbc.des_ori_pos_sin)

        # Expose the signals for easier access.
        self.des_ori_pos_rpy_sin = op.sin2
        self.des_ori_vel_sin = wbc.des_ori_vel_sin

        self.des_ori_pos_rpy_sin.value = np.array([0.0, 0.0, 0.0])

        wbc.cnt_array_sin.value = np.array([1.0, 1.0, 1.0, 1.0])

        # Impedance controllers.
        for i, imp in enumerate(wbc.imps):
            imp.gain_proportional_sin.value = np.array(
                [50.0, 50.0, 50.0, 0.0, 0.0, 0.0]
            )
            imp.gain_derivative_sin.value = np.array(
                [0.7, 0.7, 0.7, 0.0, 0.0, 0.0]
            )
            imp.gain_feed_forward_force_sin.value = 1.0

        wbc.w_com_ff_sin.value = np.array(
            [0.0, 0.0, 9.81 * 2.5, 0.0, 0.0, 0.0]
        )

        ###
        # Create the stepper.
        self.stepper = stepper = QuadrupedStepper(
            prefix + "_stepper", pin_robot, end_effector_names
        )

        ###
        # Connect the stepper with the wbc.
        dg.plug(stepper.stepper.contact_array_sout, wbc.cnt_array_sin)

        def plug_des_pos(stepper_pos, imp):
            dg.plug(
                stack_two_vectors(stepper_pos, zero_vec(4, ""), 3, 4),
                imp.desired_end_frame_placement_sin,
            )

        def plug_des_vel(stepper_pos, imp):
            dg.plug(
                stack_two_vectors(stepper_pos, zero_vec(3, ""), 3, 3),
                imp.desired_end_frame_velocity_sin,
            )

        plug_des_pos(
            stepper.stepper.front_left_foot_position_sout, wbc.imps[0]
        )
        plug_des_vel(
            stepper.stepper.front_left_foot_velocity_sout, wbc.imps[0]
        )

        plug_des_pos(
            stepper.stepper.front_right_foot_position_sout, wbc.imps[1]
        )
        plug_des_vel(
            stepper.stepper.front_right_foot_velocity_sout, wbc.imps[1]
        )

        plug_des_pos(stepper.stepper.hind_left_foot_position_sout, wbc.imps[2])
        plug_des_vel(stepper.stepper.hind_left_foot_velocity_sout, wbc.imps[2])

        plug_des_pos(
            stepper.stepper.hind_right_foot_position_sout, wbc.imps[3]
        )
        plug_des_vel(
            stepper.stepper.hind_right_foot_velocity_sout, wbc.imps[3]
        )

        self.initialize()

    def initialize(self):
        # PART 1: Positions
        # Because this controller is specific for solo12, we can hard
        # code the values here.
        self.stepper.stepper.initialize_placement(
            np.array([0.0, 0.0, 0.238, 0, 0, 0.0, 1.0]),
            np.array([0.195, 0.147, 0.015]),
            np.array([0.195, -0.147, 0.015]),
            np.array([-0.195, 0.147, 0.015]),
            np.array([-0.195, -0.147, 0.015]),
        )

        # PART 2: Parameters
        is_left_leg_in_contact = True
        l_min = -0.10
        l_max = 0.10
        w_min = -0.10
        w_max = 0.10
        t_min = 0.1
        t_max = 0.3
        l_p = 0.00  # Pelvis width
        com_height = 0.25
        weight = np.array(
            [1, 1, 5, 1000, 1000, 100000, 100000, 100000, 100000]
        )
        mid_air_foot_height = 0.10
        control_period = 0.001
        planner_loop = 0.010

        self.stepper.stepper.initialize_stepper(
            is_left_leg_in_contact,
            l_min,
            l_max,
            w_min,
            w_max,
            t_min,
            t_max,
            l_p,
            com_height,
            weight,
            mid_air_foot_height,
            control_period,
            planner_loop,
        )

        ###
        # Let the quadruped step in place for now.
        self.des_com_vel_sin = self.stepper.stepper.desired_com_velocity_sin
        self.des_com_vel_sin.value = np.array([0.0, 0.0, 0.0])

        self.set_steptime_nominal(0.2)

    def set_steptime_nominal(self, t_nom):
        self.stepper.stepper.set_steptime_nominal(t_nom)

    def set_polynomial_end_effector_trajectory(self):
        self.stepper.stepper.set_polynomial_end_effector_trajectory()

    def set_dynamical_end_effector_trajectory(self):
        self.stepper.stepper.set_dynamical_end_effector_trajectory()

    def start(self):
        self.stepper.start()

    def stop(self):
        self.stepper.stop()

    def plug(self, robot, base_position, base_velocity):
        self.robot = robot
        self.sliders.plug_slider_signal(robot.device.slider_positions)
        self.stepper.plug(robot, base_position, base_velocity)
        self.wbc.plug(robot, base_position, base_velocity)

    def plug_base_as_com(self, base_position, base_velocity_world):
        self.wbc.plug_base_as_com(base_position, base_velocity_world)
        self.wbc.des_com_pos_sin.value = np.array([0.0, 0.0, 0.27])

    def set_eff_pd(self, p, p_z, d):
        for i, imp in enumerate(self.wbc.imps):
            imp.gain_proportional_sin.value = np.array(
                [p, p, p_z, 0.0, 0.0, 0.0]
            )
            imp.gain_derivative_sin.value = np.array([d, d, d, 0.0, 0.0, 0.0])

    def trace(self):
        self.wbc.trace()

        self.robot.add_trace(
            self.stepper.stepper.name, "swing_foot_forces_sout"
        )

    def plug_swing_foot_forces(self):
        # Need to invert the frame for the swing foot forces.
        swing_foot_forces = mul_double_vec(
            -1, self.stepper.stepper.swing_foot_forces_sout
        )

        for i, feedforward in enumerate(self.wbc.imps_feedforward):
            dg.plug(
                selec_vector(swing_foot_forces, 6 * i, 6 * (i + 1)),
                feedforward,
            )


def get_controller(prefix="solo12_wbc_stepper"):
    return Solo12WBCStepper(prefix, 0.6)


if "robot" in globals():
    from dg_demos.solo12.controllers.pd_controller import (
        get_controller as get_pd_controller,
    )
    from dg_vicon_sdk.dynamic_graph.entities import ViconClientEntity

    # Init vicon.
    vicon = ViconClientEntity("vicon_entity")
    # vicon.connect_to_vicon("172.24.117.119:801")  # NYU MIM vicon.
    vicon.connect_to_vicon("10.32.27.53:801")  # MPI MIM vicon.
    vicon.add_object_to_track("solo12/solo12")

    # Create a PD controller to setup the robot at the beginning.
    pd_ctrl = get_pd_controller()

    # Setup the main controller.
    ctrl = Solo12WBCStepper("solo12_wbc_stepper", 0.6)

    # Zero the initial position from the vicon signal.
    base_posture_sin = vicon.signal("solo12_position")

    op = CreateWorldFrame("wf")
    dg.plug(base_posture_sin, op.frame_sin)
    op.set_which_dofs(np.array([1.0, 1.0, 0.0, 0.0, 0.0, 0.0]))

    base_posture_local_sin = stack_two_vectors(
        selec_vector(
            subtract_vec_vec(base_posture_sin, op.world_frame_sout), 0, 3
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

    # Set desired base rotation and velocity.
    des_yaw = 0.0
    ctrl.des_ori_pos_rpy_sin.value = np.array([0.0, 0.0, des_yaw])
    ctrl.des_com_vel_sin.value = np.array([0.0, 0.0, 0.0])

    # Setup the gamepad subscriber.
    robot.ros.ros_subscribe.add("vector", "gamepad_axes", "/gamepad_axes")
    gamepad_axes = robot.ros.ros_subscribe.signal("gamepad_axes")
    gamepad_axes.value = np.zeros(8)  # Initital value

    def go_gamepad():
        # Yaw control via the left and right trigger on the gamepad.
        yaw_integrator = VectorIntegrator("yaw_integrator")
        dg.plug(
            mul_double_vec(
                -0.75,
                subtract_vec_vec(
                    selec_vector(gamepad_axes, 7, 8),
                    selec_vector(gamepad_axes, 6, 7),
                ),
            ),
            yaw_integrator.sin,
        )

        dg.plug(
            stack_two_vectors(zero_vec(2), yaw_integrator.sout, 2, 1),
            ctrl.des_ori_pos_rpy_sin,
        )

    def go_gamepad_wbc_vel():
        """Control the velocity via the whole body controller. """
        op_quat2rpy = PoseQuaternionToPoseRPY("")
        dg.plug(base_posture_local_sin, op_quat2rpy.sin)

        op_rpy2matrix = RPYToRotationMatrix("")
        dg.plug(
            stack_two_vectors(
                zero_vec(2, ""), selec_vector(op_quat2rpy.sout, 5, 6), 2, 1
            ),
            op_rpy2matrix.sin,
        )

        vel_gain = 0.70
        dg.plug(
            multiply_mat_vec(
                op_rpy2matrix.sout,
                stack_two_vectors(
                    # Swap x and y axis from the gamepad.
                    # For the robot, x points forward while it's y on the joystick.
                    stack_two_vectors(
                        mul_double_vec(
                            vel_gain, selec_vector(gamepad_axes, 1, 2)
                        ),
                        mul_double_vec(
                            -vel_gain, selec_vector(gamepad_axes, 0, 1)
                        ),
                        1,
                        1,
                    ),
                    zero_vec(1, ""),
                    2,
                    1,
                ),
            ),
            ctrl.wbc.des_com_vel_sin,
        )

        dg.plug(
            stack_two_vectors(
                # Swap x and y axis from the gamepad.
                # For the robot, x points forward while it's y on the joystick.
                stack_two_vectors(
                    mul_double_vec(vel_gain, selec_vector(gamepad_axes, 1, 2)),
                    mul_double_vec(
                        -vel_gain, selec_vector(gamepad_axes, 0, 1)
                    ),
                    1,
                    1,
                ),
                zero_vec(1, ""),
                2,
                1,
            ),
            ctrl.des_com_vel_sin
            # ctrl.wbc.des_com_vel_sin
        )
        # ctrl.wbc.des_com_vel_sin.value = np.zeros(3)
        # ctrl.des_com_vel_sin.value = np.zeros(3)

    def go_pd():
        pd_ctrl.plug_to_robot(robot)

    def go_poly():
        ctrl.set_polynomial_end_effector_trajectory()

    def go_swing_foot_forces():
        ctrl.plug_swing_foot_forces()

    def go_stepper():
        op.update()
        ctrl.plug(robot, base_posture_local_sin, base_velocity_sin)

        # Use base as com position gives more stable result.
        ctrl.plug_base_as_com(
            base_posture_local_sin, vicon.signal("solo12_velocity_world")
        )
        ctrl.trace()

    # Start the gamepad by default.
    go_gamepad()
    go_gamepad_wbc_vel()
