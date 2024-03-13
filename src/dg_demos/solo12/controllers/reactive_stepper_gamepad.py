"""This file is a demo for using the DG whole body controller.

License BSD-3-Clause
Copyright (c) 2021, New York University and Max Planck Gesellschaft.

Author: Julian Viereck
Date:   Feb 26, 2021
"""

import time
from dg_demos.solo12.controllers.go_to import (
    get_controller as get_solo12_goto_ctrl,
)
from dg_demos.solo12.controllers.pd_controller import (
    get_controller as get_solo12_pd_ctrl,
)
from reactive_planners.dynamic_graph.quadruped_stepper import QuadrupedStepper
from mim_control.dynamic_graph.wbc_graph import WholeBodyController
from robot_properties_solo.config import Solo12Config
from dynamic_graph.sot.core.switch import SwitchVector
from dg_tools.dynamic_graph.dg_tools_entities import (
    CreateWorldFrame,
    PoseQuaternionToPoseRPY,
    PoseRPYToPoseQuaternion,
    RPYToRotationMatrix,
    VectorIntegrator,
    MultiplyInv_poseQuaternion_vector,
)
from dg_tools.utils import (
    stack_two_vectors,
    selec_vector,
    zero_vec,
    mul_double_vec,
    constVector,
)
from dynamic_graph.sot.core.math_small_entities import (
    Stack_of_vector,
    Selec_of_vector,
    Substract_of_vector,
    Multiply_double_vector,
    Multiply_matrix_vector,
    Add_of_vector,
)
from dynamic_graph.sot.core.filter_differentiator import FilterDifferentiator
import dynamic_graph as dg
import numpy as np

np.set_printoptions(suppress=True, precision=3)


class GamepadParser(object):
    def __init__(self):
        self.prefix = "GamepadParser_"

        # Max velocity.
        self.max_xy_velocity = 0.5  # 0.7
        self.max_yaw_velocity = 0.4  # 0.75

        # slice the input vector from the gamepad reader.
        self._s_left_stick_x = slice(0, 1)
        self._s_left_stick_y = slice(1, 2)
        self._s_right_stick_x = slice(2, 3)
        self._s_directional_pad = slice(4, 6)
        self._s_left_trigger = slice(6, 7)
        self._s_right_trigger = slice(7, 8)

        #
        # Filtering the input vector
        #
        # Define the filter
        self.numerator = np.array(
            [
                8.57655707e-06,
                5.14593424e-05,
                1.28648356e-04,
                1.71531141e-04,
                1.28648356e-04,
                5.14593424e-05,
                8.57655707e-06,
            ]
        )
        self.denominator = np.array(
            [
                1.0,
                -4.7871355,
                9.64951773,
                -10.46907889,
                6.44111188,
                -2.12903875,
                0.29517243,
            ]
        )
        self._gamepad_filtered = FilterDifferentiator(
            self.prefix + "gamepad_filtered"
        )
        self._gamepad_filtered.init(
            0.001,
            8,
            self.numerator,
            self.denominator,
        )

        #
        # Desired Yaw position and velocity
        #

        # with triggers
        self._left_trigger = Selec_of_vector(self.prefix + "left_trigger")
        self._left_trigger.selec(
            self._s_left_trigger.start, self._s_left_trigger.stop
        )
        dg.plug(self._gamepad_filtered.x_filtered, self._left_trigger.sin)
        self._right_trigger = Selec_of_vector(self.prefix + "right_trigger")
        self._right_trigger.selec(
            self._s_right_trigger.start, self._s_right_trigger.stop
        )
        dg.plug(self._gamepad_filtered.x_filtered, self._right_trigger.sin)
        # Left - Right trigger
        self._left_minus_right_trigger = Substract_of_vector(
            self.prefix + "left_minus_right_trigger"
        )
        dg.plug(self._left_trigger.sout, self._left_minus_right_trigger.sin1)
        dg.plug(self._right_trigger.sout, self._left_minus_right_trigger.sin2)

        # with right joystick
        self._right_stick_x = Selec_of_vector(self.prefix + "right_trigger")
        self._right_trigger.selec(
            self._s_right_stick_x.start, self._s_right_stick_x.stop
        )
        dg.plug(self._gamepad_filtered.x_filtered, self._right_stick_x.sin)

        # Multiply the difference between the joysticks
        self._yaw_angular_velocity = Multiply_double_vector(
            self.prefix + "yaw_angular_velocity"
        )
        self._yaw_angular_velocity.sin1.value = -self.max_yaw_velocity
        # dg.plug( # use the left/right trigger for the yaw control
        #     self._left_minus_right_trigger.sout,
        #     self._yaw_angular_velocity.sin2,
        # )
        dg.plug(  # use the right joystick trigger for the yaw control
            self._right_trigger.sout,
            self._yaw_angular_velocity.sin2,
        )

        #
        # Linear velocity input
        #

        # Get left stick X and Y values.
        self._left_stick_x = Selec_of_vector(self.prefix + "left_stick_x")
        self._left_stick_x.selec(
            self._s_left_stick_x.start, self._s_left_stick_x.stop
        )
        dg.plug(self._gamepad_filtered.x_filtered, self._left_stick_x.sin)
        self._left_stick_y = Selec_of_vector(self.prefix + "left_stick_y")
        self._left_stick_y.selec(
            self._s_left_stick_y.start, self._s_left_stick_y.stop
        )
        dg.plug(self._gamepad_filtered.x_filtered, self._left_stick_y.sin)

        # Scale them.
        self._forward_velocity = Multiply_double_vector(
            self.prefix + "forward_velocity"
        )
        self._forward_velocity.sin1.value = self.max_xy_velocity
        dg.plug(
            self._left_stick_y.sout,  # forward is 'y' from the gamepad.
            self._forward_velocity.sin2,
        )
        self._sideway_velocity = Multiply_double_vector(
            self.prefix + "sideway_velocity"
        )
        self._sideway_velocity.sin1.value = -self.max_xy_velocity
        dg.plug(
            self._left_stick_x.sout,  # sideway is 'x' from the gamepad.
            self._sideway_velocity.sin2,
        )

        # outputs
        self.forward_velocity_sout = self._forward_velocity.sout
        self.sideway_velocity_sout = self._sideway_velocity.sout
        self.yaw_angular_velocity_sout = self._yaw_angular_velocity.sout

    def plug(self, base_position_sout, gamepad_vector_sout):
        """Plug the base pose and the gamepad to the parser.

        inputs:
        - self._gamepad_filtered
        outputs:
        - self._yaw_angular_velocity.sout
        - self.yaw_angle_sout
        - self.rpy_output.sout
        - self.xyz_velocity.sout
        - self._angular_velocity
        - self.world_xyz_velocity.sout
        - self.quat_output.sout

        Args:
            base_position (vector_sout): Base (XYZ Quaternion) in the World.
            gamepad_vector ([type]): Inputs from the gamepad vector.
        """
        # Plug the gamepad vector.
        dg.plug(gamepad_vector_sout, self._gamepad_filtered.x)

    def trace(self, robot):
        robot.add_trace(self._forward_velocity.name, "sout")
        robot.add_trace(self._sideway_velocity.name, "sout")
        robot.add_trace(self._yaw_angular_velocity.name, "sout")
        robot.add_trace(self._gamepad_filtered.name, "x")
        robot.add_trace(self._gamepad_filtered.name, "x_filtered")


class VelocityFromLocalToWorldFrame(object):
    def __init__(self):
        self.prefix = "VelocityFromLocalToWorldFrame_"
        #
        # Yaw control via the left and right trigger on the gamepad.
        #
        self._des_yaw = VectorIntegrator(self.prefix + "des_yaw")
        # Stack up a xyz_quat form with the des yaw
        self._des_yaw_vec = Stack_of_vector(self.prefix + "des_yaw_vec")
        self._des_yaw_vec.selec1(0, 5)
        self._des_yaw_vec.selec2(0, 1)
        self._des_yaw_vec.sin1.value = np.zeros(5)
        dg.plug(self._des_yaw.sout, self._des_yaw_vec.sin2)
        # Quaternion desired trajectory
        self._pose_rpy_to_pose_quat = PoseRPYToPoseQuaternion(
            self.prefix + "pose_rpy_to_pose_quat"
        )
        dg.plug(self._des_yaw_vec.sout, self._pose_rpy_to_pose_quat.sin)
        # Select the quaternion from above
        self._des_ori_pos = Selec_of_vector(self.prefix + "des_ori_pos_sout")
        self._des_ori_pos.selec(3, 7)
        dg.plug(self._pose_rpy_to_pose_quat.sout, self._des_ori_pos.sin)

        # So3 angular velocity output
        self._angular_velocity = Stack_of_vector(
            self.prefix + "angular_velocity"
        )
        self._angular_velocity.selec1(0, 2)
        self._angular_velocity.selec2(0, 1)
        self._angular_velocity.sin1.value = np.zeros(2)

        #
        # X,Y Velocity in global frame.
        #
        # Transform the base pose/quat into pose/rpy.
        self._base_pose_rpy = PoseQuaternionToPoseRPY(
            self.prefix + "base_pose_rpy"
        )
        # Get the yaw individually.
        self._base_yaw = Selec_of_vector(self.prefix + "base_yaw")
        self._base_yaw.selec(5, 6)
        dg.plug(self._base_pose_rpy.sout, self._base_yaw.sin)
        # Fill a rpy vector with the above yaw
        self._base_yaw_vec = Stack_of_vector(self.prefix + "base_yaw_vec")
        self._base_yaw_vec.selec1(0, 2)
        self._base_yaw_vec.selec2(0, 1)
        self._base_yaw_vec.sin1.value = np.array([0.0, 0.0])
        dg.plug(self._base_yaw.sout, self._base_yaw_vec.sin2)
        # Construct a yaw rotation matrix.
        self._base_yaw_rot_mat = RPYToRotationMatrix(
            self.prefix + "base_rot_mat"
        )
        dg.plug(self._base_yaw_vec.sout, self._base_yaw_rot_mat.sin)
        # Stack up the X,Y,Z velocity.
        self._xy_velocity = Stack_of_vector(self.prefix + "xy_velocity")
        self._xy_velocity.selec1(0, 1)
        self._xy_velocity.selec2(0, 1)
        #
        self.xyz_velocity = Stack_of_vector(self.prefix + "xyz_velocity")
        self.xyz_velocity.selec1(0, 2)
        self.xyz_velocity.selec2(0, 1)
        dg.plug(self._xy_velocity.sout, self.xyz_velocity.sin1)
        self.xyz_velocity.sin2.value = np.zeros(1)
        #
        self.world_xyz_velocity = Multiply_matrix_vector(
            self.prefix + "world_xyz_velocity"
        )
        dg.plug(self._base_yaw_rot_mat.sout, self.world_xyz_velocity.sin1)
        dg.plug(self.xyz_velocity.sout, self.world_xyz_velocity.sin2)

        # inputs:
        self.forward_velocity_sin = self._xy_velocity.sin1
        self.sideway_velocity_sin = self._xy_velocity.sin2
        self.yaw_angular_velocity_sins = [
            self._des_yaw.sin,
            self._angular_velocity.sin2,
        ]
        self.world_M_base_sin = self._base_pose_rpy.sin

        # outputs:
        self.world_xyz_velocity_sout = self.world_xyz_velocity.sout
        self.orientation_sout = self._des_ori_pos.sout
        self.angular_velocity_sout = self._angular_velocity.sout

        # init
        self.set_velocity(0, 0, 0)
        self.forward_velocity_sout = None
        self.sideway_velocity_sout = None
        self.yaw_velocity_sout = None

    def set_velocity(self, vx, vy, vyaw):
        # Yaw velocity
        for yaw_velocity_sin in self.yaw_angular_velocity_sins:
            yaw_velocity_sin.value = np.array([vyaw])
        # X Velocity
        self.forward_velocity_sin.value = np.array([vx])
        # Y Velocity
        self.sideway_velocity_sin.value = np.array([vy])

    def plug_velocity(self, forward_velocity, sideway_velocity, yaw_velocity):
        # Yaw velocity
        for yaw_velocity_sin in self.yaw_angular_velocity_sins:
            dg.plug(yaw_velocity, yaw_velocity_sin)
        # X Velocity
        dg.plug(forward_velocity, self.forward_velocity_sin)
        # Y Velocity
        dg.plug(sideway_velocity, self.sideway_velocity_sin)

    def plug_world_M_base(self, world_M_base):
        dg.plug(world_M_base, self.world_M_base_sin)


class Solo12WBCStepper:
    def __init__(self, prefix, friction_coeff):
        # copy args internally
        self.prefix = prefix
        self.friction_coeff = friction_coeff

        pin_robot = Solo12Config.buildRobotWrapper()
        end_effector_names = Solo12Config.end_effector_names

        ###
        # Create the input velocity for all subgraphs.
        self.vel_local_to_world = VelocityFromLocalToWorldFrame()

        ###
        # Create the whole body controller.
        qp_penalty_weights = np.array([1e0, 1e0, 1e6, 1e6, 1e6, 1e6])
        self.wbc = wbc = WholeBodyController(
            prefix + "wbc",
            pin_robot,
            end_effector_names,
            friction_coeff,
            qp_penalty_weights,
        )
        # For the centroidal controllers.
        wbc.kc_sin.value = np.array([100.0, 100.0, 180.0])  # P_com
        wbc.dc_sin.value = np.array([15.0, 15.0, 15.0])  # D_com
        wbc.kb_sin.value = np.array([12.0, 35.0, 24.0])  # P_rpy
        wbc.db_sin.value = np.array([1.0, 11.0, 11.0])  # D_rpy

        # Desired trajectories
        self.com_height = 0.25
        self.base_height = self.com_height + 0.02
        wbc.des_com_pos_sin.value = np.array([0.0, 0.0, self.com_height])
        wbc.des_com_vel_sin.value = np.zeros(3)
        wbc.des_ori_pos_sin.value = np.array([0.0, 0.0, 0.0, 1.0])
        wbc.des_ori_vel_sin.value = np.zeros(3)
        wbc.cnt_array_sin.value = np.array([1.0, 1.0, 1.0, 1.0])
        # Plug the desired vel and pose for the CoM
        dg.plug(
            self.vel_local_to_world.orientation_sout,
            self.wbc.des_ori_pos_sin,
        )
        dg.plug(
            self.vel_local_to_world.angular_velocity_sout,
            self.wbc.des_ori_vel_sin,
        )
        dg.plug(
            self.vel_local_to_world.world_xyz_velocity_sout,
            self.wbc.des_com_vel_sin,
        )

        # Impedance controllers.
        for _, imp in enumerate(wbc.imps):
            imp.gain_proportional_sin.value = np.array(
                [50.0, 50.0, 50.0, 0.0, 0.0, 0.0]
            )
            imp.gain_derivative_sin.value = np.array(
                [0.7, 0.7, 0.7, 0.0, 0.0, 0.0]
            )
            imp.gain_feed_forward_force_sin.value = 1.0
        #
        wbc.w_com_ff_sin.value = np.array(
            [0.0, 0.0, 9.81 * 2.5, 0.0, 0.0, 0.0]
        )

        ###
        # Create the stepper.
        self.stepper = stepper = QuadrupedStepper(
            prefix + "stepper_", pin_robot, end_effector_names
        )
        self.stepper.stepper.initialize_placement(
            np.array([0.0, 0.0, 0.238, 0, 0, 0.0, 1.0]),
            np.array([0.195, 0.147, 0.015]),
            np.array([0.195, -0.147, 0.015]),
            np.array([-0.195, 0.147, 0.015]),
            np.array([-0.195, -0.147, 0.015]),
        )
        is_left_leg_in_contact = True
        l_min = -0.10
        l_max = 0.10
        w_min = -0.10
        w_max = 0.10
        t_min = 0.1
        t_max = 0.3
        l_p = 0.0  # Pelvis width
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
            self.com_height,
            weight,
            mid_air_foot_height,
            control_period,
            planner_loop,
        )
        self.set_dynamical_end_effector_trajectory()
        # Let the quadruped step in place for now.
        self.stepper_des_com_vel_sin = (
            self.stepper.stepper.desired_com_velocity_sin
        )
        self.set_steptime_nominal(0.2)
        dg.plug(
            self.vel_local_to_world.world_xyz_velocity_sout,
            self.stepper_des_com_vel_sin,
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

        # Compute the center of both feet and track this point with the com
        self._sum_feet_pos = Add_of_vector(self.prefix + "sum_feet_pos")
        self._sum_feet_pos.n_sin = 4
        dg.plug(
            stepper.stepper.front_left_foot_position_sout,
            self._sum_feet_pos.sin(0),
        )
        dg.plug(
            stepper.stepper.front_right_foot_position_sout,
            self._sum_feet_pos.sin(1),
        )
        dg.plug(
            stepper.stepper.hind_left_foot_position_sout,
            self._sum_feet_pos.sin(2),
        )
        dg.plug(
            stepper.stepper.hind_right_foot_position_sout,
            self._sum_feet_pos.sin(3),
        )
        #
        self._xy_sum_feet = Selec_of_vector(self.prefix + "xy_sum_feet")
        self._xy_sum_feet.selec(0, 2)
        dg.plug(self._sum_feet_pos.sout, self._xy_sum_feet.sin)
        #
        self._feet_center = Multiply_double_vector(self.prefix + "feet_center")
        self._feet_center.sin1.value = 1.0 / 4.0
        dg.plug(self._xy_sum_feet.sout, self._feet_center.sin2)
        self._wbc_des_com_pos = Stack_of_vector(
            self.prefix + "wbc_des_com_pos"
        )
        self._wbc_des_com_pos.selec1(0, 2)
        self._wbc_des_com_pos.selec2(0, 1)
        dg.plug(self._feet_center.sout, self._wbc_des_com_pos.sin1)
        self._wbc_des_com_pos.sin2.value = np.array([self.com_height])
        dg.plug(self._wbc_des_com_pos.sout, self.wbc.des_com_pos_sin)

        # Sub graph of the gamepad parser.
        self.user_input = GamepadParser()

    def set_steptime_nominal(self, t_nom):
        self.stepper.stepper.set_steptime_nominal(t_nom)

    def set_polynomial_end_effector_trajectory(self):
        self.stepper.stepper.set_polynomial_end_effector_trajectory()

    def set_dynamical_end_effector_trajectory(self):
        self.stepper.stepper.set_dynamical_end_effector_trajectory()

    def start(self):
        self.stepper.start()

    def stop(self):
        """ Stop stepping deactivate gampad control."""
        self.use_user_velocity(0.0, 0.0, 0.0)
        time.sleep(0.5)
        self.stepper.stop()

    def plug(
        self, robot, base_position, base_velocity, ctrl_joint_torques_sin
    ):
        self.robot = robot
        self.stepper.plug(robot, base_position, base_velocity)
        self.wbc.plug_all_signals(
            robot.device.joint_positions,
            robot.device.joint_velocities,
            base_position,
            base_velocity,
            ctrl_joint_torques_sin,
        )
        self.vel_local_to_world.plug_world_M_base(base_position)

    def plug_gamepad(self, base_position, gamepad_vector):
        self.user_input.plug(base_position, gamepad_vector)

    def plug_base_as_com(self, base_position, base_velocity_world):
        self.wbc.plug_base_as_com(base_position, base_velocity_world)
        self._wbc_des_com_pos.sin2.value = np.array([self.base_height])

    def use_gamepad(self):
        self.vel_local_to_world.plug_velocity(
            self.user_input.forward_velocity_sout,
            self.user_input.sideway_velocity_sout,
            self.user_input.yaw_angular_velocity_sout,
        )

    def use_user_velocity(
        self,
        forward_velocity,
        sideway_velocity,
        yaw_angular_velocity,
    ):
        self.vel_local_to_world.set_velocity(
            forward_velocity,
            sideway_velocity,
            yaw_angular_velocity,
        )

    def set_eff_pd(self, p, p_z, d):
        for i, imp in enumerate(self.wbc.imps):
            imp.gain_proportional_sin.value = np.array(
                [p, p, p_z, 0.0, 0.0, 0.0]
            )
            imp.gain_derivative_sin.value = np.array([d, d, d, 0.0, 0.0, 0.0])

    def trace(self, robot):
        self.user_input.trace(robot)
        self.wbc.trace(robot)
        # self.stepper.trace(robot)

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


class BaseStateEstimator(object):
    def __init__(self):
        self.prefix = "BaseEstimator_"

        # Zero the initial position from the vicon signal.
        self._world_frame_offset = CreateWorldFrame(
            self.prefix + "world_local_M_world"
        )
        self._world_frame_offset.set_which_dofs(
            np.array([1.0, 1.0, 0.0, 0.0, 0.0, 1.0])
        )

        self.base_posture_ = MultiplyInv_poseQuaternion_vector(
            self.prefix + "world_local_M_base"
        )
        self.base_velocity_world_ = MultiplyInv_poseQuaternion_vector(
            self.prefix + "world_local_V_base"
        )

        # plug the application of the transform:
        dg.plug(
            self._world_frame_offset.world_frame_sout,
            self.base_posture_.sin1,
        )
        dg.plug(
            self._world_frame_offset.world_frame_sout,
            self.base_velocity_world_.sin1,
        )

        # outputs
        self.base_posture_sout = self.base_posture_.sout
        self.base_velocity_body_sout = None
        self.base_velocity_world_sout = self.base_velocity_world_.sout

    def plug_inputs(
        self,
        base_posture_sout,
        base_velocity_body_sout,
        base_velocity_world_sout,
        imu_gyroscope_sout,
    ):
        # Base offset:
        dg.plug(base_posture_sout, self._world_frame_offset.frame_sin)

        # Base posture:
        dg.plug(base_posture_sout, self.base_posture_.sin2)

        # Create the base velocity using the IMU.
        self.base_velocity_body_sout = stack_two_vectors(
            selec_vector(base_velocity_body_sout, 0, 3),
            imu_gyroscope_sout,
            3,
            3,
            entityName=self.prefix + "body_V_base",
        )

        # Base vel world
        dg.plug(base_velocity_world_sout, self.base_velocity_world_.sin2)

    def update(self):
        self._world_frame_offset.update()

    def trace(self, robot):
        robot.add_trace(self._world_frame_offset.name, "world_frame_sout")
        robot.add_trace(self.base_posture_.name, "sout")
        robot.add_trace(self.prefix + "body_V_base", "sout")
        robot.add_trace(self.base_velocity_world_.name, "sout")


class ReactiveStepperDemo(object):
    def __init__(self):
        # robot config
        self.robot_config = Solo12Config()

        # modes
        self.idle_mode = 0
        self.go_to_mode = 1
        self.stepper_mode = 2
        self.pd_slider_mode = 3
        self.n_mode = 4

        # Switch
        self._master_switch = SwitchVector("master_switch")
        self._master_switch.n_sin = self.n_mode

        # idle mode by default
        self._master_switch.selection.value = 0

        # Idle mode
        self._master_switch.sin(0).value = np.array(12 * [0.0])

        # Create a go_to controller to go to the initial pose.
        self.go_to_ctrl = get_solo12_goto_ctrl()

        # Setup the main controller.
        self.stepper_ctrl = Solo12WBCStepper("solo12_wbc_stepper", 0.6)

        # Create a pd controller with slider.
        self.pd_ctrl = get_solo12_pd_ctrl("solo12_pd_ctrl")

        # Setup the estimator
        self.base_state_estimator = BaseStateEstimator()

    def plug(
        self,
        robot,
        base_posture_sout,
        base_velocity_body_sout,
        base_velocity_world_sout,
    ):
        # plug the estimator
        self.base_state_estimator.plug_inputs(
            base_posture_sout,
            base_velocity_body_sout,
            base_velocity_world_sout,
            robot.device.imu_gyroscope,
        )

        # Setup the gamepad subscriber.
        robot.ros.ros_subscribe.add("vector", "gamepad_axes", "/gamepad_axes")
        self._gamepad_axes = robot.ros.ros_subscribe.signal("gamepad_axes")
        self._gamepad_axes.value = np.zeros(8)  # Initital value

        # Plug the go to controller.
        self.go_to_ctrl.plug(
            robot.device.joint_positions,
            robot.device.joint_velocities,
            self._master_switch.sin(self.go_to_mode),
        )

        # Plug the stepper inputs.
        self.stepper_ctrl.plug(
            robot,
            self.base_state_estimator.base_posture_sout,
            self.base_state_estimator.base_velocity_body_sout,
            self._master_switch.sin(self.stepper_mode),
        )
        self.stepper_ctrl.plug_gamepad(
            self.base_state_estimator.base_posture_sout, self._gamepad_axes
        )

        # Set the base as CoM in the WBC.
        # self.stepper_ctrl.plug_base_as_com(
        #     self.base_state_estimator.base_posture_sout,
        #     self.base_state_estimator.base_velocity_world_sout,
        # )

        self.pd_ctrl.simple_plug(
            robot.device.slider_positions,
            robot.device.joint_positions,
            robot.device.joint_velocities,
            self._master_switch.sin(self.pd_slider_mode),
        )

        # Plug the master switch to the control.
        dg.plug(self._master_switch.sout, robot.device.ctrl_joint_torques)

    def trace(self, robot):
        self.stepper_ctrl.trace(robot)
        self.base_state_estimator.trace(robot)

    def freeze(self):
        self._master_switch.selection.value = self.go_to_mode
        self.go_to_ctrl.freeze()

    def go_init(self):
        init_joint_pos = np.array(
            [
                0.0,
                0.55,
                -1.13,
                0.0,
                0.55,
                -1.13,
                0.0,
                -0.55,
                1.13,
                0.0,
                -0.55,
                1.13,
            ]
        )
        self.go_to_ctrl.go_to(init_joint_pos, 2000)
        self._master_switch.selection.value = self.go_to_mode

    def go_zero(self):
        self.go_to_ctrl.go_to(np.array(12 * [0.0]), 2000)
        self._master_switch.selection.value = self.go_to_mode

    def go_pd_init(self):
        pos = (
            np.pi
            / 4.0
            * np.array(
                [0.0, 1.0, -2.0, 0.0, 1.0, -2.0, 0, -1.0, 2.0, 0, -1.0, 2.0]
            )
        )
        self.go_to_ctrl.go_to(pos, 2000)
        self._master_switch.selection.value = self.go_to_mode

    def go_stepper(self):
        self.base_state_estimator.update()
        time.sleep(1.0)
        self._master_switch.selection.value = self.stepper_mode

    def go_pd(self):
        self.pd_ctrl.zero_slider()
        self._master_switch.selection.value = self.pd_slider_mode

    def start(self):
        self.use_gamepad()
        self._master_switch.selection.value = self.stepper_mode
        self.stepper_ctrl.start()

    def stop(self):
        self._master_switch.selection.value = self.stepper_mode
        self.stepper_ctrl.stop()

    def use_user_velocity(self, vx, vy, vyaw):
        self.stepper_ctrl.use_user_velocity(vx, vy, vyaw)

    def use_gamepad(self):
        self.stepper_ctrl.use_gamepad()


"""
From here on we are on the real robot.
"""
if "robot" in globals():
    #
    # Init vicon.
    #
    from dg_vicon_sdk.dynamic_graph.entities import ViconClientEntity

    vicon = ViconClientEntity("vicon_entity")
    # vicon.connect_to_vicon("172.24.117.119:801")  # NYU MIM vicon.
    vicon.connect_to_vicon("10.32.27.53:801")  # MPI MIM vicon.
    vicon.add_object_to_track("solo12/solo12")

    #
    # Base state estimation.
    #
    vicon_position_sout = vicon.signal("solo12_position")
    vicon_velocity_body_sout = vicon.signal("solo12_velocity_body")
    vicon_velocity_word_sout = vicon.signal("solo12_velocity_world")
    robot.add_trace(vicon.name, "solo12_position")
    robot.add_trace(vicon.name, "solo12_velocity_body")
    robot.add_trace(vicon.name, "solo12_velocity_world")

    # Create the demo and initialize it.
    ctrl = ReactiveStepperDemo()
    ctrl.plug(
        robot,
        vicon_position_sout,
        vicon_velocity_body_sout,
        vicon_velocity_word_sout,
    )
    ctrl.trace(robot)

    # create dome shortcut here
    go_stepper = ctrl.go_stepper
    go_init = ctrl.go_init
    go_pd_init = ctrl.go_pd_init
    go_pd = ctrl.go_pd
    go_zero = ctrl.go_zero
    f = freeze = ctrl.freeze
    start = ctrl.start
    stop = ctrl.stop
