"""This file is a demo for using the DG whole body controller.

License BSD-3-Clause
Copyright (c) 2021, New York University and Max Planck Gesellschaft.

Author: Julian Viereck, Elham Daneshmand
Date:   Oct 26, 2021
"""
import numpy as np

#np.set_printoptions(suppress=True, precision=3)
#import pinocchio as pin
import dynamic_graph as dg
from dynamic_graph.sot.core.control_pd import ControlPD

from mim_control.dynamic_graph.rw_lqr_graph import RWLQRController
from robot_properties_bolt.config import BoltRWConfig

from dg_tools.utils import (
    constVector,
    stack_two_vectors,
    selec_vector,
    add_vec_vec,
    subtract_vec_vec,
    multiply_mat_vec,
    Stack_of_vector,
    zero_vec,
    mul_double_vec,
    mul_vec_vec,
    constDouble,
)
from dynamic_graph.sot.core.math_small_entities import Component_of_vector

from dynamic_graph.sot.core.math_small_entities import Add_of_double

from dg_tools.dynamic_graph.dg_tools_entities import (
    CreateWorldFrame,
    PoseRPYToPoseQuaternion,
)

class BoltRWLQRStepper:
    def __init__(self, prefix, is_real_robot):
        pin_robot = BoltRWConfig.buildRobotWrapper()
        end_effector_names = BoltRWConfig.end_effector_names

        self.is_real_robot = is_real_robot

        # Create the whole body controller.
        self.lqr = lqr = RWLQRController(
            prefix + "_lqr",
            pin_robot,
            end_effector_names,
        )

        if self.is_real_robot:
            des_com_pos = np.array(
                [0.0, 0.0, 0.38487417 - 0.05]
            )  # self.com_height
        else:
            des_com_pos = np.array(
                [0.0, 0.0, 0.35487417]
            )

        des_quat = np.array([0, 0, 0, 1])

        des_joint_pos = np.array([-0.3, 0.78539816, -1.57079633, 0.3, 0.78539816, -1.57079633, 0])

        lqr.des_robot_configuration_sin.value = np.concatenate((des_com_pos, des_quat, des_joint_pos), axis=None)
        lqr.des_robot_velocity_sin.value = np.zeros(13)

        print("BoltRWLQRStepper init done")

    def plug(self, robot, base_position, base_velocity):
        self.base_position = base_position
        self.robot = robot
        #self.sliders.plug_slider_signal(robot.device.slider_positions)
        self.lqr.plug(robot, base_position, base_velocity)
        #self.plug_swing_foot_forces()

    def plug_base_as_com(self, base_position, base_velocity_world):
        self.lqr.plug_base_as_com(base_position, base_velocity_world)
        self.lqr.des_com_pos_sin.value = np.array(
            [0.0, 0.0, self.com_height + self.base_com_offset]
        )
    
    def trace(self):
        print("robot for controller trace: " + str(self.robot))
        self.lqr.trace(self.robot)

        # self.robot.add_trace(self.stepper.stepper.name, 'swing_foot_forces_sout')
        # self.robot.add_trace(
        #     self.stepper.stepper.name, "next_support_foot_position_sout"
        # )
        # self.robot.add_trace(
        #     self.stepper.stepper.name, "left_foot_position_sout"
        # )
        # self.robot.add_trace(
        #     self.stepper.stepper.name, "right_foot_position_sout"
        # )
        # self.robot.add_trace("des_pos_l", "sout")
        # self.robot.add_trace("des_pos_r", "sout")
        # self.robot.add_trace("imp0", "sout")
        # self.robot.add_trace("imp1", "sout")
        # self.robot.add_trace("mulp0", "sout")
        # self.robot.add_trace("muld0", "sout")
        # self.robot.add_trace("mulp1", "sout")
        # self.robot.add_trace("muld1", "sout")

        # self.robot.add_trace("optitrack_entity", "1049_position")
        # self.robot.add_trace("optitrack_entity", "1049_velocity_world")
        # self.robot.add_trace("optitrack_entity", "1049_velocity_body")
        # self.robot.add_trace("des", "sout")


def get_controller(prefix="biped_lqr_stepper", is_real_robot=False):
    return BoltRWLQRStepper(prefix, is_real_robot)

if ("robot" in globals()) or ("robot" in locals()):
    from dg_optitrack_sdk.dynamic_graph.entities import OptitrackClientEntity
    #Get mocap data
    mocap = OptitrackClientEntity("optitrack_entity")
    mocap.connect_to_optitrack("1049") # give desired body ID to track
    mocap.add_object_to_track("1049") # rigid body ID for biped
    # z height while on stand: 0.74 m
    # z height with legs straight on ground: 0.537839 m
    # sim z height with legs straight: 0.47286 m
    
    # Setup the main controller.
    ctrl = get_controller("biped_lqr_stepper", True)
    # VERY IMPORTANT
    # Should be around 1 for hardware demos
    ctrl.set_kf(1)

    # quaternion order: w x y z ?
    # pose = np.array([0, 0, 0.4, 0.0, 0.0, 0.0, 1.0])
    # locked legs: [-3.83979 0.949068 0.536791] ; 
    # rotation: [w:0.00100178, x:-0.00798363, y:0.0744517, z:0.997192 ]

    # Zero the initial position from the vicon signal.
    base_posture_sin = mocap.signal("1049_position")    
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
    
    # #
    # Create the base velocity using the IMU.
    velocity = np.array([0, 0, 0, 0, 0, 0])
    biped_velocity = constVector(velocity, "")
    base_velocity_sin = biped_velocity
    # base_velocity_sin = stack_two_vectors(
    #     selec_vector(biped_velocity, 0, 3),
    #     robot.device.base_gyroscope,
    #     3,
    #     3,
    # )

    # Set desired base rotation and velocity.
    # des_yaw = 0.0
    # ctrl.des_ori_pos_rpy_sin.value = np.array([0.0, 0.0, des_yaw])
    # ctrl.des_com_vel_sin.value = np.array([0.0, 0.0, 0.0])

    def go_poly():
        ctrl.set_polynomial_end_effector_trajectory()

    def go_swing_foot_forces():
        ctrl.plug_swing_foot_forces()

    def go_stepper():
        op.update()
        ctrl.plug(robot, base_posture_local_sin, base_velocity_sin)
        print("base_posture_sin: " + str(base_posture_local_sin.value))
        # Use base as com position gives more stable result.
        ctrl.plug_base_as_com(
            base_posture_local_sin,
            base_velocity_sin,  # vicon.signal("biped_velocity_world")
        )
        ctrl.trace()
        robot.start_tracer()
        #ctrl.start()

    def set_kf(value):
        ctrl.set_kf(value)

    def start():
        ctrl.start()
        
    def stop():
        #ctrl.stop()
        robot.stop_tracer()

    print("I'm ready to get your command :)")
    print("call go_stepper() to start controller")

    # robot.start_tracer()
    # print("started tracer")

    # Start the gamepad by default.
    # go_gamepad()
    # go_gamepad_lqr_vel()
