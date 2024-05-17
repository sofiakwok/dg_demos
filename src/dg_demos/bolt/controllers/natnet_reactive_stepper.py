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

from robot_properties_bolt.config import BoltConfig
from mim_control.dynamic_graph.wbc_graph import WholeBodyController
from reactive_planners.dynamic_graph.biped_stepper import BipedStepper

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

class BoltWBCStepper:
    def __init__(self, prefix, friction_coeff, is_real_robot):
        pin_robot = BoltConfig.buildRobotWrapper()
        end_effector_names = BoltConfig.end_effector_names

        self.is_real_robot = is_real_robot

        # Create the whole body controller.
        qp_penalty_weights = np.array([1, 1, 1e6, 1e6, 1e6, 1])
        self.wbc = wbc = WholeBodyController(
            prefix + "_wbc",
            pin_robot,
            end_effector_names,
            friction_coeff,
            qp_penalty_weights,
        )

        ###
        # Specify gains for the controller.
        self.kf_eff = 1
        if self.is_real_robot:
            self.wbc.kc_sin.value = self.kf_eff * np.array([0.0, 0.0, 60.0])
            self.wbc.dc_sin.value = self.kf_eff * np.array([0.0, 0.0, 0.1])
            self.wbc.kb_sin.value = self.kf_eff * np.array([3.8, 3.2, 0.0])
            self.wbc.db_sin.value = self.kf_eff * np.array([0.2, 0.2, 0.0])
        else:
            self.wbc.kc_sin.value = self.kf_eff * np.array([0.0, 0.0, 100.0])
            self.wbc.dc_sin.value = self.kf_eff * np.array([0.0, 0.0, 10.0])
            self.wbc.kb_sin.value = self.kf_eff * np.array([100, 100, 0.0])
            self.wbc.db_sin.value = self.kf_eff * np.array([0.1, 0.1, 0.0])

        if self.is_real_robot:
            wbc.des_com_pos_sin.value = np.array(
                [0.0, 0.0, 0.38487417 - 0.05]
            )  # self.com_height
        else:
            wbc.des_com_pos_sin.value = np.array(
                [0.0, 0.0, 0.40795507 - 0.045]
            )
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

        self.wbc.cnt_array_sin.value = np.array([1.0, 1.0])

        ###
        # Create the stepper.
        self.stepper = stepper = BipedStepper(
            prefix + "_stepper", pin_robot, end_effector_names
        )

        # Impedance controllers.
        for i, imp in enumerate(wbc.imps):
            if self.is_real_robot:
                contact = Component_of_vector("imp" + str(i))
                contact.setIndex(not i)
                dg.plug(
                    constVector(
                        np.array(
                            [
                                1.0,
                                1.0,
                            ]
                        ),
                        "",
                    ),
                    contact.sin,
                )
                # dg.plug(self.stepper.stepper.contact_array_sout, contact.sin)
                contact = contact.sout
                dg.plug(
                    mul_double_vec(
                        contact,
                        constVector(
                            self.kf_eff
                            * np.array([40.0, 40.0, 97.0, 0.0, 0.0, 0.0])
                        ),
                        "mulp" + str(i),
                    ),
                    imp.gain_proportional_sin,
                )
                # imp.gain_proportional_sin.value = self.kf_eff * np.array(
                #     [40.0, 40.0, 97.0, 0.0, 0.0, 0.0]
                # )
                dg.plug(
                    mul_double_vec(
                        contact,
                        constVector(
                            self.kf_eff
                            * np.array([0.26, 0.23, 0.16, 0.0, 0.0, 0.0])
                        ),
                        "muld" + str(i),
                    ),
                    imp.gain_derivative_sin,
                )
                # imp.gain_derivative_sin.value = self.kf_eff * np.array(
                #     [0.26, 0.23, 0.16, 0.0, 0.0, 0.0]
                # )
            else:
                contact = Component_of_vector("imp" + str(i))
                contact.setIndex(not i)
                dg.plug(
                    constVector(
                        np.array(
                            [
                                1.0,
                                1.0,
                            ]
                        ),
                        "",
                    ),
                    contact.sin,
                )
                # dg.plug(self.stepper.stepper.contact_array_sout, contact.sin)
                contact = contact.sout
                dg.plug(
                    mul_double_vec(
                        contact,
                        constVector(
                            self.kf_eff
                            * np.array([150.0, 150.0, 150.0, 0.0, 0.0, 0.0])
                        ),
                        "mulp" + str(i),
                    ),
                    imp.gain_proportional_sin,
                )
                dg.plug(
                    mul_double_vec(
                        contact,
                        constVector(
                            self.kf_eff * np.array([5, 5, 5, 0.0, 0.0, 0.0])
                        ),
                        "muld" + str(i),
                    ),
                    imp.gain_derivative_sin,
                )
                # imp.gain_proportional_sin.value = self.kf_eff * np.array(
                #     [150.0, 150.0, 150.0, 0.0, 0.0, 0.0]
                # )
                # imp.gain_derivative_sin.value = self.kf_eff * np.array(
                #     [5, 5, 5, 0.0, 0.0, 0.0]
                # )

            # dg.plug(self.kf_eff.sout, imp.gain_feed_forward_force_sin)
            imp.gain_feed_forward_force_sin.value = 1.0  # centroidal gain

        self.wbc.w_com_ff_sin.value = 1 * np.array(
            [0.0, 0.0, 9.81 * 1.1, 0.0, 0.0, 0.0]
        )

        # Connect the stepper with the wbc.
        dg.plug(stepper.stepper.contact_array_sout, wbc.cnt_array_sin)

        def plug_des_pos(stepper_pos, imp):
            dg.plug(
                stack_two_vectors(stepper_pos, zero_vec(4, "zero4"), 3, 4),
                imp.desired_end_frame_placement_sin,
            )

        def plug_des_vel(stepper_pos, imp):
            dg.plug(
                stack_two_vectors(stepper_pos, zero_vec(3, "zero3"), 3, 3),
                imp.desired_end_frame_velocity_sin,
            )

        # if self.is_real_robot:
        #     eef_offset = constVector(np.array([0., 0., 0.013]))
        # else:
        #     eef_offset = constVector(np.array([0., 0., 0.0171]))
        self.initialize()

        eef_offset = constVector(np.array([0.0, 0.0, self.eff_offset]))

        plug_des_pos(
            add_vec_vec(
                stepper.stepper.left_foot_position_sout,
                eef_offset,
                "des_pos_l",
            ),
            wbc.imps[0],
        )
        plug_des_vel(stepper.stepper.left_foot_velocity_sout, wbc.imps[0])

        plug_des_pos(
            add_vec_vec(
                stepper.stepper.right_foot_position_sout,
                eef_offset,
                "des_pos_r",
            ),
            wbc.imps[1],
        )
        plug_des_vel(stepper.stepper.right_foot_velocity_sout, wbc.imps[1])

    def initialize(self):
        # PART 1: Positions
        # Because this controller is specific for bolt, we can hard
        # code the values here.
        # self.stepper.stepper.initialize_placement(
        #     np.array([0.0, 0.0, 0.238, 0, 0, 0.0, 1.0]),
        #     np.array([0.195, 0.147, 0.015]),
        #     np.array([0.195, -0.147, 0.015]),
        # )

        # PART 2: Parameters
        is_left_leg_in_contact = True
        stepping_area = 0.01
        if self.is_real_robot:
            print("real robot parameters")
            l_min = -0.15
            l_max = 0.15
            w_min = -0.05
            w_max = 0.25
            t_min = 0.15
            t_max = 0.8
            l_p = 0.075 * 1
            mid_air_foot_height = 0.06  # 0.07damp_ground#0.1Normal#.075
            self.base_com_offset = 0.064979 #0.05
            self.com_height = 0.39985317 - self.base_com_offset 
            # 0.38487417 - self.base_com_offset
            # 0.35487417 is sim com height
            v_des_list = np.array([0.0, -0.0, 0.0])
            self.eff_offset = 0.013
            # [0.1, -0.05, 0.]#[0.1, -0.025, 0.]#[0.3, 0.01, 0.]damp_ground#[0.06, -0.0, 0.]Normal
        else:
            l_min = -0.12
            l_max = 0.12
            w_min = -0.04
            w_max = 0.2
            t_min = 0.159
            t_max = 0.16
            l_p = 0.0835 * 1
            mid_air_foot_height = 0.06
            self.base_com_offset = 0.045
            self.com_height = 0.32795507 + 0.04 - self.base_com_offset
            v_des_list = np.array([0.0, -0.0, 0.0])
            self.eff_offset = 0.0171

        weight = np.array([1, 5, 10, 1000, 1000, 1, 1, 1, 1])
        control_period = 0.001
        planner_loop = 0.010

        parameter_vector = np.array([is_left_leg_in_contact, 
                                     l_min, 
                                     l_max, 
                                     w_min,
                                     w_max,
                                     t_min, 
                                     t_max, 
                                     l_p, 
                                     self.com_height, 
                                     mid_air_foot_height, 
                                     control_period, 
                                     planner_loop])

        self.stepper.stepper.initializeStepper(
            np.concatenate((parameter_vector,
            weight,
            np.array([0.0, 0.1235, self.eff_offset]),
            np.array([0.0, -0.1235, self.eff_offset])), axis=None)
        )

        ###
        # Let the biped step in place for now.
        self.des_com_vel_sin = self.stepper.stepper.desired_com_velocity_sin
        self.des_com_vel_sin.value = v_des_list
        self.stepper.stepper.base_yaw_sin.value = np.array([0.0, 0.0, 0.0])
        # self.stepper.stepper.is_closed_loop_sin.value = 0.

        self.set_steptime_nominal(0.22)
        self.set_dynamical_end_effector_trajectory()

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
        self.base_position = base_position
        self.robot = robot
        #self.sliders.plug_slider_signal(robot.device.slider_positions)
        self.stepper.plug(robot, base_position, base_velocity)
        self.wbc.plug(robot, base_position, base_velocity)
        self.plug_swing_foot_forces()

    def plug_base_as_com(self, base_position, base_velocity_world):
        self.wbc.plug_base_as_com(base_position, base_velocity_world)
        self.wbc.des_com_pos_sin.value = np.array(
            [0.0, 0.0, self.com_height + self.base_com_offset]
        )
    
    def set_kf(self, kf):
        self.kf_eff = kf
        if self.is_real_robot:
            self.wbc.kc_sin.value = self.kf_eff * np.array([0.0, 0.0, 60.0])
            self.wbc.dc_sin.value = self.kf_eff * np.array([0.0, 0.0, 0.1])
            self.wbc.kb_sin.value = self.kf_eff * np.array([3.8, 3.2, 0.0])
            self.wbc.db_sin.value = self.kf_eff * np.array([0.2, 0.2, 0.0])
            # dg.plug(stack_two_vectors(constVector(np.array([0.0, 0.0])), self.sliders.A_vec, 2, 1), self.wbc.kc_sin)
            # dg.plug(stack_two_vectors(constVector(np.array([0.0, 0.0])), self.sliders.B_vec, 2, 1), self.wbc.dc_sin)
            # dg.plug(stack_two_vectors(stack_two_vectors(self.sliders.C_vec, self.sliders.C_vec, 1, 1), constVector(np.array([0.0])), 2, 1), self.wbc.kb_sin)
            # dg.plug(stack_two_vectors(stack_two_vectors(self.sliders.D_vec, self.sliders.D_vec, 1, 1), constVector(np.array([0.0])), 2, 1), self.wbc.db_sin)
        else:
            self.wbc.kc_sin.value = self.kf_eff * np.array([0.0, 0.0, 100.0])
            self.wbc.dc_sin.value = self.kf_eff * np.array([0.0, 0.0, 10.0])
            self.wbc.kb_sin.value = self.kf_eff * np.array([100, 100, 0.0])
            self.wbc.db_sin.value = self.kf_eff * np.array([0.1, 0.1, 0.0])
        for i, imp in enumerate(self.wbc.imps):
            if self.is_real_robot:
                contact = Component_of_vector("imp" + str(i))
                contact.setIndex(not i)
                dg.plug(
                    constVector(
                        np.array(
                            [
                                1.0,
                                1.0,
                            ]
                        ),
                        "",
                    ),
                    contact.sin,
                )
                # dg.plug(self.stepper.stepper.contact_array_sout, contact.sin)
                contact = contact.sout
                dg.plug(
                    mul_double_vec(
                        contact,
                        constVector(
                            self.kf_eff
                            * np.array([40.0, 40.0, 97.0, 0.0, 0.0, 0.0])
                        ),
                        "mulp" + str(i),
                    ),
                    imp.gain_proportional_sin,
                )
                # imp.gain_proportional_sin.value = self.kf_eff * np.array(
                #     [40.0, 40.0, 97.0, 0.0, 0.0, 0.0]
                # )
                dg.plug(
                    mul_double_vec(
                        contact,
                        constVector(
                            self.kf_eff
                            * np.array([0.26, 0.23, 0.16, 0.0, 0.0, 0.0])
                        ),
                        "muld" + str(i),
                    ),
                    imp.gain_derivative_sin,
                )
                # imp.gain_derivative_sin.value = self.kf_eff * np.array(
                #     [0.26, 0.23, 0.16, 0.0, 0.0, 0.0]
                # )
            else:
                contact = Component_of_vector("imp" + str(i))
                contact.setIndex(not i)
                dg.plug(
                    constVector(
                        np.array(
                            [
                                1.0,
                                1.0,
                            ]
                        ),
                        "",
                    ),
                    contact.sin,
                )
                # dg.plug(self.stepper.stepper.contact_array_sout, contact.sin)
                contact = contact.sout
                dg.plug(
                    mul_double_vec(
                        contact,
                        constVector(
                            self.kf_eff
                            * np.array([150.0, 150.0, 150.0, 0.0, 0.0, 0.0])
                        ),
                        "mulp" + str(i),
                    ),
                    imp.gain_proportional_sin,
                )
                dg.plug(
                    mul_double_vec(
                        contact,
                        constVector(
                            self.kf_eff * np.array([5, 5, 5, 0.0, 0.0, 0.0])
                        ),
                        "muld" + str(i),
                    ),
                    imp.gain_derivative_sin,
                )
                # imp.gain_proportional_sin.value = self.kf_eff * np.array(
                #     [150.0, 150.0, 150.0, 0.0, 0.0, 0.0]
                # )
                # imp.gain_derivative_sin.value = self.kf_eff * np.array(
                #     [5, 5, 5, 0.0, 0.0, 0.0]
                # )
        self.wbc.w_com_ff_sin.value = 1 * np.array(
            [0.0, 0.0, 9.81 * 1.1, 0.0, 0.0, 0.0]
        )
    
    def trace(self):
        self.wbc.trace(self.robot)

        # self.robot.add_trace(self.stepper.stepper.name, 'swing_foot_forces_sout')
        self.robot.add_trace(
            self.stepper.stepper.name, "next_support_foot_position_sout"
        )
        self.robot.add_trace(
            self.stepper.stepper.name, "left_foot_position_sout"
        )
        self.robot.add_trace(
            self.stepper.stepper.name, "right_foot_position_sout"
        )
        self.robot.add_trace("des_pos_l", "sout")
        self.robot.add_trace("des_pos_r", "sout")
        self.robot.add_trace("imp0", "sout")
        self.robot.add_trace("imp1", "sout")
        self.robot.add_trace("mulp0", "sout")
        self.robot.add_trace("muld0", "sout")
        self.robot.add_trace("mulp1", "sout")
        self.robot.add_trace("muld1", "sout")
        # self.robot.add_ros_and_trace("vicon_entity", "biped_position")
        # self.robot.add_ros_and_trace("vicon_entity", "biped_velocity_body")
        # self.robot.add_ros_and_trace("vicon_entity", "biped_velocity_world")
        # self.robot.add_trace("des", "sout")

    def plug_swing_foot_forces(self):
        # Need to invert the frame for the swing foot forces.
        swing_foot_forces = mul_double_vec(-1, self.stepper.stepper.force_sout)

        for i, feedforward in enumerate(self.wbc.imps_feedforward):
            dg.plug(
                selec_vector(swing_foot_forces, 6 * i, 6 * (i + 1)),
                feedforward,
            )


def get_controller(prefix="biped_wbc_stepper", is_real_robot=False):
    return BoltWBCStepper(prefix, 0.6, is_real_robot)

if True: #("robot" in globals()) or ("robot" in locals()):
    from dg_optitrack_sdk.dynamic_graph.entities import OptitrackClientEntity
    #Get mocap data
    mocap = OptitrackClientEntity("optitrack_entity")
    mocap.connect_to_optitrack("1049") # give desired body ID to track
    mocap.add_object_to_track("1049") # rigid body ID for biped
    # z height while on stand: 0.74 m
    # z height with legs straight on ground: 0.537839 m
    # sim z height with legs straight: 0.47286 m
    
    # Setup the main controller.
    ctrl = get_controller("biped_wbc_stepper", True)
    # VERY IMPORTANT
    # Should be around 1 for hardware demos
    ctrl.set_kf(1)

    # quaternion order: w x y z ?
    # pose = np.array([0, 0, 0.4, 0.0, 0.0, 0.0, 1.0])

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
    des_yaw = 0.0
    ctrl.des_ori_pos_rpy_sin.value = np.array([0.0, 0.0, des_yaw])
    ctrl.des_com_vel_sin.value = np.array([0.0, 0.0, 0.0])

    def go_poly():
        ctrl.set_polynomial_end_effector_trajectory()

    def go_swing_foot_forces():
        ctrl.plug_swing_foot_forces()

    def go_pd():
        ctrl.plug_to_robot(robot)
        print("plugged robot")

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

    print("I'm ready to get your command :)")
    print("call go_stepper() to start controller")

    # robot.start_tracer()
    # print("started tracer")

    # Start the gamepad by default.
    # go_gamepad()
    # go_gamepad_wbc_vel()
