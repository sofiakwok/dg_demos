import numpy as np

import dynamic_graph as dg
from dynamic_graph.sot.core.control_pd import ControlPD
from robot_properties_bolt.config import BoltConfig
from mim_control.dynamic_graph.wbc_graph import WholeBodyController
#this is somehow stopping the script from parsing
#from reactive_planners.dynamic_graph.biped_stepper import BipedStepper

from dynamic_graph.sot.tools import Oscillator
import dynamic_graph.sot.dynamic_pinocchio as dp
from dynamic_graph.sot.core.math_small_entities import (
    Multiply_double_vector,
    Add_of_double,
    Selec_of_matrix,
)

from dynamic_graph.sot.core.math_small_entities import Component_of_vector

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
    basePoseQuat2PoseRPY,
    hom2pos,
)

from dg_tools.dynamic_graph.dg_tools_entities import (
    CreateWorldFrame, 
    PoseRPYToPoseQuaternion,
)

from reactive_planners.dynamic_graph.walking import DcmReactiveStepper

np.set_printoptions(suppress=True)


class BoltWBCStepper:
    def __init__(self, prefix, friction_coeff, is_real_robot):
        pin_robot = BoltConfig.buildRobotWrapper()
        end_effector_names = BoltConfig.end_effector_names

        self.nv = pin_robot.model.nv
        self.dg_robot = dp.DynamicPinocchio(prefix + "_pinocchio")
        self.dg_robot.setModel(pin_robot.model)
        self.dg_robot.setData(pin_robot.data)

        self.sig_eff_pos = []
        self.sig_eff_vel = []

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

        self.kf_eff = 0.0
        if self.is_real_robot:
            x = 2
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


        # Create the objects of interests from pinocchio.
        self.com = self.dg_robot.signal("com")
        self.vcom = multiply_mat_vec(
            self.dg_robot.signal("Jcom"), self.dg_robot.signal("velocity")
        )

        for endeff_name in end_effector_names:
            self.dg_robot.createPosition("pos_" + endeff_name, endeff_name)
            self.dg_robot.createJacobianEndEffWorld(
                "jac_" + endeff_name, endeff_name
            )

            # Store the endeffector position signal.
            self.sig_eff_pos.append(
                hom2pos(self.dg_robot.signal("pos_" + endeff_name))
            )

            # Compute the endeffector velocity signal.
            sel_linear = Selec_of_matrix(prefix + "_pinocchio_" + endeff_name)
            sel_linear.selecRows(0, 3)
            sel_linear.selecCols(0, self.nv + 6)
            dg.plug(self.dg_robot.signal("jac_" + endeff_name), sel_linear.sin)

            self.sig_eff_vel.append(
                multiply_mat_vec(
                    sel_linear.sout, self.dg_robot.signal("velocity")
                )
            )

        ###
        # Create the stepper.
        self.stepper = stepper = DcmReactiveStepper()

        # Setup the pinocchio input quantities for the stepper.
        dg.plug(self.com, self.stepper.com_position_sin)
        dg.plug(self.vcom, self.stepper.com_velocity_sin)
        dg.plug(
            self.sig_eff_pos[0],
            self.stepper.current_left_foot_position_sin,
        )
        dg.plug(
            self.sig_eff_vel[0],
            self.stepper.current_left_foot_velocity_sin,
        )
        dg.plug(
            self.sig_eff_pos[1],
            self.stepper.current_right_foot_position_sin,
        )
        dg.plug(
            self.sig_eff_vel[1],
            self.stepper.current_right_foot_velocity_sin,
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
                # dg.plug(self.stepper.contact_array_sout, contact.sin)
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
                # dg.plug(self.stepper.contact_array_sout, contact.sin)
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
        # dg.plug(stack_two_vectors(stack_two_vectors(constVector(np.array([0.0, 0.0])), self.sliders.A_vec, 2, 1), constVector(np.array([0.0, 0.0, 0.0])), 3, 3), wbc.w_com_ff_sin)

        self.wbc.w_com_ff_sin.value = 1 * np.array(
            [0.0, 0.0, 9.81 * 1.1, 0.0, 0.0, 0.0]
        )

        # Connect the stepper with the wbc.
        #dg.plug(stepper.contact_array_sout, wbc.cnt_array_sin)
        #dg.plug(stepper.c, wbc.cnt_array_sin)

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
                stepper.left_foot_position_sout,
                eef_offset,
                "des_pos_l",
            ),
            wbc.imps[0],
        )
        plug_des_vel(stepper.left_foot_velocity_sout, wbc.imps[0])

        plug_des_pos(
            add_vec_vec(
                stepper.right_foot_position_sout,
                eef_offset,
                "des_pos_r",
            ),
            wbc.imps[1],
        )
        plug_des_vel(stepper.right_foot_velocity_sout, wbc.imps[1])

    def initialize(self):
        # PART 1: Positions
        # Because this controller is specific for bolt, we can hard
        # code the values here.
        # self.stepper.initialize_placement(
        #     np.array([0.0, 0.0, 0.238, 0, 0, 0.0, 1.0]),
        #     np.array([0.195, 0.147, 0.015]),
        #     np.array([0.195, -0.147, 0.015]),
        # )

        # PART 2: Parameters
        is_left_leg_in_contact = True
        stepping_area = 0.01
        if self.is_real_robot:
            l_min = -0.15
            l_max = 0.15
            w_min = -0.05
            w_max = 0.25
            t_min = 0.15
            t_max = 0.8
            l_p = 0.075 * 1
            mid_air_foot_height = 0.06  # 0.07damp_ground#0.1Normal#.075
            self.base_com_offset = 0.05
            self.com_height = 0.38487417 - self.base_com_offset
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

        parameter_vector = np.array([is_left_leg_in_contact, l_min, l_max, w_min, w_max, t_min, t_max, l_p, self.com_height, mid_air_foot_height, control_period, planner_loop])

        self.stepper.initializeStepper(
            np.concatenate((parameter_vector,
            weight,
            np.array([0.0, 0.1235, self.eff_offset]),
            np.array([0.0, -0.1235, self.eff_offset])), axis=None)
        )

        ###
        # Let the biped step in place for now.
        self.des_com_vel_sin = self.stepper.desired_com_velocity_sin
        self.des_com_vel_sin.value = v_des_list
        self.stepper.base_yaw_sin.value = np.array([0.0, 0.0, 0.0])
        #self.stepper.is_closed_loop_sin.value = 0.

        self.set_steptime_nominal(0.22)
        self.set_dynamical_end_effector_trajectory()

    def set_steptime_nominal(self, t_nom):
        self.stepper.set_steptime_nominal(t_nom)

    def set_polynomial_end_effector_trajectory(self):
        self.stepper.set_polynomial_end_effector_trajectory()

    def set_dynamical_end_effector_trajectory(self):
        self.stepper.set_dynamical_end_effector_trajectory()

    def start(self):
        self.stepper.start()

    def stop(self):
        self.stepper.stop()

    def plug(self, robot, base_position, base_velocity):
        self.base_position = base_position
        self.robot = robot
        base_pose_rpy = basePoseQuat2PoseRPY(base_position)
        position = stack_two_vectors(
            base_pose_rpy, robot.device.joint_positions, 6, self.nv - 6
        )
        velocity = stack_two_vectors(
            base_velocity, robot.device.joint_velocities, 6, self.nv - 6
        )

        dg.plug(position, self.dg_robot.signal("position"))
        dg.plug(velocity, self.dg_robot.signal("velocity"))
        self.dg_robot.signal("acceleration").value = np.array(
            self.nv
            * [
                0.0,
            ]
        )

        ###
        # Plug the stepper base position.
        dg.plug(base_position, self.stepper.xyzquat_base_sin)
        
        self.wbc.plug(robot, base_position, base_velocity)
        self.plug_swing_foot_forces()

    def plug_base_as_com(self, base_position, base_velocity_world):
        print("plugging base as com")
        self.wbc.plug_base_as_com(base_position, base_velocity_world)
        print("setting desired com pos value")
        self.wbc.des_com_pos_sin.value = np.array(
            [0.0, 0.0, self.com_height + self.base_com_offset]
        )

    # def set_eff_pd(self, p, p_z, d):
    #     for i, imp in enumerate(self.wbc.imps):
    #         imp.gain_proportional_sin.value = np.array(
    #             [p, p, p_z, 0.0, 0.0, 0.0]
    #         )
    #         imp.gain_derivative_sin.value = np.array(
    #             [d, d, d, 0.0, 0.0, 0.0]
    #         )

    def set_kf(self, kf):
        self.kf_eff = kf
        if self.is_real_robot:
            x = 2
            self.wbc.kc_sin.value = self.kf_eff * np.array([0.0, 0.0, 60.0])
            self.wbc.dc_sin.value = self.kf_eff * np.array([0.0, 0.0, 0.1])
            self.wbc.kb_sin.value = self.kf_eff * np.array([3.8, 3.2, 0.0])
            self.wbc.db_sin.value = self.kf_eff * np.array([0.2, 0.2, 0.0])
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
                # dg.plug(self.stepper.contact_array_sout, contact.sin)
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
                # dg.plug(self.stepper.contact_array_sout, contact.sin)
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
        self.wbc.trace()

        # self.robot.add_trace(self.stepper.name, 'swing_foot_forces_sout')
        self.robot.add_trace(
            self.stepper.name, "next_support_foot_position_sout"
        )
        self.robot.add_trace(
            self.stepper.name, "left_foot_position_sout"
        )
        self.robot.add_trace(
            self.stepper.name, "right_foot_position_sout"
        )
        self.robot.add_trace("des_pos_l", "sout")
        self.robot.add_trace("des_pos_r", "sout")
        self.robot.add_trace("imp0", "sout")
        self.robot.add_trace("imp1", "sout")
        self.robot.add_trace("mulp0", "sout")
        self.robot.add_trace("muld0", "sout")
        self.robot.add_trace("mulp1", "sout")
        self.robot.add_trace("muld1", "sout")
        self.robot.add_ros_and_trace("vicon_entity", "biped_position")
        self.robot.add_ros_and_trace("vicon_entity", "biped_velocity_body")
        self.robot.add_ros_and_trace("vicon_entity", "biped_velocity_world")
        # self.robot.add_trace("des", "sout")

    def plug_swing_foot_forces(self):
        # Need to invert the frame for the swing foot forces.
        swing_foot_forces = mul_double_vec(-1, self.stepper.force_sout)

        for i, feedforward in enumerate(self.wbc.imps_feedforward):
            dg.plug(
                selec_vector(swing_foot_forces, 6 * i, 6 * (i + 1)),
                feedforward,
            )

def get_controller(prefix="biped_wbc_stepper", is_real_robot=True):
    #return BoltPDController(prefix="Bolt_")
    return BoltWBCStepper(prefix, 0.6, is_real_robot)


if "robot" in globals():
    #ctrl = get_controller()
    ctrl = get_controller("biped_wbc_stepper", True)

    # Zero the initial position from the mocap signal.
    pose = np.array([0, 0, 0, 0, 0, 0, 1])
    #need to convert np array to signal type for dg.plug to work
    base_posture_sin = constVector(pose, "")
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
    #
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

    def go_stepper():
        op.update()
        ctrl.plug(robot, base_posture_local_sin, base_velocity_sin)
        # Use base as com position gives more stable result.
        ctrl.plug_base_as_com(
            base_posture_local_sin,
            base_velocity_sin,  # vicon.signal("biped_velocity_world")
        )
        print("done plugging")
        #ctrl.trace()

    def go():
        ctrl.plug_to_robot(robot)

    print("#####")
    print("")
    print("Please execute `go_stepper()` function to start the controller.")
    print("")
    print("#####")
