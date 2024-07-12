import numpy as np

import dynamic_graph as dg
from mim_control.dynamic_graph.bolt_pd_graph import BoltPD
from robot_properties_bolt.config import BoltConfig

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

from dg_tools.dynamic_graph.dg_tools_entities import (
    CreateWorldFrame,
    PoseRPYToPoseQuaternion,
)

from dynamic_graph.sot.tools import Oscillator

from dynamic_graph.sot.core.math_small_entities import (
    Multiply_double_vector,
    Add_of_double,
)

from dg_tools.utils import add_vec_vec

np.set_printoptions(suppress=True)


class BoltPDController(object):
    def __init__(self, prefix, is_real_robot):
        pin_robot = BoltConfig.buildRobotWrapper()
        end_effector_names = BoltConfig.end_effector_names

        self.is_real_robot = is_real_robot

        # Create the whole body controller.
        self.pd = pd = BoltPD(
            prefix + "_pd",
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

        #des_joint_pos = np.array([-0.3, 0.78539816, -1.57079633, 0.3, 0.78539816, -1.57079633])
        des_joint_pos = np.array([0.0, 0.1, -0.2, 0.0, 0.1, -0.2])

        pd.des_robot_configuration_sin.value = np.concatenate((des_com_pos, des_quat, des_joint_pos), axis=None)
        pd.des_robot_velocity_sin.value = np.zeros(12)

        print("BoltPDStepper init done")

    def set_desired_position(self, joint_pos):
        ctrl.pd.desired_position.value = joint_pos
            
    def bend_legs(self):
        # go from straight legs to bent legs
        timescale = 50000 
        # go to bent knee position
        for i in range(timescale):
            # make a linear function from starting joint position to desired joint position
            joint_pos = ctrl.final_pos*(i/timescale)
            # Specify the desired joint positions.
            ctrl.pd.desired_position.value = joint_pos
        print(joint_pos)

    def jump_from_bend(self):

        timescale = 20000 #works without triggering driver error
        # go to straight legs
        for i in range(timescale):
            # make a linear function from starting joint position to desired joint position
            joint_pos = ctrl.final_pos - ctrl.final_pos*(i/timescale)
            # Specify the desired joint positions.
            ctrl.pd.desired_position.value = joint_pos
        print(joint_pos)

    def plug_to_robot(self, robot):
        self.plug(
            robot.device.joint_positions,
            robot.device.joint_velocities,
            robot.device.ctrl_joint_torques,
        )

    def plug(self, robot, base_position, base_velocity):
        self.base_position = base_position
        self.robot = robot
        #self.sliders.plug_slider_signal(robot.device.slider_positions)
        self.pd.plug(robot, base_position, base_velocity)
        #self.plug_swing_foot_forces()

    def trace(self):
        print("robot for controller trace: " + str(self.robot))
        self.pd.trace(self.robot)


def get_controller(prefix="biped_pd_stepper", is_real_robot=False):
    return BoltPDController(prefix, is_real_robot)


if "robot" in globals():
    from dg_optitrack_sdk.dynamic_graph.entities import OptitrackClientEntity
    #Get mocap data
    mocap = OptitrackClientEntity("optitrack_entity")
    mocap.connect_to_optitrack("1076") # give desired body ID to track
    mocap.add_object_to_track("1076") # rigid body ID for biped
    # z height while on stand: 0.74 m
    # z height with legs straight on ground: 0.537839 m
    # sim z height with legs straight: 0.47286 m
    
    # Setup the main controller.
    ctrl = get_controller("biped_pd_stepper", True)

    print("robot: " + str(robot))

    # quaternion order: x y z w
    # pose = np.array([0, 0, 0.4, 0.0, 0.0, 0.0, 1.0])
    # locked legs: [-3.83979 0.949068 0.536791] ; 
    # rotation: [x:0.00100178, y:-0.00798363, z:0.0744517, w:0.997192 ]

    # Zero the initial position from the vicon signal.
    base_posture_sin = mocap.signal("1076_position")    
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

    def go():
        ctrl.plug(robot, base_posture_local_sin, base_velocity_sin)
        print("plugged robot")
        ctrl.trace()
        robot.start_tracer()

        
    def stop():
        #ctrl.stop()
        robot.stop_tracer()

    def bend():
        ctrl.bend_legs()

    def jump():
        ctrl.jump_from_bend()

    print("#####")
    print("")
    print("Please execute `go()` function to start the controller.")
    print("")
    print("#####")