import numpy as np

import dynamic_graph as dg
from dynamic_graph.sot.core.control_pd import ControlPD
from robot_properties_bolt.config import BoltConfig
from mim_control.dynamic_graph.wbc_graph import WholeBodyController
#this is somehow stopping the script from parsing
#from reactive_planners.dynamic_graph.biped_stepper import BipedStepper

from dynamic_graph.sot.tools import Oscillator

from dynamic_graph.sot.core.math_small_entities import (
    Multiply_double_vector,
    Add_of_double,
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
)

# from dg_tools.dynamic_graph.dg_tools_entities import (
#     CreateWorldFrame,
#     PoseRPYToPoseQuaternion,
#     PoseQuaternionToPoseRPY,
#     RPYToRotationMatrix,
#     VectorIntegrator,
# )

np.set_printoptions(suppress=True)

class BoltPDController(object):
    def __init__(self, prefix=""):
        self.prefix = prefix
        # self.sliders = Sliders(4, self.prefix)
        # self.sliders.set_scale_values([1.5, 2.0, 1.0, 1.0])

        self.bolt_config = BoltConfig()

        # Setup the control graph to track the desired joint positions.
        self.pd = pd = ControlPD("PDController")

        # setup the gains
        pd.Kp.value = np.array(6 * [3.0])
        pd.Kd.value = np.array(6 * [0.05])

        pd.desired_velocity.value = np.array(
            self.bolt_config.initial_velocity[6:]
        )

        self.joint_pos = np.array(self.bolt_config.initial_configuration[7:])

        # Specify the desired joint positions.
        pd.desired_position.value = self.joint_pos
        #dg.plug(self.joint_pos.sout, self.pd.desired_position)

        #self.slider_values = self.sliders

        print("done initializing pd controller")

    def set_desired_position(self, desired_pos):
        self.pd.desired_position.value = desired_pos

    def plug_to_robot(self, robot):
        self.plug(
            robot.device.joint_positions,
            robot.device.joint_velocities,
            robot.device.ctrl_joint_torques,
        )

    def plug(
        self,
        joint_positions,
        joint_velocities,
        ctrl_joint_torques,
    ):
        dg.plug(joint_positions, self.pd.position)
        dg.plug(joint_velocities, self.pd.velocity)
        dg.plug(self.pd.control, ctrl_joint_torques)

    def record_data(self, robot):
        # Adding logging traces.
        robot.add_trace(self.pd.name, "desired_position")

def get_controller():
    return BoltPDController(prefix="Bolt_")


if "robot" in globals():
    ctrl = get_controller()
    print("controller set up")

    # Zero the initial position from the mocap signal.
    pose = np.array([0, 0, 0, 0, 0, 0, 1])
    #need to convert np array to signal type for dg.plug to work
    base_posture_sin = constVector(pose, "")
    print("converting base posture to signal")
    op = CreateWorldFrame("wf")
    print("creating world frame")
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
    print("created base posture")
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
    print("created base velocity")

    # Set desired base rotation and velocity.
    des_yaw = 0.0
    ctrl.des_ori_pos_rpy_sin.value = np.array([0.0, 0.0, des_yaw])
    ctrl.des_com_vel_sin.value = np.array([0.0, 0.0, 0.0])

    def go():
        ctrl.plug_to_robot(robot)
        print("plugged robot")

    print("#####")
    print("")
    print("Please execute `go()` function to start the controller.")
    print("")
    print("#####")
