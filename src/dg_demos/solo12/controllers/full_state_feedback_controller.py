"""
"""

from time import sleep
from os import path
import csv
import numpy as np
from enum import Enum
from dynamic_graph import plug
from robot_properties_solo.config import Solo12Config
from dynamic_graph.sot.core.switch import SwitchVector
from dg_stochastic_control.state_vector import StateVector
from dg_stochastic_control.dynamic_graph.controllers import (
    FullStateFeedbackController,
)
from dg_tools.dynamic_graph.dg_tools_entities import (
    CreateWorldFrame,
    MultiplyInv_poseQuaternion_vector,
)
from dg_demos.solo12.controllers.go_to import Solo12GoTo


class CtrlMode(Enum):
    """Control modes of the graph below."""

    IDLE = 0
    GO_TO = 1
    DDP_BALANCE_CONTROL = 2
    RISK_BALANCE_CONTROL = 3


class Solo12FullStateFeedbackController(object):
    """Graph that computes a full state feedback controller."""

    def __init__(self, name):
        self.name = name

        # Get the robot informations.
        self.solo_config = Solo12Config()
        self.nb_joints = self.solo_config.nb_joints

        # Master switch between graphs
        self.master_switch = SwitchVector(self.name + "_master_switch")
        self.master_switch.setSignalNumber(4)
        # Idle mode:
        self.master_switch.selection.value = CtrlMode.IDLE.value  # (default)
        self.master_switch.sin(CtrlMode.IDLE.value).value = np.array(
            self.nb_joints * [0.0]
        )
        # Go to mode:
        self.go_to_output_torque_sin = self.master_switch.sin(
            CtrlMode.GO_TO.value
        )
        # DDP Balance Control:
        self.ddp_balance_torque_sin = self.master_switch.sin(
            CtrlMode.DDP_BALANCE_CONTROL.value
        )
        # Risk sensitive Balance Control:
        self.risk_balance_torque_sin = self.master_switch.sin(
            CtrlMode.RISK_BALANCE_CONTROL.value
        )

        # Create offset entity for the vicon frame.
        self.world_frame = CreateWorldFrame(self.name + "_world_frame")
        # We only preserve x, y and yaw direction
        self.world_frame.set_which_dofs(np.array([1, 1, 0, 0, 0, 1]))

        # Create the transforms from the vicon frame to the world frame.
        self.base_position_in_world = MultiplyInv_poseQuaternion_vector(
            self.name + "_base_position_in_world"
        )
        self.base_velocity_in_world = MultiplyInv_poseQuaternion_vector(
            self.name + "_base_velocity_in_world"
        )

        # Create the Go To controller.
        self.go_to = Solo12GoTo(self.name + "_")
        self.go_to.set_pd_gains(Kp=5.0, Kd=0.05)  # nice gains for Solo12 MPI
        self.zero_pose = np.array(self.nb_joints * [0.0])
        self.go_to_duration = 2000

        # Create the state vector.
        self.robot_state = StateVector(
            self.name + "_fsf_ctrl_state_vector", self.solo_config.nb_joints
        )

        # Create the ddp balance controller
        self.ddp_balance = FullStateFeedbackController(
            self.name + "_ddp_balance"
        )
        self.init_pose_ddp_balance = np.array(self.nb_joints * [0.0])

        # Create the risk sensitive balance controller
        self.risk_balance = FullStateFeedbackController(
            self.name + "_risk_balance"
        )
        self.init_pose_risk_balance = np.array(self.nb_joints * [0.0])

        # Plug the graph
        # ddp balance
        plug(
            self.robot_state.state_sout,
            self.ddp_balance.full_state_measurement,
        )
        plug(self.ddp_balance.control_torque, self.ddp_balance_torque_sin)
        # risk sensitive plan
        plug(
            self.robot_state.state_sout,
            self.risk_balance.full_state_measurement,
        )
        plug(self.risk_balance.control_torque, self.risk_balance_torque_sin)
        # transfrom to the world frame
        plug(
            self.world_frame.world_frame_sout, self.base_position_in_world.sin1
        )
        plug(
            self.world_frame.world_frame_sout, self.base_velocity_in_world.sin1
        )
        plug(
            self.base_position_in_world.sout,
            self.robot_state.base_position_sin,
        )
        plug(
            self.base_velocity_in_world.sout,
            self.robot_state.base_velocity_sin,
        )

        # input/output
        self.base_position_sin = (
            self.base_position_in_world.sin2,
            self.world_frame.frame_sin,
        )
        self.joint_position_sin = self.robot_state.joint_position_sin
        self.base_velocity_sin = self.base_velocity_in_world.sin2
        self.joint_velocity_sin = self.robot_state.joint_velocity_sin
        self.control_torque_sout = self.master_switch.sout

    def initialize(self, path_to_plans):
        path_ddp_balance = path.join(path_to_plans, "balance_ddp")
        path_risk_balance = path.join(path_to_plans, "balance_risk")

        print(path_ddp_balance)
        print(path_risk_balance)
        self.ddp_balance.initialize(
            self.solo_config.urdf_path, path_ddp_balance
        )
        self.risk_balance.initialize(
            self.solo_config.urdf_path, path_risk_balance
        )

        def get_initial_position(path_to_file):
            """ Extract the first line of a file """
            assert path.isfile(path_to_file)
            with open(path_to_file) as f:
                reader = csv.reader(f)
                init_position = np.genfromtxt(next(reader), delimiter=" ")[
                    7 : 7 + 12
                ]
            print(init_position)
            print(len(init_position))
            assert len(init_position) == 12
            return init_position

        self.init_pose_ddp_balance[:] = get_initial_position(
            path.join(path_ddp_balance, "reference_state.csv")
        )
        self.go_to.go_to(self.init_pose_ddp_balance, self.go_to_duration)
        self.init_pose_risk_balance[:] = get_initial_position(
            path.join(path_risk_balance, "reference_state.csv")
        )
        self.go_to.go_to(self.init_pose_ddp_balance, self.go_to_duration)

    def go_init_ddp_plan(self):
        self.go_to.go_to(self.init_pose_ddp_balance, self.go_to_duration)
        print("Go to the initial ddp_balance position.")
        self.master_switch.selection.value = CtrlMode.GO_TO.value

    def run_ddp_plan(self):
        self.world_frame.update()
        self.ddp_balance.stopPlan()
        self.ddp_balance.resetController()
        # self.ddp_balance.runPlan()
        print("Starting the ddp balance controller.")
        self.master_switch.selection.value = CtrlMode.DDP_BALANCE_CONTROL.value

    def go_init_risk_plan(self):
        self.go_to.go_to(self.init_pose_ddp_balance, self.go_to_duration)
        print("Go to the initial ddp_balance position.")
        self.master_switch.selection.value = CtrlMode.GO_TO.value

    def run_risk_plan(self):
        self.world_frame.update()
        self.ddp_balance.stopPlan()
        self.ddp_balance.resetController()
        # self.ddp_balance.runPlan()
        print("Starting the risk sensitive balance controller.")
        self.master_switch.selection.value = (
            CtrlMode.RISK_BALANCE_CONTROL.value
        )

    def go_idle(self):
        print("Idle mode.")
        self.master_switch.selection.value = CtrlMode.IDLE.value

    def record_data(self, robot):
        # Adding logging traces.
        robot.add_trace(self.ddp_balance.name, "full_state_measurement")
        robot.add_trace(self.ddp_balance.name, "control_torque")
        robot.add_trace(self.risk_balance.name, "full_state_measurement")
        robot.add_trace(self.risk_balance.name, "control_torque")
        self.go_to.record_data(robot)
        return

    def plug(
        self,
        base_position_sout,
        joint_position_sout,
        base_velocity_sout,
        joint_velocity_sout,
        control_torque_sin,
    ):
        # plug the inputs.
        plug(base_position_sout, self.base_position_sin[0])
        plug(base_position_sout, self.base_position_sin[1])
        plug(joint_position_sout, self.joint_position_sin)
        plug(base_velocity_sout, self.base_velocity_sin)
        plug(joint_velocity_sout, self.joint_velocity_sin)

        # plug the output.
        plug(self.control_torque_sout, control_torque_sin)

        # plug the go_to controller.
        self.go_to.plug(
            joint_position_sout,
            joint_velocity_sout,
            self.go_to_output_torque_sin,
        )


def get_controller():
    return Solo12FullStateFeedbackController("ctrl")


if "robot" in globals():
    from dg_vicon_sdk.dynamic_graph.entities import ViconClientEntity

    #
    # load controller
    #
    traj_path = "/home/bhammoud/Downloads"

    print("loading ", traj_path)

    ctrl = get_controller()
    ctrl.record_data(robot)
    ctrl.initialize(traj_path)

    vicon = ViconClientEntity("vicon")
    vicon.connect_to_vicon("10.32.27.53:801")
    vicon.add_object_to_track("solo12/solo12")

    ctrl.plug(
        vicon.solo12_position,
        robot.device.joint_positions,
        vicon.solo12_velocity_body,
        robot.device.joint_velocities,
        robot.device.ctrl_joint_torques,
    )

    print("#####")
    print("")
    print("")
    print("")
    print("#####")
