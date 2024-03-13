import os
import csv
import numpy as np
import dynamic_graph as dg
from dynamic_graph.sot.core.switch import SwitchVector
from dynamic_graph.sot.core.reader import Reader
from dynamic_graph.sot.core.control_pd import ControlPD
from dynamic_graph.sot.core.math_small_entities import (
    Add_of_vector,
    Multiply_double_vector,
)
from robot_properties_solo.config import Solo12Config
from dg_demos.solo12.controllers.go_to import Solo12GoTo
import rospkg
from dg_demos.simulation_tools import write_current_control_graph
from dynamic_graph_manager.vicon_sdk import ViconClientEntity


class Solo12Vicon(object):
    """Small helper for the vicon entity use """

    def __init__(self, prefix=""):
        # Parameters
        self.vicon_host_ip = "10.32.27.53"
        self.vicon_object_name = "solo12"

        # Create the Vicon entity
        self.vicon_client = ViconClientEntity(prefix + "vicon_client")
        self.vicon_client.connect_to_vicon(self.vicon_host_ip)
        self.vicon_client.add_object_to_track(
            "{}/{}".format(self.vicon_object_name, self.vicon_object_name)
        )

        self.position_signal_name = self.vicon_object_name + "_position"
        self.velocity_world_signal_name = (
            self.vicon_object_name + "_velocity_world"
        )
        self.velocity_body_signal_name = (
            self.vicon_object_name + "_velocity_body"
        )

        self.position_sout = self.vicon_client.signal(
            self.position_signal_name
        )
        self.velocity_world_sout = self.vicon_client.signal(
            self.velocity_world_signal_name
        )
        self.velocity_body_sout = self.vicon_client.signal(
            self.velocity_body_signal_name
        )

    def record_data(self, robot):
        robot.add_trace(self.vicon_client.name, self.position_signal_name)
        robot.add_trace(
            self.vicon_client.name, self.velocity_world_signal_name
        )
        robot.add_trace(self.vicon_client.name, self.velocity_body_signal_name)


class TrajectoryReplay(object):
    def __init__(self, prefix=""):
        # Establish a prefix for each entities in this controller.
        self.prefix = prefix

        # User defined parameters
        self.use_feed_forward_torques = False

        # Master switch between graphs
        self.master_switch = SwitchVector(self.prefix + "master_switch")
        self.master_switch.setSignalNumber(3)
        self.output_torque_sout = self.master_switch.sout
        # Idle mode: signal 0 (default)
        self.idle_index = 0
        self.master_switch.selection.value = self.idle_index
        self.master_switch.sin0.value = 12 * [0.0]
        # Go to is on signal 1
        self.go_to_index = 1
        self.go_to_output_torque_sin = self.master_switch.sin1
        # Follow the trajectory on signal 2
        self.follow_trajectory_index = 2
        self.follow_traj_output_torque_sin = self.master_switch.sin2

        # Get the robot informations.
        self.solo_config = Solo12Config()

        # Init configuration from the input sequence.
        self.init_position = 12 * [0.0]

        # Sequence players.
        self.reader_joint_positions = Reader(
            self.prefix + "reader_joint_positions"
        )
        self.reader_joint_velocities = Reader(
            self.prefix + "reader_joint_velocities"
        )
        self.reader_joint_torques = Reader(
            self.prefix + "reader_joint_torques"
        )

        # Specify which of the columns to select.
        # NOTE: This is selecting the columns in reverse order.
        #       Hence the last number is the first column in the file.
        #       So here we do not read the *first* column of the file.
        #       The generated vector is read in the order of the file though.
        self.column_selec = self.solo_config.nb_joints * "1" + "0"
        self.reader_joint_positions.selec.value = self.column_selec
        self.reader_joint_velocities.selec.value = self.column_selec
        self.reader_joint_torques.selec.value = self.column_selec

        # Create the pd controller.
        self.pd_ctrl = ControlPD(self.prefix + "pd_controller")

        # Gains
        self.pd_ctrl.Kp.value = self.solo_config.nb_joints * (0.01,)
        self.pd_ctrl.Kd.value = self.solo_config.nb_joints * (0.001,)

        # Plug the desired trajectories.
        dg.plug(
            self.reader_joint_positions.vector, self.pd_ctrl.desired_position
        )
        dg.plug(
            self.reader_joint_velocities.vector, self.pd_ctrl.desired_velocity
        )

        # Scale the desired feedforward torques by 1 or 0 in case we use them
        # or not.
        self.scale_torque = Multiply_double_vector(
            self.prefix + "scale_torque"
        )
        if self.use_feed_forward_torques:
            self.scale_torque.sin1.value = 1.0
        else:
            self.scale_torque.sin1.value = 0.0
        dg.plug(self.reader_joint_torques.vector, self.scale_torque.sin2)

        # Add pd.control and the read desired torques.
        self.add_pd_and_desired_torques = Add_of_vector(
            self.prefix + "add_pd_and_desired_torques"
        )
        dg.plug(self.scale_torque.sout, self.add_pd_and_desired_torques.sin1)
        dg.plug(self.pd_ctrl.control, self.add_pd_and_desired_torques.sin2)

        # GoTo controller:
        self.go_to = Solo12GoTo(self.prefix)

    def file_exists(self, filename):
        if os.path.isfile(filename):
            print("The file %s exists" % filename)
            return True
        else:
            print("The file %s does not exist" % filename)
            return False

    def set_gains(self, Kp, Kd, use_feed_forward_torques):
        self.pd_ctrl.Kp.value = self.solo_config.nb_joints * (Kp,)
        self.pd_ctrl.Kd.value = self.solo_config.nb_joints * (Kd,)
        self.use_feed_forward_torques = use_feed_forward_torques
        if self.use_feed_forward_torques:
            self.scale_torque.sin1.value = 1.0
        else:
            self.scale_torque.sin1.value = 0.0
        self.go_to.set_pd_gains(Kp, Kd)

    def set_desired_trajectory_files(
        self,
        desired_positions_file,
        desired_velocity_file,
        desired_torques_file,
    ):
        # Checkout the initial configuration and go there.
        assert self.file_exists(desired_positions_file)
        with open(desired_positions_file) as f:
            reader = csv.reader(f)
            self.init_position = np.genfromtxt(next(reader), delimiter=" ")[1:]
        assert len(self.init_position) == 12
        self.go_to.go_to(self.init_position, 2000)

        # Load the desired trajectories.
        self.reader_joint_positions.load(desired_positions_file)
        #
        assert self.file_exists(desired_velocity_file)
        self.reader_joint_velocities.load(desired_velocity_file)
        #
        assert self.file_exists(desired_torques_file)
        self.reader_joint_torques.load(desired_torques_file)

    def plug(self, joint_positions_sout, joint_velocities_sout, torques_sin):
        # plug the desired quantity signals in the pd controller.
        dg.plug(joint_positions_sout, self.pd_ctrl.position)
        dg.plug(joint_velocities_sout, self.pd_ctrl.velocity)
        dg.plug(
            self.add_pd_and_desired_torques.sout,
            self.follow_traj_output_torque_sin,
        )

        self.go_to.plug(
            joint_positions_sout,
            joint_velocities_sout,
            self.go_to_output_torque_sin,
        )

        dg.plug(self.output_torque_sout, torques_sin)

    def go_zero(self):
        self.go_to.go_to(self.init_position, 2000)
        print("Start going to the initial configuration.")
        self.master_switch.selection.value = self.go_to_index
        print("Master switch: switched to 'go_to_index'.")

    def run_trajectory(self):
        # Safety check
        # current_position = self.pd_ctrl.position.value
        # if(np.linalg.norm(current_position - self.init_position) > 0.05):
        #     print("The robot is not at the initial configuration,",
        #           " please run ctrl.go_zero() first.",
        #           np.linalg.norm(current_position - self.init_position),
        #           "> 0.01")
        #     print("current_position = ", current_position)
        #     print("initial_position = ", self.init_position)
        #     return
        print("Warning safety check disabled!!")
        self.reader_joint_positions.rewind()
        self.reader_joint_velocities.rewind()
        self.reader_joint_torques.rewind()
        print("Start following the trajectories.")
        self.master_switch.selection.value = self.follow_trajectory_index
        print("Master switch: switched to 'follow_trajectory_index'.")

    def record_data(self, robot):
        """ Adding logging traces. """

        # Add the PD controller data
        robot.add_trace(self.pd_ctrl.name, "desired_position")
        robot.add_trace(self.pd_ctrl.name, "desired_velocity")
        robot.add_trace(self.pd_ctrl.name, "position")
        robot.add_trace(self.pd_ctrl.name, "velocity")
        robot.add_trace(self.pd_ctrl.name, "control")

        # Add the PD+
        robot.add_trace(self.add_pd_and_desired_torques.name, "sin1")
        robot.add_trace(self.add_pd_and_desired_torques.name, "sin2")
        robot.add_trace(self.add_pd_and_desired_torques.name, "sout")

        # Add the readers
        robot.add_trace(self.reader_joint_positions.name, "vector")
        robot.add_trace(self.reader_joint_velocities.name, "vector")
        robot.add_trace(self.reader_joint_torques.name, "vector")

        # Add the master switch
        robot.add_trace(self.master_switch.name, "sin0")
        robot.add_trace(self.master_switch.name, "sin1")
        robot.add_trace(self.master_switch.name, "sin2")
        robot.add_trace(self.master_switch.name, "sout")
        robot.add_trace(self.master_switch.name, "selection")

        # Add the GoTo
        self.go_to.record_data(robot)


def get_controller():
    return TrajectoryReplay(prefix="solo12_")


if "robot" in globals():
    #
    # load controller
    #
    ctrl = get_controller()
    ctrl.set_gains(Kp=5.0, Kd=0.05, use_feed_forward_torques=False)
    ctrl.plug(
        robot.device.joint_positions,
        robot.device.joint_velocities,
        robot.device.ctrl_joint_torques,
    )

    # load the trajectories
    file_paths = os.path.join(
        rospkg.RosPack().get_path("dg_demos"),
        "python",
        "dg_demos",
        "solo12",
        "simulations",
    )
    # file_paths = os.path.join(file_paths, "com_oscillation")
    file_paths = os.path.join(file_paths, "stamping")
    ctrl.set_desired_trajectory_files(
        os.path.join(file_paths, "joint_positions.dat"),
        os.path.join(file_paths, "joint_velocities.dat"),
        os.path.join(file_paths, "joint_torques.dat"),
    )

    # Record the data
    ctrl.record_data(robot)

    # Create the vicon drivers and acquire the data
    vicon = Solo12Vicon(prefix="solo12_")
    vicon.record_data(robot)

    #
    # Write the graph for debugging
    #
    write_current_control_graph("solo12", "trajectory_replay")

    def angles():
        import numpy as np

        np.set_printoptions(suppress=True)
        print(np.array(robot.device.joint_positions.value).reshape(4, 3))

    print("#####")
    print("")
    print("- 'ctrl.go_zero()' to go to the starting configuration.")
    print("- 'ctrl.run_trajectory()' to perform the tracking.")
    print("")
    print("#####")
