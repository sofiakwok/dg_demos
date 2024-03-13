#! /usr/bin/env/python

"""Simple example showing how to get gamepad events."""

from pathlib import Path
from sensor_msgs.msg import Joy
import rclpy
from mim_msgs.srv import JointCalibration
from dynamic_graph_manager.dynamic_graph_manager_client import (
    DynamicGraphManagerClient,
)
from dg_demos.solo12.controllers import reactive_stepper_gamepad


class GamepadCommandManager(rclpy.node.Node):
    """
    Managing the buttons from the gamepad in order to interact with the
    controller.

    receiving:
        buttons:
            0: A
            1: B
            2: X
            3: Y
            4: LB
            5: RB
            6: back
            7: start
            8: central
            9: left joystick
           10: right joystick"""

    def __init__(self, dgm_client):
        super().__init__("gamepad_commands")
        self._dgm_client = dgm_client
        self._subscriber = self.create_subscription(
            Joy, "/joy", self._joy_callback, 10
        )

        self._last_action = "stop"
        self._fifo_start = []

    def _on_start_reslease(self):
        if self._last_action == "stop":
            self._dgm_client.run_python_command("ctrl.start()")
            self._last_action = "start"
        else:
            self._dgm_client.run_python_command("ctrl.stop()")
            self._last_action = "stop"

    def _joy_callback(self, msg_in):
        """ Use the input from /joy_node to trigger events."""
        # We first manage the fifo size.
        self._fifo_start.append(msg_in.buttons[7])
        while len(self._fifo_start) > 6:
            self._fifo_start.pop(0)

        # We then manage the start/stop event.
        if self._fifo_start == [1, 1, 1, 0, 0, 0]:
            self._on_start_reslease()


class CalibrationClient(rclpy.node.Node):
    """Call for the robot calibration.

    Args:
        rclpy (ROS): Inherite from the ROS2 Node interface.
    """

    def __init__(self, node_name="calibration_client"):
        super().__init__(node_name)
        self.calibrate_client = self.create_client(
            JointCalibration, "/dynamic_graph_manager/calibrate_joint_position"
        )
        while not self.calibrate_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(
                "'/dynamic_graph_manager/calibrate_joint_position' "
                "service not available, waiting again..."
            )
        self.calibrate_request = JointCalibration.Request()

    def calibrate(self):
        future = self.calibrate_client.call_async(self.calibrate_request)
        rclpy.spin_until_future_complete(self, future)


def press_enter_to_continue(msg):
    print("######################")
    print("")
    input("Press enter to\033[1;36;40m " + msg + "\033[0m.\n")
    print("")
    print("---------------------")


def main(args=None):
    rclpy.init(args=args)

    print("Creating the dynamic_graph_manager client.")
    dgm_client = DynamicGraphManagerClient()
    print("Creating gamepad command manager.")
    gamepad_command_manager = GamepadCommandManager(dgm_client)

    # Load control graph
    print("Load graph!")
    graph_script = str(Path(reactive_stepper_gamepad.__file__))
    dgm_client.run_python_script(graph_script)
    dgm_client.run_python_command("print('Graph loaded.')")

    # Create the remote calibration client.
    calibrate_client = CalibrationClient()
    press_enter_to_continue("calibrate the robot")
    calibrate_client.calibrate()

    press_enter_to_continue("start the dynamic graph and the tracer")
    dgm_client.start_tracer()
    dgm_client.start_dynamic_graph()

    press_enter_to_continue("go to the initial position")
    dgm_client.run_python_command("ctrl.go_init()")

    press_enter_to_continue("use the stepping controller")
    dgm_client.run_python_command("ctrl.go_stepper()")

    print("Please use the gampad to control the robot.")
    rclpy.spin(gamepad_command_manager)

    print("Dump the logs on disk.")
    dgm_client.stop_tracer()

    print("Shutting down.")
    rclpy.shutdown()


if __name__ == "__main__":
    main()
