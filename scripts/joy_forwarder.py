#! /usr/bin/env/python

"""Simple example showing how to get gamepad events."""

import rclpy
from rclpy.node import Node
from mim_msgs.msg import Vector
from sensor_msgs.msg import Joy


class JoyForwarder(Node):
    """
    Publishes the values from the gamepad in a single vector.
    Following this drawing for naming:
    https://en.wikipedia.org/wiki/Xbox_360_controller#/media/File:360_controller.svg

    receiving:
        Floats:
            0: left stick X
            1: left stick Y
            2: left trigger
            3: right stick X
            4: right stick Y
            5: right trigger
            6: arrow X
            7: arrow Y
        Ints:
            all buttons. Not used here.
    broadcasting:
        Floats:
            0: left stick X
            1: left stick Y
            2: right sitck X
            3: right sitck Y
            4: direcional pad (arrow) X
            5: direcional pad (arrow) Y
            6: left trigger
            7: right trigger
    """

    def __init__(self):
        super().__init__("gamepad_publisher")
        self.subscriber = self.create_subscription(
            Joy, "/joy", self._joy_calllback, 10
        )
        self.publisher_ = self.create_publisher(Vector, "gamepad_axes", 10)

    def _joy_calllback(self, msg_in):
        """ Forward the input from /joy_node to DG."""
        msg_out = Vector()
        msg_out.data = [
            -msg_in.axes[0],
            msg_in.axes[1],
            -msg_in.axes[3],
            msg_in.axes[4],
            msg_in.axes[6],
            msg_in.axes[7],
            msg_in.axes[2],
            msg_in.axes[5],
        ]
        self.publisher_.publish(msg_out)


def main(args=None):
    rclpy.init(args=args)

    print("Creating gamepad publisher...")
    minimal_publisher = JoyForwarder()
    print("Creating gamepad publisher... Done.")

    print("Publishing...")
    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
