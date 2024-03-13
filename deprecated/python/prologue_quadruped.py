from dynamic_graph_manager.device import Device
from dynamic_graph_manager.device.robot import Robot
from py_robot_properties_quadruped.sot_torque_control_config import (
    QuadrupedConfig,
)

# first of all get some configuration from the robot
config = QuadrupedConfig()
local_device = Device(config.robot_name)
local_device.initialize(config.yaml_path)
robot = Robot(name=local_device.name, device=local_device)

__all__ = ["robot"]
