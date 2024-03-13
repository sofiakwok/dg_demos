import numpy as np

# import the controller
from dg_demos.bolt.controllers.pd_controller import get_controller

# import the simulated robot
from bolt.dg_bolt_bullet import get_bolt_robot, BoltConfig


def simulate(with_gui=True):
    #
    # setup and run simulation
    #
    robot = get_bolt_robot(use_fixed_base=True, init_sliders_pose=4 * [0.0])
    bolt_config = BoltConfig()

    # Update the initial state of the robot.
    q0 = bolt_config.q0.copy()
    q0.fill(0.0)
    q0[2] = 1.0
    q0[6] = 1.0
    robot.reset_state(q0, bolt_config.v0)

    # load controller
    ctrl = get_controller()
    ctrl.plug(
        robot.device.slider_positions,
        robot.device.joint_positions,
        robot.device.joint_velocities,
        robot.device.ctrl_joint_torques,
    )

    # run the Simulation
    robot.run(500000)


if __name__ == "__main__":
    simulate()
