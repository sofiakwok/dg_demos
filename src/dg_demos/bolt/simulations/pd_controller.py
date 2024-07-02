import numpy as np
import pybullet as p
# import the controller
from dg_demos.bolt.controllers.bolt_pd_controller import get_controller

# import the simulated robot
from bolt.dg_bolt_bullet import get_bolt_robot, BoltConfig


def simulate(with_gui=True):
    #
    # setup and run simulation
    #
    robot = get_bolt_robot(use_fixed_base=False, init_sliders_pose=4 * [0.0])
    bolt_config = BoltConfig()

    sim_freq = 10000
    p.resetDebugVisualizerCamera(1.3, 60, -35, (0.0, 0.0, 0.0))
    p.setTimeStep(1.0 / sim_freq)
    p.setRealTimeSimulation(0)

    # Update the initial state of the robot.
    q0 = np.matrix(BoltConfig.initial_configuration).T
    qdot = np.matrix(BoltConfig.initial_velocity).T
    #q0.fill(0.0)
    #q0[2] = 1.0
    #q0[6] = 1.0
    robot.reset_state(q0, qdot)

    # load controller
    ctrl = get_controller(is_real_robot=False)
    ctrl.plug(robot, *robot.base_signals())
    print("plugged signals")

    ctrl.trace()
    print("trace")
    robot.start_tracer()

    # run the Simulation
    robot.run(10000, 0.0001)

    robot.stop_tracer()


if __name__ == "__main__":
    simulate()
