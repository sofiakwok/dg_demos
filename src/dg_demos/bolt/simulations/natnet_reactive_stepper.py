"""
Simulation demo for solo12 reactive stepper.

License BSD-3-Clause
Copyright (c) 2021, New York University and Max Planck Gesellschaft.

Author: Julian Viereck, Elham Daneshmand
Date:   Feb 26, 2021
"""

import numpy as np
import pybullet as p

import dynamic_graph as dg
#from dg_demos.bolt.controllers.reactive_stepper_no_mocap import get_controller
from dg_demos.bolt.controllers.natnet_reactive_stepper import get_controller
# import the simulated robot
from bolt.dg_bolt_bullet import get_bolt_robot, BoltConfig

from dg_tools.dynamic_graph.dg_tools_entities import (
    CreateWorldFrame,
)
from dg_tools.utils import (
    stack_two_vectors,
    selec_vector,
    subtract_vec_vec,
    constVector,
)


def simulate(with_gui=True):
    #
    # setup and run simulation
    #
    time = 0
    sim_freq = 10000  # Hz
    ctrl_freq = 1000
    plan_freq = 1000 

    from dg_optitrack_sdk.dynamic_graph.entities import OptitrackClientEntity
    #Get mocap data
    mocap = OptitrackClientEntity("optitrack_entity")
    mocap.connect_to_optitrack("1049") # give desired body ID to track
    mocap.add_object_to_track("1049") # rigid body ID for biped
    # Zero the initial position from the vicon signal.
    base_posture_sin = mocap.signal("1049_position")    
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
    velocity = np.array([0, 0, 0, 0, 0, 0])
    biped_velocity = constVector(velocity, "")

    #load position and velocity data from txt file

    robot = get_bolt_robot(use_fixed_base=False, init_sliders_pose=4 * [1.0])
    print("sim robot: " + str(robot))
    p.resetDebugVisualizerCamera(1.3, 60, -35, (0.0, 0.0, 0.0))
    p.setTimeStep(1.0 / sim_freq)
    p.setRealTimeSimulation(0)

    # Update the initial state of the robot.
    q0 = np.matrix(BoltConfig.initial_configuration).T
    qdot = np.matrix(BoltConfig.initial_velocity).T
    # q0[0] = -0.1
    # q0[1] = 0.0
    #q0[2] = 0.536895 - 0.0649 #0.357222 - 0.064979
    q0[3] = 0.0018172
    q0[4] = -0.00820817
    q0[5] = 0.0750234
    q0[6] = 0.997146
    print(q0)

    # mocap: translation: [-3.83942 0.949264 0.536983]
    # rotation: [x:0.00281365, y:-0.00689991, z:0.0752908, w:0.997134 ]

    robot.reset_state(q0, qdot)
    ctrl = get_controller(is_real_robot=False)

    ctrl.plug(robot, *robot.base_signals())
    #print("base posture: " + str(base_posture_local_sin.value))
    #ctrl.plug(robot, base_posture_local_sin, biped_velocity)

    ctrl.trace()
    robot.start_tracer()

    # robot.run(1000)

    # robot.run(100,0.01)
    ctrl.set_kf(1)
    ctrl.start()
    robot.run(10000, 0.01)
    #TODO: use mocap signal from robot for run() 
    # print("after start")
    # from dynamic_graph import writeGraph
    # writeGraph("/tmp/my_graph.dot")
    # robot.run(1000,0.01)
    print("Finished normally!")
    ctrl.stop()
    robot.stop_tracer()


if __name__ == "__main__":
    simulate()
