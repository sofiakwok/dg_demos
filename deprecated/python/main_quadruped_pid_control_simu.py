import numpy as np
from pinocchio.utils import zero

from dg_demos.quadruped_pid_control import quadruped_pid_control
from dg_demos.prologue_quadruped import robot

config, controller_manager, pid = quadruped_pid_control(robot, nb_sliders=1)

q = zero(config.nb_joints)
dq = zero(config.nb_joints)
q_out = q.T.tolist()[0]
dq_out = dq.T.tolist()[0]
ddq = zero(config.nb_joints)
joint_torques = zero(config.nb_joints)
err_pid = 8 * (0.0,)

q[:] = np.asmatrix(robot.device.joint_positions.value).T
dq[:] = np.asmatrix(robot.device.joint_velocities.value).T

dt = config.dt

robot.device.slider_positions.value = (0.0, 1.0, 0.5, 0.75)


for i in range(4000):
    # fill the sensors
    robot.device.joint_positions.value = q.T.tolist()[0][:]
    robot.device.joint_velocities.value = dq.T.tolist()[0][:]

    # "Execute the dynamic graph"
    robot.device.executeGraph()

    #
    joint_torques[:] = np.asmatrix(robot.device.ctrl_joint_torques.value).T

    # integrate the configuration from the computed torques
    q = q + dt * dq + dt * dt * 0.5 * joint_torques / config.motor_inertia
    dq = dq + dt * joint_torques / config.motor_inertia

    if (i % 1000) == 0:
        # print "qref =     ", robot.pid_control.pose_controller.qRef.value
        # print ("q =        ",
        #        robot.pid_control.pose_controller.base6d_encoders.value[6:])
        # print "err_pid =  ", robot.pid_control.pose_controller.qError.value
        # print "currents = ", robot.device.ctrl_joint_torques.value
        pass


print
print "End of simulation"
print "qref =     ", pid.controller.qRef.value
print "q =        ",
print pid.controller.base6d_encoders.value[6:]
print "err_pid =  ", pid.controller.qError.value
print "currents = ", robot.device.ctrl_joint_torques.value
