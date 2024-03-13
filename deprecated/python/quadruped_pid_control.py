# local imports specific to the quadruped
from dynamic_graph import plug
from py_robot_properties_quadruped.config import QuadrupedConfig

from pid_controller import (
    PID,
    construct_and_initialize_the_control_manager,
    construct_the_pid_controller,
    plug_1_sliders_to_the_pid_controller,
    plug_4_sliders_to_the_pid_controller,
    plug_the_current_state_to_the_pid_controller,
    plug_the_gains_state_to_the_pid_controller,
)


def quadruped_pid_control(robot, nb_sliders=1):
    """
    This program uses the dynamic graph framework to create an instance of a PID
    controller on all present joint. The reference of the joints are given by
    an input signal from the device as well.
    :return:
    """
    # Manages arguments
    description_str = (
        "This program uses the dynamic graph framework to create an instance \n"
        "of a PID controller on all present joint. The reference of the \n"
        "joints are given by an input signal from the device as well.\n"
    )
    print(description_str)

    # create a small object that for only purpose to store the pointer to the
    # different created entities. This way the pid controllers related object
    # are localised in pid.XXXX
    pid = PID()

    # first of all get some configuration from the robot
    config = QuadrupedConfig()

    # here we initialize the eco-system of the sot-torque-control package.
    controller_manager = construct_and_initialize_the_control_manager(config)

    # construct the pid controller entity.
    pid.controller = construct_the_pid_controller(pid, config)

    # The pid control plugging is done via several steps. In all these steps
    # the idea is to feed the PD controller with the requiered data like the
    # gains, the current state and teh desired state.
    # Step1:
    pid = plug_the_current_state_to_the_pid_controller(robot, pid, config)
    # Step2:
    # Here we have several ways of getting the information from the sliders
    # So we put a small state machine.
    if nb_sliders == 1:
        pid = plug_1_sliders_to_the_pid_controller(robot, pid, config)
    else:
        pid = plug_4_sliders_to_the_pid_controller(robot, pid, config)
    # Step3:
    pid = plug_the_gains_state_to_the_pid_controller(pid, config)
    # Step4:initialize the pid controller after signal are plugged
    pid.controller.init(config.dt, config.robot_name)

    # plug the ouput of the pid to the controls
    plug(pid.controller.pwmDes, robot.device.ctrl_joint_torques)

    # get the info on the tracking error
    robot.device.after.addSignal("{0}.qError".format(pid.controller.name))

    return config, controller_manager, pid
