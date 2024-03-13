import numpy as np
from dynamic_graph import plug

# this is a Finite impendance filter (see wikipedia ;))
from dynamic_graph.sot.core.fir_filter import FIRFilter_Vector_double
from dynamic_graph.sot.core.operator import (
    Multiply_double_vector,
    Selec_of_vector,
    Stack_of_vector,
    Substract_of_vector,
)

# sot core imports
from dynamic_graph.sot.core.vector_constant import VectorConstant

# get the controller manager to be able to use the
# sot-torque-control.pose_controller.
from dynamic_graph.sot.torque_control.control_manager import ControlManager

# This entity is the pd controller.
from dynamic_graph.sot.torque_control.position_controller import (
    PositionController,
)


class PID:
    """
    This class has for sole purpose to contain the pointers of all entities
    related to the pid controller.
    """

    def __init__(self):
        pass


def construct_and_initialize_the_control_manager(config):
    """
    Setup the controller manager which creates some global variables in a shape
    of singletons. This steps is necessary in order to use the eco-systems of
    Andrea Del Prete (sot-torque-control).
    """
    # then create the controller manager.
    controller_manager = ControlManager("ctrl_man")
    # initialize the control mamager so all singletons are initialized.
    controller_manager.init(config.dt, config.urdf_path, config.robot_name)
    return controller_manager


def construct_the_pid_controller(pid, config):
    # create the pid control
    controller = PositionController("PIDController")
    # reset and redimension all input signals:
    controller.base6d_encoders.value = (config.nb_joints + 6) * (0,)
    controller.jointsVelocities.value = config.nb_joints * (0,)
    controller.qRef.value = config.nb_joints * (0,)
    controller.dqRef.value = config.nb_joints * (0,)
    controller.Kp.value = config.nb_joints * (0,)
    controller.Kd.value = config.nb_joints * (0,)
    controller.Ki.value = config.nb_joints * (0,)
    return controller


def plug_the_current_state_to_the_pid_controller(robot, pid, config):
    """
    In this method we acquire the current state (whole body configuration and
    velocity) and feed it to the pid control.
    """
    # Feeding the current configuration
    ###################################

    # The PID object needs a 6+N vector, 6 being the DoF of the free flyer and
    # N the number of joints. Hence first we create a fake free flyer.
    #
    # In order to do so we create an entity that will concatenate 2 vectors.
    # the first one is gonna be a constant 6D vector and the other one the
    # encoder values
    #
    # The entity will put "sin1" on top of "sin2" and construct "sout" from
    # them. So "sin1" stands for "Signal Input 1", "sin2" for "Signal Input 2",
    # and "sout" for "Signal output".
    pid.base6d_encoders = Stack_of_vector("pid_base6d_encoders")
    # here we just say that we slice sin1 from index 0 to index 6.
    pid.base6d_encoders.selec1(0, 6)
    # idem
    pid.base6d_encoders.selec2(0, config.nb_joints)
    # here we set sin1 to a null vector of dim 6.
    pid.base6d_encoders.sin1.value = 6 * (0.0,)
    # here we plug the encoders to sin2.
    plug(robot.device.joint_positions, pid.base6d_encoders.sin2)
    # And finally we plug the concatenated vector in the pid controller
    plug(pid.base6d_encoders.sout, pid.controller.base6d_encoders)

    # Feeding the current joint velocity
    ####################################
    plug(robot.device.joint_velocities, pid.controller.jointsVelocities)

    return pid


def plug_4_sliders_to_the_pid_controller(robot, pid, config):
    """
    In this part we define the desired position the joints must reach from
    analogue input. These inputs are 4 sliders (linear potentiometers).
    Because we have only 4 data for 8 joints we will have decompose the signal
    in 4 different signal and then concatenate them to get a vector of dim 8.
    """

    # Center the sliders. The value of the sliders is in [0, 1].
    # in our case we prefer [-0.5, 0.5]. In order to do this we use this
    # Substract_of_vector entity that will subrstract sin2 from sin1
    # We substract 0.5 to all element of the sliders.
    pid.centered_slider = Substract_of_vector("centered_slider")
    plug(robot.device.slider_positions, pid.centered_slider.sin1)
    pid.centered_slider.sin2.value = [0.5, 0.5, 0.5, 0.5]

    # Filter the centered sliders
    # Hence we create a "Finite Impendance Response" filter.
    # the filter is in the following form:
    # out = sum_{i=0}^{N} data_i * alpha_i
    #   - the data_i are the collected elements, their number grows until the
    #     size of the filter is reached.
    #   - the alpha_i are the gains of the filter, they are defined by the
    #     method "setElement(index, value)"
    # in the end here we do an averaging filter on 200 points.
    pid.slider_filtered = FIRFilter_Vector_double("slider_fir_filter")
    filter_size = 200
    pid.slider_filtered.setSize(filter_size)
    for i in range(filter_size):
        pid.slider_filtered.setElement(i, 1.0 / float(filter_size))
    # we plug the centered sliders output to the input of the filter.
    plug(pid.centered_slider.sout, pid.slider_filtered.sin)

    # Now we want the slider to be in [-qref, qref]
    # So we multiply all sliders by a constant which is max_qref.
    pid.scaled_slider = Multiply_double_vector("scaled_slider")
    pid.scaled_slider.sin1.value = config.max_qref
    plug(pid.slider_filtered.sout, pid.scaled_slider.sin2)

    # Now we need to solve the problem that we have 4 sliders for 8 motors.
    # Hence we will map each slider value to 2 motors.
    for i, leg in enumerate(["fr", "hr", "hl", "fl"]):
        # first of all we define the references for the hip joint:
        pid.__dict__[leg + "_hip_qref"] = Selec_of_vector(leg + "_hip_qref")
        pid.__dict__[leg + "_hip_qref"].selec(i, i + 1)
        plug(pid.scaled_slider.sout, pid.__dict__[leg + "_hip_qref"].sin)

        # Then we define the reference for the knee joint. We want the knee to move
        # twice as much as the hip and on the opposite direction
        pid.__dict__[leg + "_knee_qref"] = Multiply_double_vector(
            leg + "_knee_qref"
        )
        pid.__dict__[leg + "_knee_qref"].sin1.value = -2.0
        plug(
            pid.__dict__[leg + "_hip_qref"].sout,
            pid.__dict__[leg + "_knee_qref"].sin2,
        )

        # now we need to stack the signals 2 by 2:
        pid.__dict__[leg + "_qref"] = Stack_of_vector(leg + "_qref")
        pid.__dict__[leg + "_qref"].selec1(0, 1)
        pid.__dict__[leg + "_qref"].selec2(0, 1)
        # first element is the hip
        plug(
            pid.__dict__[leg + "_hip_qref"].sout,
            pid.__dict__[leg + "_qref"].sin1,
        )
        # second element is the knee
        plug(
            pid.__dict__[leg + "_knee_qref"].sout,
            pid.__dict__[leg + "_qref"].sin2,
        )

    # We stack fr and hr legs together
    pid.fl_hr_qref = Stack_of_vector("fl_hr_qref")
    pid.fl_hr_qref.selec1(0, 2)
    pid.fl_hr_qref.selec2(0, 2)
    plug(pid.fl_qref.sout, pid.fl_hr_qref.sin1)
    plug(pid.hr_qref.sout, pid.fl_hr_qref.sin2)
    # We stack hl and fl legs together
    pid.hl_fr_qref = Stack_of_vector("hl_fr_qref")
    pid.hl_fr_qref.selec1(0, 2)
    pid.hl_fr_qref.selec2(0, 2)
    plug(pid.hl_qref.sout, pid.hl_fr_qref.sin1)
    plug(pid.fr_qref.sout, pid.hl_fr_qref.sin2)
    # Now we can stack the four leg together
    pid.qRef = Stack_of_vector("qref")
    pid.qRef.selec1(0, 4)
    pid.qRef.selec2(0, 4)
    plug(pid.fl_hr_qref.sout, pid.qRef.sin1)
    plug(pid.hl_fr_qref.sout, pid.qRef.sin2)
    # finally we plug the qref in the controller.
    plug(pid.qRef.sout, pid.controller.qRef)

    return pid


def plug_1_sliders_to_the_pid_controller(robot, pid, config):
    """
    In this part we define the desired position the joints must reach from
    analogue input. These inputs are 4 sliders (linear potentiometers).
    Because we have only 4 data for 8 joints we will have decompose the signal
    in 4 different signal and then concatenate them to get a vector of dim 8.
    """

    # Center the sliders. The value of the sliders is in [0, 1].
    # in our case we prefer [-0.5, 0.5]. In order to do this we use this
    # Substract_of_vector entity that will subrstract sin2 from sin1
    # We substract 0.5 to all element of the sliders.
    pid.centered_slider = Substract_of_vector("centered_slider")
    plug(robot.device.slider_positions, pid.centered_slider.sin1)
    pid.centered_slider.sin2.value = [0.5, 0.5, 0.5, 0.5]

    # Filter the centered sliders
    # Hence we create a "Finite Impendance Response" filter.
    # the filter is in the following form:
    # out = sum_{i=0}^{N} data_i * alpha_i
    #   - the data_i are the collected elements, their number grows until the
    #     size of the filter is reached.
    #   - the alpha_i are the gains of the filter, they are defined by the
    #     method "setElement(index, value)"
    # in the end here we do an averaging filter on 200 points.
    pid.slider_filtered = FIRFilter_Vector_double("slider_fir_filter")
    filter_size = 200
    pid.slider_filtered.setSize(filter_size)
    for i in range(filter_size):
        pid.slider_filtered.setElement(i, 1.0 / float(filter_size))
    # we plug the centered sliders output to the input of the filter.
    plug(pid.centered_slider.sout, pid.slider_filtered.sin)

    # Now we want the slider to be in [-qref, qref]
    # So we multiply all sliders by a constant which is max_qref.
    pid.scaled_slider = Multiply_double_vector("scaled_slider")
    pid.scaled_slider.sin1.value = config.max_qref
    plug(pid.slider_filtered.sout, pid.scaled_slider.sin2)

    # Now we need to solve the problem that we want 1 sliders for 8 motors.
    # Hence we will map slider_a value to the 8 motors.
    # first let us get the value for all hips from slider A.
    pid.hip_qRef = Selec_of_vector("hip_qRef")
    pid.hip_qRef.selec(0, 1)  # this is slider A
    plug(pid.scaled_slider.sout, pid.hip_qRef.sin)
    # Then we define the reference for the knee joint. We want the knee to move
    # twice as much as the hip and on the opposite direction
    pid.knee_qRef = Multiply_double_vector("knee_qRef")
    pid.knee_qRef.sin1.value = -2.0
    plug(pid.hip_qRef.sout, pid.knee_qRef.sin2)

    for i, leg in enumerate(["fr", "hr", "hl", "fl"]):
        # now we need to stack the signals 2 by 2:
        pid.__dict__[leg + "_qref"] = Stack_of_vector(leg + "_qref")
        pid.__dict__[leg + "_qref"].selec1(0, 1)
        pid.__dict__[leg + "_qref"].selec2(0, 1)
        # first element is the hip
        plug(pid.hip_qRef.sout, pid.__dict__[leg + "_qref"].sin1)
        # second element is the knee
        plug(pid.knee_qRef.sout, pid.__dict__[leg + "_qref"].sin2)

    # We stack fr and hr legs together
    pid.fl_hr_qref = Stack_of_vector("fl_hr_qref")
    pid.fl_hr_qref.selec1(0, 2)
    pid.fl_hr_qref.selec2(0, 2)
    plug(pid.fl_qref.sout, pid.fl_hr_qref.sin1)
    plug(pid.hr_qref.sout, pid.fl_hr_qref.sin2)
    # We stack hl and fl legs together
    pid.hl_fl_qref = Stack_of_vector("hl_fl_qref")
    pid.hl_fl_qref.selec1(0, 2)
    pid.hl_fl_qref.selec2(0, 2)
    plug(pid.hl_qref.sout, pid.hl_fl_qref.sin1)
    plug(pid.fl_qref.sout, pid.hl_fl_qref.sin2)
    # Now we can stack the four leg together
    pid.qRef = Stack_of_vector("qref")
    pid.qRef.selec1(0, 4)
    pid.qRef.selec2(0, 4)
    plug(pid.fl_hr_qref.sout, pid.qRef.sin1)
    plug(pid.hl_fl_qref.sout, pid.qRef.sin2)
    # finally we plug the qref in the controller.
    plug(pid.qRef.sout, pid.controller.qRef)

    return pid


def plug_the_gains_state_to_the_pid_controller(pid, config):
    # Set the gains to the desired values:
    pid.controller.Kp.value = config.nb_joints * (config.kp,)
    pid.controller.Kd.value = config.nb_joints * (config.kd,)
    pid.controller.Ki.value = config.nb_joints * (config.ki,)
    return pid
