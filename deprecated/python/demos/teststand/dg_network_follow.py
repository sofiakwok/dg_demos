# Copyright 2019 Max Planck Society. All rights reserved.
# Julian Viereck

import numpy as np

from dynamic_graph import plug
from dynamic_graph.sot.core.control_pd import ControlPD
from dynamic_graph.sot.core.joint_limitator import JointLimitator

from py_dg_tools.utils import (
    constVector,
    subtract_vec_vec,
    stack_two_vectors,
    add_vec_vec,
    mul_double_vec,
)
from dynamic_graph_manager.dg_tools import (
    HistoryRecorder,
    PreviousValue,
    MemoryReplay,
    Upsampler,
)

# from dg_blmc_robots.teststand import get_teststand_robot

from dynamic_graph_manager.dg_tensorflow import TFRunnerEntity

from learn_to_control.trajectory import build_trajectory_from_target_points
from dynamic_graph.sot.core.reader import Reader

###
# Compute the desired trajectory and window.
N_ACTION_HISTORY = 5
N_STATE_HISTORY = 5

N_DES_WINDOW = 9

da = 2
dq = 2

arr2tup = lambda a: tuple([tuple(b) for b in a])


class InputDesiredTrajectoryGenerator(object):
    def __init__(
        self,
        desired_traj,
        history_size=4,
        history_next_size=None,
        include_future_point=None,
    ):

        self.desired_traj = desired_traj

        if history_next_size is None:
            history_next_size = history_size
            history_prev_size = history_size
        else:
            history_next_size = history_next_size
            history_prev_size = history_size

        self.window_O = (-history_prev_size, history_next_size + 1)

        self.window_current = (0, 1)

        self.include_future_point = include_future_point
        if include_future_point is not None:
            self.window_future_point = (
                include_future_point,
                include_future_point + 1,
            )
        else:
            self.window_future_point = (0, 0)

    def generate_window(self):
        res = []
        for it in range(self.desired_traj.T):
            res.append(
                np.vstack(
                    [
                        self.desired_traj.trajectory_slice(it, self.window_O),
                        self.desired_traj.trajectory_slice(
                            it, self.window_future_point
                        ),
                    ]
                ).reshape(-1)
            )
        return np.array(res)


history_next_size = 4
history_prev_size = 4
include_future_point = 20

# Create a desired trajectory.
traj = build_trajectory_from_target_points(
    75, np.array([0, 0]), np.array([1.0, -2.0])
)
gen = InputDesiredTrajectoryGenerator(
    traj, history_next_size, history_prev_size, include_future_point
)

# Create trajectory entity to reply the desired trajectory window information
traj_desired_window = MemoryReplay("traj_desired_window")
traj_desired_window.init(arr2tup(gen.generate_window().tolist()))

# Create desired trajectory entity.
traj_desired_q = MemoryReplay("traj_desired_q")
traj_desired_q.init(arr2tup(gen.desired_traj.states))

# # Get the robot corresponding to the quadruped.
# if not 'robot' in globals():
#     robot = get_teststand_robot(fixed_slider=True)
#
# q, v = robot.wrapper.get_state()
# q[0] = 0.4
# robot.reset_state(q, v)

# Compute the error between current and desired q:
q_error = subtract_vec_vec(traj_desired_q.sout, robot.device.joint_positions)

################################################################################
# Action history.
action_history = HistoryRecorder("action_history")
action_history.init(N_ACTION_HISTORY)

action_previous = PreviousValue("action_previous")
action_previous.init(da)

plug(action_previous.sprev, action_history.sin)

################################################################################
# State history.
q_error_history = HistoryRecorder("q_error_history")
q_error_history.init(N_STATE_HISTORY)

plug(q_error, q_error_history.sin)

# Joint the input signals together.
vecs = [
    (traj_desired_window.sout, dq * N_DES_WINDOW + dq),
    (action_history.sout, da * N_ACTION_HISTORY),
    (q_error_history.sout, dq * N_STATE_HISTORY),
    (robot.device.joint_positions, dq),
]
prev_vec = vecs[0][0]
prev_size = vecs[0][1]

for (sig, size) in vecs[1:]:
    prev_vec = stack_two_vectors(prev_vec, sig, prev_size, size)
    prev_size += size

################################################################################
# Create the position controller
pd = ControlPD("PDController")
# setup the gains
pd.Kp.value = 2 * (5.0,)
pd.Kd.value = 2 * (0.1,)

pd.desired_velocity.value = 2 * (0.0,)

# plug the desired quantity signals in the pd controller.
plug(robot.device.joint_positions, pd.position)
plug(robot.device.joint_velocities, pd.velocity)

################################################################################
# Pass the network output through the recorder.
# Load the tensorflow network.
policy = TFRunnerEntity("runner")
policy.load_saved_model("saved_policy", "EVALUATION")
policy.add_input("sin", "input/Ob")
policy.add_output("sout", "output/action")

policy.sin.value = 42 * [0.0]

# Plug an upsampler to run the neural network and all the dependent entities
# at lower sampling rate.
upsampler = Upsampler("upsampler")
upsampler.init(5)

# plug(policy.sout, action_previous.sin)
# plug(action_previous.sout, upsampler.sin)

plug(policy.sout, upsampler.sin)

relative_desired_pos = mul_double_vec(0, upsampler.sout)


def warmup_upsampler():
    relative_desired_pos.recompute(robot.device.joint_positions.time)
    print(relative_desired_pos.value)


warmup_upsampler()

# plug(relative_desired_pos, robot.device.ctrl_joint_torques)

# plug(relative_desired_pos, robot.device.ctrl_joint_torques)
# plug(add_vec_vec(relative_desired_pos, relative_desired_pos), robot.device.ctrl_joint_torques)
# plug(add_vec_vec(relative_desired_pos, robot.device.joint_positions), robot.device.ctrl_joint_torques)

plug(
    add_vec_vec(robot.device.joint_positions, relative_desired_pos),
    pd.desired_position,
)


def warmup_pdcontrol():
    print("warmup pd begin")
    pd.control.recompute(robot.device.joint_positions.time)
    print("finish warmup pdcontrol", pd.control.value)


warmup_pdcontrol()

# plug(pd.control, robot.device.ctrl_joint_torques)


def plug_network():
    # plug(prev_vec, policy.sin)

    # relative_desired_pos = mul_double_vec(0, action_previous.sout)
    # relative_desired_pos = mul_double_vec(0, policy.sout)

    # Add the current position to the desired offset as target position.

    plug(pd.control, robot.device.ctrl_joint_torques)
