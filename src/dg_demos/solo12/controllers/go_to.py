"""
"""

from robot_properties_solo.config import Solo12Config
from mim_control.dynamic_graph.go_to import GoTo


def get_controller():
    config = Solo12Config()
    ctrl = GoTo(config.nb_joints, prefix="solo12_")
    ctrl.set_pd_gains(Kp=5.0, Kd=0.05)  # nice gains for Solo12 MPI
    return ctrl


if "robot" in globals():
    ctrl = get_controller()
    ctrl.record_data()

    ctrl.plug(
        robot.device.joint_positions,
        robot.device.joint_velocities,
        robot.device.ctrl_joint_torques,
    )

    def angles():
        import numpy as np

        np.set_printoptions(suppress=True)
        print(np.array(robot.device.joint_positions.value).reshape(4, 3))

    print("#####")
    print("")
    print(
        "Please execute `ctrl.go_to(desired_joint_positions, nb_iteration)` function to move the joints."
    )
    print("")
    print("#####")
