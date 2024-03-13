from dg_demos.quadruped_pid_control import quadruped_pid_control

config, controller_manager, pid = quadruped_pid_control(robot, nb_sliders=1)

__all__ = ["config", "controller_manager", "pid"]
