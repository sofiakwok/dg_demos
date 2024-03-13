globals()["robot"] = 1

# import the controller
from dg_demos.solo.controllers.pd_controller import PDController

# import the simulated robot
from robot_properties_solo.config import SoloConfig
from dg_blmc_robots.solo.solo_bullet import get_robot

#
# setup and run simulation
#
robot = get_robot(use_fixed_base=False)

# load controller
ctrl = PDController("solo8")
ctrl.follow_slider()
ctrl.plug(robot)
# init_pos(ctrl)

# get the robot config
config = SoloConfig()
# Update the initial state of the robot.
robot.reset_state(config.q0, config.v0)

# run the Simulation
robot.run(5000, 1.0 / 60.0)
