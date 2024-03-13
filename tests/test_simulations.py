"""
@package dg_demos
@file test_simulations.py
@author Maximilien Naveau
@license License BSD-3-Clause
@copyright Copyright (c) 2019, New York University and Max Planck Gesellschaft.
@date 2019-02-06
@brief This file contains dynamic graph tools specific to the robot Solo
"""

import os
import unittest
import importlib
from multiprocessing import Process
import dg_demos.teststand.simulations


def _run_simulation(script_module_name):
    """
    run in a different process in order to have a clean dynamic-graph
    """
    print("run_simulation")

    def _run_simulation_in_process(script_module_name):
        simulation_script = importlib.import_module(script_module_name)
        simulation_script.simulate(with_gui=False)
        return True

    p = Process(target=_run_simulation_in_process, args=(script_module_name,))
    p.start()
    p.join()
    return True


def _get_script_module_names(robot_name, ignore_files):
    # get the path to the simualtion module
    simulations_module_name = "dg_demos." + robot_name + ".simulations"
    simulations_module = importlib.import_module(
        simulations_module_name, package="dg_demos"
    )
    module_path = os.path.dirname(simulations_module.__file__)

    simulation_script_names = []
    for file in os.listdir(module_path):
        if file.endswith(".py") and file not in ignore_files:
            name, _ = os.path.splitext(file)
            simulation_script_names += [simulations_module_name + "." + name]

    return simulation_script_names


def test_simulations():
    robot_names = ["teststand"]
    ignore_files = ["__init__.py", "track_planned_motion.py"]
    for robot_name in robot_names:
        script_module_names = _get_script_module_names(
            robot_name, ignore_files
        )
        for script_module_name in script_module_names:
            yield _run_simulation, script_module_name


if __name__ == "__main__":
    unittest.main()
