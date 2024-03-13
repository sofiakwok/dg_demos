"""
@package dg_demos
@file foot_circle.py
@author Elham Daneshmand
@license License BSD-3-Clause
@copyright Copyright (c) 2019, New York University and Max Planck Gesellschaft.
@date 2019-02-06
@brief Contains some usefull tools to setup a simulation.
"""

import os.path
import subprocess
from dynamic_graph import writeGraph


def write_current_control_graph(robot_name, script_name):
    #
    # Write the graph for debugging
    #
    script_name, _ = os.path.splitext(os.path.basename(script_name))
    graph_name = robot_name + "_" + script_name + "_graph"
    graph_dot_file = "/tmp/" + graph_name + ".dot"
    graph_pdf_file = "/tmp/" + graph_name + ".pdf"

    writeGraph(graph_dot_file)
    subprocess.call(
        "dot -Tpdf " + graph_dot_file + " > " + graph_pdf_file, shell=True
    )
