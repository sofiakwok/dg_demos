import sys
import launch
import launch.actions
import launch.substitutions
import launch_ros.actions


def generate_launch_description():
    return launch.LaunchDescription(
        [
            launch_ros.actions.Node(
                package="joy",
                node_executable="joy_node",
            ),
            launch_ros.actions.Node(
                package="dg_demos",
                node_executable="joy_forwarder.py",
            ),
            launch_ros.actions.Node(
                package="dg_demos",
                node_executable="solo12_reactive_stepper_gamepad_sequencer.py",
                output="screen",
                prefix=[
                    'xterm -geometry 80x20+100+100 -fa "Ubuntu Mono" -fs 14 -hold -e'
                ],
            ),
            launch_ros.actions.Node(
                package="dynamic_graph_manager",
                node_executable="remote_python_client",
                output="screen",
                prefix=[
                    'xterm -geometry 80x20+100+700 -fa "Ubuntu Mono" -fs 14 -hold -e'
                ],
            ),
            # launch_ros.actions.Node(
            #     package="solo",
            #     node_executable="dg_main_solo12",
            #     output="screen",
            #     prefix=['xterm -fa "Ubuntu Mono" -fs 14 -hold -e "sudo -E env LD_LIBRARY_PATH=$LD_LIBRARY_PATH PYTHONPATH=$PYTHONPATH""'],
            # )
        ]
    )
