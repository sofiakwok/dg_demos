# Reactive Stepper for solo12

## 1/ Installation procedure

We assume here you have installed the standard dependencies of the machines-in-motion organization:
- dynamic-graph(-python)
- pinocchio
- ROS2
- treep
- colcon
- ...

In order to install the reactive stepper controller you will need all the package for running Solo12 with dynamic-graph plus the controllers themselves:

```bash
mkdir ~/devel
cd ~/devel
git clone git@github.com:machines-in-motion/treep_machines_in_motion.git
treep --clone DG_SOLO
treep --clone REACTIVE_PLANNERS
```

Then build the packages we use colcon:

```bash
source /opt/openrobots/setup.bash
source /opt/ros/dashing/setup.bash
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
```

Finally activate your own workspace:

```bash
source ~/devel/workspace/install/setup.bash
```

## 2/ Standard execution of the demo

In order to start the demo you must do the following:

- Calibrate the motion-capture system.

- Powering on the robot:
    - switch on both power supply on the desk,
    - make sure the (hard) e-stop and the (soft, silder box) e-stop are disarmed,

note: for a power cycle afterwards just arm/disarm the (hard) e-stop.

### 2.1/ Using ros2 launch

- terminal 1: start the robot
    ```bash
    ros2 run --prefix="sudo -E env LD_LIBRARY_PATH=$LD_LIBRARY_PATH PYTHONPATH=$PYTHONPATH" solo dg_main_solo12  dgm_parameters_solo12_mpi_is.yaml
    ```

- terminal 2: Start the demo:
    ```bash
    ros2 launch dg_demos solo12_reactive_stepper_gamepad.launch.py
    ```
    Two termimals will spawn, one is the DynamicGraphMnager python client and the other is the demos sequencer.
    In the demo sequencer terminal you need to press enter to go to the next step of the demo.
    All steps are explained in the terminal.
    In essence it boils down to perform the operations below.

### 2.2/ Doing all of it by hand:

In terms of software you need to run the following:

- terminal 1: start the robot
    ```bash
    ros2 run --prefix="sudo -E env LD_LIBRARY_PATH=$LD_LIBRARY_PATH PYTHONPATH=$PYTHONPATH" solo  dg_main_solo12  dgm_parameters_solo12_mpi_is.yaml
    ```

- terminal 2: Calibrate the robot
    ```bash
    source /opt/openrobots/setup.bash
    source ~/devel/workspace/install/setup.bash
    ros2 service call /dynamic_graph_manager/calibrate_joint_position mim_msgs/srv/JointCalibration
    # wait until finished
    ```

- terminal 3: Load the graph
    ```bash
    source /opt/openrobots/setup.bash
    source ~/devel/workspace/install/setup.bash
    cd ~/devel/workspace/src/dg_demos/src/dg_demos/solo12/controllers
    ros2 run dynamic_graph_manager remote_python_client reactive_stepper.py
    ```
    This should display:
    ```bash
    [INFO] [RosPythonInterpreterClient]: Waiting for service /dynamic_graph_manager/run_python_command ...
    [INFO] [RosPythonInterpreterClient]: Successfully connected to /dynamic_graph_manager/run_python_command
    [INFO] [RosPythonInterpreterClient]: Waiting for service /dynamic_graph_manager/run_python_file ...
    [INFO] [RosPythonInterpreterClient]: Successfully connected to /dynamic_graph_manager/run_python_file
    File parsed
    print('File parsed')
    Output:File parsed


    Interacting with remote python server.
    >>>
    ```
    Here you can interact with the controller and start the demo.

- terminal 4: Start the gamepad controller

    ```bash
    source /opt/openrobots/setup.bash
    source ~/devel/workspace/install/setup.bash
    ros2 run dg_demos  gamepad_publisher.py
    ```
    the terminal won't display anything if not errors occurs.

- terminal 3: Use the PD mode for the initial configuration.
    ```bash
    >>> go_pd()
    ```

- **note: make sure all sliders are at 0 !!!**

- terminal 2: Start the graph
    ```bash
    ros2 service call /dynamic_graph_manager/start_dynamic_graph std_srvs/srv/Empty
    ```

- Use silder B for bending the legs. The slider A is widening the stance and C and D are not used.

- Put the robot on the floor **ALIGNED with the MOCAP frame!!!**

- terminal 3: Start the stepper conrtoller

    ```bash
    >>> go_stepper()
    ```

- terminal 3: Start/Stop stepping
    ```bash
    >>> ctrl.start()
    >>> ctrl.stop()
    ```

- Use the gamepad to control the robot:
    - Left joystick inputs a linear velocity in the base frame.
    - The triggers (LT, RT) inputs a yaw velocity.

## Enjoy and be carefull! a hand always on the (hard) e-stop.
