dg_demos
--------

### What is it

This package contains the robot demos or the experiments done during research
that needs execution on a real/simulated hardware. All these demos must be
composed of configuration files or python scripts. This makes this package
installable everywhere.

The dependencies for each demos should be described inside a readme files!!!!

The execution of each demos must be done via minimal operator intervention.

### Authors

- Maximilien Naveau
- Julian Viereck

### Copyrights

Copyright (c) 2019, New York University and Max Planck Gesellschaft.

### License

License BSD-3-Clause

### Installation:

We usually use the multipurpose super build `colcon` in oder to build our
workspaces. Otherwize this is a standard python package.

    `colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release`

### Usage:

checkout the roslaunch folder:
all the demo are available there!

### Documentation:

See demo and unit tests for more information on the API.
Doxygen informations are available by calling:

	`catkin_make -DBUILD_DOCUMENTATION=ON`
