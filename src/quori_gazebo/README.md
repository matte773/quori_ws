# ROS 2 quori_gazebo

## Overview

This package launches the quori model in gazebo.

## Prerequisites



### Supported ROS 2 Distributions

- Humble Hawksbill

## Installation


### Building from Source

#### Dependencies

quori_description

#### Building

# Source ROS 2 environment
source /opt/ros/humble/setup.bash

# Build your package
cd ~/ros2_ws
colcon build --packages-select [your_package_name]

# Source the local workspace
source install/setup.bash
source ~/ros2_ws/install/setup.bash

#Launch command
ros2 launch quori_gazebo empty_world.launch.py #launches quori in an empty world

#Gazebo Log Files Location
cd ~/.gazebo

#Command to Change from .xacro to .urdf
xacro quori.xacro > quori.urdf



