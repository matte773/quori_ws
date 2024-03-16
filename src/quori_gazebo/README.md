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

#set Gazebo Path
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/[your path]/quori_ws/src/quori_description/models

# Source ROS 2 environment
source /opt/ros/humble/setup.bash

# Build your package
cd ~/ros2_ws
colcon build --packages-select [your_package_name]

# Source the local workspace
source install/setup.bash
source ~/ros2_ws/install/setup.bash

#Launch command
#launches quori in an empty world

ros2 launch quori_gazebo empty_world.launch.py 

#Gazebo Log Files Location
cd ~/.gazebo

#Command to Change from .xacro to .urdf
xacro quori.xacro > quori.urdf



