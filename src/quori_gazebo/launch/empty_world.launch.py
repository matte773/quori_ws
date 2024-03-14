#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch_ros.actions import Node

def generate_launch_description():
    # Define the directory of the 'quori_gazebo' package
    quori_launch_dir = os.path.join(get_package_share_directory('quori_gazebo'), 'launch')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    # Launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    x_pose = LaunchConfiguration('x_pose', default='0.0')
    y_pose = LaunchConfiguration('y_pose', default='0.0')

    # Specify the world file location
    world = os.path.join(
        get_package_share_directory('quori_gazebo'),
        'worlds',
        'empty.world'  # Update this to the correct world file for Quori
    )

    # Gazebo server launch command
    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={'world': world}.items()
    )

    # Gazebo client launch command
    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
        )
    )

    # Robot state publisher command
    robot_state_publisher_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(quori_launch_dir, 'robot_state_publisher.launch.py')  # Update this path if necessary
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    # Command to spawn the Quori robot in Gazebo
    spawn_quori_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(quori_launch_dir, 'spawn_quori.launch.py')  # Update this path if necessary
        ),
        launch_arguments={
            'x_pose': x_pose,
            'y_pose': y_pose
        }.items()
    )

    

    ld = LaunchDescription()

    # Declare launch arguments
    ld.add_action(DeclareLaunchArgument('x_pose', default_value='0.0', description='Initial x pose'))
    ld.add_action(DeclareLaunchArgument('y_pose', default_value='0.0', description='Initial y pose'))

    # Add the commands to the launch description
    ld.add_action(gzserver_cmd)
    ld.add_action(gzclient_cmd)
    ld.add_action(robot_state_publisher_cmd)
    ld.add_action(spawn_quori_cmd)

    return ld
