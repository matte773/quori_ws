import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node


def generate_launch_description():
    # Define the path to the Quori robot's URDF or Xacro file
    quori_urdf_file = os.path.join(
        get_package_share_directory('quori_description'),
        'urdf',
        'quori.urdf'  # Or 'quori.xacro' if you're using a Xacro file
    )

    # Check if it's a Xacro file, and convert it to URDF format on-the-fly if necessary
    if quori_urdf_file.endswith('.xacro'):
        robot_desc = Command(['xacro ', quori_urdf_file])
    else:
        with open(quori_urdf_file, 'r') as infp:
            robot_desc = infp.read()

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'robot_description': robot_desc
            }],
        ),
    ])
