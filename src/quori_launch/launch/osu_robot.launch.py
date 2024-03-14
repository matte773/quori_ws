import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    # Define paths to the packages and launch files
    quori_controller_launch_path = os.path.join(
        get_package_share_directory('quori_controller'),
        'launch',
        'quori_control_holo.launch.py'
    )

    quori_teleop_launch_path = os.path.join(
        get_package_share_directory('quori_teleop'),
        'launch',
        'quori_teleop.launch.py'
    )

    astra_ros_launch_path = os.path.join(
        get_package_share_directory('astra_ros'),
        'launch',
        'default.launch.py'
    )

    respeaker_ros_launch_path = os.path.join(
        get_package_share_directory('respeaker_ros'),
        'launch',
        'sample_respeaker.launch.py'
    )

    quori_launch_path = os.path.join(
        get_package_share_directory('quori_launch'),
        'launch',
        'mapping.launch.py'
    )

    # Start creating the LaunchDescription
    ld = LaunchDescription()

    # Include the necessary launch files
    ld.add_action(IncludeLaunchDescription(PythonLaunchDescriptionSource(quori_controller_launch_path)))
    ld.add_action(IncludeLaunchDescription(PythonLaunchDescriptionSource(quori_teleop_launch_path)))
    ld.add_action(IncludeLaunchDescription(PythonLaunchDescriptionSource(astra_ros_launch_path)))
    ld.add_action(IncludeLaunchDescription(PythonLaunchDescriptionSource(respeaker_ros_launch_path)))
    ld.add_action(IncludeLaunchDescription(PythonLaunchDescriptionSource(quori_launch_path)))

    # Launch RViz
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', '/opt/quori/src/quori_osu/launch/quori.rviz'],
    )
    ld.add_action(rviz_node)

    return ld

