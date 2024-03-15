import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Path to the quori URDF file
    quori_urdf_path = os.path.join(
        get_package_share_directory('quori_description'),
        'urdf',
        'quori.urdf'
    )

    #Path to the quori mesh files
    quori_mesh_path = os.path.join(
        get_package_share_directory('quori_description'),
       'models', 'quori', 'meshes'
    )

    # Read the URDF file
    with open(quori_urdf_path, 'r') as file:
        robot_description_content = file.read()

    # Launch configuration variables specific to simulation
    x_pose = LaunchConfiguration('x_pose', default='0.0')
    y_pose = LaunchConfiguration('y_pose', default='0.0')

    # Declare the launch arguments
    declare_x_position_cmd = DeclareLaunchArgument(
        'x_pose', default_value='0.0',
        description='X position for spawning the robot')

    declare_y_position_cmd = DeclareLaunchArgument(
        'y_pose', default_value='0.0',
        description='Y position for spawning the robot')
    
    # Set GAZEBO_MODEL_PATH environment variable to use quori_mesh_path
    set_gazebo_model_path_cmd = SetEnvironmentVariable(
        name='GAZEBO_MODEL_PATH',
        value=quori_mesh_path  # Use the quori_mesh_path variable here
    )

    # Node to spawn the robot in Gazebo
    start_gazebo_ros_spawner_cmd = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'quori',
            '-topic', 'robot_description',
            '-x', x_pose,
            '-y', y_pose,
            '-z', '0.01'
        ],
        output='screen',
    )

    # Set the robot_description parameter from the URDF file content
    set_robot_description_cmd = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'use_sim_time': True, 'robot_description': robot_description_content}],
    )

    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_x_position_cmd)
    ld.add_action(declare_y_position_cmd)

    # Set GAZEBO_MODEL_PATH environment variable
    ld.add_action(set_gazebo_model_path_cmd)

    # Add the nodes to the launch description
    ld.add_action(set_robot_description_cmd)
    ld.add_action(start_gazebo_ros_spawner_cmd)

    return ld
