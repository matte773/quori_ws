from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command, FindExecutable, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Xacro processing to generate URDF
    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name='xacro')]),
        ' ',
        PathJoinSubstitution([FindPackageShare('quori_description'), 'urdf', 'quori.xacro'])
    ])
    robot_description = {'robot_description': robot_description_content}

    # Controller configurations
    quori_control_holo_yaml = PathJoinSubstitution(
        [FindPackageShare('quori_controller'), 'config', 'quori_control_holo.yaml']
    )
    quori_controller_params_yaml = PathJoinSubstitution(
        [FindPackageShare('quori_controller'), 'config', 'quori_controller_params.yaml']
    )
    calibration_yaml = PathJoinSubstitution(
        [FindPackageShare('quori_controller'), 'config', 'calibration.yaml']
    )

    return LaunchDescription([
        # Declare the robot description
        ExecuteProcess(
            cmd=['ros2', 'param', 'set', '/robot_description', robot_description],
            shell=True
        ),

        # Load controller configurations
        ExecuteProcess(
            cmd=['ros2', 'param', 'load', '/quori', quori_control_holo_yaml],
            shell=True
        ),

        # Quori Controller Node
        Node(
            package='quori_controller',
            executable='node',
            namespace='/quori',
            output='screen',
            parameters=[quori_controller_params_yaml, calibration_yaml],
            remappings=[('/quori/joint_states', '/joint_states')]
        ),

        # Controller Spawner
        Node(
            package='controller_manager',
            executable='spawner',
            namespace='/quori',
            output='screen',
            arguments=['joint_trajectory_controller', 'joint_state_controller', 'base_controller']
        ),

        # Robot State Publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen'
        ),

        # Base Serial Node
        Node(
            package='rosserial_python',
            executable='serial_node.py',
            output='screen',
            respawn=True,
            parameters=[{
                'port': '/dev/quori/base',
                'baud': 115200
            }]
        )
    ])

