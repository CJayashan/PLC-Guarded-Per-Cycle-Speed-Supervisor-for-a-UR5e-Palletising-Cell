from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution, FindExecutable
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    desc_pkg = get_package_share_directory('ur5e_cell_description')
    bringup_pkg = get_package_share_directory('ur5e_cell_bringup')

    # Build robot_description by expanding our top-level xacro (vendor injects FakeSystem inside)
    xacro_file = PathJoinSubstitution([desc_pkg, 'urdf', 'ur5e_cell.urdf.xacro'])
    robot_description_content = ParameterValue(
        Command([FindExecutable(name='xacro'), ' ', xacro_file]),
        value_type=str
    )
    robot_description = {'robot_description': robot_description_content}

    # Robot State Publisher (publishes TF & /robot_description)
    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    )

    # ros2_control controller_manager (reads robot_description; loads controllers.yaml)
    cm = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            robot_description,  # kept for Humble compatibility (may log deprecation warning)
            PathJoinSubstitution([bringup_pkg, 'config', 'controllers.yaml'])
        ],
        output='screen'
    )

    # Spawners: JSB first, then JTC
    spawner_jsb = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        output='screen'
    )
    spawner_jtc = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_trajectory_controller', '--controller-manager', '/controller_manager'],
        output='screen'
    )

    # Small delay before spawning helps when laptops are under load
    delayed_spawners = TimerAction(period=1.0, actions=[spawner_jsb, spawner_jtc])

    return LaunchDescription([rsp, cm, delayed_spawners])
