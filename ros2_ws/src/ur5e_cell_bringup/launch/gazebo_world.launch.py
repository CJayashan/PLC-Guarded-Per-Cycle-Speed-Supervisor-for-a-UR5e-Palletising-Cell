from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, Command
from ament_index_python.packages import get_package_share_directory
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import FindExecutable
import os


def generate_launch_description():
    # 1) Locate bringup share directory
    bringup_share = get_package_share_directory('ur5e_cell_bringup')

    # 2) World file path (installed)
    world_path = os.path.join(bringup_share, 'worlds', 'ur5e_cell.world')

    # 3) Locate Gazebo main launch file
    gazebo_launch = PathJoinSubstitution([
        FindPackageShare('gazebo_ros'),
        'launch',
        'gazebo.launch.py'
    ])

    # 4) Start Gazebo with our world
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gazebo_launch),
        launch_arguments={'world': world_path}.items()
    )

    # 5) Robot description (URDF from xacro)
    urdf_xacro_path = PathJoinSubstitution([
        FindPackageShare('ur5e_cell_description'),
        'urdf',
        'ur5e_cell_gazebo.urdf.xacro'
    ])


    robot_description = Command([
        FindExecutable(name='xacro'), ' ',
        urdf_xacro_path
    ])

    # 6) Robot State Publisher - now with use_sim_time=True
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{
            'robot_description': ParameterValue(robot_description, value_type=None),
            'use_sim_time': True
        }],
        output='screen'
    )

    # 7) Spawn entity into Gazebo from robot_description
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'ur5e_cell'
        ],
        output='screen'
    )

    # 8) Cycle signals node: reads FollowJointTrajectory status and publishes
    #    zone_busy + cycle_ok for the PLC/OPC UA bridge
    cycle_signals_node = TimerAction(
        period=7.5,  # start after JSB (6s) and JTC (7s)
        actions=[
            Node(
                package='ur5e_cell_bringup',        
                executable='cycle_signals_node',    
                name='cycle_signals_node',
                parameters=[{
                    'status_topic': '/joint_trajectory_controller/follow_joint_trajectory/_action/status',
                    'use_sim_time': True,
                    }],
                output='screen',
                )
            ],
    )

    # Delay spawn_entity so Gazebo has time to load GazeboRosFactory / spawn service
    spawn_delayed = TimerAction(
        period=5.0,  # seconds; increase if needed
        actions=[spawn_entity]
    )

    spawner_jsb = TimerAction(
        period=6.0,
        actions=[Node(
            package='controller_manager',
            executable='spawner',
            arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
            output='screen'
        )]
    )

    spawner_jtc = TimerAction(
        period=7.0,
        actions=[Node(
            package='controller_manager',
            executable='spawner',
            arguments=['joint_trajectory_controller', '--controller-manager', '/controller_manager'],
            output='screen'
        )]
    )

    return LaunchDescription([
        gazebo,
        robot_state_publisher,
        spawn_delayed,
        spawner_jsb,
        spawner_jtc,
        cycle_signals_node,
    ])
