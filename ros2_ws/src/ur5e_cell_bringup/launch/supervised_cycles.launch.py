from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # ------------------------
    # Launch-time arguments
    # ------------------------
    mode = LaunchConfiguration("mode")
    num_cycles = LaunchConfiguration("num_cycles")
    manual_speed = LaunchConfiguration("manual_speed")
    constant_speed = LaunchConfiguration("constant_speed")

    mode_arg = DeclareLaunchArgument(
        "mode",
        default_value="manual",  # manual | constant | external
        description="Speed supervisor mode: manual, constant, or external.",
    )

    num_cycles_arg = DeclareLaunchArgument(
        "num_cycles",
        default_value="5",
        description="Number of A->B->A cycles to run.",
    )

    manual_speed_arg = DeclareLaunchArgument(
        "manual_speed",
        default_value="0.5",
        description="Speed value used when mode==manual (0.0..1.0).",
    )

    constant_speed_arg = DeclareLaunchArgument(
        "constant_speed",
        default_value="0.5",
        description="Speed value used when mode==constant (0.0..1.0).",
    )

    run_id_arg = DeclareLaunchArgument(
        "run_id",
        default_value="constant_0_3_test",
        description="Identifier string stored in the CSV log."
    )
    run_id = LaunchConfiguration("run_id")

    # 1) Speed supervisor node
    speed_supervisor = Node(
        package="ur5e_cell_bringup",
        executable="speed_supervisor_node",
        name="speed_supervisor",
        output="screen",
        parameters=[
            {"mode": mode},                    # manual | constant | external
            {"manual_speed": manual_speed},    # now from LaunchArg
            {"constant_speed": constant_speed},
            {"speed_topic": "/speed_set"},
            {"use_sim_time": True},
        ],
    )

    # 2) Cycle signals node
    cycle_signals = Node(
        package="ur5e_cell_bringup",
        executable="cycle_signals_node",
        name="cycle_signals",
        output="screen",
        parameters=[
            {
                "status_topic": "/joint_trajectory_controller/follow_joint_trajectory/_action/status",
            },
            {"zone_busy_topic": "/zone_busy"},
            {"cycle_ok_topic": "/cycle_ok"},
            {"use_sim_time": True},
        ],
    )

    # 3) MoveIt speed executor (delayed start)
    moveit_speed_executor = Node(
        package="ur5e_cell_bringup",
        executable="moveit_speed_executor_node",
        name="moveit_speed_executor",
        output="screen",
        parameters=[
            {"num_cycles": num_cycles},
            {"speed_topic": "/speed_set"},
            {"plan_service": "/plan_kinematic_path"},
            {
                "jtc_action_name": "/joint_trajectory_controller/follow_joint_trajectory",
            },
            {"min_speed_scale": 0.1},
            {"max_speed_scale": 1.0},
            {"use_sim_time": True},
        ],
    )

    cycle_logger = Node(
        package='ur5e_cell_bringup',
        executable='cycle_logger_node',
        name='cycle_logger',
        output='screen',
        parameters=[
            {"csv_path": "/home/withanage/projects/project_root/logs/cycle_log_constant_0_3.csv"},
            {"run_id": run_id},
            {"use_sim_time": False},
        ],
    )

    

    delayed_executor = TimerAction(
        period=15.0,
        actions=[moveit_speed_executor],
    )

    return LaunchDescription(
        [
            mode_arg,
            num_cycles_arg,
            manual_speed_arg,
            constant_speed_arg,
            run_id_arg,
            speed_supervisor,
            cycle_signals,
            cycle_logger,
            delayed_executor,

        ]
    )
