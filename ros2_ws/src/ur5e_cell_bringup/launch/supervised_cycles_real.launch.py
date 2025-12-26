from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    mode = LaunchConfiguration("mode")
    num_cycles = LaunchConfiguration("num_cycles")
    manual_speed = LaunchConfiguration("manual_speed")
    constant_speed = LaunchConfiguration("constant_speed")
    run_id = LaunchConfiguration("run_id")

    return LaunchDescription([
        DeclareLaunchArgument("mode", default_value="manual"),
        DeclareLaunchArgument("num_cycles", default_value="5"),
        DeclareLaunchArgument("manual_speed", default_value="0.5"),
        DeclareLaunchArgument("constant_speed", default_value="0.5"),
        DeclareLaunchArgument("run_id", default_value="real_test"),

        # Speed supervisor (same topics as before: speed_set, speed_set_external, opcua_* etc.)
        Node(
            package="ur5e_cell_bringup",
            executable="speed_supervisor_node",
            name="speed_supervisor",
            output="screen",
            parameters=[
                {"mode": mode},
                {"manual_speed": manual_speed},
                {"constant_speed": constant_speed},
                {"use_sim_time": False},
            ],
        ),

        # (Optional) keep this commented until we “do cycle signals later”
        # Node(
        #     package="ur5e_cell_bringup",
        #     executable="cycle_signals_node",
        #     name="cycle_signals",
        #     output="screen",
        #     parameters=[
        #         {"status_topic": "/scaled_joint_trajectory_controller/follow_joint_trajectory/_action/status"},
        #         {"zone_busy_topic": "/zone_busy"},
        #         {"cycle_ok_topic": "/cycle_ok"},
        #         {"use_sim_time": False},
        #     ],
        # ),

        Node(
            package="ur5e_cell_bringup",
            executable="cycle_logger_node",
            name="cycle_logger",
            output="screen",
            parameters=[
                {"csv_path": "/home/withanage/projects/project_root/logs/cycle_log_real.csv"},
                {"run_id": run_id},
                {"use_sim_time": False},
            ],
        ),

        TimerAction(
            period=3.0,
            actions=[
                Node(
                    package="ur5e_cell_bringup",
                    executable="moveit_speed_executor_node",
                    name="moveit_speed_executor",
                    output="screen",
                    parameters=[
                        {"num_cycles": num_cycles},
                        {"speed_topic": "/speed_set"},
                        {"plan_service": "/plan_kinematic_path"},
                        # KEY CHANGE: scaled controller action server
                        {"jtc_action_name": "/scaled_joint_trajectory_controller/follow_joint_trajectory"},
                        {"min_speed_scale": 0.1},
                        {"max_speed_scale": 1.0},
                        {"use_sim_time": False},
                    ],
                )
            ],
        ),
    ])
