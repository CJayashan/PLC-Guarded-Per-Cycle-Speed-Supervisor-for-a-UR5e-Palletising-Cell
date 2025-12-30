from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, TimerAction, RegisterEventHandler, Shutdown
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration, PythonExpression

def generate_launch_description():
    # ------------------------
    # Launch-time arguments
    # ------------------------
    mode = LaunchConfiguration("mode")
    ppo_mode = LaunchConfiguration("ppo_mode")
    model_dir = LaunchConfiguration("model_dir")
    payload_mass = LaunchConfiguration("payload_mass")
    is_ppo = PythonExpression(["'", mode, "' == 'ppo'"])
    num_cycles = LaunchConfiguration("num_cycles")
    manual_speed = LaunchConfiguration("manual_speed")
    constant_speed = LaunchConfiguration("constant_speed")

    mode_arg = DeclareLaunchArgument(
        "mode",
        default_value="manual",  # manual | constant | external | ppo
        description="Speed supervisor mode: manual, constant, external, or ppo.",
    )
    ppo_mode_arg = DeclareLaunchArgument(
        "ppo_mode",
        default_value="train",
        description="PPO mode: train | finetune | eval",
    )

    model_dir_arg = DeclareLaunchArgument(
        "model_dir",
        default_value="/home/withanage/projects/project_root/logs",
        description="Directory to save PPO model/logs",
    )

    payload_mass_arg = DeclareLaunchArgument(
        "payload_mass",
        default_value="5.0",
        description="Payload mass in kg (used only in PPO mode).",
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
        parameters=[{"mode": mode}, {"manual_speed": manual_speed}, {"constant_speed": constant_speed}, {"use_sim_time": True}],
        condition=UnlessCondition(is_ppo),
    )

    ppo_node = Node(
        package="ur5e_cell_bringup",
        executable="ppo_speed_supervisor_node",
        name="ppo_speed_supervisor",
        output="screen",
        parameters=[{"use_sim_time": True},
                    {"mode": ppo_mode},
                    {"run_id": run_id},
                    {"payload_mass": payload_mass},
                    {"model_dir": model_dir}],
        condition=IfCondition(is_ppo),
    )

    speed_supervisor_ppo = Node(
        package="ur5e_cell_bringup",
        executable="speed_supervisor_node",
        name="speed_supervisor",
        output="screen",
        parameters=[{"mode": "ppo"}, {"use_sim_time": True}],
        condition=IfCondition(is_ppo),
    )

    shutdown_on_ppo_exit = RegisterEventHandler(
        OnProcessExit(target_action=ppo_node, on_exit=[Shutdown(reason="PPO node exited")]),
        condition=IfCondition(is_ppo),
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

    cycle_logger = Node(
        package='ur5e_cell_bringup',
        executable='cycle_logger_node',
        name='cycle_logger',
        output='screen',
        parameters=[
            {"csv_path": ["/home/withanage/projects/project_root/logs/", run_id, ".csv"]},
            {"run_id": run_id},
            {"payload_mass": payload_mass},
            {"use_sim_time": True},
        ],
        condition=UnlessCondition(is_ppo),
    )

    cycle_logger_ppo = Node(
        package='ur5e_cell_bringup',
        executable='cycle_logger_node',
        name='cycle_logger',
        output='screen',
        parameters=[
            {"csv_path": ["/home/withanage/projects/project_root/logs/", run_id, ".csv"]},
            {"run_id": run_id},
            {"payload_mass": 0.0},  # overridden by /payload_mass from PPO
            {"use_sim_time": True},
        ],
        condition=IfCondition(is_ppo),
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
        condition=UnlessCondition(is_ppo),
    )

    moveit_speed_executor_ppo = Node(
        package="ur5e_cell_bringup",
        executable="moveit_speed_executor_node",
        name="moveit_speed_executor",
        output="screen",
        parameters=[
            {"num_cycles": 1000000},  # effectively infinite
            {"speed_topic": "/speed_set"},
            {"plan_service": "/plan_kinematic_path"},
            {
                "jtc_action_name": "/joint_trajectory_controller/follow_joint_trajectory",
            },
            {"min_speed_scale": 0.1},
            {"max_speed_scale": 1.0},
            {"use_sim_time": True},
        ],
        condition=IfCondition(is_ppo),
    )
    

    delayed_executor = TimerAction(period=20.0, actions=[moveit_speed_executor], condition=UnlessCondition(is_ppo))
    delayed_executor_ppo = TimerAction(period=20.0, actions=[moveit_speed_executor_ppo], condition=IfCondition(is_ppo))

    return LaunchDescription(
        [
            mode_arg,
            num_cycles_arg,
            manual_speed_arg,
            constant_speed_arg,
            payload_mass_arg,
            run_id_arg,
            ppo_mode_arg,
            model_dir_arg,
            speed_supervisor,
            ppo_node,
            speed_supervisor_ppo,
            shutdown_on_ppo_exit,
            cycle_signals,
            cycle_logger,
            cycle_logger_ppo,
            delayed_executor,
            delayed_executor_ppo,
        ]
    )
