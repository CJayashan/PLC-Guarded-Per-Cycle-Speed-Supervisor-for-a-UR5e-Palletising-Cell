import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    ur_type = LaunchConfiguration("ur_type")
    use_sim_time = LaunchConfiguration("use_sim_time")

    declared_arguments = []
    declared_arguments.append(DeclareLaunchArgument("ur_type", default_value="ur5e"))
    declared_arguments.append(DeclareLaunchArgument("use_sim_time", default_value="false"))

    cfg_pkg_share  = get_package_share_directory("ur5e_cell_moveit_config")
    desc_pkg_share = get_package_share_directory("ur5e_cell_description")

    # --- Use the same xacro as your sim setup (joint names match real UR driver) ---
    xacro_path = os.path.join(desc_pkg_share, "urdf", "ur5e_cell.urdf.xacro")

    controllers_path = os.path.join(cfg_pkg_share, "config", "controllers_real.yaml")
    kinematics_path  = os.path.join(cfg_pkg_share, "config", "kinematics.yaml")
    joint_limits_path = os.path.join(cfg_pkg_share, "config", "joint_limits.yaml")

    xacro_mappings = {
        "ur_type": ur_type,
        "joint_limit_params": os.path.join(
            get_package_share_directory("ur_description"), "config", "ur5e", "joint_limits.yaml"
        ),
        "kinematics_params": os.path.join(
            get_package_share_directory("ur_description"), "config", "ur5e", "default_kinematics.yaml"
        ),
        "physical_params": os.path.join(
            get_package_share_directory("ur_description"), "config", "ur5e", "physical_parameters.yaml"
        ),
        "visual_params": os.path.join(
            get_package_share_directory("ur_description"), "config", "ur5e", "visual_parameters.yaml"
        ),
    }

    moveit_config = (
        MoveItConfigsBuilder("ur5e_cell", package_name="ur5e_cell_moveit_config")
        .robot_description(file_path=xacro_path, mappings=xacro_mappings)
        .robot_description_semantic(file_path="config/ur5e_cell.srdf")
        .robot_description_kinematics(file_path=kinematics_path)
        .joint_limits(file_path=joint_limits_path)
        .trajectory_execution(file_path=controllers_path)
        .planning_pipelines(pipelines=["ompl"])
        .to_moveit_configs()
    )

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            ParameterFile(controllers_path),     # <-- THIS is what fixes "No controller_names"
            {"use_sim_time": use_sim_time},
        ],
    )

    return LaunchDescription(declared_arguments + [move_group_node])
