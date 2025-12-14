import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder
from launch_ros.parameter_descriptions import ParameterFile

def generate_launch_description():
    # --- Paths to description and config packages ---
    desc_pkg_share = get_package_share_directory("ur5e_cell_description")
    cfg_pkg_share  = get_package_share_directory("ur5e_cell_moveit_config")

    xacro_path        = os.path.join(desc_pkg_share, "urdf", "ur5e_cell.urdf.xacro")
    srdf_path         = os.path.join(cfg_pkg_share,  "config", "ur5e_cell.srdf")
    kinematics_path   = os.path.join(cfg_pkg_share,  "config", "kinematics.yaml")
    joint_limits_path = os.path.join(cfg_pkg_share,  "config", "joint_limits.yaml")
    controllers_path  = os.path.join(cfg_pkg_share,  "config", "controllers.yaml")

    # If your xacro expects these (as in your current description), pass defaults here.
    # Adjust/remove keys if your xacro args differ.
    xacro_mappings = {
        "ur_type": "ur5e",
        "kinematics_params": "$(find ur_description)/config/ur5e/default_kinematics.yaml",
        "joint_limit_params": "$(find ur_description)/config/ur5e/joint_limits.yaml",
        "physical_params": "$(find ur_description)/config/ur5e/physical_parameters.yaml",
        "visual_params": "$(find ur_description)/config/ur5e/visual_parameters.yaml",
    }

    moveit_config = (
        MoveItConfigsBuilder("ur5e_cell", package_name="ur5e_cell_moveit_config")
        .robot_description(file_path=xacro_path, mappings=xacro_mappings)
        .robot_description_semantic(file_path=srdf_path)
        .robot_description_kinematics(file_path=kinematics_path)
        .joint_limits(file_path=joint_limits_path)
        .trajectory_execution(file_path=controllers_path)
        .planning_pipelines(pipelines=["ompl"])
        .to_moveit_configs()
    )

    move_group = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            ParameterFile(controllers_path),
        ],
    )

    return LaunchDescription([move_group])