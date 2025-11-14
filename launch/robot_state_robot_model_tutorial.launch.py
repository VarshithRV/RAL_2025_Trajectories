from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
import yaml
import xacro


def generate_launch_description():
    ur_type = "ur16e"

    ur_description_pkg = get_package_share_directory("ur_description")
    urdf_xacro_path = os.path.join(ur_description_pkg, "urdf", "ur.urdf.xacro")

    robot_description_raw = xacro.process_file(
        urdf_xacro_path,
        mappings={
            "ur_type": ur_type,
            "name": "ur",
            "prefix": "",
        },
    ).toxml()

    robot_description = {"robot_description": robot_description_raw}

    ur_moveit_pkg = get_package_share_directory("ur_moveit_config")
    srdf_xacro_path = os.path.join(ur_moveit_pkg, "srdf", "ur.srdf.xacro")

    robot_description_semantic_raw = xacro.process_file(
        srdf_xacro_path,
        mappings={"name": "ur"},
    ).toxml()

    robot_description_semantic = {
        "robot_description_semantic": robot_description_semantic_raw
    }

    kinematics_yaml_path = os.path.join(ur_moveit_pkg, "config", "kinematics.yaml")

    with open(kinematics_yaml_path, "r") as f:
        kinematics_yaml = yaml.safe_load(f)

    robot_description_kinematics = {
    "robot_description_kinematics": {
        "ur_manipulator": {
            "kinematics_solver": "kdl_kinematics_plugin/KDLKinematicsPlugin",
            "kinematics_solver_attempts": 3,
            "kinematics_solver_search_resolution": 0.005,
            "kinematics_solver_timeout": 0.005,
        }
    }
}

    tutorial_node = Node(
        package="ral_2025_trajectories",
        executable="robot_model_and_robot_state_tutorial",
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
        ],
    )

    return LaunchDescription([tutorial_node])
