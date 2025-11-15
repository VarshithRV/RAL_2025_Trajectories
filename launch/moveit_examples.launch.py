from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
import yaml
import xacro


def generate_launch_description():

    robot_description_kinematics = {
    "robot_description_kinematics": {
        "ur_manipulator": {
            "kinematics_solver": "kdl_kinematics_plugin/KDLKinematicsPlugin",
            "kinematics_solver_attempts": 3,
            "kinematics_solver_search_resolution": 0.005,
            "kinematics_solver_timeout": 0.005,}
        }
    }

    tutorial_node = Node(
        package="ral_2025_trajectories",
        executable="moveit2_examples",
        output="screen",
        parameters=[
            robot_description_kinematics,
        ],
    )

    return LaunchDescription([tutorial_node])
