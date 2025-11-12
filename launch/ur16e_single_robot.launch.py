from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    ur_driver_pkg = FindPackageShare('ur_robot_driver').find('ur_robot_driver')
    ur_moveit_pkg = FindPackageShare('ur_moveit_config').find('ur_moveit_config')

    ur_control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(ur_driver_pkg,'launch','ur_control.launch.py')),
        launch_arguments={
            'launch_rviz':'false',
            'ur_type':'ur16e',
            'robot_ip':'localhost',
            'use_fake_hardware':'true'
        }.items()
    )

    ur_moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(ur_moveit_pkg,'launch','ur_moveit.launch.py')),
        launch_arguments={
            'ur_type':'ur16e',
            'launch_rviz':'true'
        }.items()        
    )

    return LaunchDescription([
        ur_control_launch,
        ur_moveit_launch
    ])