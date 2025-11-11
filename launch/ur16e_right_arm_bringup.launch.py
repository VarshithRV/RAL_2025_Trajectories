from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
import os

# whats needed to know when writing a launch file : 
# how to launch a node, with parameters, remapping capabilities and ns
# how to include a launch file, with arguments, remapping capabilities and ns
# how to define launch arguments and pass them to the launching command
# how to launch Params, urdf xml etc

def generate_launch_description():
    ur_driver_pkg = FindPackageShare('ur_robot_driver').find('ur_robot_driver')
    ur_moveit_pkg = FindPackageShare('ur_moveit_config').find('ur_moveit_config')

    ur_control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ur_driver_pkg, 'launch', 'ur_control.launch.py')
        ),
        launch_arguments={
            'launch_rviz': 'false',
            'ur_type': 'ur16e',
            'robot_ip':'192.168.1.7',
            'use_fake_hardware': 'false'
        }.items()
    )

    # ur_moveit_launch = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         os.path.join(ur_moveit_pkg, 'launch', 'ur_moveit.launch.py')
    #     ),
    #     launch_arguments={
    #         'ur_type': 'ur16e',
    #         'launch_rviz': 'true'
    #     }.items()
    # )

    return LaunchDescription([
        ur_control_launch,
        # ur_moveit_launch
    ])
