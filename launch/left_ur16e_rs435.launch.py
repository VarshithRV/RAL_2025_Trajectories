""" Static transform publisher acquired via MoveIt 2 hand-eye calibration """
""" EYE-IN-HAND: wrist_3_link -> camera_color_optical_frame """
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    nodes = [
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            output="log",
            arguments=[
                "--frame-id",
                "wrist_3_link",
                "--child-frame-id",
                "camera_color_optical_frame",
                "--x",
                "-0.0166512",
                "--y",
                "-0.0954157",
                "--z",
                "0.023471",
                "--qx",
                "0.0148196",
                "--qy",
                "-0.0150615",
                "--qz",
                "0.0161584",
                "--qw",
                "0.999646",
                # "--roll",
                # "0.0301332",
                # "--pitch",
                # "-0.0296377",
                # "--yaw",
                # "0.032772",
            ],
        ),
    ]
    return LaunchDescription(nodes)
