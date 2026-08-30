import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("so101_description")

    urdf_path = os.path.join(pkg_share, "urdf", "so101_full.urdf")
    with open(urdf_path, "r", encoding="utf-8") as f:
        robot_description = f.read()

    return LaunchDescription(
        [
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                parameters=[{"robot_description": robot_description}],
                output="screen",
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="world_to_base",
                arguments=["0", "0", "0", "0", "0", "0", "world", "base_link"],
                output="screen",
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                arguments=[
                    "-d",
                    os.path.join(pkg_share, "config", "so101.rviz"),
                ],
            ),
        ]
    )
