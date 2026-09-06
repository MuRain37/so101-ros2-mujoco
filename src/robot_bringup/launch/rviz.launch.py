import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    pkg_share = get_package_share_directory("robot_description")

    urdf_path = os.path.join(pkg_share, "urdf", "so101_new_calib.urdf")
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
            DeclareLaunchArgument(
                "use_demo",
                default_value="true",
                description="Publish demo joint states instead of reading the real arm",
            ),
            Node(
                package="so101_leader_bridge",
                executable="so101_leader_driver",
                name="so101_leader_driver",
                output="screen",
                condition=UnlessCondition(LaunchConfiguration("use_demo")),
            ),
            Node(
                package="robot_description",
                executable="joint_state_demo",
                name="joint_state_demo",
                output="screen",
                condition=IfCondition(LaunchConfiguration("use_demo")),
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
