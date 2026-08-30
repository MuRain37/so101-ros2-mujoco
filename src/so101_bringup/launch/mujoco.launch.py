from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "model_path",
                default_value="",
                description="Optional path to a self-contained MuJoCo XML scene.",
            ),
            Node(
                package="so101_mujoco_sim",
                executable="so101_mujoco_viewer",
                name="so101_mujoco_viewer",
                output="screen",
                parameters=[{"model_path": LaunchConfiguration("model_path")}],
            ),
        ]
    )
