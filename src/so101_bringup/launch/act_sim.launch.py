from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    project_root = Path.cwd()
    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("so101_mujoco_sim"), "launch", "display.launch.py"]
            )
        ),
        launch_arguments={
            "random_seed": LaunchConfiguration("random_seed"),
            "task_id": LaunchConfiguration("task_id"),
        }.items(),
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "policy_path",
                default_value=str(
                    project_root
                    / "outputs/train/act_so101_red_blue_cubes/checkpoints/100000/pretrained_model"
                ),
            ),
            DeclareLaunchArgument("device", default_value="cuda"),
            DeclareLaunchArgument("inference_rate", default_value="30.0"),
            DeclareLaunchArgument("task_id", default_value="red_blue_cubes_to_targets"),
            DeclareLaunchArgument("random_seed", default_value="-1"),
            DeclareLaunchArgument("policy_gui_enabled", default_value="true"),
            simulation,
            Node(
                package="so101_policy",
                executable="so101_act_policy",
                name="so101_act_policy",
                output="screen",
                prefix=[str(project_root / ".venv/bin/python")],
                parameters=[
                    {
                        "policy_path": LaunchConfiguration("policy_path"),
                        "device": LaunchConfiguration("device"),
                        "inference_rate": LaunchConfiguration("inference_rate"),
                    }
                ],
            ),
            Node(
                package="so101_policy",
                executable="so101_policy_gui",
                name="so101_policy_gui",
                output="screen",
                condition=IfCondition(LaunchConfiguration("policy_gui_enabled")),
            ),
        ]
    )
