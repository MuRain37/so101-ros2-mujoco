from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler, EmitEvent
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from launch.actions import OpaqueFunction
from robot_bringup.validation import validate_task, configure_initial_pose


def generate_launch_description():
    project_root = Path.cwd()
    return LaunchDescription(
        [
            DeclareLaunchArgument("robot_id", default_value="auto"),
            DeclareLaunchArgument(
                "follower_workspace",
                default_value="0.10 0.45 -0.25 0.25 0.015 0.35",
            ),
            DeclareLaunchArgument("gripper_open_fraction", default_value="1.0"),
            RegisterEventHandler(OnProcessExit(on_exit=[
                EmitEvent(event=Shutdown(reason="inference component exited"))
            ])),
            DeclareLaunchArgument(
                "policy_path",
                description="Local LeRobot pretrained_model directory (ACT, SmolVLA or PI0).",
            ),
            DeclareLaunchArgument("device", default_value="cuda"),
            DeclareLaunchArgument("task_instruction", default_value=""),
            DeclareLaunchArgument("inference_rate", default_value="30.0"),
            DeclareLaunchArgument("task_id", default_value="red_blue_cubes_to_targets"),
            DeclareLaunchArgument("random_seed", default_value="-1"),
            DeclareLaunchArgument("policy_gui_enabled", default_value="true"),
            OpaqueFunction(function=validate_task),
            OpaqueFunction(function=configure_initial_pose),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [FindPackageShare("mujoco_sim"), "launch", "display.launch.py"]
                    )
                ),
                launch_arguments={
                    "robot_id": LaunchConfiguration("robot_id"),
                    "random_seed": LaunchConfiguration("random_seed"),
                    "task_id": LaunchConfiguration("task_id"),
                    "follower_workspace": LaunchConfiguration("follower_workspace"),
                    "gripper_open_fraction": LaunchConfiguration("gripper_open_fraction"),
                    "initial_joint_positions": LaunchConfiguration("initial_joint_positions"),
                }.items(),
            ),
            Node(
                package="robot_policy",
                executable="policy_inference",
                name="policy_inference",
                output="screen",
                prefix=[str(project_root / ".venv/bin/python")],
                parameters=[
                    {
                        "policy_path": LaunchConfiguration("policy_path"),
                        "task_id": LaunchConfiguration("task_id"),
                        "robot_id": LaunchConfiguration("robot_id"),
                        "device": LaunchConfiguration("device"),
                        "task_instruction": ParameterValue(LaunchConfiguration("task_instruction"), value_type=str),
                        "inference_rate": LaunchConfiguration("inference_rate"),
                    }
                ],
            ),
            Node(
                package="robot_policy",
                executable="robot_policy_gui",
                name="robot_policy_gui",
                output="screen",
                condition=IfCondition(LaunchConfiguration("policy_gui_enabled")),
            ),
        ]
    )
