from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler, EmitEvent
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import OpaqueFunction
from robot_bringup.validation import validate_task


def generate_launch_description():
    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("mujoco_sim"),
                "launch",
                "display.launch.py",
            ])
        ),
        launch_arguments={
            "robot_id": LaunchConfiguration("robot_id"),
            "random_seed": LaunchConfiguration("random_seed"),
            "task_id": LaunchConfiguration("dataset_task_id"),
        }.items(),
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument("robot_id", default_value="auto"),
            DeclareLaunchArgument("position_scale", default_value="2.0"),
            DeclareLaunchArgument("orientation_scale", default_value="1.0"),
            DeclareLaunchArgument("leader_min_height", default_value="0.011"),
            DeclareLaunchArgument("follower_min_height", default_value="0.015"),
            DeclareLaunchArgument("leader_range", default_value=""),
            DeclareLaunchArgument("follower_range", default_value=""),
            RegisterEventHandler(OnProcessExit(on_exit=[
                EmitEvent(event=Shutdown(reason="teleop component exited"))
            ])),
            DeclareLaunchArgument(
                "port",
                default_value="/dev/ttyACM0",
                description="Serial port connected to the SO-101 leader arm.",
            ),
            DeclareLaunchArgument(
                "calibration_file",
                default_value=(
                    "~/.cache/huggingface/lerobot/calibration/"
                    "teleoperators/so_leader/zihao_leader_arm.json"
                ),
                description="LeRobot leader-arm calibration JSON file.",
            ),
            DeclareLaunchArgument(
                "rate",
                default_value="50.0",
                description="Leader-arm serial polling rate in Hz.",
            ),
            DeclareLaunchArgument("dataset_output_dir", default_value="dataset/raw"),
            DeclareLaunchArgument(
                "dataset_task_id", default_value="red_cube_to_red_target"
            ),
            DeclareLaunchArgument(
                "random_seed",
                default_value="-1",
                description="Task randomization seed; negative selects a random seed.",
            ),
            DeclareLaunchArgument(
                "dataset_gui_enabled",
                default_value="true",
                description="Open the dataset start/stop control window.",
            ),
            OpaqueFunction(function=validate_task, kwargs={"task_argument": "dataset_task_id"}),
            Node(
                package="so101_leader_bridge",
                executable="so101_leader_driver",
                remappings=[("joint_states", "/leader/joint_states")],
                name="so101_leader_driver",
                output="screen",
                parameters=[
                    {
                        "port": LaunchConfiguration("port"),
                        "calibration_file": LaunchConfiguration("calibration_file"),
                        "rate": LaunchConfiguration("rate"),
                    }
                ],
            ),
            simulation,
            Node(
                package="teleop_retargeting", executable="retarget",
                name="teleop_retargeting", output="screen",
                parameters=[{
                    "robot_id": LaunchConfiguration("robot_id"),
                    "task_id": LaunchConfiguration("dataset_task_id"),
                    "position_scale": LaunchConfiguration("position_scale"),
                    "orientation_scale": LaunchConfiguration("orientation_scale"),
                    "leader_min_height": LaunchConfiguration("leader_min_height"),
                    "follower_min_height": LaunchConfiguration("follower_min_height"),
                    "leader_range": LaunchConfiguration("leader_range"),
                    "follower_range": LaunchConfiguration("follower_range"),
                }],
            ),
            Node(
                package="vla_dataset",
                executable="dataset_recorder",
                name="dataset_recorder",
                output="screen",
                parameters=[
                    {
                        "output_dir": LaunchConfiguration("dataset_output_dir"),
                        "robot_id": LaunchConfiguration("robot_id"),
                        "task_id": LaunchConfiguration("dataset_task_id"),
                    }
                ],
            ),
            Node(
                package="vla_dataset",
                executable="dataset_gui",
                name="dataset_gui",
                output="screen",
                condition=IfCondition(LaunchConfiguration("dataset_gui_enabled")),
            ),
        ]
    )
