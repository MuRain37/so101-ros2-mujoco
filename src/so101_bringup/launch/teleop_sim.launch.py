from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("so101_mujoco_sim"),
                "launch",
                "display.launch.py",
            ])
        ),
        launch_arguments={
            "random_seed": LaunchConfiguration("random_seed"),
            "task_id": LaunchConfiguration("dataset_task_id"),
        }.items(),
    )
    return LaunchDescription(
        [
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
            DeclareLaunchArgument("dataset_task", default_value="把红色方块放到红色区域"),
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
            Node(
                package="so101_leader_bridge",
                executable="so101_leader_driver",
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
                package="so101_vla_dataset",
                executable="so101_dataset_recorder",
                name="so101_dataset_recorder",
                output="screen",
                parameters=[
                    {
                        "output_dir": LaunchConfiguration("dataset_output_dir"),
                        "task": LaunchConfiguration("dataset_task"),
                        "task_id": LaunchConfiguration("dataset_task_id"),
                    }
                ],
            ),
            Node(
                package="so101_vla_dataset",
                executable="so101_dataset_gui",
                name="so101_dataset_gui",
                output="screen",
                condition=IfCondition(LaunchConfiguration("dataset_gui_enabled")),
                parameters=[{"task": LaunchConfiguration("dataset_task")}],
            ),
        ]
    )
