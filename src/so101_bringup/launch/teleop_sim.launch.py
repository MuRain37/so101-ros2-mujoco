from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
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
        )
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
                    }
                ],
            ),
        ]
    )
