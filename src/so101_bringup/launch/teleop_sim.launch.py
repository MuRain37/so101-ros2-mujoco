from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
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
            DeclareLaunchArgument(
                "joint_state_topic",
                default_value="/joint_states",
                description="Leader JointState target topic for the simulator.",
            ),
            DeclareLaunchArgument(
                "realtime_factor",
                default_value="1.0",
                description="Ratio of MuJoCo simulation time to wall time.",
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
            Node(
                package="so101_mujoco_sim",
                executable="so101_mujoco_viewer",
                name="so101_mujoco_viewer",
                output="screen",
                parameters=[
                    {
                        "joint_state_topic": LaunchConfiguration("joint_state_topic"),
                        "realtime_factor": LaunchConfiguration("realtime_factor"),
                    }
                ],
            ),
        ]
    )
