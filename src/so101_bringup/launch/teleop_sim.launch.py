from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


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
            DeclareLaunchArgument(
                "camera_enabled",
                default_value="true",
                description="Enable wrist-camera rendering and ROS image publication.",
            ),
            DeclareLaunchArgument(
                "camera_width", default_value="640", description="Published image width."
            ),
            DeclareLaunchArgument(
                "camera_height", default_value="480", description="Published image height."
            ),
            DeclareLaunchArgument(
                "camera_publish_rate",
                default_value="30.0",
                description="Wrist-camera publication rate in simulation-time Hz.",
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
                        "camera_enabled": ParameterValue(
                            LaunchConfiguration("camera_enabled"), value_type=bool
                        ),
                        "camera_width": ParameterValue(
                            LaunchConfiguration("camera_width"), value_type=int
                        ),
                        "camera_height": ParameterValue(
                            LaunchConfiguration("camera_height"), value_type=int
                        ),
                        "camera_publish_rate": ParameterValue(
                            LaunchConfiguration("camera_publish_rate"), value_type=float
                        ),
                    }
                ],
            ),
        ]
    )
