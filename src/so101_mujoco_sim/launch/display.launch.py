from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "model_path",
                default_value="",
                description="Optional absolute or relative path to a MuJoCo XML scene.",
            ),
            DeclareLaunchArgument(
                "joint_state_topic",
                default_value="/joint_states",
                description="JointState target topic for the simulator.",
            ),
            DeclareLaunchArgument(
                "realtime_factor",
                default_value="1.0",
                description="Ratio of MuJoCo simulation time to wall time.",
            ),
            DeclareLaunchArgument(
                "random_seed",
                default_value="-1",
                description="Task randomization seed; negative selects a random seed.",
            ),
            DeclareLaunchArgument(
                "task_id",
                default_value="red_cube_to_red_target",
                description="Registered task that owns scene reset behavior.",
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
            DeclareLaunchArgument(
                "d435_enabled",
                default_value="true",
                description="Enable global D435 aligned RGB-D publication.",
            ),
            DeclareLaunchArgument("d435_width", default_value="640"),
            DeclareLaunchArgument("d435_height", default_value="480"),
            DeclareLaunchArgument("d435_publish_rate", default_value="30.0"),
            DeclareLaunchArgument(
                "camera_preview_enabled", default_value="true",
                description="Publish JPEG-compressed RGB topics for preview tools.",
            ),
            Node(
                package="so101_mujoco_sim",
                executable="so101_mujoco_simulator",
                name="so101_mujoco_simulator",
                output="screen",
                parameters=[
                    {
                        "model_path": LaunchConfiguration("model_path"),
                        "joint_state_topic": LaunchConfiguration("joint_state_topic"),
                        "realtime_factor": ParameterValue(
                            LaunchConfiguration("realtime_factor"), value_type=float
                        ),
                        "random_seed": ParameterValue(
                            LaunchConfiguration("random_seed"), value_type=int
                        ),
                        "task_id": LaunchConfiguration("task_id"),
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
                        "d435_enabled": ParameterValue(
                            LaunchConfiguration("d435_enabled"), value_type=bool
                        ),
                        "d435_width": ParameterValue(
                            LaunchConfiguration("d435_width"), value_type=int
                        ),
                        "d435_height": ParameterValue(
                            LaunchConfiguration("d435_height"), value_type=int
                        ),
                        "d435_publish_rate": ParameterValue(
                            LaunchConfiguration("d435_publish_rate"), value_type=float
                        ),
                    }
                ],
            ),
            Node(
                package="image_transport", executable="republish",
                name="wrist_preview_republisher",
                condition=IfCondition(LaunchConfiguration("camera_preview_enabled")),
                parameters=[{"in_transport": "raw", "out_transport": "compressed"}],
                remappings=[
                    ("in", "/wrist_cam/image_raw"),
                    ("out/compressed", "/wrist_cam/image_preview/compressed"),
                ],
            ),
            Node(
                package="image_transport", executable="republish",
                name="d435_preview_republisher",
                condition=IfCondition(LaunchConfiguration("camera_preview_enabled")),
                parameters=[{"in_transport": "raw", "out_transport": "compressed"}],
                remappings=[
                    ("in", "/d435/color/image_raw"),
                    ("out/compressed", "/d435/color/image_preview/compressed"),
                ],
            ),
        ]
    )
