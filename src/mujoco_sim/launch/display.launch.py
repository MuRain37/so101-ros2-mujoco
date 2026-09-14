from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from mujoco_sim.tasks import create_task


def camera_preview_nodes(context):
    task = create_task(LaunchConfiguration("task_id").perform(context))
    enabled = IfCondition(LaunchConfiguration("camera_preview_enabled"))
    nodes = []
    for camera in task.cameras:
        stream = camera.rgb
        if stream is None or stream.preview_topic is None:
            continue
        nodes.append(Node(
            package="image_transport",
            executable="republish",
            name=f"{camera.camera_id}_preview_republisher",
            condition=enabled,
            parameters=[{"in_transport": "raw", "out_transport": "compressed"}],
            remappings=[
                ("in", stream.topic),
                ("out/compressed", stream.preview_topic),
            ],
        ))
    return nodes


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("robot_id", default_value="auto"),
            DeclareLaunchArgument("initial_joint_positions", default_value=""),
            DeclareLaunchArgument(
                "model_path",
                default_value="",
                description="Optional absolute or relative path to a MuJoCo XML scene.",
            ),
            DeclareLaunchArgument(
                "joint_state_topic",
                default_value="/robot/joint_targets",
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
                description="Registered task that owns its scene and camera configuration.",
            ),
            DeclareLaunchArgument(
                "follower_workspace",
                default_value="0.10 0.45 -0.25 0.25 0.015 0.35",
                description="Workspace used when deriving a mapped reset pose.",
            ),
            DeclareLaunchArgument(
                "gripper_open_fraction",
                default_value="1.0",
                description="Leader opening fraction used for the mapped reset pose.",
            ),
            DeclareLaunchArgument(
                "camera_enabled",
                default_value="true",
                description="Enable all image streams declared by the selected task.",
            ),
            DeclareLaunchArgument(
                "camera_preview_enabled",
                default_value="true",
                description="Publish compressed previews for task RGB streams.",
            ),
            Node(
                package="mujoco_sim",
                executable="mujoco_simulator",
                name="mujoco_simulator",
                output="screen",
                parameters=[
                    {
                        "robot_id": LaunchConfiguration("robot_id"),
                        "initial_joint_positions": ParameterValue(
                            LaunchConfiguration("initial_joint_positions"), value_type=str
                        ),
                        "model_path": LaunchConfiguration("model_path"),
                        "joint_state_topic": LaunchConfiguration("joint_state_topic"),
                        "realtime_factor": ParameterValue(
                            LaunchConfiguration("realtime_factor"), value_type=float
                        ),
                        "random_seed": ParameterValue(
                            LaunchConfiguration("random_seed"), value_type=int
                        ),
                        "task_id": LaunchConfiguration("task_id"),
                        "follower_workspace": LaunchConfiguration(
                            "follower_workspace"
                        ),
                        "gripper_open_fraction": ParameterValue(
                            LaunchConfiguration("gripper_open_fraction"),
                            value_type=float,
                        ),
                        "camera_enabled": ParameterValue(
                            LaunchConfiguration("camera_enabled"), value_type=bool
                        ),
                    }
                ],
            ),
            OpaqueFunction(function=camera_preview_nodes),
        ]
    )
