"""Preflight configuration before any hardware process starts."""
from launch.substitutions import LaunchConfiguration
from launch.actions import SetLaunchConfiguration
from mujoco_sim.tasks import create_task


def validate_task(context, task_argument="task_id"):
    task = create_task(LaunchConfiguration(task_argument).perform(context))
    robot_id = task.resolve_robot(LaunchConfiguration("robot_id").perform(context))
    return [SetLaunchConfiguration("robot_id", robot_id)]


def configure_initial_pose(context):
    """Resolve optional checkpoint pose before starting inference components."""
    import json
    import math
    from pathlib import Path
    from robot_adapters import get_robot

    path = Path(LaunchConfiguration("policy_path").perform(context)).expanduser() / "initial_pose.json"
    positions = []
    if path.exists():
        try:
            pose = json.loads(path.read_text())
            robot = get_robot(LaunchConfiguration("robot_id").perform(context))
            if pose["robot_id"] != robot.robot_id:
                raise ValueError("robot_id does not match the selected robot")
            if pose["task_id"] != LaunchConfiguration("task_id").perform(context):
                raise ValueError("task_id does not match the selected task")
            if pose["unit"] != "rad":
                raise ValueError("unit must be rad")
            names, values = pose["joint_names"], pose["positions"]
            if not isinstance(names, list) or set(names) != set(robot.joint_names):
                raise ValueError("joint_names must contain exactly the robot's joints")
            if not isinstance(values, list) or any(
                type(v) not in (int, float) or not math.isfinite(v) for v in values
            ):
                raise ValueError("positions must be finite numbers")
            positions = robot.ordered(names, values).tolist()
        except (OSError, ValueError, KeyError, TypeError) as exc:
            raise ValueError(f"Invalid initial pose {path}: {exc}") from exc
    return [
        SetLaunchConfiguration("initial_joint_positions", " ".join(map(str, positions))),
    ]
