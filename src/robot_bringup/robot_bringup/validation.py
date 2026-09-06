"""Preflight configuration before any hardware process starts."""
from launch.substitutions import LaunchConfiguration
from launch.actions import SetLaunchConfiguration
from mujoco_sim.tasks import create_task


def validate_task(context, task_argument="task_id"):
    task = create_task(LaunchConfiguration(task_argument).perform(context))
    robot_id = task.resolve_robot(LaunchConfiguration("robot_id").perform(context))
    return [SetLaunchConfiguration("robot_id", robot_id)]
