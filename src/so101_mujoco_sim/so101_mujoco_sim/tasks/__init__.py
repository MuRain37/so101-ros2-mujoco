"""Registry of available MuJoCo training tasks."""

from .red_cube_to_target import RedCubeToTargetTask
from .red_blue_cubes_to_targets import RedBlueCubesToTargetsTask

TASKS = {
    RedCubeToTargetTask.task_id: RedCubeToTargetTask,
    RedBlueCubesToTargetsTask.task_id: RedBlueCubesToTargetsTask,
}


def create_task(task_id: str):
    try:
        return TASKS[task_id]()
    except KeyError as error:
        available = ", ".join(sorted(TASKS))
        raise ValueError(
            f"unknown task_id {task_id!r}; available tasks: {available}"
        ) from error
