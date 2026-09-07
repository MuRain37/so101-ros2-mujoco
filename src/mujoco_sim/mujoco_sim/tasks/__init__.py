"""Registry of available MuJoCo training tasks."""

from .red_cube_to_target import RedCubeToTargetTask
from .red_blue_cubes_to_targets import RedBlueCubesToTargetsTask
from .red_cube_in_drawer import RedCubeInDrawerTask
from .ur5e_red_blue_cubes_to_targets import UR5eRedBlueCubesToTargetsTask
from .ur5e_red_cube_in_drawer import UR5eRedCubeInDrawerTask
from .ur5e_red_cube_to_target import UR5eRedCubeToTargetTask

TASKS = {
    UR5eRedCubeToTargetTask.task_id: UR5eRedCubeToTargetTask,
    UR5eRedBlueCubesToTargetsTask.task_id: UR5eRedBlueCubesToTargetsTask,
    UR5eRedCubeInDrawerTask.task_id: UR5eRedCubeInDrawerTask,
    RedCubeToTargetTask.task_id: RedCubeToTargetTask,
    RedBlueCubesToTargetsTask.task_id: RedBlueCubesToTargetsTask,
    RedCubeInDrawerTask.task_id: RedCubeInDrawerTask,
}


def create_task(task_id: str):
    try:
        return TASKS[task_id]()
    except KeyError as error:
        available = ", ".join(sorted(TASKS))
        raise ValueError(
            f"unknown task_id {task_id!r}; available tasks: {available}"
        ) from error
