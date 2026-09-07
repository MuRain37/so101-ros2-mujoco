"""UR5e version of the red-cube-to-target task."""
from .red_cube_to_target import RedCubeToTargetTask


class UR5eRedCubeToTargetTask(RedCubeToTargetTask):
    task_id = "ur5e_red_cube_to_target"
    robot_id = "ur5e"
    scene_file = "tasks/ur5e_red_cube_to_target.xml"
