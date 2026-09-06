"""UR5e tabletop scene with a fixed initial red cube pose."""
from .base import SimulationTask


class UR5eRedCubeToTargetTask(SimulationTask):
    task_id = "ur5e_red_cube_to_target"
    robot_id = "ur5e"
    scene_file = "tasks/ur5e_red_cube_to_target.xml"
    language_instruction = "把红色方块放到红色区域"

    def reset(self, model, data, rng):
        joint = model.joint("red_cube_free")
        q, v = int(joint.qposadr[0]), int(joint.dofadr[0])
        data.qpos[q:q + 7] = model.qpos0[q:q + 7]
        data.qvel[v:v + 6] = 0
        data.qacc_warmstart[v:v + 6] = 0
        return "restored fixed red cube pose"
