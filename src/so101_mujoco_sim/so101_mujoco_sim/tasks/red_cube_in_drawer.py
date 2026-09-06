"""Fixed-start scene for opening, filling, and closing a tabletop drawer."""

from .base import SimulationTask


class RedCubeInDrawerTask(SimulationTask):
    task_id = "red_cube_in_drawer"
    scene_file = "tasks/red_cube_in_drawer.xml"
    language_instruction = "拉开抽屉，把红色方块放进抽屉，再关上抽屉"

    def reset(self, model, data, rng):
        """Restore the cube's XML pose and close the passive drawer."""
        import mujoco

        for name, nq, nv in (("red_cube_free", 7, 6), ("drawer_slide", 1, 1)):
            joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
            if joint_id < 0:
                raise ValueError(f"model is missing task joint {name!r}")
            q = int(model.jnt_qposadr[joint_id])
            v = int(model.jnt_dofadr[joint_id])
            data.qpos[q : q + nq] = model.qpos0[q : q + nq]
            data.qvel[v : v + nv] = 0.0
            data.qacc_warmstart[v : v + nv] = 0.0

        return "restored fixed red cube pose and closed drawer"
