"""Reset logic for moving the red cube to the red target."""

from .base import SimulationTask

CUBES = (
    ("red_cube", "red_cube_free", "red_cube_geom"),
)


class RedCubeToTargetTask(SimulationTask):
    task_id = "red_cube_to_red_target"
    language_instruction = "把红色方块放到红色区域"
    scene_file = "tasks/red_cube_to_target.xml"

    def reset(self, model, data, rng):
        """Randomize the red cube inside the scene named spawn area."""
        import mujoco

        site_id = mujoco.mj_name2id(
            model, mujoco.mjtObj.mjOBJ_SITE, "cube_spawn_area"
        )
        if site_id < 0:
            raise ValueError("model is missing site cube_spawn_area")

        center = data.site_xpos[site_id]
        size = model.site_size[site_id]
        x_range = (center[0] - size[0], center[0] + size[0])
        y_range = (center[1] - size[1], center[1] + size[1])

        red_xy = (rng.uniform(*x_range), rng.uniform(*y_range))

        positions = {}
        for (cube_name, joint_name, _), (x, y) in zip(
            CUBES, (red_xy,)
        ):
            joint_id = mujoco.mj_name2id(
                model, mujoco.mjtObj.mjOBJ_JOINT, joint_name
            )
            if joint_id < 0:
                raise ValueError(f"model is missing cube joint {joint_name!r}")

            qpos_address = int(model.jnt_qposadr[joint_id])
            dof_address = int(model.jnt_dofadr[joint_id])
            data.qpos[qpos_address : qpos_address + 7] = model.qpos0[
                qpos_address : qpos_address + 7
            ]
            data.qvel[dof_address : dof_address + 6] = 0.0
            data.qpos[qpos_address] = x
            data.qpos[qpos_address + 1] = y
            positions[cube_name] = tuple(
                float(value)
                for value in data.qpos[qpos_address : qpos_address + 3]
            )

        return "randomized objects: " + ", ".join(
            f"{name}=({position[0]:.3f}, {position[1]:.3f}, {position[2]:.3f})"
            for name, position in positions.items()
        )
