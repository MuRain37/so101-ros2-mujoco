"""Reset logic for moving the red cube to the red target."""

from .base import SimulationTask

CUBES = (
    ("red_cube", "red_cube_free", "red_cube_geom"),
    ("blue_cube", "blue_cube_free", "blue_cube_geom"),
)
CUBE_MINIMUM_GAP = 0.01
MAX_ATTEMPTS = 1_000


class RedCubeToTargetTask(SimulationTask):
    task_id = "red_cube_to_red_target"
    scene_file = "scene.xml"

    def reset(self, model, data, rng):
        """Randomize both cubes inside the scene named spawn area."""
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

        geom_sizes = []
        for _, _, geom_name in CUBES:
            geom_id = mujoco.mj_name2id(
                model, mujoco.mjtObj.mjOBJ_GEOM, geom_name
            )
            if geom_id < 0:
                raise ValueError(f"model is missing cube geom {geom_name!r}")
            geom_sizes.append(model.geom_size[geom_id])

        minimum_x_separation = (
            geom_sizes[0][0] + geom_sizes[1][0] + CUBE_MINIMUM_GAP
        )
        minimum_y_separation = (
            geom_sizes[0][1] + geom_sizes[1][1] + CUBE_MINIMUM_GAP
        )
        for _ in range(MAX_ATTEMPTS):
            red_xy = (rng.uniform(*x_range), rng.uniform(*y_range))
            blue_xy = (rng.uniform(*x_range), rng.uniform(*y_range))
            if (
                abs(red_xy[0] - blue_xy[0]) >= minimum_x_separation
                or abs(red_xy[1] - blue_xy[1]) >= minimum_y_separation
            ):
                break
        else:
            raise RuntimeError("failed to find non-overlapping cube positions")

        positions = {}
        for (cube_name, joint_name, _), (x, y) in zip(
            CUBES, (red_xy, blue_xy)
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
