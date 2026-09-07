"""Open, fill, and close a tabletop drawer with a randomly placed red cube."""

from .base import CameraSpec, ImageStreamSpec, SimulationTask

CUBE = ("red_cube", "red_cube_free", "red_cube_geom")


class RedCubeInDrawerTask(SimulationTask):
    task_id = "red_cube_in_drawer"
    scene_file = "tasks/red_cube_in_drawer.xml"
    cameras = (
        CameraSpec(
            camera_id="front",
            mjcf_name="global_d435_camera",
            frame_id="global_d435_optical_frame",
            rgb=ImageStreamSpec(
                topic="/d435/color/image_raw",
                info_topic="/d435/color/camera_info",
                observation_key="observation.images.front",
                preview_topic="/d435/color/image_preview/compressed",
            ),
        ),
        CameraSpec(
            camera_id="wrist",
            mjcf_name="wrist_cam",
            frame_id="wrist_cam_optical_frame",
            rgb=ImageStreamSpec(
                topic="/wrist_cam/image_raw",
                info_topic="/wrist_cam/camera_info",
                observation_key="observation.images.wrist",
                preview_topic="/wrist_cam/image_preview/compressed",
            ),
        ),
    )
    primary_camera_id = "front"
    language_instruction = "拉开抽屉，把红色方块放进抽屉，再关上抽屉"

    def reset(self, model, data, rng):
        """Close the passive drawer and randomize the cube in its spawn area."""
        import mujoco

        # Close the drawer and clear any residual velocity.
        joint_id = mujoco.mj_name2id(
            model, mujoco.mjtObj.mjOBJ_JOINT, "drawer_slide"
        )
        if joint_id < 0:
            raise ValueError("model is missing task joint 'drawer_slide'")
        q = int(model.jnt_qposadr[joint_id])
        v = int(model.jnt_dofadr[joint_id])
        data.qpos[q] = model.qpos0[q]
        data.qvel[v] = 0.0
        data.qacc_warmstart[v] = 0.0

        # Sample the cube's x/y inside the scene named spawn area.
        site_id = mujoco.mj_name2id(
            model, mujoco.mjtObj.mjOBJ_SITE, "cube_spawn_area"
        )
        if site_id < 0:
            raise ValueError("model is missing site cube_spawn_area")
        center = data.site_xpos[site_id]
        size = model.site_size[site_id]
        x_range = (center[0] - size[0], center[0] + size[0])
        y_range = (center[1] - size[1], center[1] + size[1])
        cube_xy = (rng.uniform(*x_range), rng.uniform(*y_range))

        cube_name, joint_name, _ = CUBE
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
        data.qacc_warmstart[dof_address : dof_address + 6] = 0.0
        data.qpos[qpos_address] = cube_xy[0]
        data.qpos[qpos_address + 1] = cube_xy[1]
        position = tuple(
            float(value) for value in data.qpos[qpos_address : qpos_address + 3]
        )

        return (
            f"randomized objects: {cube_name}="
            f"({position[0]:.3f}, {position[1]:.3f}, {position[2]:.3f})"
        )
