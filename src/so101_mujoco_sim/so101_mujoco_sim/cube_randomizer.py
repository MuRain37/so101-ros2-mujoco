"""Randomize the red and blue cube positions in a MuJoCo scene."""

from __future__ import annotations

import random

# The target-facing edges are y=-0.115 and y=0.115, giving a=0.230 m.
# Inset that square by 0.025 m on every side, retaining the centre (0.25, 0.0).
CUBE_X_RANGE = (0.16, 0.34)
CUBE_Y_RANGE = (-0.09, 0.09)
CUBE_SIZE = 0.024
CUBE_MINIMUM_GAP = 0.01
MAX_ATTEMPTS = 1_000

CUBE_NAMES = ("red_cube", "blue_cube")
JOINT_NAMES = ("red_cube_free", "blue_cube_free")


def randomize_cube_positions(
    model,
    data,
) -> dict[str, tuple[float, float, float]]:
    """Randomize both free-joint qpos values and return their new positions."""
    import mujoco

    minimum_separation = CUBE_SIZE + CUBE_MINIMUM_GAP
    for _ in range(MAX_ATTEMPTS):
        red_xy = (
            random.uniform(*CUBE_X_RANGE),
            random.uniform(*CUBE_Y_RANGE),
        )
        blue_xy = (
            random.uniform(*CUBE_X_RANGE),
            random.uniform(*CUBE_Y_RANGE),
        )
        if (
            abs(red_xy[0] - blue_xy[0]) >= minimum_separation
            or abs(red_xy[1] - blue_xy[1]) >= minimum_separation
        ):
            break
    else:
        raise RuntimeError("failed to find non-overlapping cube positions")

    positions = {}
    for cube_name, joint_name, (x, y) in zip(
        CUBE_NAMES, JOINT_NAMES, (red_xy, blue_xy)
    ):
        joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, joint_name)
        if joint_id < 0:
            raise ValueError(f"model is missing cube joint '{joint_name}'")
        qpos_address = int(model.jnt_qposadr[joint_id])
        data.qpos[qpos_address] = x
        data.qpos[qpos_address + 1] = y
        positions[cube_name] = (
            float(data.qpos[qpos_address]),
            float(data.qpos[qpos_address + 1]),
            float(data.qpos[qpos_address + 2]),
        )

    return positions
