"""MuJoCo physics state and robot-independent actuator control."""
import numpy as np

from robot_adapters import get_robot
from robot_adapters.models import load_robot_model, load_scene
from robot_adapters.retargeting import PoseRetargeter


class MujocoSimulation:
    def __init__(
        self,
        model_path,
        assets,
        task,
        rng,
        robot_id="auto",
        follower_workspace=(0.10, 0.45, -0.25, 0.25, 0.015, 0.35),
        gripper_open_fraction=1.0,
    ):
        import mujoco
        self._mujoco = mujoco
        self.model = load_scene(model_path, assets)
        self.data = mujoco.MjData(self.model)
        self.robot = get_robot(task.resolve_robot(robot_id)).bind(self.model)
        self._task, self._rng = task, rng
        self.reset_positions = np.array(self.robot.home, dtype=float)
        if task.reset_source_robot_id is not None:
            source = get_robot(task.reset_source_robot_id)
            mapper = PoseRetargeter(
                load_robot_model(source),
                self.model,
                self.robot.robot_id,
                follower_workspace=follower_workspace,
                gripper_open_fraction=gripper_open_fraction,
                source_robot_id=source.robot_id,
            )
            self.set_reset_positions(
                mapper.goal(source.home, self.robot.home)
            )
        self.reset_robot()
        mujoco.mj_forward(self.model, self.data)

    def set_targets(self, names, positions):
        self._targets = self.robot.clamp(self.robot.ordered(names, positions))

    def apply_targets(self):
        self.robot.apply(self.data, self._targets)

    def set_reset_positions(self, positions):
        positions = np.asarray(positions, dtype=float)
        if positions.shape != (len(self.robot.joint_names),):
            raise ValueError("reset pose has the wrong number of joints")
        if not np.isfinite(positions).all():
            raise ValueError("reset pose joints must be finite")
        self.reset_positions = self.robot.clamp(positions)

    def reset_robot(self):
        self.robot.reset(self.data)
        self.robot.set_positions(self.data, self.reset_positions)
        self.robot.apply(self.data, self.reset_positions)
        self._targets = self.reset_positions.copy()

    def step(self):
        self._mujoco.mj_step(self.model, self.data)

    def joint_positions(self):
        return self.robot.positions(self.data).tolist()

    def action_positions(self):
        # Public joint targets, not gripper actuator bytes.
        return self._targets.tolist()

    def reset_task(self):
        self.reset_robot()
        summary = self._task.reset(self.model, self.data, self._rng)
        self._mujoco.mj_forward(self.model, self.data)
        return summary
