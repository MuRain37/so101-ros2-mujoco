"""MuJoCo physics state and robot-independent actuator control."""
import numpy as np

from robot_adapters import get_robot
from robot_adapters.models import load_scene


class MujocoSimulation:
    def __init__(self, model_path, assets, task, rng, robot_id="auto"):
        import mujoco
        self._mujoco = mujoco
        self.model = load_scene(model_path, assets)
        self.data = mujoco.MjData(self.model)
        self.robot = get_robot(task.resolve_robot(robot_id)).bind(self.model)
        self._task, self._rng = task, rng
        self.reset_robot()
        mujoco.mj_forward(self.model, self.data)

    def set_targets(self, names, positions):
        self._targets = self.robot.clamp(self.robot.ordered(names, positions))

    def apply_targets(self):
        self.robot.apply(self.data, self._targets)

    def reset_robot(self):
        self.robot.reset(self.data)
        self._targets = np.array(self.robot.home, dtype=float)

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
