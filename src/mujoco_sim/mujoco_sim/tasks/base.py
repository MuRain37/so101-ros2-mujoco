"""Common interface for scene-specific training tasks."""


class SimulationTask:
    """Base interface implemented by each MuJoCo training task."""

    task_id = ""
    scene_file = ""
    language_instruction = ""
    robot_id = "so101"

    def resolve_robot(self, requested="auto"):
        if requested not in ("auto", self.robot_id):
            raise ValueError(f"task {self.task_id!r} requires robot {self.robot_id!r}, not {requested!r}")
        return self.robot_id

    def scene_for(self, robot_id="auto"):
        self.resolve_robot(robot_id)
        return self.scene_file

    def reset(self, model, data, rng):
        raise NotImplementedError

    def evaluate(self, model, data):
        """Return a future success result; detection is not implemented yet."""
        return None
