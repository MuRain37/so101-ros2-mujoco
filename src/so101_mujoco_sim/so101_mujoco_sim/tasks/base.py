"""Common interface for scene-specific training tasks."""


class SimulationTask:
    """Base interface implemented by each MuJoCo training task."""

    task_id = ""
    scene_file = ""

    def reset(self, model, data, rng):
        raise NotImplementedError

    def evaluate(self, model, data):
        """Return a future success result; detection is not implemented yet."""
        return None
