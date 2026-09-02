"""MuJoCo model state, robot control, and task reset behavior."""

JOINTS = (
    "shoulder_pan",
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
    "wrist_roll",
    "gripper",
)


class MujocoSimulation:
    """Own one MuJoCo model and its mutable physics state."""

    def __init__(self, model_path, assets, task, rng):
        import mujoco

        self._mujoco = mujoco
        self.model = mujoco.MjModel.from_xml_path(str(model_path), assets=assets)
        self.data = mujoco.MjData(self.model)
        self._task = task
        self._rng = rng
        self._joint_qpos_addresses = {}
        self._actuator_ids = {}
        self._targets = {}
        self._configure_robot()
        self.apply_targets()
        mujoco.mj_forward(self.model, self.data)

    def _configure_robot(self):
        for name in JOINTS:
            joint_id = self._mujoco.mj_name2id(
                self.model, self._mujoco.mjtObj.mjOBJ_JOINT, name
            )
            actuator_id = self._mujoco.mj_name2id(
                self.model, self._mujoco.mjtObj.mjOBJ_ACTUATOR, name
            )
            if joint_id < 0 or actuator_id < 0:
                continue
            qpos_address = int(self.model.jnt_qposadr[joint_id])
            self._joint_qpos_addresses[name] = (joint_id, qpos_address)
            self._actuator_ids[name] = actuator_id
            self._targets[name] = float(self.data.qpos[qpos_address])

        missing = set(JOINTS) - set(self._targets)
        if missing:
            raise ValueError(
                f"model is missing joint or actuator definitions: {sorted(missing)}"
            )

    def set_targets(self, names, positions):
        """Clamp and store the latest requested robot joint targets."""
        for name, position in zip(names, positions):
            joint = self._joint_qpos_addresses.get(name)
            if joint is None:
                continue
            joint_id, _ = joint
            lower, upper = self.model.jnt_range[joint_id]
            self._targets[name] = min(upper, max(lower, position))

    def apply_targets(self):
        """Copy the stored targets into MuJoCo position actuators."""
        for name, target in self._targets.items():
            self.data.ctrl[self._actuator_ids[name]] = target

    def step(self):
        self._mujoco.mj_step(self.model, self.data)

    def joint_positions(self):
        return [
            float(self.data.qpos[self._joint_qpos_addresses[name][1]])
            for name in JOINTS
        ]

    def action_positions(self):
        return [float(self.data.ctrl[self._actuator_ids[name]]) for name in JOINTS]

    def reset_task(self):
        summary = self._task.reset(self.model, self.data, self._rng)
        self._mujoco.mj_forward(self.model, self.data)
        return summary
