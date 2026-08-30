#!/usr/bin/env python3
"""Run the SO-101 MuJoCo physics scene in MuJoCo's native viewer."""

import time
from pathlib import Path

import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import JointState


JOINTS = (
    "shoulder_pan",
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
    "wrist_roll",
    "gripper",
)


class SO101MujocoViewer(Node):
    """Drive the SO-101 position actuators from ROS joint-state targets."""

    def __init__(self) -> None:
        super().__init__("so101_mujoco_viewer")
        self.declare_parameter("model_path", "")
        self.declare_parameter("joint_state_topic", "joint_states")
        self.declare_parameter("sim_joint_state_topic", "sim/joint_states")
        self.declare_parameter("state_publish_rate", 50.0)
        self.declare_parameter("realtime_factor", 1.0)
        self.declare_parameter("max_substeps", 20)

        input_topic = self.get_parameter("joint_state_topic").value
        output_topic = self.get_parameter("sim_joint_state_topic").value
        self._target_subscription = self.create_subscription(
            JointState, input_topic, self._on_joint_state, 10
        )
        self._state_publisher = self.create_publisher(JointState, output_topic, 10)
        self._clock_publisher = self.create_publisher(Clock, "/clock", 10)

        self._model = None
        self._data = None
        self._joint_qpos_addresses = {}
        self._actuator_ids = {}
        self._targets = {}
        self._last_state_publish_time = float("-inf")
        self.get_logger().info(
            f"receiving targets on '{input_topic}' and publishing simulated state on "
            f"'{output_topic}'"
        )

    def resolve_model_path(self) -> Path:
        """Return the requested model file or the packaged default scene."""
        configured_path = self.get_parameter("model_path").value.strip()
        if configured_path:
            path = Path(configured_path).expanduser()
        else:
            path = self.default_model_path()

        path = path.resolve()
        if not path.is_file():
            raise FileNotFoundError(f"MuJoCo model file does not exist: {path}")
        return path

    @staticmethod
    def default_model_path() -> Path:
        """Return the installed default scene path."""
        package_share = Path(get_package_share_directory("so101_mujoco_sim"))
        return (package_share / "mujoco" / "scene.xml").resolve()

    @staticmethod
    def description_mesh_assets() -> dict[str, bytes]:
        """Read shared meshes for MuJoCo's virtual file system."""
        description_share = Path(get_package_share_directory("so101_description"))
        mesh_dir = description_share / "meshes"
        assets = {mesh.name: mesh.read_bytes() for mesh in mesh_dir.glob("*.stl")}
        if not assets:
            raise FileNotFoundError(f"no STL meshes found in {mesh_dir}")
        return assets

    def _configure_model(self, model, data) -> None:
        """Cache the qpos and actuator slots of every SO-101 joint."""
        import mujoco

        self._model = model
        self._data = data
        self._joint_qpos_addresses = {}
        self._actuator_ids = {}
        self._targets = {}

        for name in JOINTS:
            joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
            actuator_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, name)
            if joint_id < 0 or actuator_id < 0:
                continue
            qpos_address = model.jnt_qposadr[joint_id]
            self._joint_qpos_addresses[name] = (joint_id, qpos_address)
            self._actuator_ids[name] = actuator_id
            self._targets[name] = data.qpos[qpos_address]

        missing = set(JOINTS) - set(self._targets)
        if missing:
            raise ValueError(f"model is missing joint or actuator definitions: {sorted(missing)}")

    def _on_joint_state(self, message: JointState) -> None:
        """Store the latest leader pose as PD actuator targets, never as qpos."""
        if self._model is None:
            return

        for name, position in zip(message.name, message.position):
            joint = self._joint_qpos_addresses.get(name)
            if joint is None:
                continue
            joint_id, _ = joint
            lower, upper = self._model.jnt_range[joint_id]
            self._targets[name] = min(upper, max(lower, position))

    def _apply_targets(self) -> None:
        """Copy the most recent leader pose into MuJoCo position controls."""
        for name, target in self._targets.items():
            self._data.ctrl[self._actuator_ids[name]] = target

    @staticmethod
    def _sim_stamp(sim_time: float):
        sec = int(sim_time)
        nanosec = int((sim_time - sec) * 1_000_000_000)
        return sec, nanosec

    def _publish_simulated_state(self) -> None:
        """Publish actual post-physics joint positions and MuJoCo simulation time."""
        sec, nanosec = self._sim_stamp(self._data.time)

        clock = Clock()
        clock.clock.sec = sec
        clock.clock.nanosec = nanosec
        self._clock_publisher.publish(clock)

        publish_rate = self.get_parameter("state_publish_rate").value
        if publish_rate <= 0 or self._data.time - self._last_state_publish_time < 1.0 / publish_rate:
            return

        message = JointState()
        message.header.stamp.sec = sec
        message.header.stamp.nanosec = nanosec
        message.name = list(JOINTS)
        message.position = [
            float(self._data.qpos[self._joint_qpos_addresses[name][1]]) for name in JOINTS
        ]
        self._state_publisher.publish(message)
        self._last_state_publish_time = self._data.time

    def run(self) -> None:
        """Load the physics scene and advance it in real time until Viewer closes."""
        try:
            import mujoco
            import mujoco.viewer
        except ImportError as exc:
            raise RuntimeError(
                "MuJoCo is not installed for the ROS Python interpreter. "
                "Install it with: /usr/bin/python3 -m pip install --user "
                "--break-system-packages mujoco==3.3.7"
            ) from exc

        model_path = self.resolve_model_path()
        assets = (
            self.description_mesh_assets()
            if model_path == self.default_model_path()
            else None
        )
        model = mujoco.MjModel.from_xml_path(str(model_path), assets=assets)
        data = mujoco.MjData(model)
        self._configure_model(model, data)
        self._apply_targets()
        mujoco.mj_forward(model, data)

        timestep = model.opt.timestep
        realtime_factor = self.get_parameter("realtime_factor").value
        max_substeps = self.get_parameter("max_substeps").value
        if realtime_factor <= 0:
            raise ValueError("realtime_factor must be greater than zero")
        if max_substeps < 1:
            raise ValueError("max_substeps must be at least one")
        self.get_logger().info(
            f"loaded physics model ({model.njnt} joints, {model.nu} actuators, "
            f"timestep={timestep}s)"
        )

        accumulator = 0.0
        last_wall_time = time.perf_counter()
        with mujoco.viewer.launch_passive(model, data) as native_viewer:
            while rclpy.ok() and native_viewer.is_running():
                rclpy.spin_once(self, timeout_sec=0.001)
                now = time.perf_counter()
                accumulator += (now - last_wall_time) * realtime_factor
                last_wall_time = now

                substeps = 0
                while accumulator >= timestep and substeps < max_substeps:
                    self._apply_targets()
                    mujoco.mj_step(model, data)
                    accumulator -= timestep
                    substeps += 1
                if accumulator >= timestep:
                    accumulator = 0.0
                    self.get_logger().warn("physics loop fell behind; discarded excess time")

                self._publish_simulated_state()
                native_viewer.sync()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SO101MujocoViewer()
    try:
        node.run()
    except Exception as exc:  # noqa: BLE001
        node.get_logger().fatal(str(exc))
        raise
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
