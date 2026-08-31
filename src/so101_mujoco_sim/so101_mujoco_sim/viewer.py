#!/usr/bin/env python3
"""Run the SO-101 MuJoCo physics scene in MuJoCo's native viewer."""

import time
from math import radians, tan
from pathlib import Path

import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import CameraInfo, Image, JointState


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
        self.declare_parameter("camera_enabled", True)
        self.declare_parameter("camera_name", "wrist_cam")
        self.declare_parameter("camera_frame_id", "wrist_cam_optical_frame")
        self.declare_parameter("camera_image_topic", "wrist_cam/image_raw")
        self.declare_parameter("camera_info_topic", "wrist_cam/camera_info")
        self.declare_parameter("camera_width", 640)
        self.declare_parameter("camera_height", 480)
        self.declare_parameter("camera_publish_rate", 30.0)

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
        self._camera_image_publisher = None
        self._camera_info_publisher = None
        self._camera_id = -1
        self._camera_name = ""
        self._camera_frame_id = ""
        self._camera_width = 0
        self._camera_height = 0
        self._camera_publish_period = 0.0
        self._next_camera_publish_time = 0.0
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

    def _create_camera_renderer(self, mujoco, model):
        """Create the offscreen renderer and ROS publishers for the wrist camera."""
        if not self.get_parameter("camera_enabled").value:
            self.get_logger().info("wrist camera rendering is disabled")
            return None

        self._camera_name = self.get_parameter("camera_name").value.strip()
        self._camera_frame_id = self.get_parameter("camera_frame_id").value.strip()
        self._camera_width = int(self.get_parameter("camera_width").value)
        self._camera_height = int(self.get_parameter("camera_height").value)
        publish_rate = float(self.get_parameter("camera_publish_rate").value)
        if not self._camera_name:
            raise ValueError("camera_name must not be empty")
        if not self._camera_frame_id:
            raise ValueError("camera_frame_id must not be empty")
        if self._camera_width < 1 or self._camera_height < 1:
            raise ValueError("camera_width and camera_height must be positive")
        if publish_rate <= 0:
            raise ValueError("camera_publish_rate must be greater than zero")

        self._camera_id = mujoco.mj_name2id(
            model, mujoco.mjtObj.mjOBJ_CAMERA, self._camera_name
        )
        if self._camera_id < 0:
            raise ValueError(f"model does not contain camera {self._camera_name!r}")

        model.vis.global_.offwidth = max(model.vis.global_.offwidth, self._camera_width)
        model.vis.global_.offheight = max(model.vis.global_.offheight, self._camera_height)
        image_topic = self.get_parameter("camera_image_topic").value
        info_topic = self.get_parameter("camera_info_topic").value
        self._camera_image_publisher = self.create_publisher(
            Image, image_topic, qos_profile_sensor_data
        )
        self._camera_info_publisher = self.create_publisher(
            CameraInfo, info_topic, qos_profile_sensor_data
        )
        self._camera_publish_period = 1.0 / publish_rate
        self._next_camera_publish_time = 0.0
        self.get_logger().info(
            f"publishing {self._camera_name!r} at {self._camera_width}x"
            f"{self._camera_height} {publish_rate:g} Hz on {image_topic!r}"
        )
        return mujoco.Renderer(
            model, height=self._camera_height, width=self._camera_width
        )

    def _publish_camera_frame(self, renderer) -> None:
        """Render and publish one RGB frame according to MuJoCo simulation time."""
        if renderer is None or self._data.time + 1e-12 < self._next_camera_publish_time:
            return

        renderer.update_scene(self._data, camera=self._camera_name)
        pixels = renderer.render()
        sec, nanosec = self._sim_stamp(self._data.time)

        image = Image()
        image.header.stamp.sec = sec
        image.header.stamp.nanosec = nanosec
        image.header.frame_id = self._camera_frame_id
        image.height = self._camera_height
        image.width = self._camera_width
        image.encoding = "rgb8"
        image.is_bigendian = False
        image.step = self._camera_width * 3
        image.data = pixels.tobytes()
        self._camera_image_publisher.publish(image)

        fovy = radians(float(self._model.cam_fovy[self._camera_id]))
        focal = 0.5 * self._camera_height / tan(0.5 * fovy)
        cx = 0.5 * (self._camera_width - 1)
        cy = 0.5 * (self._camera_height - 1)
        info = CameraInfo()
        info.header.stamp.sec = sec
        info.header.stamp.nanosec = nanosec
        info.header.frame_id = self._camera_frame_id
        info.height = self._camera_height
        info.width = self._camera_width
        info.distortion_model = "plumb_bob"
        info.d = [0.0, 0.0, 0.0, 0.0, 0.0]
        info.k = [focal, 0.0, cx, 0.0, focal, cy, 0.0, 0.0, 1.0]
        info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        info.p = [focal, 0.0, cx, 0.0, 0.0, focal, cy, 0.0, 0.0, 0.0, 1.0, 0.0]
        self._camera_info_publisher.publish(info)

        while self._next_camera_publish_time <= self._data.time + 1e-12:
            self._next_camera_publish_time += self._camera_publish_period

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

        renderer = self._create_camera_renderer(mujoco, model)
        accumulator = 0.0
        last_wall_time = time.perf_counter()
        try:
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
                        self.get_logger().warn(
                            "physics loop fell behind; discarded excess time"
                        )

                    self._publish_simulated_state()
                    self._publish_camera_frame(renderer)
                    native_viewer.sync()
        finally:
            if renderer is not None:
                renderer.close()


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
