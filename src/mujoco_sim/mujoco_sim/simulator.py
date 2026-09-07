#!/usr/bin/env python3
"""Run the SO-101 MuJoCo physics scene in MuJoCo's native viewer."""

import random
import time
from pathlib import Path

import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger

from .camera import MujocoCameraPublisher
from .simulation import MujocoSimulation
from robot_adapters import get_robot
from robot_adapters.models import legacy_assets
from robot_adapters.retargeting import parse_workspace
from .tasks import create_task


class MujocoSimulator(Node):
    """Drive the selected robot actuators from epoch-tagged joint targets."""

    def __init__(self) -> None:
        super().__init__("mujoco_simulator")
        self.declare_parameter("model_path", "")
        self.declare_parameter("task_id", "red_cube_to_red_target")
        self._task = create_task(str(self.get_parameter("task_id").value))
        self.declare_parameter("robot_id", "auto")
        self._robot_id = self._task.resolve_robot(str(self.get_parameter("robot_id").value))
        self._robot = get_robot(self._robot_id)
        self._epoch = 0
        self.declare_parameter("joint_state_topic", "/robot/joint_targets")
        self.declare_parameter("sim_joint_state_topic", "sim/joint_states")
        self.declare_parameter("state_publish_rate", 50.0)
        self.declare_parameter("realtime_factor", 1.0)
        self.declare_parameter("max_substeps", 20)
        self.declare_parameter("random_seed", -1)
        self.declare_parameter(
            "follower_workspace", "0.10 0.45 -0.25 0.25 0.015 0.35"
        )
        self.declare_parameter("gripper_open_fraction", 1.0)
        self.declare_parameter("camera_enabled", True)
        self._cameras = (
            [MujocoCameraPublisher(self, spec) for spec in self._task.cameras]
            if self.get_parameter("camera_enabled").value else []
        )

        input_topic = self.get_parameter("joint_state_topic").value
        output_topic = self.get_parameter("sim_joint_state_topic").value
        self._target_subscription = self.create_subscription(
            JointState, input_topic, self._on_joint_state, 10
        )
        self._action_publisher = self.create_publisher(JointState, "sim/action", 10)
        self._state_publisher = self.create_publisher(JointState, output_topic, 10)
        self._clock_publisher = self.create_publisher(Clock, "/clock", 10)

        self._simulation = None
        self._last_state_publish_time = float("-inf")
        seed = int(self.get_parameter("random_seed").value)
        self._rng = random.Random(None if seed < 0 else seed)
        self._reset_service = self.create_service(
            Trigger, "sim/reset_task", self._on_reset_task
        )
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

    def default_model_path(self) -> Path:
        """Return the selected task default scene path."""
        package_share = Path(get_package_share_directory("mujoco_sim"))
        return (package_share / "mujoco" / self._task.scene_for(self._robot_id)).resolve()

    @staticmethod
    def default_model_assets() -> dict[str, bytes]:
        """Read robot and global-camera meshes for MuJoCo's virtual file system."""
        return legacy_assets()

    def _on_joint_state(self, message: JointState) -> None:
        """Store the latest leader pose as PD actuator targets, never as qpos."""
        if self._simulation is None:
            return
        if message.header.frame_id != self.command_epoch:
            return
        try:
            self._simulation.set_targets(message.name, message.position)
        except ValueError as error:
            self.get_logger().warning(str(error), throttle_duration_sec=2.0)

    @property
    def command_epoch(self):
        # Joint-space messages use frame_id as an epoch token, not a TF frame.
        return f"{self._robot_id}/epoch/{self._epoch}"

    @staticmethod
    def _sim_stamp(sim_time: float):
        sec = int(sim_time)
        nanosec = int((sim_time - sec) * 1_000_000_000)
        return sec, nanosec

    def _publish_simulated_state(self) -> None:
        """Publish actual post-physics joint positions and MuJoCo simulation time."""
        simulation = self._simulation
        sec, nanosec = self._sim_stamp(simulation.data.time)

        clock = Clock()
        clock.clock.sec = sec
        clock.clock.nanosec = nanosec
        self._clock_publisher.publish(clock)

        publish_rate = self.get_parameter("state_publish_rate").value
        if (
            publish_rate <= 0
            or simulation.data.time - self._last_state_publish_time < 1.0 / publish_rate
        ):
            return

        message = JointState()
        message.header.stamp.sec = sec
        message.header.stamp.nanosec = nanosec
        message.header.frame_id = self.command_epoch
        message.name = list(self._robot.joint_names)
        message.position = simulation.joint_positions()
        self._state_publisher.publish(message)

        action = JointState()
        action.header.stamp.sec = sec
        action.header.stamp.nanosec = nanosec
        action.header.frame_id = self.command_epoch
        action.name = list(self._robot.joint_names)
        action.position = simulation.action_positions()
        self._action_publisher.publish(action)
        self._last_state_publish_time = simulation.data.time

    def _reset_task(self, reason: str) -> None:
        """Run the selected task's reset logic and refresh MuJoCo state."""
        summary = self._simulation.reset_task()
        self._epoch += 1
        self._last_state_publish_time = float("-inf")
        self.get_logger().info(f"{reason}: {summary}")

    def _on_reset_task(self, request, response):
        """Reset the robot and selected task before acknowledging the request."""
        del request
        if self._simulation is None:
            response.success = False
            response.message = "simulation model is not loaded"
            return response
        self._reset_task("episode reset")
        response.success = True
        response.message = "episode reset complete"
        return response

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
            self.default_model_assets()
            if model_path == self.default_model_path()
            else None
        )
        simulation = MujocoSimulation(
            model_path,
            assets,
            self._task,
            self._rng,
            self._robot_id,
            follower_workspace=parse_workspace(
                self.get_parameter("follower_workspace").value
            ),
            gripper_open_fraction=float(
                self.get_parameter("gripper_open_fraction").value
            ),
        )
        self._simulation = simulation
        model = simulation.model
        data = simulation.data

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

        try:
            for camera in self._cameras:
                camera.start(mujoco, model)
        except Exception:
            for camera in self._cameras:
                camera.close()
            raise
        accumulator = 0.0
        last_wall_time = time.perf_counter()
        try:
            with mujoco.viewer.launch_passive(model, data) as native_viewer:
                self._reset_task("startup")
                native_viewer.sync()
                last_sim_time = data.time
                while rclpy.ok() and native_viewer.is_running():
                    rclpy.spin_once(self, timeout_sec=0.001)
                    if data.time < last_sim_time:
                        self._reset_task("reset")
                        for camera in self._cameras:
                            camera.reset_timing(data.time)
                        accumulator = 0.0
                        last_wall_time = time.perf_counter()

                    now = time.perf_counter()
                    accumulator += (now - last_wall_time) * realtime_factor
                    last_wall_time = now

                    substeps = 0
                    while accumulator >= timestep and substeps < max_substeps:
                        simulation.apply_targets()
                        simulation.step()
                        accumulator -= timestep
                        substeps += 1
                    if accumulator >= timestep:
                        accumulator = 0.0
                        self.get_logger().warn(
                            "physics loop fell behind; discarded excess time"
                        )

                    self._publish_simulated_state()
                    for camera in self._cameras:
                        camera.publish_if_due(data, self._sim_stamp)
                    last_sim_time = data.time
                    native_viewer.sync()
        finally:
            for camera in self._cameras:
                camera.close()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MujocoSimulator()
    try:
        node.run()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception as exc:  # noqa: BLE001
        node.get_logger().fatal(str(exc))
        raise
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
