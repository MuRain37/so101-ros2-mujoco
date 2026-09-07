#!/usr/bin/env python3
"""Run a task-compatible LeRobot ACT policy against the selected robot."""

from pathlib import Path
import json

import numpy as np
import rclpy
import torch
from lerobot.policies.act.modeling_act import ACTPolicy
from lerobot.policies.factory import make_pre_post_processors
from lerobot.utils.control_utils import predict_action
from mujoco_sim.camera import depth_training_image, rgb_image_array
from mujoco_sim.tasks import create_task
from mujoco_sim.tasks.base import camera_streams
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Image, JointState
from std_srvs.srv import Trigger
from robot_adapters import get_robot


def validate_policy(config, robot, policy_path, task):
    expected = (len(robot.joint_names),)
    for key, features in (("observation.state", config.input_features),
                          ("action", config.output_features)):
        if key not in features or tuple(features[key].shape) != expected:
            raise ValueError(
                f"policy {key} must have shape {expected} for {robot.robot_id}"
            )

    expected_images = {
        stream.observation_key: (3, camera.height, camera.width)
        for camera, _, stream in camera_streams(task.cameras)
    }
    supplied_images = {
        key: tuple(feature.shape)
        for key, feature in config.input_features.items()
        if key.startswith("observation.images.")
    }
    if supplied_images != expected_images:
        raise ValueError(
            f"policy camera inputs {supplied_images} do not match task "
            f"camera inputs {expected_images}"
        )

    metadata = Path(policy_path) / "robot.json"
    if metadata.is_file():
        supplied = json.loads(metadata.read_text())
        for key, value in robot.metadata().items():
            if supplied.get(key) != value:
                raise ValueError(f"policy robot metadata mismatch: {key}")


class ACTPolicyNode(Node):
    """Bridge task-configured MuJoCo observations to an ACT policy action."""

    def __init__(self) -> None:
        super().__init__("act_policy")
        self.declare_parameter("policy_path", "")
        self.declare_parameter("task_id", "red_cube_to_red_target")
        self.declare_parameter("robot_id", "auto")
        self.declare_parameter("device", "cuda")
        self.declare_parameter("inference_rate", 30.0)
        self.declare_parameter("state_topic", "/sim/joint_states")
        self.declare_parameter("command_topic", "/robot/joint_targets")

        self._task = create_task(str(self.get_parameter("task_id").value))
        self._robot = get_robot(
            self._task.resolve_robot(str(self.get_parameter("robot_id").value))
        )
        self._epoch = None
        policy_path = Path(
            str(self.get_parameter("policy_path").value)
        ).expanduser()
        if not policy_path.is_dir():
            raise FileNotFoundError(f"policy directory does not exist: {policy_path}")
        device_name = str(self.get_parameter("device").value)
        if device_name == "cuda" and not torch.cuda.is_available():
            raise RuntimeError("device is cuda but CUDA is unavailable")
        self._device = torch.device(device_name)

        from lerobot.configs.policies import PreTrainedConfig
        config = PreTrainedConfig.from_pretrained(policy_path)
        validate_policy(config, self._robot, policy_path, self._task)
        self._policy = ACTPolicy.from_pretrained(
            policy_path, config=config
        ).to(self._device)
        self._preprocessor, self._postprocessor = make_pre_post_processors(
            policy_cfg=self._policy.config,
            pretrained_path=str(policy_path),
            preprocessor_overrides={
                "device_processor": {"device": device_name}
            },
        )
        self._policy.reset()

        self._images = {
            stream.observation_key: None
            for _, _, stream in camera_streams(self._task.cameras)
        }
        self._state = None
        self._waiting_logged = False
        self._reset_future = None
        self._image_subscriptions = []
        for camera, modality, stream in camera_streams(self._task.cameras):
            callback = self._image_callback(
                stream.observation_key, modality, camera.depth_range_m
            )
            self._image_subscriptions.append(
                self.create_subscription(Image, stream.topic, callback, 1)
            )
        self.create_subscription(
            JointState,
            str(self.get_parameter("state_topic").value),
            self._on_state,
            10,
        )
        self._publisher = self.create_publisher(
            JointState, str(self.get_parameter("command_topic").value), 10
        )
        self._sim_reset_client = self.create_client(Trigger, "/sim/reset_task")
        self._reset_service = self.create_service(
            Trigger, "/policy/reset_episode", self._on_reset_episode
        )
        rate = float(self.get_parameter("inference_rate").value)
        if rate <= 0:
            raise ValueError("inference_rate must be greater than zero")
        self.create_timer(1.0 / rate, self._infer)
        self.get_logger().info(
            f"loaded ACT policy for {self._task.task_id} from "
            f"{policy_path} on {device_name}"
        )

    def _image_callback(self, key, modality, depth_range_m):
        def callback(message):
            try:
                self._images[key] = (
                    rgb_image_array(message)
                    if modality == "rgb"
                    else depth_training_image(message, depth_range_m)
                )
            except ValueError as error:
                self.get_logger().warning(str(error), throttle_duration_sec=2.0)
        return callback

    def _on_state(self, message: JointState) -> None:
        if not message.header.frame_id.startswith(self._robot.robot_id + "/epoch/"):
            return
        if self._epoch != message.header.frame_id:
            self._epoch = message.header.frame_id
            self._policy.reset()
            self._clear_observations()
        try:
            self._state = self._robot.ordered(
                message.name, message.position
            ).astype(np.float32)
        except ValueError as error:
            self.get_logger().warning(str(error), throttle_duration_sec=2.0)

    def _on_reset_episode(self, request, response):
        del request
        if self._reset_future is not None:
            response.success = False
            response.message = "episode reset already in progress"
            return response
        if not self._sim_reset_client.service_is_ready():
            response.success = False
            response.message = "MuJoCo reset service is unavailable"
            return response

        self._policy.reset()
        self._clear_observations()
        self._reset_future = self._sim_reset_client.call_async(Trigger.Request())
        response.success = True
        response.message = "episode reset requested"
        return response

    def _clear_observations(self) -> None:
        for key in self._images:
            self._images[key] = None
        self._state = None
        self._waiting_logged = False

    def _finish_reset(self) -> bool:
        if self._reset_future is None:
            return False
        if not self._reset_future.done():
            return True

        future = self._reset_future
        self._reset_future = None
        try:
            result = future.result()
        except Exception as error:  # noqa: BLE001
            self.get_logger().error(f"episode reset failed: {error}")
        else:
            log = self.get_logger().info if result.success else self.get_logger().error
            log(result.message)
        self._clear_observations()
        return True

    def _infer(self) -> None:
        if self._finish_reset():
            return
        if self._state is None or any(image is None for image in self._images.values()):
            if not self._waiting_logged:
                self.get_logger().info(
                    "waiting for task cameras and simulated joint state"
                )
                self._waiting_logged = True
            return
        observation = {**self._images, "observation.state": self._state}
        action = predict_action(
            observation,
            self._policy,
            self._device,
            self._preprocessor,
            self._postprocessor,
            use_amp=False,
            robot_type=self._robot.robot_id,
        ).squeeze(0).cpu().numpy()
        if action.shape != (len(self._robot.joint_names),) or not np.isfinite(action).all():
            self.get_logger().error(f"invalid policy action: shape={action.shape}")
            return
        message = JointState()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = self._epoch
        message.name = list(self._robot.joint_names)
        message.position = action.tolist()
        self._publisher.publish(message)


def main(args=None):
    rclpy.init(args=args)
    node = ACTPolicyNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
