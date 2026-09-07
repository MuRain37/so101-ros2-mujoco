#!/usr/bin/env python3
"""Run a dimension-compatible LeRobot ACT policy against the selected robot."""

from pathlib import Path
import json

import numpy as np
import rclpy
import torch
from lerobot.policies.act.modeling_act import ACTPolicy
from lerobot.policies.factory import make_pre_post_processors
from lerobot.utils.control_utils import predict_action
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Image, JointState
from std_srvs.srv import Trigger
from robot_adapters import get_robot


def validate_policy(config, robot, policy_path):
    expected = (len(robot.joint_names),)
    for key, features in (("observation.state", config.input_features),
                          ("action", config.output_features)):
        if key not in features or tuple(features[key].shape) != expected:
            raise ValueError(f"policy {key} must have shape {expected} for {robot.robot_id}")
    metadata = Path(policy_path) / "robot.json"
    if metadata.is_file():
        supplied = json.loads(metadata.read_text())
        for key, value in robot.metadata().items():
            if supplied.get(key) != value:
                raise ValueError(f"policy robot metadata mismatch: {key}")


def image_array(message: Image) -> np.ndarray:
    """Decode a ROS RGB image without cv_bridge's NumPy 1.x dependency."""
    if message.encoding not in ("rgb8", "bgr8"):
        raise ValueError(f"unsupported image encoding: {message.encoding}")
    rows = np.frombuffer(message.data, dtype=np.uint8).reshape(
        message.height, message.step
    )
    image = rows[:, : message.width * 3].reshape(
        message.height, message.width, 3
    ).copy()
    return image if message.encoding == "rgb8" else image[:, :, ::-1].copy()


class ACTPolicyNode(Node):
    """Bridge MuJoCo observations to a LeRobot ACT policy action."""

    def __init__(self) -> None:
        super().__init__("act_policy")
        self.declare_parameter("policy_path", "")
        self.declare_parameter("robot_id", "so101")
        self._robot = get_robot(str(self.get_parameter("robot_id").value))
        self._epoch = None
        self.declare_parameter("device", "cuda")
        self.declare_parameter("inference_rate", 30.0)
        self.declare_parameter("front_image_topic", "/d435/color/image_raw")
        self.declare_parameter("wrist_image_topic", "/wrist_cam/image_raw")
        self.declare_parameter("state_topic", "/sim/joint_states")
        self.declare_parameter("command_topic", "/robot/joint_targets")

        policy_path = Path(str(self.get_parameter("policy_path").value)).expanduser()
        if not policy_path.is_dir():
            raise FileNotFoundError(f"policy directory does not exist: {policy_path}")
        device_name = str(self.get_parameter("device").value)
        if device_name == "cuda" and not torch.cuda.is_available():
            raise RuntimeError("device is cuda but CUDA is unavailable")
        self._device = torch.device(device_name)

        from lerobot.configs.policies import PreTrainedConfig
        config = PreTrainedConfig.from_pretrained(policy_path)
        validate_policy(config, self._robot, policy_path)
        self._policy = ACTPolicy.from_pretrained(policy_path, config=config).to(self._device)
        self._preprocessor, self._postprocessor = make_pre_post_processors(
            policy_cfg=self._policy.config,
            pretrained_path=str(policy_path),
            preprocessor_overrides={
                "device_processor": {"device": device_name}
            },
        )
        self._policy.reset()

        self._front_image = None
        self._wrist_image = None
        self._state = None
        self._waiting_logged = False
        self._reset_future = None
        self.create_subscription(
            Image,
            str(self.get_parameter("front_image_topic").value),
            self._on_front_image,
            1,
        )
        self.create_subscription(
            Image,
            str(self.get_parameter("wrist_image_topic").value),
            self._on_wrist_image,
            1,
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
        self.get_logger().info(f"loaded ACT policy from {policy_path} on {device_name}")

    def _on_front_image(self, message: Image) -> None:
        self._front_image = image_array(message)

    def _on_wrist_image(self, message: Image) -> None:
        self._wrist_image = image_array(message)

    def _on_state(self, message: JointState) -> None:
        if not message.header.frame_id.startswith(self._robot.robot_id + "/epoch/"):
            return
        if self._epoch != message.header.frame_id:
            self._epoch = message.header.frame_id
            self._policy.reset()
            self._clear_observations()
        try:
            self._state = self._robot.ordered(message.name, message.position).astype(np.float32)
        except ValueError as error:
            self.get_logger().warning(str(error), throttle_duration_sec=2.0)

    def _on_reset_episode(self, request, response):
        """Pause inference and ask MuJoCo to start a fresh episode."""
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
        self._front_image = None
        self._wrist_image = None
        self._state = None
        self._waiting_logged = False

    def _finish_reset(self) -> bool:
        """Return true while inference must remain paused for a reset."""
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

        # Discard observations delivered while MuJoCo handled the reset.
        self._clear_observations()
        return True

    def _infer(self) -> None:
        if self._finish_reset():
            return
        if any(
            item is None
            for item in (self._front_image, self._wrist_image, self._state)
        ):
            if not self._waiting_logged:
                self.get_logger().info(
                    "waiting for both cameras and simulated joint state"
                )
                self._waiting_logged = True
            return
        observation = {
            "observation.images.front": self._front_image,
            "observation.images.wrist": self._wrist_image,
            "observation.state": self._state,
        }
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
