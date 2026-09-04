#!/usr/bin/env python3
"""Run a trained LeRobot ACT policy against the SO-101 simulation."""

from pathlib import Path

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


JOINTS = (
    "shoulder_pan",
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
    "wrist_roll",
    "gripper",
)


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


class SO101ACTPolicy(Node):
    """Bridge MuJoCo observations to a LeRobot ACT policy action."""

    def __init__(self) -> None:
        super().__init__("so101_act_policy")
        self.declare_parameter("policy_path", "")
        self.declare_parameter("device", "cuda")
        self.declare_parameter("inference_rate", 30.0)
        self.declare_parameter("front_image_topic", "/d435/color/image_raw")
        self.declare_parameter("wrist_image_topic", "/wrist_cam/image_raw")
        self.declare_parameter("state_topic", "/sim/joint_states")
        self.declare_parameter("command_topic", "/joint_states")

        policy_path = Path(str(self.get_parameter("policy_path").value)).expanduser()
        if not policy_path.is_dir():
            raise FileNotFoundError(f"policy directory does not exist: {policy_path}")
        device_name = str(self.get_parameter("device").value)
        if device_name == "cuda" and not torch.cuda.is_available():
            raise RuntimeError("device is cuda but CUDA is unavailable")
        self._device = torch.device(device_name)

        self._policy = ACTPolicy.from_pretrained(policy_path).to(self._device)
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
        positions = dict(zip(message.name, message.position))
        if all(joint in positions for joint in JOINTS):
            self._state = np.asarray(
                [positions[joint] for joint in JOINTS], dtype=np.float32
            )

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
            robot_type="so101",
        ).squeeze(0).cpu().numpy()
        if action.shape != (len(JOINTS),) or not np.isfinite(action).all():
            self.get_logger().error(f"invalid policy action: shape={action.shape}")
            return
        message = JointState()
        message.header.stamp = self.get_clock().now().to_msg()
        message.name = list(JOINTS)
        message.position = action.tolist()
        self._publisher.publish(message)


def main(args=None):
    rclpy.init(args=args)
    node = SO101ACTPolicy()
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
