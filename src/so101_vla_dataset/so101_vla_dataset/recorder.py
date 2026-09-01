#!/usr/bin/env python3
"""Synchronize SO-101 ROS observations and save raw episodes."""

import json
from collections import deque
from pathlib import Path

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image, JointState
from std_srvs.srv import Trigger

CAMERA_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
    reliability=ReliabilityPolicy.RELIABLE,
)

JOINTS = ("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper")


class SO101DatasetRecorder(Node):
    """Image-driven raw recorder; LeRobot conversion runs in its own environment."""

    def __init__(self) -> None:
        super().__init__("so101_dataset_recorder")
        self.declare_parameter("output_dir", "data/raw")
        self.declare_parameter("task", "把红色方块放到红色区域")
        self.declare_parameter("sync_tolerance", 0.05)
        self.declare_parameter("front_topic", "/d435/color/image_raw")
        self.declare_parameter("wrist_topic", "/wrist_cam/image_raw")
        self.declare_parameter("state_topic", "/sim/joint_states")
        self.declare_parameter("action_topic", "/sim/action")
        self._recording = False
        self._episode_dir = None
        self._metadata_file = None
        self._frames = 0
        self._state_cache = deque(maxlen=100)
        self._action_cache = deque(maxlen=100)
        self._wrist_cache = deque(maxlen=10)
        self.create_subscription(Image, self.get_parameter("front_topic").value, self._on_front, CAMERA_QOS)
        self.create_subscription(Image, self.get_parameter("wrist_topic").value, self._on_wrist, CAMERA_QOS)
        self.create_subscription(JointState, self.get_parameter("state_topic").value, self._on_state, 50)
        self.create_subscription(JointState, self.get_parameter("action_topic").value, self._on_action, 50)
        self.create_service(Trigger, "dataset/start_episode", self._start_episode)
        self.create_service(Trigger, "dataset/stop_episode", self._stop_episode)
        self.get_logger().info("ready; call /dataset/start_episode to begin recording")

    @staticmethod
    def _stamp(message):
        return float(message.header.stamp.sec) + message.header.stamp.nanosec * 1e-9

    @staticmethod
    def _joint_vector(message):
        values = dict(zip(message.name, message.position))
        if any(name not in values for name in JOINTS):
            return None
        return np.asarray([values[name] for name in JOINTS], dtype=np.float32)

    @staticmethod
    def _rgb(message):
        if message.encoding not in ("rgb8", "RGB8") or message.step != message.width * 3:
            return None
        if len(message.data) != message.height * message.step:
            return None
        return np.frombuffer(message.data, dtype=np.uint8).reshape(message.height, message.width, 3).copy()

    @staticmethod
    def _nearest(cache, stamp, tolerance):
        if not cache:
            return None
        item = min(cache, key=lambda entry: abs(entry[0] - stamp))
        return item[1] if abs(item[0] - stamp) <= tolerance else None

    def _on_state(self, message):
        vector = self._joint_vector(message)
        if vector is not None:
            self._state_cache.append((self._stamp(message), vector))

    def _on_action(self, message):
        vector = self._joint_vector(message)
        if vector is not None:
            self._action_cache.append((self._stamp(message), vector))

    def _on_wrist(self, message):
        image = self._rgb(message)
        if image is not None:
            self._wrist_cache.append((self._stamp(message), image))

    def _on_front(self, message):
        if not self._recording:
            return
        front = self._rgb(message)
        if front is None:
            return
        stamp = self._stamp(message)
        tolerance = float(self.get_parameter("sync_tolerance").value)
        wrist = self._nearest(self._wrist_cache, stamp, tolerance)
        state = self._nearest(self._state_cache, stamp, tolerance)
        action = self._nearest(self._action_cache, stamp, tolerance)
        if wrist is None or state is None or action is None:
            return
        index = self._frames
        front_name = f"front_{index:06d}.npy"
        wrist_name = f"wrist_{index:06d}.npy"
        np.save(self._episode_dir / front_name, front)
        np.save(self._episode_dir / wrist_name, wrist)
        record = {
            "frame_index": index,
            "timestamp": stamp,
            "state": state.tolist(),
            "action": action.tolist(),
            "front": front_name,
            "wrist": wrist_name,
            "task": self.get_parameter("task").value,
        }
        self._metadata_file.write(json.dumps(record, ensure_ascii=False) + "\n")
        self._metadata_file.flush()
        self._frames += 1

    def _next_episode_dir(self, root):
        root.mkdir(parents=True, exist_ok=True)
        indices = [int(path.name.split("_")[-1]) for path in root.glob("episode_*") if path.name.split("_")[-1].isdigit()]
        return root / f"episode_{(max(indices) + 1 if indices else 0):06d}"

    def _start_episode(self, request, response):
        del request
        if self._recording:
            response.success = False
            response.message = "an episode is already recording"
            return response
        root = Path(self.get_parameter("output_dir").value).expanduser()
        self._episode_dir = self._next_episode_dir(root)
        self._episode_dir.mkdir(parents=False)
        self._metadata_file = (self._episode_dir / "metadata.jsonl").open("w", encoding="utf-8")
        manifest = {"task": self.get_parameter("task").value, "joints": list(JOINTS), "fps": 30, "format": "so101_raw_v1"}
        (self._episode_dir / "episode.json").write_text(json.dumps(manifest, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
        self._frames = 0
        self._recording = True
        response.success = True
        response.message = f"recording to {self._episode_dir}"
        return response

    def _stop_episode(self, request, response):
        del request
        if not self._recording:
            response.success = False
            response.message = "no episode is recording"
            return response
        episode_dir = self._episode_dir
        count = self._frames
        self._metadata_file.close()
        self._metadata_file = None
        self._episode_dir = None
        self._recording = False
        self._frames = 0
        response.success = True
        response.message = f"saved raw episode with {count} frames to {episode_dir}"
        return response


def main(args=None):
    rclpy.init(args=args)
    node = SO101DatasetRecorder()
    try:
        rclpy.spin(node)
    except ExternalShutdownException:
        pass
    finally:
        if node._metadata_file is not None:
            node._metadata_file.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
