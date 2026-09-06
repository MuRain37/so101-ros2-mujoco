#!/usr/bin/env python3
"""Control a rosbag2 recording with the existing episode services."""

import json
import signal
import shutil
import subprocess
import time
from datetime import datetime
from pathlib import Path

import rclpy
from mujoco_sim.tasks import create_task
from robot_core import get_robot
import rosbag2_py
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_srvs.srv import Trigger


DEFAULT_BAG_TOPICS = (
    "/clock", "/leader/joint_states", "/robot/joint_targets", "/sim/joint_states", "/sim/action",
    "/wrist_cam/image_raw", "/wrist_cam/camera_info",
    "/d435/color/image_raw", "/d435/color/camera_info",
    "/d435/depth/image_raw", "/d435/depth/camera_info",
)
CAMERA_IMAGE_TOPICS = (
    "/wrist_cam/image_raw",
    "/d435/color/image_raw",
    "/d435/depth/image_raw",
)


class DatasetRecorder(Node):
    """Start and stop one rosbag2 episode without changing the public API."""

    def __init__(self) -> None:
        super().__init__("dataset_recorder")
        self.declare_parameter("output_dir", "dataset/raw")
        self.declare_parameter("task_id", "red_cube_to_red_target")
        self.declare_parameter("robot_id", "auto")
        self._task = create_task(str(self.get_parameter("task_id").value))
        self._robot = get_robot(self._task.resolve_robot(str(self.get_parameter("robot_id").value)))
        self.declare_parameter("bag_topics", list(DEFAULT_BAG_TOPICS))
        self.declare_parameter("startup_timeout", 5.0)
        self.declare_parameter("minimum_camera_rate", 27.0)
        self._process = None
        self._episode_dir = None
        self._bag_dir = None
        self._manifest = None
        self._reset_client = self.create_client(Trigger, "/sim/reset_task")
        self.create_service(Trigger, "dataset/start_episode", self._start_episode)
        self.create_service(Trigger, "dataset/stop_episode", self._stop_episode)
        self.create_service(Trigger, "dataset/cancel_episode", self._cancel_episode)
        self.get_logger().info("ready; call /dataset/start_episode to begin recording")

    def _next_episode_dir(self, root: Path) -> Path:
        root.mkdir(parents=True, exist_ok=True)
        episode_id = datetime.now().astimezone().strftime("%Y%m%d_%H%M%S")
        episode_dir = root / f"episode_{episode_id}"
        suffix = 1
        while episode_dir.exists():
            episode_dir = root / f"episode_{episode_id}_{suffix:02d}"
            suffix += 1
        return episode_dir

    def _start_episode(self, request, response):
        del request
        if self._process is not None and self._process.poll() is None:
            response.success = False
            response.message = "an episode is already recording"
            return response
        self._process = None

        topics = list(self.get_parameter("bag_topics").value)
        available = {name for name, _ in self.get_topic_names_and_types()}
        missing = sorted(set(topics) - available)
        if missing:
            response.success = False
            response.message = "cannot record; missing topics: " + ", ".join(missing)
            return response

        root = Path(self.get_parameter("output_dir").value).expanduser().resolve()
        episode_dir = self._next_episode_dir(root)
        episode_dir.mkdir()
        bag_dir = episode_dir / "rosbag"
        command = [
            "ros2", "bag", "record", "--output", str(bag_dir),
            "--storage", "mcap", "--storage-preset-profile", "zstd_fast",
            "--max-cache-size", "1073741824", "--use-sim-time", "--topics",
            *topics,
        ]
        try:
            process = subprocess.Popen(command)
        except OSError as error:
            response.success = False
            response.message = f"failed to start rosbag2: {error}"
            return response

        timeout = float(self.get_parameter("startup_timeout").value)
        deadline = time.monotonic() + timeout
        while process.poll() is None and not bag_dir.is_dir() and time.monotonic() < deadline:
            time.sleep(0.05)
        time.sleep(0.2)
        if process.poll() is not None or not bag_dir.is_dir():
            if process.poll() is None:
                process.terminate()
                process.wait(timeout=5)
            response.success = False
            response.message = f"rosbag2 failed to start (exit code {process.returncode})"
            return response

        self._process = process
        self._episode_dir = episode_dir
        self._bag_dir = bag_dir
        self._manifest = {
            **self._robot.metadata(),
            "episode_id": episode_dir.name.removeprefix("episode_"),
            "recorded_at": datetime.now().astimezone().isoformat(timespec="seconds"),
            "task_id": self.get_parameter("task_id").value,
            "task": self._task.language_instruction,
            "format": "rosbag2_mcap",
            "topics": topics,
            "status": "recording",
        }
        self._write_manifest()
        response.success = True
        response.message = f"recording to {episode_dir}"
        return response

    def _write_manifest(self) -> None:
        (self._episode_dir / "episode.json").write_text(
            json.dumps(self._manifest, ensure_ascii=False, indent=2) + "\n",
            encoding="utf-8",
        )

    def _stop_process(self) -> int:
        process = self._process
        process.send_signal(signal.SIGINT)
        try:
            process.wait(timeout=30)
        except subprocess.TimeoutExpired:
            process.terminate()
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait()
        return process.returncode

    def _validate_bag(self):
        try:
            metadata = rosbag2_py.Info().read_metadata(str(self._bag_dir), "mcap")
        except Exception as error:  # noqa: BLE001
            return False, {"errors": [f"cannot read bag metadata: {error}"]}

        duration = metadata.duration.nanoseconds * 1e-9
        counts = {
            item.topic_metadata.name: item.message_count
            for item in metadata.topics_with_message_count
        }
        errors = []
        if duration <= 0:
            errors.append("recording duration is zero")
        missing = [topic for topic in self._manifest["topics"] if counts.get(topic, 0) == 0]
        if missing:
            errors.append("missing or empty topics: " + ", ".join(missing))

        minimum_rate = float(self.get_parameter("minimum_camera_rate").value)
        camera_rates = {}
        if duration > 0:
            for topic in CAMERA_IMAGE_TOPICS:
                if topic in self._manifest["topics"]:
                    rate = counts.get(topic, 0) / duration
                    camera_rates[topic] = round(rate, 3)
                    if rate < minimum_rate:
                        errors.append(
                            f"{topic} rate {rate:.2f} Hz is below {minimum_rate:.2f} Hz"
                        )

        return not errors, {
            "duration_seconds": round(duration, 3),
            "message_counts": counts,
            "camera_rates_hz": camera_rates,
            "errors": errors,
        }

    def _finish_episode(self):
        returncode = self._stop_process()
        valid, validation = self._validate_bag() if returncode == 0 else (False, {
            "errors": [f"rosbag stopped with exit code {returncode}"]
        })
        episode_dir = self._episode_dir
        self._manifest.update({
            "stopped_at": datetime.now().astimezone().isoformat(timespec="seconds"),
            "status": "complete" if valid else "invalid",
            "validation": validation,
        })
        self._write_manifest()
        self._process = None
        self._episode_dir = None
        self._bag_dir = None
        self._manifest = None
        return valid, episode_dir, validation["errors"]

    def _request_task_reset(self) -> bool:
        if not self._reset_client.service_is_ready():
            self.get_logger().error(
                "cannot reset task; /sim/reset_task is unavailable"
            )
            return False
        future = self._reset_client.call_async(Trigger.Request())
        future.add_done_callback(self._report_reset_result)
        return True

    def _report_reset_result(self, future) -> None:
        try:
            response = future.result()
        except Exception as error:  # noqa: BLE001
            self.get_logger().error(f"simulation reset failed: {error}")
            return
        log = self.get_logger().info if response.success else self.get_logger().error
        log(response.message)

    def _cancel_episode(self, request, response):
        del request
        if self._process is None:
            response.success = False
            response.message = "no episode is recording"
            return response

        episode_dir = self._episode_dir
        self._stop_process()
        self._process = None
        self._episode_dir = None
        self._bag_dir = None
        self._manifest = None
        try:
            shutil.rmtree(episode_dir)
        except OSError as error:
            response.success = False
            response.message = f"recording stopped but episode cleanup failed: {error}"
            self._request_task_reset()
            return response

        self._request_task_reset()
        response.success = True
        response.message = f"discarded episode {episode_dir}"
        return response


    def _stop_episode(self, request, response):
        del request
        if self._process is None:
            response.success = False
            response.message = "no episode is recording"
            return response

        valid, episode_dir, errors = self._finish_episode()
        reset_requested = self._request_task_reset()
        response.success = valid
        response.message = (
            f"saved and validated rosbag episode to {episode_dir}"
            if valid
            else f"saved invalid episode to {episode_dir}: {'; '.join(errors)}"
        )
        if not reset_requested:
            response.message += "; task was not reset"
        return response


def main(args=None):
    rclpy.init(args=args)
    node = DatasetRecorder()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node._process is not None:
            valid, episode_dir, errors = node._finish_episode()
            if not valid:
                node.get_logger().error(
                    f"saved invalid episode to {episode_dir}: {'; '.join(errors)}"
                )
        try:
            node.destroy_node()
        except KeyboardInterrupt:
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
