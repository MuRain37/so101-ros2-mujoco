#!/usr/bin/env python3
"""Convert task-configured rosbag2 MCAP episodes to a LeRobot dataset."""

import argparse
import bisect
import json
import shutil
from pathlib import Path

import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import Image, JointState

from mujoco_sim.camera import depth_training_image, rgb_image_array
from mujoco_sim.tasks import create_task
from mujoco_sim.tasks.base import camera_manifest, camera_streams
from robot_adapters import get_robot

STATE_TOPICS = {
    "/sim/joint_states": JointState,
    "/sim/action": JointState,
}


def stamp(message):
    return message.header.stamp.sec + message.header.stamp.nanosec * 1e-9


def nearest(values, times, target, tolerance):
    index = bisect.bisect_left(times, target)
    candidates = [i for i in (index - 1, index) if 0 <= i < len(values)]
    if not candidates:
        return None
    best = min(candidates, key=lambda i: abs(times[i] - target))
    return values[best] if abs(times[best] - target) <= tolerance else None


def joint_positions(message, robot):
    return robot.ordered(message.name, message.position).astype(np.float32)


def validate_manifest(manifest, robot, task):
    if manifest.get("robot_id") != robot.robot_id:
        raise ValueError("episode robot_id does not match selected robot")
    for key, expected in robot.metadata().items():
        if manifest.get(key) != expected:
            raise ValueError(f"episode {key} does not match selected robot")
    if manifest.get("cameras") != camera_manifest(task.cameras):
        raise ValueError("episode camera schema does not match selected task")
    if manifest.get("primary_camera_id") != task.primary_camera_id:
        raise ValueError("episode primary camera does not match selected task")


def read_episode(episode_dir, task):
    manifest_path, bag_dir = episode_dir / "episode.json", episode_dir / "rosbag"
    if not manifest_path.is_file() or not bag_dir.is_dir():
        raise ValueError(f"{episode_dir} must contain episode.json and rosbag/")
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    topic_types = dict(STATE_TOPICS)
    topic_types.update(
        (stream.topic, Image) for _, _, stream in camera_streams(task.cameras)
    )
    streams = {topic: [] for topic in topic_types}
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag_dir), storage_id="mcap"),
        rosbag2_py.ConverterOptions("", ""),
    )
    while reader.has_next():
        topic, data, _ = reader.read_next()
        if topic in streams:
            streams[topic].append(deserialize_message(data, topic_types[topic]))
    for values in streams.values():
        values.sort(key=stamp)
    return manifest, streams


def convert_episode(dataset, episode_dir, task, tolerance, robot):
    manifest, streams = read_episode(episode_dir, task)
    validate_manifest(manifest, robot, task)
    missing = [topic for topic, values in streams.items() if not values]
    if missing:
        raise ValueError(
            f"{episode_dir}: missing or empty topics: {', '.join(missing)}"
        )

    primary = next(
        camera for camera in task.cameras
        if camera.camera_id == task.primary_camera_id
    )
    reference = streams[primary.rgb.topic]
    stream_times = {
        topic: [stamp(message) for message in values]
        for topic, values in streams.items()
    }
    task_text = manifest.get("task", task.task_id)
    count = 0
    for reference_message in reference:
        timestamp = stamp(reference_message)
        matched = {}
        for topic, values in streams.items():
            if topic == primary.rgb.topic:
                matched[topic] = reference_message
            else:
                matched[topic] = nearest(
                    values, stream_times[topic], timestamp, tolerance
                )
        if any(message is None for message in matched.values()):
            continue

        frame = {
            "observation.state": joint_positions(
                matched["/sim/joint_states"], robot
            ),
            "action": joint_positions(matched["/sim/action"], robot),
            "task": task_text,
        }
        for camera, modality, stream in camera_streams(task.cameras):
            message = matched[stream.topic]
            frame[stream.observation_key] = (
                rgb_image_array(message)
                if modality == "rgb"
                else depth_training_image(message, camera.depth_range_m)
            )
        dataset.add_frame(frame)
        count += 1
    if count:
        dataset.save_episode()
    return count


def dataset_features(task, robot):
    features = {
        "observation.state": {
            "dtype": "float32",
            "shape": (len(robot.joint_names),),
            "names": {"axes": list(robot.joint_names)},
        },
        "action": {
            "dtype": "float32",
            "shape": (len(robot.joint_names),),
            "names": {"axes": list(robot.joint_names)},
        },
    }
    for camera, modality, stream in camera_streams(task.cameras):
        features[stream.observation_key] = {
            "dtype": "video" if modality == "rgb" else "image",
            "shape": (camera.height, camera.width, 3),
            "names": ["height", "width", "channels"],
        }
    return features


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", type=Path, default=Path("dataset/raw"))
    parser.add_argument(
        "--output", type=Path, default=Path("dataset/lerobot/so101_red_cube")
    )
    parser.add_argument("--repo-id", default="so101_red_cube")
    parser.add_argument("--task-id", default="red_cube_to_red_target")
    parser.add_argument("--robot-id", default="auto")
    parser.add_argument("--fps", type=int, default=30)
    parser.add_argument("--tolerance", type=float, default=0.02)
    args = parser.parse_args(argv)

    task = create_task(args.task_id)
    robot = get_robot(task.resolve_robot(args.robot_id))
    if not args.input.is_dir():
        parser.error(f"input directory does not exist: {args.input}")
    episodes = sorted(path for path in args.input.glob("episode_*") if path.is_dir())
    if not episodes:
        parser.error(f"no episode directories found in {args.input}")

    selected = []
    for episode in episodes:
        manifest = json.loads(
            (episode / "episode.json").read_text(encoding="utf-8")
        )
        if manifest.get("task_id") == args.task_id:
            validate_manifest(manifest, robot, task)
            selected.append(episode)
    if not selected:
        parser.error(
            f"no episodes for robot {robot.robot_id}, task {args.task_id}"
        )
    if args.output.exists():
        parser.error(f"output already exists: {args.output}; choose another --output")

    from lerobot.datasets.lerobot_dataset import LeRobotDataset

    args.output.parent.mkdir(parents=True, exist_ok=True)
    dataset = LeRobotDataset.create(
        repo_id=args.repo_id,
        root=args.output,
        fps=args.fps,
        robot_type=robot.robot_id,
        features=dataset_features(task, robot),
        use_videos=True,
    )
    total = sum(
        convert_episode(dataset, episode, task, args.tolerance, robot)
        for episode in selected
    )
    if total == 0:
        shutil.rmtree(args.output)
        raise RuntimeError(f"no synchronized frames for task_id={args.task_id!r}")
    dataset.finalize()
    metadata = args.output / "meta"
    (metadata / "robot.json").write_text(
        json.dumps(robot.metadata(), ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    (metadata / "cameras.json").write_text(
        json.dumps({
            "primary_camera_id": task.primary_camera_id,
            "cameras": camera_manifest(task.cameras),
        }, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    print(
        f"converted {total} frames from {len(selected)} episodes to {args.output}"
    )


if __name__ == "__main__":
    main()
