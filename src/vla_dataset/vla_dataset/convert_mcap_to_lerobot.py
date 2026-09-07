#!/usr/bin/env python3
"""Convert recorded rosbag2 MCAP episodes to a LeRobot dataset."""

import argparse
import bisect
import json
import shutil
from pathlib import Path

import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import Image, JointState
from robot_adapters import get_robot
from mujoco_sim.tasks import create_task

TOPICS = {"/d435/color/image_raw": Image, "/wrist_cam/image_raw": Image,
          "/sim/joint_states": JointState, "/sim/action": JointState}

def stamp(message):
    return message.header.stamp.sec + message.header.stamp.nanosec * 1e-9

def read_episode(episode_dir):
    manifest_path, bag_dir = episode_dir / "episode.json", episode_dir / "rosbag"
    if not manifest_path.is_file() or not bag_dir.is_dir():
        raise ValueError(f"{episode_dir} must contain episode.json and rosbag/")
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    streams = {topic: [] for topic in TOPICS}
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(bag_dir), storage_id="mcap"),
                rosbag2_py.ConverterOptions("", ""))
    while reader.has_next():
        topic, data, _ = reader.read_next()
        if topic in streams:
            streams[topic].append(deserialize_message(data, TOPICS[topic]))
    for values in streams.values(): values.sort(key=stamp)
    return manifest, streams

def nearest(values, times, target, tolerance):
    index = bisect.bisect_left(times, target)
    candidates = [i for i in (index - 1, index) if 0 <= i < len(values)]
    if not candidates: return None
    best = min(candidates, key=lambda i: abs(times[i] - target))
    return values[best] if abs(times[best] - target) <= tolerance else None

def image_array(message):
    if message.encoding not in ("rgb8", "bgr8"):
        raise ValueError(f"unsupported image encoding: {message.encoding}")
    row = np.frombuffer(message.data, dtype=np.uint8).reshape(message.height, message.step)
    image = row[:, : message.width * 3].reshape(message.height, message.width, 3).copy()
    return image if message.encoding == "rgb8" else image[:, :, ::-1]

def joint_positions(message, robot):
    return robot.ordered(message.name, message.position).astype(np.float32)

def validate_manifest(manifest, robot):
    if manifest.get("robot_id", "so101") != robot.robot_id:
        raise ValueError("episode robot_id does not match selected robot")
    for key, expected in robot.metadata().items():
        if key in manifest and manifest[key] != expected:
            raise ValueError(f"episode {key} does not match selected robot")

def convert_episode(dataset, episode_dir, task_id, tolerance, robot):
    manifest, streams = read_episode(episode_dir)
    if manifest.get("task_id") != task_id: return 0
    validate_manifest(manifest, robot)
    missing = [topic for topic in TOPICS if not streams[topic]]
    if missing: raise ValueError(f"{episode_dir}: missing or empty topics: {', '.join(missing)}")
    front, wrist = streams["/d435/color/image_raw"], streams["/wrist_cam/image_raw"]
    state, action = streams["/sim/joint_states"], streams["/sim/action"]
    wrist_times, state_times, action_times = ([stamp(item) for item in values] for values in (wrist, state, action))
    task, count = manifest.get("task", task_id), 0
    for message in front:
        wrist_msg = nearest(wrist, wrist_times, stamp(message), tolerance)
        state_msg = nearest(state, state_times, stamp(message), tolerance)
        action_msg = nearest(action, action_times, stamp(message), tolerance)
        if any(item is None for item in (wrist_msg, state_msg, action_msg)): continue
        dataset.add_frame({"observation.images.front": image_array(message),
                           "observation.images.wrist": image_array(wrist_msg),
                           "observation.state": joint_positions(state_msg, robot),
                           "action": joint_positions(action_msg, robot), "task": task})
        count += 1
    if count: dataset.save_episode()
    return count

def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", type=Path, default=Path("dataset/raw"))
    parser.add_argument("--output", type=Path, default=Path("dataset/lerobot/so101_red_cube"))
    parser.add_argument("--repo-id", default="so101_red_cube")
    parser.add_argument("--task-id", default="red_cube_to_red_target")
    parser.add_argument("--robot-id", default="auto")
    parser.add_argument("--fps", type=int, default=30)
    parser.add_argument("--tolerance", type=float, default=0.02)
    args = parser.parse_args(argv)
    robot = get_robot(create_task(args.task_id).resolve_robot(args.robot_id))
    if not args.input.is_dir(): parser.error(f"input directory does not exist: {args.input}")
    episodes = sorted(path for path in args.input.glob("episode_*") if path.is_dir())
    if not episodes: parser.error(f"no episode directories found in {args.input}")
    selected = []
    for episode in episodes:
        manifest = json.loads((episode / "episode.json").read_text(encoding="utf-8"))
        if (manifest.get("task_id") == args.task_id
                and manifest.get("robot_id", "so101") == robot.robot_id):
            validate_manifest(manifest, robot)
            selected.append(episode)
    episodes = selected
    if not episodes: parser.error(f"no episodes for robot {robot.robot_id}, task {args.task_id}")
    if args.output.exists(): parser.error(f"output already exists: {args.output}; choose another --output")
    from lerobot.datasets.lerobot_dataset import LeRobotDataset
    shape = (480, 640, 3)
    features = {
        "observation.images.front": {"dtype": "video", "shape": shape, "names": ["height", "width", "channels"]},
        "observation.images.wrist": {"dtype": "video", "shape": shape, "names": ["height", "width", "channels"]},
        "observation.state": {"dtype": "float32", "shape": (len(robot.joint_names),), "names": {"axes": list(robot.joint_names)}},
        "action": {"dtype": "float32", "shape": (len(robot.joint_names),), "names": {"axes": list(robot.joint_names)}},
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    dataset = LeRobotDataset.create(repo_id=args.repo_id, root=args.output, fps=args.fps,
                                     robot_type=robot.robot_id, features=features, use_videos=True)
    total = sum(convert_episode(dataset, episode, args.task_id, args.tolerance, robot) for episode in episodes)
    if total == 0:
        shutil.rmtree(args.output)
        raise RuntimeError(f"no valid episodes found for task_id={args.task_id!r}")
    dataset.finalize()
    (args.output / "meta" / "robot.json").write_text(
        json.dumps(robot.metadata(), ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
    )
    print(f"converted {total} frames from {len(episodes)} episodes to {args.output}")

if __name__ == "__main__": main()
