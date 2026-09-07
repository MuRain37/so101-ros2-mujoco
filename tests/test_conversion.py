"""Small real MCAP -> LeRobot round trips, isolated from user datasets."""
import json

import numpy as np
import pytest
import rosbag2_py
from rclpy.serialization import serialize_message
from sensor_msgs.msg import Image, JointState

from mujoco_sim.tasks import create_task
from mujoco_sim.tasks.base import camera_manifest, camera_streams
from robot_adapters import get_robot
from vla_dataset.convert_mcap_to_lerobot import STATE_TOPICS, main


def write_episode(path, robot_id):
    path.mkdir(parents=True)
    robot = get_robot(robot_id)
    task_id = (
        "ur5e_red_cube_to_target"
        if robot_id == "ur5e" else "red_cube_to_red_target"
    )
    task = create_task(task_id)
    manifest = {
        **robot.metadata(),
        "task_id": task_id,
        "task": "把红色方块放到红色区域",
        "cameras": camera_manifest(task.cameras),
        "primary_camera_id": task.primary_camera_id,
    }
    (path / "episode.json").write_text(json.dumps(manifest))

    topic_types = dict(STATE_TOPICS)
    topic_types.update(
        (stream.topic, Image) for _, _, stream in camera_streams(task.cameras)
    )
    writer = rosbag2_py.SequentialWriter()
    writer.open(
        rosbag2_py.StorageOptions(uri=str(path / "rosbag"), storage_id="mcap"),
        rosbag2_py.ConverterOptions("", ""),
    )
    for i, (topic, cls) in enumerate(topic_types.items()):
        writer.create_topic(rosbag2_py.TopicMetadata(
            id=i,
            name=topic,
            type=f"sensor_msgs/msg/{cls.__name__}",
            serialization_format="cdr",
        ))
    for i in range(6):
        timestamp = 1_000_000_000 + round(i * 1_000_000_000 / 30)
        for topic, cls in topic_types.items():
            msg = cls()
            msg.header.stamp.sec, msg.header.stamp.nanosec = divmod(
                timestamp, 1_000_000_000
            )
            if cls is Image:
                msg.height, msg.width, msg.step, msg.encoding = 480, 640, 1920, "rgb8"
                msg.data = np.full((480, 640, 3), i * 30, np.uint8).tobytes()
            else:
                msg.name, msg.position = list(robot.joint_names), list(robot.home)
            writer.write(topic, serialize_message(msg), timestamp)
    writer.close()


@pytest.mark.parametrize("robot_id", ["so101", "ur5e"])
def test_mcap_to_lerobot(tmp_path, robot_id):
    from lerobot.datasets.lerobot_dataset import LeRobotDataset

    raw = tmp_path / "raw"
    write_episode(raw / "episode_1", "so101")
    write_episode(raw / "episode_2", "ur5e")
    output = tmp_path / "export"
    task_id = (
        "ur5e_red_cube_to_target"
        if robot_id == "ur5e" else "red_cube_to_red_target"
    )
    main([
        "--input", str(raw), "--output", str(output),
        "--repo-id", "test/roundtrip", "--task-id", task_id,
    ])
    metadata = json.loads((output / "meta/robot.json").read_text())
    cameras = json.loads((output / "meta/cameras.json").read_text())
    assert metadata == get_robot(robot_id).metadata()
    assert cameras["cameras"] == camera_manifest(create_task(task_id).cameras)

    dataset = LeRobotDataset("test/roundtrip", root=output, video_backend="pyav")
    assert dataset.num_frames == 6 and dataset.num_episodes == 1
    frame = dataset[2]
    assert tuple(frame["action"].shape) == (len(get_robot(robot_id).joint_names),)
    assert tuple(frame["observation.images.front"].shape) == (3, 480, 640)
    assert tuple(frame["observation.images.wrist"].shape) == (3, 480, 640)
    assert frame["action"].numpy() == pytest.approx(get_robot(robot_id).home)


def test_rgbd_episode_conversion(tmp_path):
    from types import SimpleNamespace
    from lerobot.datasets.lerobot_dataset import LeRobotDataset
    from mujoco_sim.tasks.base import CameraSpec, ImageStreamSpec
    from vla_dataset.convert_mcap_to_lerobot import convert_episode, dataset_features

    camera = CameraSpec(
        camera_id="front",
        mjcf_name="front_cam",
        frame_id="front_optical_frame",
        width=32,
        height=32,
        rgb=ImageStreamSpec(
            "/front/color/image_raw", "/front/color/camera_info",
            "observation.images.front",
        ),
        depth=ImageStreamSpec(
            "/front/depth/image_raw", "/front/depth/camera_info",
            "observation.images.front_depth",
        ),
        depth_range_m=(0.1, 1.1),
    )
    task = SimpleNamespace(
        task_id="rgbd_test", cameras=(camera,), primary_camera_id="front"
    )
    robot = get_robot("so101")
    episode = tmp_path / "episode_rgbd"
    episode.mkdir()
    manifest = {
        **robot.metadata(), "task_id": task.task_id, "task": "RGB-D test",
        "cameras": camera_manifest(task.cameras),
        "primary_camera_id": task.primary_camera_id,
    }
    (episode / "episode.json").write_text(json.dumps(manifest))

    topic_types = {
        **STATE_TOPICS,
        camera.rgb.topic: Image,
        camera.depth.topic: Image,
    }
    writer = rosbag2_py.SequentialWriter()
    writer.open(
        rosbag2_py.StorageOptions(uri=str(episode / "rosbag"), storage_id="mcap"),
        rosbag2_py.ConverterOptions("", ""),
    )
    for index, (topic, cls) in enumerate(topic_types.items()):
        writer.create_topic(rosbag2_py.TopicMetadata(
            id=index, name=topic, type=f"sensor_msgs/msg/{cls.__name__}",
            serialization_format="cdr",
        ))
    for index in range(3):
        timestamp = 1_000_000_000 + index * 33_333_333
        for topic, cls in topic_types.items():
            message = cls()
            message.header.stamp.sec, message.header.stamp.nanosec = divmod(
                timestamp, 1_000_000_000
            )
            if cls is JointState:
                message.name = list(robot.joint_names)
                message.position = list(robot.home)
            elif topic == camera.rgb.topic:
                message.height = message.width = 32
                message.step, message.encoding = 96, "rgb8"
                message.data = np.full((32, 32, 3), index * 20, np.uint8).tobytes()
            else:
                message.height = message.width = 32
                message.step, message.encoding = 128, "32FC1"
                message.data = np.full((32, 32), 0.1 + index * 0.5, np.float32).tobytes()
            writer.write(topic, serialize_message(message), timestamp)
    writer.close()

    output = tmp_path / "rgbd_dataset"
    dataset = LeRobotDataset.create(
        repo_id="test/rgbd", root=output, fps=30, robot_type="so101",
        features=dataset_features(task, robot), use_videos=True,
    )
    assert convert_episode(dataset, episode, task, 0.02, robot) == 3
    dataset.finalize()
    loaded = LeRobotDataset("test/rgbd", root=output, video_backend="pyav")
    depth = loaded[1]["observation.images.front_depth"]
    assert tuple(depth.shape) == (3, 32, 32)
    assert float(depth.mean()) == pytest.approx(127 / 255, abs=0.01)
