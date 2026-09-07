"""Small real MCAP -> LeRobot round trips, isolated from user datasets."""
import json
import numpy as np
import pytest
import rosbag2_py
from rclpy.serialization import serialize_message
from sensor_msgs.msg import Image, JointState

from robot_adapters import get_robot
from vla_dataset.convert_mcap_to_lerobot import main, TOPICS


def write_episode(path, robot_id, legacy=False):
    path.mkdir(parents=True)
    robot = get_robot(robot_id)
    task_id = "ur5e_red_cube_to_target" if robot_id == "ur5e" else "red_cube_to_red_target"
    manifest = {"task_id": task_id, "task": "把红色方块放到红色区域"}
    if not legacy:
        manifest.update(robot.metadata())
    (path / "episode.json").write_text(json.dumps(manifest))
    writer = rosbag2_py.SequentialWriter()
    writer.open(rosbag2_py.StorageOptions(uri=str(path / "rosbag"), storage_id="mcap"),
                rosbag2_py.ConverterOptions("", ""))
    for i, (topic, cls) in enumerate(TOPICS.items()):
        writer.create_topic(rosbag2_py.TopicMetadata(id=i, name=topic,
                            type=f"sensor_msgs/msg/{cls.__name__}", serialization_format="cdr"))
    for i in range(6):
        timestamp = 1_000_000_000 + round(i * 1_000_000_000 / 30)
        for topic, cls in TOPICS.items():
            msg = cls()
            msg.header.stamp.sec, msg.header.stamp.nanosec = divmod(timestamp, 1_000_000_000)
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
    # Separate scenes in one input tree: export must never mix dimensions.
    write_episode(raw / "episode_1", "so101", legacy=True)
    write_episode(raw / "episode_2", "ur5e")
    output = tmp_path / "export"
    task_id = "ur5e_red_cube_to_target" if robot_id == "ur5e" else "red_cube_to_red_target"
    main(["--input", str(raw), "--output", str(output), "--repo-id", "test/roundtrip",
          "--task-id", task_id])
    metadata = json.loads((output / "meta/robot.json").read_text())
    assert metadata == get_robot(robot_id).metadata()
    dataset = LeRobotDataset("test/roundtrip", root=output, video_backend="pyav")
    assert dataset.num_frames == 6 and dataset.num_episodes == 1
    frame = dataset[2]
    assert tuple(frame["action"].shape) == (len(get_robot(robot_id).joint_names),)
    assert tuple(frame["observation.images.wrist"].shape) == (3, 480, 640)
    assert frame["action"].numpy() == pytest.approx(get_robot(robot_id).home)
