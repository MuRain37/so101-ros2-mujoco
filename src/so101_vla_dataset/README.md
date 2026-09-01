# SO-101 VLA dataset recorder

ROS 2 recorder for raw episodes. It synchronizes the global RGB camera, wrist RGB
camera, simulated joint state, and simulated action using the image timestamp,
then saves NumPy images and JSONL metadata without requiring LeRobot.

Build and run:

```bash
colcon build --packages-select so101_vla_dataset --symlink-install
source install/setup.zsh
ros2 run so101_vla_dataset so101_dataset_recorder
```

Control recording:

```bash
ros2 service call /dataset/start_episode std_srvs/srv/Trigger "{}"
ros2 service call /dataset/stop_episode std_srvs/srv/Trigger "{}"
```

Parameters include `output_dir`, `task`, and `sync_tolerance`. This first
version intentionally has no reset or automatic success detection.
