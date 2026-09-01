# SO-101 ROS 2 + MuJoCo

A ROS 2 Jazzy workspace for visualizing and teleoperating the LeRobot SO-101
follower arm in MuJoCo. The simulation includes a wooden table, red and blue
cubes, and matching placement areas.

## Packages

- `so101_description`: shared workcell URDF, meshes, sensor models, and RViz configuration.
- `so101_leader_bridge`: publishes a physical SO-101 leader arm as ROS joint states.
- `so101_mujoco_sim`: MuJoCo model with a wrist camera, physics viewer, and ROS joint control.
- `so101_bringup`: launch files composing the packages.

## Requirements

- Ubuntu 24.04
- ROS 2 Jazzy
- Python 3.12 used by ROS 2
- MuJoCo 3.3.7

Install the Python dependency for the ROS interpreter:

```bash
/usr/bin/python3 -m pip install --user --break-system-packages mujoco==3.3.7
```

## Build

```bash
source /opt/ros/jazzy/setup.zsh
colcon build --symlink-install
source install/setup.zsh
```

## Run

Start the MuJoCo scene:

```bash
ros2 launch so101_mujoco_sim display.launch.py
```

Start leader-arm teleoperation and MuJoCo together:

```bash
ros2 launch so101_bringup teleop_sim.launch.py
```

The default calibration file is resolved from the current user's home directory.
Override it with `calibration_file:=/path/to/calibration.json` when needed.

## License

Apache-2.0. Third-party model attribution is retained in
`src/so101_mujoco_sim/THIRD_PARTY_LICENSES_SO_ARM100.txt`.
