# SO-101 MuJoCo viewer

This ROS 2 Python package runs the SO-101 follower arm in MuJoCo's native
interactive viewer. It reuses the STL meshes provided by the
`so101_description` package and includes the MuJoCo-specific MJCF scene and
robot model from [TheRobotStudio/SO-ARM100](https://github.com/TheRobotStudio/SO-ARM100), with manipulation collision geometry adapted from [MuJoCo Menagerie robotstudio_so101](https://github.com/google-deepmind/mujoco_menagerie/tree/main/robotstudio_so101),
licensed under Apache-2.0; the bundled license is in
`THIRD_PARTY_LICENSES_SO_ARM100.txt`.

## Prerequisite

Install MuJoCo for the same Python interpreter used by ROS 2 Jazzy:

```bash
/usr/bin/python3 -m pip install --user --break-system-packages mujoco==3.3.7
```

## Run

```bash
colcon build --packages-select so101_mujoco_sim
source install/setup.zsh
ros2 launch so101_mujoco_sim display.launch.py
```

To load another self-contained MJCF scene, provide `model_path`:

```bash
ros2 launch so101_mujoco_sim display.launch.py model_path:=/path/to/scene.xml
```

The viewer needs a desktop OpenGL session. It always advances MuJoCo physics at
the model's fixed 2 ms step, publishes `/clock` and `/sim/joint_states`, and
contains a wooden table, the SO-101 on its tabletop, a free 24 mm-wide red cube and blue cube, plus non-colliding red and blue
placement targets. No task cameras are included yet.

## Leader-arm teleoperation

The viewer receives a `sensor_msgs/JointState` target, clamps it to the joint
limits, and writes it to the MuJoCo position-actuator controls. The published
`/sim/joint_states` topic reports the post-physics pose, which can differ from
the leader target during contact.

```bash
ros2 run so101_mujoco_sim so101_mujoco_viewer \
  --ros-args -p joint_state_topic:=/joint_states
```

For the leader-arm bridge and Viewer together, use
`ros2 launch so101_bringup teleop_sim.launch.py`.
