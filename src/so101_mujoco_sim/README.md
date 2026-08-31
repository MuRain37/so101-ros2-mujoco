# SO-101 MuJoCo viewer

This ROS 2 Python package runs the SO-101 follower arm in MuJoCo's native
interactive viewer. It reuses the STL meshes provided by the
`so101_description` package and includes the MuJoCo-specific MJCF scene and
robot model from [TheRobotStudio/SO-ARM100](https://github.com/TheRobotStudio/SO-ARM100), with manipulation collision geometry and a wrist-camera mount adapted from [MuJoCo Menagerie robotstudio_so101](https://github.com/google-deepmind/mujoco_menagerie/tree/main/robotstudio_so101),
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
placement targets. The follower carries the Menagerie wrist-camera mount and an original visual model of the recommended InnoMaker `U20CAM-1080P-S1` module. Its single black STL preserves the 32 mm PCB, 2.2 mm holes, fitted 26.0286 mm mounting pitch, M12 lens interface, and aligned camera optical axis. The simulator renders this camera and publishes its images with MuJoCo simulation-time stamps. Published dimensions come from the [manufacturer manual](https://github.com/INNO-MAKER/U20CAM-1080P-S1/blob/main/U20CAM-1080P-S1%20UserManual-v1.1.pdf), and the module selection follows the [official SO-101 installation guide](https://github.com/TheRobotStudio/SO-ARM100/blob/main/Optional/SO101_Wrist_Cam_Hex-Nut_Mount_32x32_UVC_Module/README.md).

## Wrist-camera topics

The Viewer publishes `rgb8` images on `/wrist_cam/image_raw` and matching
`CameraInfo` on `/wrist_cam/camera_info`, using sensor-data QoS and the
`wrist_cam_optical_frame` frame ID. Both messages use the same MuJoCo time stamp
as `/clock`. Defaults are 640 x 480 at 30 simulation-time Hz. Configure them at
launch time, for example:

```bash
ros2 launch so101_mujoco_sim display.launch.py \
  camera_width:=640 camera_height:=480 camera_publish_rate:=30.0
```

Disable offscreen rendering with `camera_enabled:=false`. Check the stream with:

```bash
ros2 topic hz /wrist_cam/image_raw
ros2 topic echo /wrist_cam/camera_info --once
```

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
