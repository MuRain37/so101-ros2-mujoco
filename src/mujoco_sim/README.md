# Multi-robot MuJoCo simulator

The default remains SO101. Select the independent UR5e + Robotiq scene with
`task_id:=ur5e_red_cube_to_target`; the task determines the robot. An explicit
`robot_id` must match the task. Shared arm XML is in
`robot_description/mjcf/{so101,ur5e}`; this package owns environment and task scenes.
See the workspace README for the multi-robot control/data contract. The detailed
dimensions below describe the original SO101 scenes, not the UR5e variant.

This ROS 2 Python package runs the SO-101 follower arm in MuJoCo's native
interactive viewer. It reuses the STL meshes provided by the
`robot_description` package and includes the MuJoCo-specific MJCF scene and
robot model from [TheRobotStudio/SO-ARM100](https://github.com/TheRobotStudio/SO-ARM100), with manipulation collision geometry and a wrist-camera mount adapted from [MuJoCo Menagerie robotstudio_so101](https://github.com/google-deepmind/mujoco_menagerie/tree/main/robotstudio_so101),
licensed under Apache-2.0.

## Prerequisite

Install MuJoCo for the same Python interpreter used by ROS 2 Jazzy:

```bash
/usr/bin/python3 -m pip install --user --break-system-packages mujoco==3.3.7
```

## Run

```bash
colcon build --packages-select mujoco_sim
source install/setup.zsh
ros2 launch mujoco_sim display.launch.py
```

To load another self-contained MJCF scene, provide `model_path`:

```bash
ros2 launch mujoco_sim display.launch.py model_path:=/path/to/scene.xml
```

When `model_path` is empty, the selected task supplies its default scene file.
An explicit `model_path` overrides that default.

The red and blue cubes are randomized at every startup. Their centre positions
use the scene's named `cube_spawn_area`, currently a centred `0.18 m x 0.18 m`
square with `x=[0.16, 0.34]` m and `y=[-0.09, 0.09]` m. The sampler keeps a
1 cm edge-to-edge gap so the cubes cannot overlap. Calling
`/sim/reset_task` resets the robot home pose, delegates object reset to the task,
and advances the command epoch without changing the clock. The random task reads bounds
from the named `cube_spawn_area` site. Set `random_seed` to a non-negative integer to
reproduce the random sequence; its default is `-1`. Success evaluation is reserved by
the task interface but is not enabled yet.

The simulator needs a desktop OpenGL session. It always advances MuJoCo physics at
the model's fixed 2 ms step, publishes `/clock` and `/sim/joint_states`, and
contains a wooden table, the SO-101 on its tabletop, a free 24 mm-wide red cube and blue cube, plus non-colliding red and blue
placement targets. The follower carries the Menagerie wrist-camera mount and an original visual model of the recommended InnoMaker `U20CAM-1080P-S1` module. Its single black STL preserves the 32 mm PCB, 2.2 mm holes, fitted 26.0286 mm mounting pitch, M12 lens interface, and aligned camera optical axis. The simulator renders this camera and publishes its images with MuJoCo simulation-time stamps. Published dimensions come from the [manufacturer manual](https://github.com/INNO-MAKER/U20CAM-1080P-S1/blob/main/U20CAM-1080P-S1%20UserManual-v1.1.pdf), and the module selection follows the [official SO-101 installation guide](https://github.com/TheRobotStudio/SO-ARM100/blob/main/Optional/SO101_Wrist_Cam_Hex-Nut_Mount_32x32_UVC_Module/README.md).

The scene also contains a suspended global Intel RealSense D435 visual model,
positioned above the front of the table and aimed at the manipulation workspace.
It is held by a non-colliding table-mounted stand with a base, rear post, boom,
and fitted cradle. The structure follows the modular layout of the
[open-source SO-100/101 overhead camera mount](https://github.com/TheRobotStudio/SO-ARM100/tree/main/Optional/Overhead_Cam_Mount_Webcam),
while its dimensions are adapted to this scene's fixed D435 pose and enclosure.
It is a single white STL merged from the [MuJoCo Menagerie](https://github.com/google-deepmind/mujoco_menagerie/tree/main/realsense_d435i)
visual meshes, is Apache-2.0 licensed, and has no collision geometry. A virtual
camera inside the body publishes aligned RGB-D images. The bundled license is installed by the
`robot_description` package.

## Global D435 topics

The fixed D435 publishes aligned streams at 640 x 480 and 30 simulation-time Hz:

```text
/d435/color/image_raw       sensor_msgs/msg/Image (rgb8)
/d435/color/camera_info     sensor_msgs/msg/CameraInfo
/d435/depth/image_raw       sensor_msgs/msg/Image (32FC1, metres)
/d435/depth/camera_info     sensor_msgs/msg/CameraInfo
```

All four messages use `global_d435_optical_frame` and the same MuJoCo timestamp.
Use `d435_enabled:=false` to disable this renderer, or configure
`d435_width`, `d435_height`, and `d435_publish_rate`. For example:

```bash
ros2 topic hz /d435/color/image_raw
ros2 topic hz /d435/depth/image_raw
rqt_image_view /d435/color/image_raw
```

## Compressed RGB preview

For smooth local preview, use the JPEG topics instead of the raw recording topics:

```bash
rqt_image_view /wrist_cam/image_preview/compressed
rqt_image_view /d435/color/image_preview/compressed
```

Set `camera_preview_enabled:=false` to disable both preview republishers. Raw RGB-D topics remain unchanged for rosbag recording.

## Wrist-camera topics

The Viewer publishes `rgb8` images on `/wrist_cam/image_raw` and matching
`CameraInfo` on `/wrist_cam/camera_info`, using sensor-data QoS and the
`wrist_cam_optical_frame` frame ID. Both messages use the same MuJoCo time stamp
as `/clock`. Defaults are 640 x 480 at 30 simulation-time Hz. Configure them at
launch time, for example:

```bash
ros2 launch mujoco_sim display.launch.py \
  camera_width:=640 camera_height:=480 camera_publish_rate:=30.0
```

Disable offscreen rendering with `camera_enabled:=false`. Check the stream with:

```bash
ros2 topic hz /wrist_cam/image_raw
ros2 topic echo /wrist_cam/camera_info --once
```

## Leader-arm teleoperation

The simulator receives a `sensor_msgs/JointState` target, clamps it to the joint
limits, and writes it to the MuJoCo position-actuator controls. The published
`/sim/action` topic reports applied public joint targets (radians, not Robotiq
actuator bytes), while `/sim/joint_states` reports the post-physics pose, which can
differ from the target during PD response or contact. Both messages use the same
simulation timestamp.

```bash
ros2 run mujoco_sim mujoco_simulator \
  --ros-args -p joint_state_topic:=/robot/joint_targets
```

Custom command publishers must copy the current `/sim/joint_states` epoch from
`header.frame_id`; missing or obsolete epochs are rejected.

For the leader-arm bridge and Viewer together, use
`ros2 launch robot_bringup teleop_sim.launch.py`.

## Red cube in a drawer

Task `red_cube_in_drawer`: open the drawer, put the red cube inside, then close it.
The initial cube pose is fixed at `(0.12, -0.16, 0.013)` metres. It has a free
joint and remains graspable. The cabinet is fixed at `(0.34, 0, 0)` on the
table, directly in front of the arm. This scene mounts the arm at `(-0.04, 0, 0)`
using MuJoCo model attachment, leaving the shared arm model unchanged.
Its handle faces the robot, and the passive slide opens along world -X
through 0.096 m. Cabinet dimensions are approximately 0.142 m wide, 0.12 m deep,
and 0.101 m high (excluding the handle).

The model is adapted at 60% scale from
[Meta-World's drawer](https://github.com/Farama-Foundation/Metaworld/blob/6e01ad7e2ffb2302e4dca04f796fcd8837df8540/metaworld/assets/objects/assets/drawer.xml).
Meshes live in `robot_description/meshes/metaworld_drawer*.stl`; attribution and
the MIT license are in `robot_description/THIRD_PARTY_LICENSE_METAWORLD.txt`.
The hollow drawer and handle use separate primitive collision geoms, while
the original STL meshes provide appearance. No drawer motor is added.

Build the new resources from the workspace root:

```bash
colcon build --symlink-install \
  --packages-select robot_description mujoco_sim
source install/setup.zsh
```

View the scene:

```bash
ros2 launch robot_bringup mujoco.launch.py \
  task_id:=red_cube_in_drawer
```

Teleoperate and record:

```bash
ros2 launch robot_bringup teleop_sim.launch.py \
  dataset_task_id:=red_cube_in_drawer
```

The existing `/sim/reset_task` service restores the robot home pose, cube start
pose, and closed drawer, including task-joint velocities. Adjust placement in
`mujoco/tasks/red_cube_in_drawer.xml`; drawer geometry and travel are defined in
`mujoco/objects/metaworld_drawer.xml`. Existing camera names and robot action
dimensions are preserved. This scene tilts the global camera toward the drawer
so the complete cabinet remains visible. This adds the physical scene and recording task;
automatic completion detection and a trained drawer policy are not included.
