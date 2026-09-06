# ROS 2 + MuJoCo 机器人学习工作空间

支持 SO101 和 UR5e + Robotiq 2F-85 仿真、SO101 主臂遥操作、
MCAP 数据录制、LeRobot 格式转换以及 ACT 策略推理。
运行环境：Ubuntu 24.04 / ROS 2 Jazzy / Python 3.12 / MuJoCo 3.3.7 / LeRobot 0.5.1。

## 功能包

| 功能包 | 职责 |
| --- | --- |
| `robot_description` | 机械臂 MJCF、URDF 和网格资源；上游模型及许可证 |
| `robot_core` | SO101 / UR5e 状态与动作适配器、初始姿态、模型资源加载 |
| `mujoco_sim` | 物理仿真、相机、环境，以及任务场景选择和复位 |
| `so101_leader_bridge` | 通过串口采集实体 SO101 主臂数据，并应用标定 |
| `teleop_retargeting` | SO101 直接关节映射，或固定桌面基准的 UR5e 绝对位姿重定向 |
| `vla_dataset` | 录制界面与服务、回合元数据、数据格式转换 |
| `robot_policy` | ACT 策略推理及复位界面 |
| `robot_bringup` | 组合启动各节点，并预先检查配置 |

原有的通用功能包 `so101_description`、`so101_mujoco_sim`、`so101_bringup`、
`so101_vla_dataset`、`so101_policy` 已重命名，旧包名不再用于启动。
已有的 SO101 数据集和模型检查点保留在原位置。

## 环境与构建

请在项目根目录运行命令，不要在模型检查点目录中运行。已有环境直接使用 `.venv`。
首次安装时，创建能够访问系统 ROS 包的 Python 3.12 虚拟环境：

```bash
/usr/bin/python3 -m venv --system-site-packages .venv
source .venv/bin/activate
python -m pip install -r requirements.txt
python -m pip install mujoco==3.3.7 scipy pyserial
```

ROS 依赖包括 `ros-jazzy-rosbag2-storage-mcap`、
`ros-jazzy-compressed-image-transport` 以及已有的 Qt/rqt 相关依赖。
重命名功能包或移动工作空间后，请打开新终端，排除旧 `build`/`install`
构建产物的影响，再重新构建。不要删除数据集或训练输出。

```bash
cd /home/murain/workspace/so101_project
source /opt/ros/jazzy/setup.zsh
source .venv/bin/activate
python -m colcon build --symlink-install
source install/setup.zsh
```

## 遥操作与录制

启动 UR5e 场景，包含初始位置固定、边长 40 mm 的红色方块和红色放置区域：

```bash
ros2 launch robot_bringup teleop_sim.launch.py \
  dataset_task_id:=ur5e_red_cube_to_target
```

启动原有的 SO101 红蓝方块任务：

```bash
ros2 launch robot_bringup teleop_sim.launch.py \
  robot_id:=so101 \
  dataset_task_id:=red_blue_cubes_to_targets
```

SO101 还支持 `red_cube_to_red_target` 和 `red_cube_in_drawer`。
UR5e 使用独立场景 `ur5e_red_cube_to_target`，不再复用 SO101 的任务 ID。
每个任务通过 `robot_id` 和 `scene_file` 声明绑定的机械臂和场景 XML。
启动参数 `robot_id` 默认是 `auto`，由任务确定；也可显式指定以检查是否匹配。
不匹配时会在打开串口前报错。已有 SO101 启动命令保持兼容。
不连接主臂，只显示仿真场景：

```bash
ros2 launch mujoco_sim display.launch.py \
  task_id:=ur5e_red_cube_to_target
```

需要时可通过 `port` 和 `calibration_file` 覆盖串口与标定文件配置。
同一个串口设备只允许一个主臂驱动进程打开。
遥操作或推理启动组中的必要进程退出时，会关闭同组其他进程，避免残留驱动。

UR5e 使用固定桌面基准的绝对位姿映射：对 SO101 主臂做正运动学得到夹爪参考点
的位置和姿态。水平位置以两台机械臂各自的 home 位姿为固定参考点，按
`position_scale:=2.0` 缩放；高度方向先把主臂高度减去 `leader_min_height`，
缩放后再加回 `follower_min_height`，保证主臂夹爪降到最低位置时，UR5e 夹爪
也降到自己的最低抓取位置。姿态以两台机械臂各自的“夹爪竖直向下”姿态为基准，
把主臂相对该基准的世界旋转 1:1 映射到 UR5e（倍率为 `orientation_scale:=1.0`），
因此主臂夹爪指向桌面时 UR5e 夹爪也指向桌面。映射只依赖主臂当前关节角和模型常量，
与遥操作启动时主臂或从臂摆在哪里无关；复位或重启不会改变对应关系。
夹爪开合继续跟随主臂，并受关节速度限制；收到首帧数据后 UR5e 即以限速移动到
主臂当前姿态对应的目标位姿。
`leader_min_height` 与 `follower_min_height` 的默认值由当前仿真夹爪几何推导
（夹爪参考点在其指尖刚好接触桌面时相对桌面的高度），实体安装时应按实际
夹爪重新标定并覆盖这两个启动参数。
如果需要把 SO101 的整段操作范围线性映射到 UR5e 的目标范围并限制其输出不越界，
可同时给出 `leader_range` 与 `follower_range`，格式为
`xmin xmax ymin ymax zmin zmax`；设置后这两个范围取代位置倍率与最低高度公式，
姿态仍按 `orientation_scale` 跟随主臂。
UR5e 输入中断超过 0.25 秒时保持最后的目标；恢复新数据后按新的绝对目标
继续，不会自动归位。

SO101 收到有效主臂消息后立即转发完整关节目标，不做 FK/IK、相对位姿锚定或
渐进限速，也不经过额外的定时转发。保留关节限位、无效数据检查和复位轮次隔离。
输入中断时保持最后的目标，恢复后直接跟随；仿真状态过期时暂停转发。
这不会消除原本物理 PD 控制和 ROS 通信的响应时间，但不会再人为逐步追赶目标。

录制界面保留开始、结束和取消功能，空格键用于取消本次录制。
复位会调用任务的 `reset()`，并将所选机械臂恢复到初始姿态：

```bash
ros2 service call /sim/reset_task std_srvs/srv/Trigger '{}'
```

## 控制链路与数据约定

```text
SO101 主臂 -> /leader/joint_states -> 重定向 --+
                                            +-> /robot/joint_targets -> 仿真
ACT 策略（单独启动，不与遥操作同时运行）-------+
仿真 -> /sim/joint_states（实际姿态）、/sim/action（已应用的关节目标）
```

对外发布的状态和动作统一使用弧度。SO101 保持原来的六个关节名称及顺序。
UR5e 的顺序为 `shoulder_pan_joint, shoulder_lift_joint, elbow_joint,
wrist_1_joint, wrist_2_joint, wrist_3_joint, gripper`。
第七维表示夹爪等效驱动角度：0 为张开，0.8 为闭合。
只有适配器内部会将其转换为 Robotiq 的 0–255 控制值。
夹爪连杆中的被动关节不作为策略动作维度。

状态、动作和目标消息的 `JointState.header.frame_id` 保存 `robot_id/epoch/N`，
它表示控制轮次，而不是 TF 坐标系。每次复位都会更新轮次，旧轮次的命令会被拒绝。
自定义控制器必须复制当前状态消息中的轮次标识。
相机消息仍使用光学坐标系标识和仿真时间戳；通过服务复位时，`/clock` 保持单调递增。
图像话题仍为 `/wrist_cam/image_raw` 和 `/d435/color/image_raw`，
同时保留全局深度图和压缩预览话题。

UR5e 重定向将关节速度限制为 0.8 rad/s，并检查逆运动学（IK）是否收敛及关节限位。
它**不是无碰撞运动规划器**，也不控制实体 UR 机械臂。
操作时应避免机械臂穿过桌面或物体，以及进入奇异姿态。
UR 腕部相机外壳是简化的仿真外观，不是可直接加工制造的精确安装组件。

## LeRobot 格式转换与推理

```bash
ros2 run vla_dataset convert_mcap_to_lerobot \
  --task-id ur5e_red_cube_to_target \
  --repo-id ur5e_red_cube \
  --output dataset/lerobot/ur5e_red_cube
```

默认输入目录为 `dataset/raw`。转换时由任务确定机械臂，再同时按机械臂和任务筛选，
不会混合 SO101 的六维动作与 UR5e 的七维动作。
新录制回合包含 `robot_id`、`joint_names`、`joint_units`；
缺少机械臂元数据的旧回合按 SO101 处理。
导出结果包含 `meta/robot.json`。请将该文件复制到训练检查点的 `config.json`
所在目录，文件名保持为 `robot.json`，以便明确校验机械臂身份。
ACT 始终检查状态和动作维度，不能用 SO101 检查点控制 UR5e。
训练 UR5e 策略需要重新录制 UR5e 示范数据，不能直接使用现有 SO101 数据。

```bash
ros2 launch robot_bringup act_sim.launch.py \
  task_id:=ur5e_red_cube_to_target \
  policy_path:=/absolute/path/to/ur5e/pretrained_model
```

以上是命令模板，目前尚未训练 UR5e 模型检查点。
使用旧 SO101 检查点时，应指定 `robot_id:=so101` 及对应任务。
推理界面的复位功能会清空策略动作队列，并在下一轮开始前复位机械臂和任务。

## 扩展其他机械臂

将模型添加到 `robot_description/mjcf`，在 `robot_core/robots.py` 中实现并注册适配器，
再创建独立任务类，声明 `task_id`、`robot_id`、`scene_file` 和复位逻辑，注册到任务表。
不同场景可以复用同一机械臂模型，但每个场景只绑定一款机械臂。
适配器需要声明对外关节名称、用于读取限位的模型关节、执行器映射、初始姿态和 TCP。
当前 IK 假设机械臂是由旋转关节组成的串联结构，动作最后一维为夹爪；
其他运动学结构需要适合它的重定向求解器。
任务与机械臂的兼容关系需要显式配置，不会自动生成。

## 验证

完成构建并加载工作空间环境后运行：

```bash
python -m pytest tests -q
```

测试使用临时生成的录制数据，不访问真实串口硬件，也不修改用户数据集或训练输出。
正式采集示范数据前，还应手动验证实体主臂遥操作以及任务执行效果。

## 模型来源与许可证

项目代码采用 Apache-2.0 许可证。UR5e 和 Robotiq 模型来自 MuJoCo Menagerie，
固定使用的提交版本及各自的 BSD 许可证记录在
`src/robot_description/mjcf/SOURCES.md` 和对应上游模型目录中。
已有 SO101、D435 和 MetaWorld 资源的来源说明及许可证保留在相应资源目录中。
