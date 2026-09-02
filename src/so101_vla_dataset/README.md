# SO-101 VLA dataset recorder

保留原有的两个 ROS 2 服务，但内部使用 rosbag2 的 Zstd 无损压缩 MCAP 格式。启动
`teleop_sim` 后录制器节点和控制窗口会自动运行，默认不开始录制。窗口提供
“开始录制”和“停止录制”两个按钮、录制计时，并显示服务返回的保存路径或错误。
每次开始前可以在“任务指令”输入框中修改本次 Episode 的语言指令。
按 `R` 可开始录制，按 `S` 可停止录制；按钮不可用时对应快捷键也不会执行。
机械臂始终跟随主臂；开始和停止按钮只控制数据录制。可以在未录制状态下
将机械臂调整到合适的安全起始姿态。

无界面运行时关闭窗口：

```bash
ros2 launch so101_bringup teleop_sim.launch.py dataset_gui_enabled:=false
```

也可以单独启动窗口：

```bash
ros2 run so101_vla_dataset so101_dataset_gui
```

正在录制时关闭窗口，会先询问是否安全停止；确认后等待 rosbag 写盘和验证完成再退出。
停止录制并完成写盘后，录制器会调用 `/sim/reset_task`，由当前 `task_id` 执行
场景重置，同时保持机械臂跟随和仿真时钟连续。
启动时设置 `random_seed:=123` 可复现方块随机序列；默认 `-1` 为非确定随机。
命令行服务调用方式仍然保留：

```bash
ros2 service call /dataset/start_episode std_srvs/srv/Trigger "{}"
ros2 service call /dataset/stop_episode std_srvs/srv/Trigger "{}"
```

每次开始服务会在 `dataset/raw/episode_YYYYMMDD_HHMMSS/` 创建一个 rosbag 目录，并录制
`/clock`、关节状态、动作、腕部 RGB、D435 RGB、D435 深度及对应的
`CameraInfo`。可通过 `dataset_output_dir`、`dataset_task_id` 和 `dataset_task`
启动参数修改输出目录、任务实现和语言描述。`episode.json` 会同时记录 `task_id`
与语言指令；成功检测暂未启用。

开始服务会先确认全部话题存在并确认 rosbag2 成功启动。停止服务会检查必要
话题非空，以及腕部 RGB、D435 RGB 和 D435 深度是否达到默认最低 27 Hz。
检查结果和每个话题的消息数量会写入 `episode.json`；不合格数据会标记为
`status: invalid`，并让停止服务返回失败。

查看录制内容：

```bash
ros2 bag info dataset/raw/episode_20260902_124500/rosbag
```
