# SO-101 VLA dataset recorder

提供开始、停止和取消三个 ROS 2 服务，但内部使用 rosbag2 的 Zstd 无损压缩 MCAP 格式。启动
`teleop_sim` 后录制器节点和控制窗口会自动运行，默认不开始录制。窗口提供
“开始录制”和“停止录制”两个按钮、录制计时，并显示服务返回的保存路径或错误。
任务语言指令由仿真任务定义统一提供，不在 UI 中编辑。
按 `R` 可开始录制，按 `S` 可停止录制，按空格可取消当前录制；按钮不可用时对应快捷键也不会执行。
机械臂始终跟随主臂；开始和停止按钮只控制数据录制。可以在未录制状态下
将机械臂调整到合适的安全起始姿态。

无界面运行时关闭窗口：

```bash
ros2 launch robot_bringup teleop_sim.launch.py dataset_gui_enabled:=false
```

也可以单独启动窗口：

```bash
ros2 run vla_dataset dataset_gui
```

正在录制时关闭窗口，会先询问是否安全停止；确认后等待 rosbag 写盘和验证完成再退出。
停止录制并完成写盘后，录制器会调用 `/sim/reset_task`，由当前 `task_id` 执行
场景重置，同时保持机械臂跟随和仿真时钟连续。
启动时设置 `random_seed:=123` 可复现方块随机序列；默认 `-1` 为非确定随机。
命令行服务调用方式仍然保留：

```bash
ros2 service call /dataset/start_episode std_srvs/srv/Trigger "{}"
ros2 service call /dataset/stop_episode std_srvs/srv/Trigger "{}"
ros2 service call /dataset/cancel_episode std_srvs/srv/Trigger "{}"
```

每次开始服务会在 `dataset/raw/episode_YYYYMMDD_HHMMSS/` 创建一个 rosbag 目录。
`/clock`、关节状态和动作固定录制；图像与 `CameraInfo` 话题由所选 Task 的
相机配置自动追加。当前四个 Task 录制腕部 RGB 和全局 D435 RGB，不录制深度。
`episode.json` 会保存 `task_id`、语言指令和完整相机 schema。

开始服务会确认 Task 要求的全部话题存在。停止服务会检查必要话题非空，并按
相机声明频率的 90% 验证每条图像流。检查结果和消息数量写入 `episode.json`；
不合格数据标记为 `status: invalid`。

查看录制内容：

```bash
ros2 bag info dataset/raw/episode_20260902_124500/rosbag
```
