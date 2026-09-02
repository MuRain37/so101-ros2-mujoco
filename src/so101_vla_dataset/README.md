# SO-101 VLA dataset recorder

保留原有的两个 ROS 2 服务，但内部使用 rosbag2 的 Zstd 无损压缩 MCAP 格式。启动
`teleop_sim` 后录制器节点会自动运行，默认不开始录制。

```bash
ros2 service call /dataset/start_episode std_srvs/srv/Trigger "{}"
ros2 service call /dataset/stop_episode std_srvs/srv/Trigger "{}"
```

每次开始服务会在 `dataset/raw/episode_YYYYMMDD_HHMMSS/` 创建一个 rosbag 目录，并录制
`/clock`、关节状态、动作、腕部 RGB、D435 RGB、D435 深度及对应的
`CameraInfo`。可通过 `dataset_output_dir` 和 `dataset_task` 启动参数修改输出目录和任务描述。

开始服务会先确认全部话题存在并确认 rosbag2 成功启动。停止服务会检查必要
话题非空，以及腕部 RGB、D435 RGB 和 D435 深度是否达到默认最低 27 Hz。
检查结果和每个话题的消息数量会写入 `episode.json`；不合格数据会标记为
`status: invalid`，并让停止服务返回失败。

查看录制内容：

```bash
ros2 bag info dataset/raw/episode_20260902_124500/rosbag
```
