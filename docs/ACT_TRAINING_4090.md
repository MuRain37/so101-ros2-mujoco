# RTX 4090 上的 ACT 训练参数与实测记录

记录日期：2026-09-08。

## 推荐结论

本项目 UR5e 数据集当前推荐使用 **batch size 8、8 个数据加载进程、OMP 线程数 1、混合精度**，两个任务分别训练 ACT。

这是本次已测试配置中，兼顾训练吞吐、保留优化更新次数和避免额外重启成本的选择，**不是经过完整搜索或任务成功率评估的全局最佳参数**。batch 32 的样本吞吐略高，但不能据此认定模型效果更好。

## 环境与数据

远端 Conda 环境：`so101`，路径 `/root/miniconda3/envs/so101`。

| 项目 | 配置 |
|---|---|
| GPU | 单张 NVIDIA RTX 4090，24 GB 显存 |
| 主机资源 | 平台标示 16 核 CPU、120 GB 内存 |
| Python | 3.12.14 |
| LeRobot | 0.5.1 |
| PyTorch | 2.10.0+cu128 |
| torchvision | 0.25.0+cu128 |
| huggingface-hub | 1.30.0 |
| PyAV | 15.1.0 |
| 数据格式 | LeRobot v3.0 |
| 观测与动作 | 7 维状态、7 维动作；front / wrist 双路 RGB |
| 视频 | 640×480、30 FPS、AV1 |

| 任务目录 | 示范回合 | 帧数 |
|---|---:|---:|
| `ur5e_red_blue_cubes` | 57 | 14,933 |
| `ur5e_red_cube_in_drawer` | 52 | 18,832 |

本地数据位于项目的 `ur5e_dataset/`；本次远端数据直接位于 `/root/autodl-tmp/so101-ros2-mujoco/` 下。

环境注意事项：远端 `pip check` 通过。项目现有 `requirements.txt` 将 huggingface-hub 固定为 0.35.3，与已安装 LeRobot 0.5.1 声明的 `huggingface-hub>=1.0.0` 不一致；复现本次训练应使用上述已验证的 Conda 环境，不要直接将其降级。此次没有修改项目依赖文件或 LeRobot 源码。

## 推荐训练配置

| 参数 | 值 | 说明 |
|---|---|---|
| `policy.type` | `act` | 每个任务单独训练 |
| `policy.device` | `cuda` | GPU 训练 |
| `policy.use_amp` | `true` | 混合精度 |
| `batch_size` | `8` | 本轮正式训练配置 |
| `num_workers` | `8` | 原先为 4，调整后数据等待显著减少 |
| `OMP_NUM_THREADS` | `1` | 环境变量；原先为 4，与 workers 一起调整 |
| `dataset.video_backend` | `pyav` | 实测能正常读取 AV1 视频 |
| `policy.chunk_size` | `100` | 一次预测 100 帧动作 |
| `policy.n_action_steps` | `100` | 推理动作执行窗口 |
| `policy.optimizer_lr` | `1e-5` | 本次 ACT 默认学习率 |
| `policy.optimizer_lr_backbone` | `1e-5` | 视觉骨干学习率 |
| `steps` | `50000` | 每个任务的当前训练预算，不是已验证的收敛要求 |
| `save_freq` | `10000` | 保留中间检查点用于后续评估 |
| `log_freq` | `100` | 每 100 步记录训练指标 |
| `eval_freq` | `0` | 训练期间不自动执行仿真评估 |
| `wandb.enable` | `false` | 日志保存在远端本地 |
| `policy.push_to_hub` | `false` | 不自动上传模型 |

其他 ACT 配置沿用 LeRobot 0.5.1 默认值，包括 ResNet18 及 ImageNet 预训练权重。本次环境已经缓存骨干权重。

TorchCodec 0.10.0 在该环境中因缺少 FFmpeg 动态库而导入失败，因此使用 PyAV。此次未安装新的解码依赖，也未修改解码实现。

## 吞吐测试结果

以下结果来自红蓝分类数据集；抽屉任务尚未做独立吞吐测试。

| 配置 | 每步更新耗时 | 每步数据等待 | 训练速度 | 样本吞吐 | 进程显存 |
|---|---:|---:|---:|---:|---:|
| batch 8 / workers 4 / OMP 4 | 约 75 ms | 约 40 ms | 约 8.5 步/秒 | 约 68 样本/秒 | 约 4.8 GB |
| **batch 8 / workers 8 / OMP 1** | **约 77–80 ms** | **约 3–6 ms** | **约 12–12.5 步/秒** | **约 96–100 样本/秒** | **约 4.7–4.9 GB** |
| batch 32 / workers 8 / OMP 1 | 约 262–270 ms | 约 24–42 ms | 稳定约 3.3–3.4 步/秒 | 稳定约 107 样本/秒 | 约 15.1 GB |

- 调整 workers 和 OMP 后，吞吐比原配置提高约 40%，数据等待大幅减少。由于两个参数同时调整，不能把收益单独归因于其中一个参数。
- 推荐配置连续 20 秒 GPU 利用率采样平均约 83%，范围 68–93%；功耗多数约 300–313 W，温度约 61–64°C。
- batch 32 测试运行 300 步，无报错；总用时约 96 秒，含预热的平均速度为 3.12 步/秒。表中稳定吞吐根据预热后日志计算。
- batch 32 的 GPU 利用率多次达到 94–100%，但稳定样本吞吐只比推荐配置高约 9%。显存用得更多不代表训练成本成比例降低。
- 测试 batch 32 时，正式训练及其加载进程通过暂停信号挂起，测试结束后恢复。正式训练保留约 4.9 GB 显存，因此当时 GPU 总显存约 20 GB，其中测试进程自身约 15.1 GB。
- 短测只验证运行稳定性和吞吐，未验证最终任务成功率。正式训练已恢复 batch 8，暂停前进度没有丢失。

## 可复现训练命令

以下命令在远端执行。仅当没有重复训练进程且目标输出目录不存在时启动新训练。两个任务依次运行，各 5 万步。

```bash
conda activate so101
cd /root/autodl-tmp/so101-ros2-mujoco
export OMP_NUM_THREADS=1
export PYTHONUNBUFFERED=1
mkdir -p logs

for task in ur5e_red_blue_cubes ur5e_red_cube_in_drawer; do
  lerobot-train \
    --dataset.repo_id="local/${task}" \
    --dataset.root="$PWD/${task}" \
    --dataset.video_backend=pyav \
    --policy.type=act \
    --policy.device=cuda \
    --policy.use_amp=true \
    --policy.push_to_hub=false \
    --policy.chunk_size=100 \
    --policy.n_action_steps=100 \
    --policy.optimizer_lr=1e-5 \
    --policy.optimizer_lr_backbone=1e-5 \
    --batch_size=8 \
    --num_workers=8 \
    --steps=50000 \
    --save_freq=10000 \
    --log_freq=100 \
    --eval_freq=0 \
    --wandb.enable=false \
    --output_dir="outputs/train/act_${task}" \
    --job_name="act_${task}" \
    > "logs/train_${task}.log" 2>&1 || break

  for checkpoint in "outputs/train/act_${task}"/checkpoints/*/pretrained_model; do
    cp "${task}/meta/robot.json" "$checkpoint/robot.json"
    cp "${task}/meta/cameras.json" "$checkpoint/cameras.json"
  done
done
```

远端已有 `train_act.sh` 封装上述训练流程，并已应用推荐加载参数。新启动后台训练可使用：

```bash
nohup bash train_act.sh train > train_queue.log 2>&1 < /dev/null &
```

该脚本目前仅保存在远端。项目的本文档提供独立的参数与命令记录。模型元数据在每个任务训练结束后复制到检查点；若提前取用中间检查点，应手动复制对应数据集的 `meta/robot.json` 至该检查点的 `pretrained_model/robot.json`，以满足项目推理校验。

## 时间预算与效果判断

推荐配置按 12 步/秒估算，每任务 5 万步纯训练约 69 分钟。两个任务可先预留约 2.5 小时，实际以各任务的稳定速度和检查点保存耗时为准，避免使用启动预热阶段的 ETA。

batch 8 × 50,000 步共处理约 40 万个训练样本。batch 32 × 12,500 步处理相同数量样本，按本次稳定速度约需 62 分钟，理论上每任务节省约 6 分钟；这不是模型效果等价的保证，也未计入切换和评估成本。

最终应在对应 MuJoCo 场景中比较各检查点的任务成功率，使用一致的初始条件和重复试验。红蓝任务检查两个方块是否分别放到对应区域；抽屉任务检查是否完整完成打开、放入和关闭三个阶段。训练 loss 下降或 GPU 跑满均不能替代任务成功率评估。
