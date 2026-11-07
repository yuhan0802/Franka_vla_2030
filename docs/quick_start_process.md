```bash
ROBOT_IP=172.16.0.2 ./run_robot.sh
```

```bash
./run_vr.sh
```

| LeRobot episode 采集 | `./run_lerobot_record.sh [数据集名称] [秒数] [任务描述]` | 三路 RGB 编码为 MP4；同一数据集自动追加 `episode_000000`、`episode_000001`… 的 LeRobot v2.1 风格 episode。只有 Quest 右手柄 `A` 键会保存本次 episode；达到时长后会停止采样并等待 `A` 键确认，`Ctrl+C` 或异常退出会丢弃本次录制（需同时运行 `./run_vr.sh`） |
