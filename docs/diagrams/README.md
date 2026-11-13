# 架构图（PNG）

| 图 | 看什么 |
| --- | --- |
| [01_overall_architecture.png](01_overall_architecture.png) | **整体分层**：入口 → 配置 → 编排 → 模型 ⇅ ROS 2 → 控制 → 硬件。每层的组件、层间传递的内容、节拍与安全闸门 |
| [02_weight_config.png](02_weight_config.png) | **权重如何配置**：以 `task40 × universal-mulaw9` 为例，从 JSON 三个区块到合并结果、环境变量、远端服务启动参数、适配器握手 |
| [03_control_paths.png](03_control_paths.png) | **三条控制链路**：direct 关节速度（默认）/ MoveIt Servo（遥操作与采集）/ 遗留 JointJog，逐节点、逐话题对比 |
| [04_end_to_end_example.png](04_end_to_end_example.png) | **完整示例**：登记一个新权重的 5 步，以及 `./run_eef_task.sh task1 act-v2 run` 从命令到日志的 9 步 |

图中的数值（端口、GPU、checkpoint、话题名）取自 2026-11-13 的 `config/eef_tasks.json` 和代码。改了注册表或链路后请同步更新源文件并重新生成：

```bash
docs/diagrams/render.sh                      # 全部重新生成（需要本机安装 Chrome / Chromium）
docs/diagrams/render.sh 02_weight_config     # 只生成一张
```

源文件在 `src/*.html`，共用样式在 `src/style.css`。图片宽 1800 px，以 2× 分辨率输出。
