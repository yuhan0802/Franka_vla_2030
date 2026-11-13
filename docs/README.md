# 文档索引

按「先理解 → 再操作 → 查参考 → 看历史」组织。新增文档请放进对应目录，并在这里加一行。

## 顶层：先读这三篇

| 文档 | 内容 |
| --- | --- |
| [architecture.md](architecture.md) | 系统架构：部署与网络、软件分层、三条控制链路、推理时序、配置解析、数据闭环、CPU 布局、模型协议、安全层 |
| [weights_and_tasks.md](weights_and_tasks.md) | 两份注册表的字段说明、增量扩展手册、当前模型速查表、常见坑 |
| [scripts.md](scripts.md) | 根目录与 `scripts/` 的全部脚本：分类、状态（主流程/支撑/快捷/遗留）、调用关系 |

## guides/ — 操作手册

| 文档 | 内容 |
| --- | --- |
| [guides/eef_task_launcher.md](guides/eef_task_launcher.md) | `run_eef_task.sh` 的各模式、启动与停止顺序、日志位置 |
| [guides/teleop_sim_recording.md](guides/teleop_sim_recording.md) | Gazebo、RViz、键盘、Quest 3、RealSense、LeRobot 采集、真机 Servo 栈 |
| [guides/troubleshooting.md](guides/troubleshooting.md) | 常见问题与排障（控制器、RViz、夹爪、VR、X11、FCI 网络） |
| [guides/frankavr_joint_vla_deploy.md](guides/frankavr_joint_vla_deploy.md) | 旧 FrankaVR 三视角关节动作模型（`run_vla.sh`，20 号机 8093）的部署手册 |
| [guides/frankavr_server_deploy.md](guides/frankavr_server_deploy.md) | 上述 8093 服务端部署 |

## reference/ — 参考资料

| 文档 | 内容 |
| --- | --- |
| [reference/eef_pose_control_adapter.md](reference/eef_pose_control_adapter.md) | EEF 位姿控制的精确语义：训练协议、服务端与 bridge 的改动、安全层、上线检查清单 |
| [reference/robot_vla_basics.md](reference/robot_vla_basics.md)（[HTML 版](reference/robot_vla_basics.html)） | 入门知识：坐标系、姿态表示、左乘/右乘、运动学、控制方式、ROS 2、相机、夹爪、VLA、数据格式 |
| [reference/pi05_real_vs_sim2real_alignment.md](reference/pi05_real_vs_sim2real_alignment.md) | pi0.5 真机权重与 Sim2Real 权重的对齐对比 |

## reports/ — 按日期的分析报告

报告是**当时的结论**，不一定代表现状；与代码或注册表冲突时，以代码和注册表为准。

| 日期 | 文档 | 一句话 |
| --- | --- | --- |
| 2026-10-24 | [reports/task0_v4_stall_report_20261024.md](reports/task0_v4_stall_report_20261024.md) | Task0 v4 在试管上方停住不夹取：codec 底噪与动作放大 |
| 2026-10-25 | [reports/task1_v5_version_record_20261025.md](reports/task1_v5_version_record_20261025.md) | Task1 v5 实机成功的两套配置（x3 / maxabs） |
| 2026-10-27 | [reports/experiments/20261027_oct25_baseline_retest.md](reports/experiments/20261027_oct25_baseline_retest.md) | 10-25 基线复测，含日志盘 A/B |
| 2026-10-30 | [reports/task1_eef_failure_analysis_20261030.md](reports/task1_eef_failure_analysis_20261030.md) | Task1 EEF 实机失败分析与离线评估（证据在 `reports/experiments/20261030_task1_eef_offline_eval/`） |
| 2026-11-03 | [reports/task3_gripper_label_bug_20261103.md](reports/task3_gripper_label_bug_20261103.md) | Task3 夹爪标签错误：根因与修复（`scripts/data_fix/`） |

## archive/ — 早期文档

早期 pi0.5 与检测脚本的文档，仅供追溯：[archive/](archive/)。

## 写文档的约定

- 中文为主，命令、路径、话题名、字段名保持原样并用反引号包裹。
- 新报告放在 `reports/`，文件名为 `<主题>_<YYYYMMDD>.md`，开头先写结论；证据（图片、JSON、脚本）放进同名子目录或 `reports/experiments/<YYYYMMDD>_<主题>/`。
- 操作类内容放 `guides/`，长期有效的原理与协议放 `reference/`。
- 改了话题名、端口、注册表字段或脚本入口时，同步更新 `architecture.md`、`weights_and_tasks.md`、`scripts.md` 中受影响的部分。
- 移动文档后运行链接检查（见 [AGENTS.md](../AGENTS.md)）。
