# AGENTS.md — 给 AI agent 的工作说明

本仓库控制一台**真实的** Franka FR3 机械臂。读完本页再动手。

## 0. 先读什么（按顺序，约 10 分钟）

1. [docs/architecture.md](docs/architecture.md)：系统怎么连起来，重点看 §3 控制链路和 §9 安全层
2. [docs/weights_and_tasks.md](docs/weights_and_tasks.md)：权重和任务怎么配置、怎么扩展
3. [docs/scripts.md](docs/scripts.md)：每个脚本做什么、会不会动机械臂
4. 与任务相关的 `docs/reports/*`：历史问题，避免重蹈覆辙

## 1. 安全边界（硬规则）

| 级别 | 命令示例 | agent 可以执行？ |
| --- | --- | --- |
| **离线** | `python3 scripts/validate_task_registries.py`、`python3 scripts/check_doc_links.py`、`./run_eef_task.sh list`、`… show`、`./run_task.sh <id> show`、`scripts/test_*.py`、读代码与日志 | ✅ 随时可以 |
| **只读远端** | `./run_eef_task.sh <t> <m> server-status`、`ssh mac_20 'ss -ltn; nvidia-smi'` | ✅ 可以 |
| **启动服务/相机，不发运动** | `server`、`dry-run`、`probe`、`run_vla_cameras.sh`、`run_realsense.sh` | ⚠️ 仅当用户在本次会话中明确要求时 |
| **会动机械臂或夹爪** | `run`、`axis-test`、`home`、`stop`（会回位）、`fci_init.sh`、`homing.sh`、`task*.sh`、`tk*.sh`、`run_robot*.sh`、`run_vr.sh` | ❌ 不要执行；把命令交给现场操作者 |

此外：

- **不要**手动设置确认令牌（`I_UNDERSTAND_DIRECT_EEF`、`I_UNDERSTAND_FULL_DIRECT_TASK`、`I_ACCEPT_VELOCITY_INTERFACE_RISK` 等），也不要设置 `EEF_DIRECT_SKIP_COMMISSION=true`。这些令牌由入口脚本在正确时机设置。
- **不要**为了"让它跑起来"去放宽安全限幅：`eef_task_config.py` 中的范围校验、`pi05_direct_velocity.yaml`、`eef_workspace_bounds.yaml`、控制器 jerk 限制等。如果确实需要调整，单独提交，写明原因，并请人审核。
- **不要**终止 GPU 服务器上不是本次任务启动的 tmux 会话或进程。服务器是共用的。
- **不要**修改上游代码：`workspace/src/` 下除 `franka_vr/` 以外的包、`vendor/`、`upstream-libfranka-docker/`。
- 机器人主机的路径（`/media/mac/Elements1`、`/home/mac/.ssh/…`）和 GPU 服务器的路径只在那些机器上存在；在别的机器上找不到属于正常现象。

## 2. 增量工作法（怎样一步步扩展本项目）

本项目的原则是**追加优先于修改，配置优先于代码，小步快验**。

### 2.1 先确定改动落在哪一层

| 想做的事 | 改哪里 | 不要改哪里 |
| --- | --- | --- |
| 换权重、加模型、加任务、调执行参数 | `config/eef_tasks.json` 或 `vla_instruction_weights.json` | shell 脚本 |
| 新模型协议或服务端 | `server/deploy_*.py` + 协议常量 + 文档 | 现有服务端的握手字段 |
| 适配器行为（安全检查、积分、夹爪逻辑） | `workspace/src/franka_vr/franka_vr/scripts/` 中**不依赖 ROS 的核心模块**（`eef_pose_safety.py`、`eef_rtc_plan.py`、`eef_direct_kinematics.py`、`eef_direct_completion.py`、`*_safety.py`），ROS 节点只做接线 | 把逻辑写进 ROS 回调里 |
| 底层执行与限幅 | `src/pi05_direct_velocity_executor.cpp`、`src/pi05_watchdog_velocity_controller.cpp`、`config/*.yaml` | — （高风险，需人审） |
| 新的启动方式 | 优先给现有入口加模式；实在需要时，组件放 `scripts/`，用户入口放根目录 | 复制粘贴一份现有的 `run_*.sh` |
| 数据问题 | `scripts/data_fix/`，并写报告 | 原始数据（只读） |

### 2.2 追加，不覆盖

- **新权重 = 新 model key**。不要改已有 key 的 `checkpoint`：历史日志里的 `selected_profile.json` 和报告都按 key 引用模型，覆盖之后旧结果就无法复现了。旧模型不再使用时，从 `tasks.<key>.models` 中移除即可，`models` 里的条目可以保留。
- 同理，pi0.5 新权重新增一个 `id`，参考 `task0-fsp-demo10/20` 的做法。
- 调参优先写在模型或任务级的 `runtime`/`adapter` 里，不要改 `defaults`，因为它会影响所有模型。

### 2.3 每一步都要验证

```bash
python3 scripts/validate_task_registries.py      # 注册表
./run_eef_task.sh <task> <model> show            # 合并后的配置是否符合预期
python3 scripts/test_eef_task_runner.py          # 相关离线测试（需要 numpy、bash ≥ 4，在机器人主机上运行）
python3 scripts/check_doc_links.py               # 动过文档就跑
```

然后把 `dry-run` → `axis-test` → `run` 的命令交给操作者，并说明预期现象和停止方法。

### 2.4 一次提交只做一件事

- 提交信息格式参考历史：`config: add Task0 FSP demo checkpoints`、`adapter: …`、`docs: …`。
- 提交内容和配套文档一起改：
  - 动了注册表 → 更新 [docs/weights_and_tasks.md §5](docs/weights_and_tasks.md) 的速查表；
  - 动了脚本入口 → 更新 [docs/scripts.md](docs/scripts.md)；
  - 动了话题、端口或链路 → 更新 [docs/architecture.md](docs/architecture.md)；
  - 做了实验或排查 → 新建 `docs/reports/<主题>_<YYYYMMDD>.md`，先写结论，再写证据。
- 当前开发分支：`feat/eef-pose-control`。

## 3. 代码地图（最常改的地方）

```
config/eef_tasks.json                       EEF 注册表（任务 × 模型）
vla_instruction_weights.json                pi0.5 注册表
run_eef_task.sh                             EEF 主入口 → run_eef_direct.sh / run_eef_pose.sh
scripts/eef_task_config.py                  EEF 解析与校验（新增字段/方法时改这里）
server/deploy_franka_task0_qwen_actioncodec_eef.py   EEF 模型服务（WebSocket，在 GPU 机上跑）
workspace/src/franka_vr/franka_vr/
  scripts/eef_pose_policy_adapter.py        EEF 适配器主体（协议、积分、夹爪、日志）
  scripts/eef_pose_direct_adapter.py        direct 后端：DirectPoseTracker 代替 Servo
  scripts/eef_pose_rtc_adapter.py           RTC 异步交接
  scripts/eef_pose_safety.py                协议常量与安全核心（numpy-only，可离线测试）
  scripts/lerobot_episode_recorder.py       数据采集
  src/pi05_direct_velocity_executor.cpp     100 Hz 关节速度安全执行器
  src/pi05_watchdog_velocity_controller.cpp 1 kHz ros2_control 控制器插件
  src/franka_vr_vel.cpp                     MoveIt Servo + /set_target_pose（VR/键盘/采集）
  launch/franka_pi05_direct_velocity.launch.py   direct 栈
  launch/franka_twist.launch.py             Servo 栈
  config/                                   控制器、Servo、工作空间边界、相机标定
```

## 4. 环境速记

| 名称 | 说明 |
| --- | --- |
| 机器人主机 | Ubuntu + PREEMPT_RT，用户 `mac`，仓库所在机器；Docker 镜像 `franka-vr:humble` |
| FR3 | FCI `172.16.0.2`（`.env` 中的 `ROBOT_IP`） |
| `mac_20` / `10.24.116.20` | GPU 服务器，大部分 EEF 模型 |
| `mac_63` / `10.24.116.63` | GPU 服务器，`act-v2`、`universal-*`、task3 等 |
| `66` / `10.24.116.66` | GPU 服务器，openpi pi0.5（端口 8094，`tk*` 用 8101-8106） |
| 日志 | `$FRANKA_LOG_ROOT/eef_<task>_<model>_<时间>_<pid>/`，含 `selected_profile.json`、`dry_run/`、`execute/`、`policy.jsonl` |
| tmux 会话 | 本机 `franka_vr_eef_direct_robot`、`franka_vr_eef_direct_cameras` 等；远端每个端口一个会话 |

## 5. 排查问题时

1. 先看本次运行的日志目录：`policy.jsonl` 中的 `event` 字段（`armed`、`fault`、`complete`），以及 tmux 抓取的 `*_tmux.log`。
2. 再查 [docs/guides/troubleshooting.md](docs/guides/troubleshooting.md) 和 `docs/reports/` 中是否有同类问题。
3. 涉及夹爪状态时，先读 [reports/task3_gripper_label_bug_20261103.md](docs/reports/task3_gripper_label_bug_20261103.md)。涉及模型在目标上方停住时，先读 [reports/task0_v4_stall_report_20261024.md](docs/reports/task0_v4_stall_report_20261024.md)。
4. 结论写进 `docs/reports/`，不要只留在对话里。
