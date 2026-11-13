# 脚本目录

根目录有约 45 个 `*.sh`。它们**故意留在根目录**，原因有三：

- 脚本通过 `$root_dir/<兄弟脚本>.sh` 互相调用；
- `docker compose` 的项目标签（`com.docker.compose.project.working_dir`）被 stop/status 用来识别容器；
- `scripts/test_*.py` 也按根目录路径引用它们。

移动任何一个都要同步改调用方并在机器人主机上回归，所以这里用一张分类表代替搬家。

图例：**●** 日常主流程　**○** 支撑/底层组件（通常不直接调用）　**◇** 快捷方式（一行转发）　**△** 遗留/复现旧实验（不要用于新工作）

## 1. 主机与机器人准备

| 脚本 | 状态 | 作用 | 会动机械臂？ |
| --- | --- | --- | --- |
| `configure_fci_realtime_host.sh` | ○ | 一次性配置 GRUB 隔离 CPU 与 FCI 网卡中断，需要 sudo 并重启 | 否 |
| `fci_init.sh` | ● | 上电后初始化：夹爪重新初始化 + 机械臂回到 `config/default_initial_pose.json` | **是** |
| `homing.sh` | ● | 夹爪 homing（夹爪宽度异常、录制时报"未回零"时使用） | 夹爪 |
| `detect.sh` | △ | FrankaVR 三视角 VLA 只读预检（2026-10 的九项检测） | 否 |

## 2. 遥操作、仿真与数据采集（MoveIt Servo 链路）

| 脚本 | 状态 | 作用 |
| --- | --- | --- |
| `run_robot.sh` | ● | 真机 Servo 栈（`franka_twist.launch.py`），VR/键盘/采集的基础 |
| `run_vr.sh` | ● | Quest 3 桥接；需先启动 `run_robot.sh` 或 `run_gazebo_vr.sh` |
| `run_lerobot_record.sh <数据集> <秒数> <任务描述>` | ● | 录制 LeRobot episode，按 Quest `A` 键保存 |
| `run_merge_lerobot_datasets.sh` | ○ | 合并本地录制的数据集（不改动输入） |
| `run_robot_manual_control.sh` | ○ | 真机键盘控制（第二终端） |
| `run_gazebo.sh` / `run_gazebo_vr.sh` | ● | Gazebo 仿真 + MoveIt（+ Servo） |
| `run_manual_control.sh` | ○ | Gazebo 键盘控制（第二终端） |
| `run_realsense.sh` | ○ | 单独启动 RealSense，不控制机器人 |
| `run_vla_cameras.sh` | ○ | 三路 VLA 相机（左右 C922 + RealSense），被各 VLA 运行器调用 |

## 3. EEF 位姿 VLA（当前主流程，注册表 `config/eef_tasks.json`）

```
run_eef_task.sh ─┬─ scripts/eef_task_config.py   (解析/校验)
                 ├─ run_eef_direct.sh  ── run_robot_pi05_direct.sh + run_vla_cameras.sh   (默认 direct)
                 ├─ run_eef_pose.sh    ── run_robot.sh + run_vla_cameras.sh               (EEF_CONTROL=servo)
                 │      └─ server|server-status|server-stop: ssh/scp server/deploy_*.py 到 GPU 机
                 └─ fci_init.sh       (仅 stop 时)
```

| 脚本 | 状态 | 作用 |
| --- | --- | --- |
| `run_eef_task.sh <task> <model> <mode>` | ● | **唯一推荐入口**。模式：`list` `show` `server-status` `dry-run` `axis-test ±x/y/z/rx/ry/rz` `home` `probe` `run` `stop` `stop-only` `status` |
| `run_eef_direct.sh` | ○ | direct 后端运行器（无 Servo） |
| `run_eef_pose.sh` | ○ | Servo 后端运行器 + 远端 EEF 模型服务管理 |
| `task1-act-v2.sh` `task1-act.sh` `task1-act-1k.sh` `task1-act2.sh` `task1-droid.sh` | ◇ | `run_eef_task.sh task1 <model> "$@"` |
| `task1-test.sh` | ○ | 批量重复试验：服务只部署一次，结果写入 `results.csv` |
| `task1-replay.sh` | ○ | 不跑模型，把训练数据轨迹原样回放到真机（用 `server/replay_eef_trajectory_server.py`） |

## 4. pi0.5 关节速度（注册表 `vla_instruction_weights.json`）

```
task1.sh / task2.sh / task6.sh / task0-fsp-demo*.sh
   └─ run_task.sh <id> ── scripts/resolve_direct_task.py
         └─ run_pi05_direct.sh full-execute ── run_vla_pi05_droid_66.sh (远端服务)
                                             └─ run_robot_pi05_direct.sh + run_vla_cameras.sh
```

| 脚本 | 状态 | 作用 |
| --- | --- | --- |
| `run_task.sh <id> [run\|stop\|stop-only\|status\|show]` | ● | pi0.5 关节任务入口，按 id 读注册表 |
| `task1.sh` `task2.sh` `task6.sh` `task0-fsp-demo10.sh` `task0-fsp-demo20.sh` | ◇ | `run_task.sh <id> "$@"` |
| `task.sh exc1\|exc2\|stop` | ● | Task1+Task2 联合权重（20 号机 8094，**权重写死在脚本中**） |
| `tk1.sh` `tk2.sh` `tk6.sh` | ● | 复用 66 号机常驻服务（8101/8102/8106），**参数写死在 `scripts/run_tk_direct.sh`** |
| `run_pi05_direct.sh` | ○ | direct 关节速度运行器（server/probe/dry-run/commission/execute/full-execute） |
| `run_robot_pi05_direct.sh` | ○ | direct 速度机器人栈（EEF direct 也用它） |
| `run_vla_pi05_droid_66.sh` | ○ | 66 号机 pi0.5 服务启停 |

## 5. 遗留入口（复现旧实验用）

| 脚本 | 说明 |
| --- | --- |
| `run_20k.sh` | 复现 2026-10-15 的 pi0.5 20k 成功试验，见 [archive/pi05_success.md](archive/pi05_success.md) |
| `run_7k.sh` | pi0.5 checkpoint-7000，独立端口 8095 |
| `run_pi05.sh` → `run_pi05_joint.sh` → `run_robot_pi05_joint.sh` | pi0.5 JointJog 经 MoveIt Servo（旧链路） |
| `run_sim2real.sh` | 仿真训练的粉色试管权重上真机，见 [reference/pi05_real_vs_sim2real_alignment.md](reference/pi05_real_vs_sim2real_alignment.md) |
| `run_vla.sh`、`run_franka_vr_vla_server.sh`、`run_franka_vr_vla_bridge.sh` | 旧 FrankaVR 三视角 ActionCodec 关节动作模型（20 号机 8093），见 [guides/frankavr_joint_vla_deploy.md](guides/frankavr_joint_vla_deploy.md) |

## 6. `scripts/` 子目录

| 位置 | 内容 |
| --- | --- |
| `scripts/validate_task_registries.py` | **改完注册表先跑它**，离线校验两份注册表 |
| `scripts/eef_task_config.py`、`resolve_direct_task.py`、`select_vla_task.py` | 注册表解析器（启动器调用） |
| `scripts/test_*.py` | 离线单元/集成测试（需要 numpy 和 Linux bash 4+，在机器人主机或容器内运行） |
| `scripts/run_tk_direct.sh`、`fci_arm_reset.sh`、`pi05_*.sh`、`runtime_log_root.sh` | 被根目录脚本调用的组件 |
| `scripts/*probe*`、`check_*.py`、`detect_vla_model_io.py`、`wait_tk_cameras.py` | 只读诊断工具 |
| `scripts/*.cpp` | 夹爪/FCI 底层探测与复位小程序（libfranka） |
| `scripts/data_fix/` | 数据集夹爪标签核验与修复 |
| `scripts/offline_eval/` | 在 GPU 机上做的离线开环/闭环评估 |

## 7. 新增脚本的约定

- **优先不新增**。换权重、加任务走注册表（[weights_and_tasks.md](weights_and_tasks.md)）。
- 快捷方式只允许一行 `exec` 转发，不写逻辑。
- 新的底层组件放进 `scripts/`；只有用户日常直接敲的入口才放根目录。
- 开头用注释说明：做什么、会不会动机械臂、依赖哪个栈。
- 会动机械臂的模式必须先有 dry-run 路径，且沿用现有确认令牌机制。
- 新增或改名后更新本页。
