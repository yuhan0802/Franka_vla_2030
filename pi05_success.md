# pi0.5 20k 试管抓取成功配置与复现

记录日期：2026-10-15，Asia/Shanghai。本文件保存本次成功测试的参数与操作方法，后续复测以此为基线。

## 一条命令运行

已封装为 `run_20k.sh`。复位实物后执行：

```bash
cd /home/mac/franka_vr_docker
./run_20k.sh
```

默认执行真实运动：固定成功配置，启动服务和相机，等待新机器人控制栈就绪，先 dry-run 检查，再执行最多 240 步，自动保存日志。脚本自行启动的机器人控制栈会在结束后退出；相机和模型服务保留。若复用了原有机器人控制栈，则保留该栈。

`./run_20k.sh dry-run` 仅检查推理、不发出动作；`./run_20k.sh stop` 断开此配置的机器人、相机和 20k 服务；`./run_20k.sh status` 查看状态。详细分步命令仍保留在下文。

## 成功证据

- 测试目录：`logs/pi05_20k_relaxed_20261015_201147/`。
- [执行日志](logs/pi05_20k_relaxed_20261015_201147/execute.log)、[机器人日志](logs/pi05_20k_relaxed_20261015_201147/robot/robot.log)、[结束画面](logs/pi05_20k_relaxed_20261015_201147/after.jpg)。
- 结束画面确认：试管架上已无该试管，试管在红色篮子里。
- 第 109 步发出闭合命令，驱动报告闭合成功；随后继续运动，模型发出张开命令，驱动报告张开成功。
- 共执行 240 步，达到上限后停止。FCI `control_command_success_rate=1.0`，`robot_mode=2`，采样到的 current/last motion errors 均未置位。
- 速度限幅 31 次、加速度限幅 30 次。因此本次成功包含执行保护和 Servo 平滑，不能描述为模型动作完全未经修改。
- 这是一次实机成功记录，不代表每次都成功。上一轮相同配置驱动也曾报告成功，但图像显示试管仍在架上；必须同时核对实际物体位置。

## 固定参数

| 项目 | 本次值 |
|---|---|
| 工作目录 | `/home/mac/franka_vr_docker` |
| 机械臂 FCI IP | `172.16.0.2` |
| 远端 | `vla@10.24.116.66`，SSH 端口 `1172` |
| SSH 私钥文件 | `/home/mac/下载/id_rsa_66`，权限 `600`；不把密钥内容写入文档 |
| checkpoint step | `20000` |
| checkpoint | `/mnt/82_store/lyh/VLA/openpi-main/checkpoints/pi05_franka_tube_pick_v5_finetune/tube_pick_v5_full_bs32_20k/20000` |
| policy config | `pi05_franka_tube_pick_v5_finetune` |
| 推理端点 | `10.24.116.66:8094` |
| 推理 GPU / 远端 tmux | `0` / `pi05-droid-66-8094` |
| 指令 | `put the red test tube into the basket` |
| 外部相机 | 左相机 `/camera_left/image_raw` |
| 腕部相机 | `/camera/camera_front/color/image_raw` |
| 图像 | RGB，640×480 原图送入服务器；本机不重复缩放 |
| 输入 state | 7 维关节位置 rad ＋ 1 维夹爪闭合度 |
| 模型动作块 | `(16, 8)`，7 维关节速度编码 ＋ 1 维夹爪闭合度 |
| 动作频率 | `15 Hz` |
| 动作单位 | `droid`，服务器反归一化后关节速度乘 `3` 得 rad/s |
| 调度 | 严格串行，`prefetch-actions=0`；当前块执行并停止后取新观测、再推理 |
| 本次总步数上限 | `240`，即最多 15 个完整动作块 |
| 关节速度上限 | `0.15 rad/s` |
| 关节加速度上限 | `0.5 rad/s²` |
| 关节限位余量 | `0.15 rad` |
| 观测最大年龄 | `0.5 s` |
| 夹爪控制 | 启用 |
| 夹爪力 / 速度 | `10 N` / `0.02 m/s` |
| 闭合 / 张开阈值 | `0.6` / `0.4`，带滞回 |
| 抓取内侧容差 | `0.010 m`，本次已从原来的 `0.005 m` 放宽 |
| 抓取外侧容差 | `0.030 m` |
| 抓取目标宽度 | 使用模型解码值，本次闭合时约 `0.010 m`；不是固定硬编码宽度 |

右相机不参与本次推理。虽然相机启动脚本可以同时采集右相机，但适配器只取左相机和腕部图像。

夹爪内侧容差定义：允许实际宽度比模型目标宽度更窄 10 mm；外侧允许更宽 30 mm。容差仅影响驱动的抓取成功判定，不改变模型目标宽度、夹爪力或速度。驱动失败仍会触发停止；放宽后可能接受空夹，不能只凭驱动 success 判定任务成功。

## 数据处理链路

| 项目 | 训练处理 | 本次部署处理 |
|---|---|---|
| 关节速度 | 采集的已发送速度 rad/s ÷3，转为 DROID action，再按 action 统计量归一化 | 服务器使用 checkpoint 的 action 统计量反归一化；适配器再 ×3 恢复 rad/s，随后执行保护和 Servo |
| 夹爪 state | `1 - 实测总宽度 / 0.08`，限制到 `[0,1]` | 同样计算闭合度输入模型；state 使用 state 统计量归一化 |
| 夹爪 action | `1 - 目标总宽度 / 0.08` | 服务器反归一化后，`(1 - clip(闭合度,0,1)) * 0.08` 还原目标宽度 m |
| 图像 | 训练管线缩放、补边至模型尺寸 | 原始 RGB 图像交服务器执行相同缩放补边管线 |

不能用 state 统计量反归一化 action。×3 是训练数据中 ÷3 的逆操作。模型服务器已经做统计反归一化，客户端不再重复做一次。

严格串行在推理间隙会停车；240 步不等于整段墙钟时间只有 16 秒，推理等待和夹爪动作也消耗时间。夹爪运动时暂停手臂，成功后继续当前动作块。

## 终端复现

先恢复接近本次起始的机械臂姿态、张开夹爪，把试管直立放在架上、篮子放回原位。保持左相机和腕部相机位置不变。参考 [本次起始图](logs/pi05_20k_relaxed_20261015_201147/before_left.jpg) 与 [腕部起始图](logs/pi05_20k_relaxed_20261015_201147/before_wrist.jpg)。随机推理和实物复位误差意味着不保证逐步动作完全相同。

### 1. 建立本轮日志目录

在后续运行命令的同一个终端执行：

```bash
cd /home/mac/franka_vr_docker
run_dir="$PWD/logs/pi05_20k_relaxed_$(date +%Y%m%d_%H%M%S)"
mkdir -p "$run_dir"
```

### 2. 启动或核实 20k 服务

```bash
VLA_SSH_KEY='/home/mac/下载/id_rsa_66' \
PI05_CHECKPOINT_STEP=20000 \
VLA_SERVER_PORT=8094 \
PI05_SERVER_GPU=0 \
PI05_DROID_TMUX_SESSION=pi05-droid-66-8094 \
./run_pi05.sh server 2>&1 | tee "$run_dir/server.log"
```

输出必须确认 checkpoint 以 `/20000` 结尾，以及 `8094` 和 `/healthz` 正常。已运行时复用该服务。

### 3. 启动相机和机器人，等控制器就绪

```bash
tmux has-session -t franka_vr_pi05_joint_cameras 2>/dev/null || \
  tmux new-session -d -s franka_vr_pi05_joint_cameras \
  'cd /home/mac/franka_vr_docker && exec ./run_vla_cameras.sh'

tmux has-session -t franka_vr_pi05_joint_robot 2>/dev/null || \
  tmux new-session -d -s franka_vr_pi05_joint_robot \
  "cd /home/mac/franka_vr_docker && PI05_LOG_DIR='$run_dir/robot' ROBOT_IP=172.16.0.2 exec ./run_robot_pi05_joint.sh"
```

机器人启动会编译两个包，通常需二十多秒。执行前检查日志：

```bash
tail -n 40 "$run_dir/robot/robot.log"
```

确认 `fr3_arm_controller`、`joint_state_broadcaster`、`franka_robot_state_broadcaster` 已 active，Servo initialized successfully。不要在编译尚未完成时立刻运行适配器，否则可能报 `/servo_node/switch_command_type is unavailable`。本次实际操作是在该服务出现后才执行。

如需查看前台启动输出：`tmux attach -t franka_vr_pi05_joint_robot`，用 Ctrl-B 再按 D 退出查看而保持运行。

### 4. 执行本次相同参数

此命令会产生真实机械臂与夹爪动作：

```bash
set -o pipefail
VLA_INSTRUCTION='put the red test tube into the basket' \
VLA_SSH_KEY='/home/mac/下载/id_rsa_66' \
PI05_DROID_TMUX_SESSION=pi05-droid-66-8094 \
PI05_CHECKPOINT_STEP=20000 \
VLA_SERVER_PORT=8094 \
PI05_SERVER_GPU=0 \
PI05_ENABLE_REAL_ROBOT_EXECUTION=I_UNDERSTAND \
PI05_ENABLE_GRIPPER=true \
PI05_MAX_ACTIONS=240 \
PI05_MAX_JOINT_VELOCITY=0.15 \
PI05_MAX_JOINT_ACCELERATION=0.5 \
PI05_GRIPPER_FORCE=10 \
./run_pi05.sh execute 2>&1 | tee "$run_dir/execute.log"
```

当前代码默认值提供：`action-units=droid`、15 Hz、串行零预取、夹爪速度 0.02、inner 0.010、outer 0.030。启动日志应出现 `action_units=droid; serial=True; image_resize=server`；闭合日志应出现 `epsilon_inner=0.010m epsilon_outer=0.030m`。`PI05_MAX_ACTIONS=240` 和 `PI05_ENABLE_GRIPPER=true` 必须明确设置，脚本自身默认只有 16 步且不启用夹爪。

命令结束后不要只看退出码：日志中若有 `FAULT LATCHED`，表示执行失败停止；达到 240 步只代表测试周期结束。必须观察试管确实进入篮子。

### 5. 停止并释放连接

仅退出机器人控制栈、保留相机和模型服务（本次成功测试后的处理）：

```bash
tmux send-keys -t franka_vr_pi05_joint_robot C-c
```

等待机器人 tmux 和对应 `franka_robot` 容器退出后再手动复位。需要全部断开本轮相机、机器人和 20k 服务时：

```bash
VLA_SSH_KEY='/home/mac/下载/id_rsa_66' \
VLA_SERVER_PORT=8094 \
PI05_DROID_TMUX_SESSION=pi05-droid-66-8094 \
./run_pi05.sh stop
```

此 stop 只针对指定端口/会话，不会停掉服务器上的其他权重服务。运行适配器的终端按 Ctrl-C 可以中止该执行进程，但要释放 FCI 连接仍需退出机器人控制栈。

## 代码与控制栈基线

- `run_pi05.sh` → `run_pi05_joint.sh` → `pi05_joint_velocity_adapter.py`。
- 适配器与速度保护：`workspace/src/franka_vr/franka_vr/scripts/pi05_joint_velocity_adapter.py`、`pi05_joint_velocity_safety.py`。
- 真实机器人入口：`run_robot_pi05_joint.sh`，不是旧 Cartesian 控制入口 `run_robot.sh`。
- 独立 launch：`franka_pi05_joint_velocity.launch.py`；配置：`fr3_pi05_servo.yaml`、`pi05_state_publisher.yaml`。
- Servo：`speed_units`、100 Hz 发布周期、输出 JointTrajectory 至 `/fr3_arm_controller/joint_trajectory`，启用 AccelerationLimitedPlugin 平滑与碰撞检查。`scale.joint=0.15` 为配置值，不应误认为 DROID 反解码额外又乘 0.15。
- FCI 1 kHz；CPU 4 用于 ros2_control，FCI IRQ CPU 2，其余节点 CPUs `0-1,3,6-31`；pi05 Compose overlay cpuset `0-31`。
- state broadcaster 使用本项目已经修改的非阻塞发布逻辑；启动脚本编译 `franka_robot_state_broadcaster` 和 `franka_vr`。
- 本次夹爪容差改动已写入适配器默认值，不需要每次重新改代码。
- 训练数据处理的进一步核验见 [pi05_training_alignment.md](pi05_training_alignment.md)。
