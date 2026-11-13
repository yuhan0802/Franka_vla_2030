# FrankaVR 真机 VLA 部署手册

本手册用于三视角 FrankaVR 真机模型。当前 `mac_20` 只做 WebSocket 推理；本机负责
相机、ROS、工作区保护和 FCI。旧 LIBERO/8092 服务不适用于此流程。

> [!CAUTION]
> 这是实验性真机控制，不是安全认证系统。执行时急停必须可达，工作区必须清空，
> 操作者必须全程在场。任何异常先停止新目标，必要时立即使用机器人急停。

## 架构与通信链路

```text
RealSense(front) + C922(left/right) + FR3 joint state
                    │ ROS 2
                    ▼
franka_vr_vla_bridge.py (本机：观测、限幅、边界、动作调度)
                    │ MessagePack/WebSocket :8093
                    ▼
mac_20 ActionCodec policy (仅推理)
                    │ action[15, 7]
                    ▼
本机桥接器 ── /set_target_pose ──► Servo ──► effort trajectory controller ──► FCI/FR3
```

模型请求包含三张 `uint8 RGB` 图像、`float32[8]`（7 个关节角 + 固定为零的
夹爪状态）及任务文本。响应为 `float32[15,7]`，每步是
`xyz + rpy + gripper`。

本机只顺序执行每块的前 10 步：动作以 **15 Hz** 下发（约 67 ms/步），推理
请求上限为 **30 Hz**。推理慢于约 33 ms 时，实际请求频率会自动下降；执行期间
只保留最新预测块，避免陈旧动作堆积。第 11–15 步不执行。

每一动作先按 `xyz × 0.075 m`、`rpy × 0.15 rad` 换算，再受单步上限、
`workspace_bounds_world` 和 Servo 碰撞/奇异性保护约束。推理等待或目标被拒绝时，
桥接器每 0.2 秒刷新最后一个安全位姿，避免控制指令间隙。

| 项目 | 当前值 |
| --- | --- |
| 远端模型 | `mac_20` / `10.24.116.20:8093` |
| 远端项目 | `/mnt/82_store/lyh/VLA/Ours_VQAQ_VLA` |
| 本机入口 | `./run_vla.sh` |
| 本机桥接器 | `workspace/src/franka_vr/franka_vr/scripts/franka_vr_vla_bridge.py` |
| 训练动作块 | 15 步 × 7 维 |
| 本机回放 | 前 10 步，15 Hz；请求上限 30 Hz |

## 最简使用流程

首次使用或 Dockerfile 变更后构建一次：

```bash
cd /home/mac/franka_vr_docker
docker compose build
```

确认急停、台面、相机和 FCI 均已准备好后，按以下顺序操作：

```bash
cd /home/mac/franka_vr_docker

# 1. 查看远端服务与本地会话状态
./run_vla.sh status

# 2. 用真实图像、关节状态做空跑；不会调用 /set_target_pose
VLA_INSTRUCTION='put the red test tube into the basket' ./run_vla.sh dry-run

# 3. 确认输出方向、相机视角和边界后，按 Ctrl-C 结束空跑，再执行真机
VLA_INSTRUCTION='put the red test tube into the basket' ./run_vla.sh execute
```

`execute` 自动处理远端服务、本机真机栈和三路相机。服务已经监听时会复用它，
不会重复启动。运行中桥接器处于前台；按 `Ctrl-C` 仅停止新 VLA 目标，真机控制栈
继续保持。用下面命令结束整套服务：

```bash
./run_vla.sh stop
```

## 真机前置检查

1. 急停可达；工作区没有人、障碍物或不受控的夹具。
2. 不得同时运行 Gazebo、fake hardware 或第二个真实控制栈；它们会争用
   `/controller_manager`/FCI。
3. 使用专用 FCI 网口。出现 `communication_constraints_violation` 时，先修复
   网卡节能、链路、PREEMPT_RT、CPU governor 与 IRQ 亲和性，再测试模型。
4. 三路图像、关节状态与 `world -> fr3_link8` TF 必须正常。
5. 必须实测工作区，不能为追求范围而放宽未知区域。

边界文件是
`workspace/src/franka_vr/franka_vr/config/vla_calibration.yaml`：

```yaml
workspace_bounds_world:
  min: [x_min, y_min, z_min]
  max: [x_max, y_max, z_max]
```

它使用 ROS `world` 坐标系和米。当前实际配置为 `x ≤ 0.75`；任何改动都应先
通过空跑和低速、空载测试验证。

## 运行时判读与可调参数

正常日志会显示已连接
`qwen3vl2b_franka_vr_actionfix_v2`，随后出现：

```text
Queued 10 actions from FrankaVR 15-step chunk; first=[...]
Starting 10-action VLA block at 15 Hz
```

默认值及其覆盖方式：

```bash
VLA_POLICY_HZ=30 VLA_ACTIONS_PER_CHUNK=10 VLA_ACTION_HZ=15 \
VLA_MAX_TRANSLATION=0.010 VLA_MAX_ROTATION=0.08 \
VLA_INSTRUCTION='put the red test tube into the basket' ./run_vla.sh execute
```

默认使用实时 7 维关节与夹爪状态构造模型观测。夹爪观测按总开口归一化：
`0=关闭`、`1=完全打开（0.12 m）`。仅在复现实验时才设置
`VLA_OBSERVATION_STATE='[...8 个数...]'` 固定模型输入；这不会绕过真实 TF、
工作区或 Servo 安全检查。

发送给后端前，第 8 维始终量化为严格的 `0` 或 `1`：总开口低于 `0.06 m`
传 `0`，达到或超过 `0.06 m` 传 `1`；固定观测状态也使用相同阈值。

不要仅因动作方向异常而提高单步限幅或边界；应回到 `dry-run` 检查相机左右对应、
相机安装、`world` 坐标和训练时输入顺序。

夹爪默认启用。动作的第 7 维（Python 索引 `6`）采用 `0=关闭、1=打开`：
`≥0.50` 发打开命令，`≤0.40` 发关闭命令（0.10 滞回）。不要把它与输入
`joint_state` 的第 8 维混淆；该输入维度同样是 `0=关闭、1=打开`。
默认宽度为关闭 `0.0 m`、打开 `0.12 m`、速度 `0.02 m/s`。如需临时禁用：

```bash
VLA_ENABLE_GRIPPER=false \
VLA_INSTRUCTION='put the red test tube into the basket' ./run_vla.sh execute
```

## 停止与异常处理

| 情况 | 操作 |
| --- | --- |
| 只结束 VLA 动作 | 桥接器前台按 `Ctrl-C`。 |
| 结束整套服务 | `./run_vla.sh stop`。 |
| 方向、边界、碰撞或相机异常 | 先按桥接器 `Ctrl-C`；必要时急停。不要继续执行。 |
| FCI 告警或断开 | 立即停止执行，检查实时链路；模型服务不能修复 FCI 问题。 |
| 机器人安全退出等待中 | 保留容器和 tmux，查看 ROS 日志后重试 `./run_vla.sh stop`。 |

真机控制栈必须通过 `Ctrl-C` 或 `./run_vla.sh stop` 退出。后者发送 SIGINT 并
等待 ROS 完成受控停机。禁止用 `docker stop`、短超时停止或 `SIGKILL` 强杀控制
容器，否则可能在控制器完成安全交接前中断它。

## 排障速查

```bash
# 本机服务/会话
./run_vla.sh status
docker ps --filter "label=com.docker.compose.project.working_dir=$(pwd)"

# 远端模型端口和 GPU
ssh mac_20 'ss -ltnp | grep :8093 || true; nvidia-smi'
```

| 现象 | 优先检查 |
| --- | --- |
| 等待三路图像 | 相机 ROS 话题与 C922 的 `/dev/v4l/by-id` 路径。 |
| 等待关节或 TF | 真机栈、`/joint_states` 的 `fr3_joint1..7`、`world -> fr3_link8`。 |
| 8093 不可达 | SSH、端口监听、GPU 显存；不要启两个服务。 |
| 工作区拒绝 | 模型方向和实测边界；不要删除边界保护。 |
| 动作块不连续 | 远端推理耗时、网络、相机/关节更新；日志中的实际请求间隔。 |

## 维护边界

修改 ROS 包后，启动脚本会在容器内重建 `franka_vr`。只有修改 Dockerfile、系统
依赖或基础镜像时才需要 `docker compose build`。修改
`run_deploy_franka_simulation.sh` 或其部署依赖后，需同步到 mac_20 并重启 8093；详见
远端项目内的 `deploy.md`。
