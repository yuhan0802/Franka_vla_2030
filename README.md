# Franka VR Docker

面向 Franka FR3 的 Docker 化 ROS 2 控制与仿真环境。项目集成 ROS 2 Humble、MoveIt 2、Gazebo Sim 和 Meta Quest 3，可用于 Gazebo 轨迹规划、键盘/VR 遥操作以及受控真机连接。

> [!WARNING]
> 本项目不是安全认证的机器人控制系统。真机运行前，必须完成 Franka 官方要求的安装、网络和实时性检查；保持急停可用，并确保工作区无人。首次测试应低速、无负载进行。

## 功能

- Gazebo Sim：FR3、Franka Hand、重力、碰撞、摩擦与可抓取方块
- MoveIt 2：在 RViz 设置末端目标、规划并执行轨迹
- MoveIt Servo：键盘与 Quest 的连续笛卡尔目标跟踪
- RViz 夹爪开合交互控件
- Meta Quest 3：通过 Oculus Reader 读取右手控制器位姿与按键
- 真机模式：连接 Franka FCI，并保留容器化的 ROS 2 工作空间

## 环境要求

- Linux 主机、Docker Engine、Docker Compose v2
- X11 图形桌面（使用 Gazebo 或 RViz 时需要）
- VR 模式：Meta Quest 3、USB 数据线、开发者模式与 USB 调试授权
- 真机模式：已按 [Franka Linux 安装指南](https://frankaemika.github.io/docs/installation_linux.html) 验证实时内核、FCI 网络和机器人状态

## 快速开始（Gazebo）

```bash
git clone https://github.com/221zhong/franka_vr_control_2026.git
cd franka_vr_control_2026
cp .env.example .env

# 仅首次或 X11 授权失效后执行
xhost +local:docker

docker compose build
./run_gazebo.sh
```

Gazebo 和 RViz 打开、终端出现 `You can start planning now!` 后即可操作。结束时在启动终端按 `Ctrl+C`；若启动终端已关闭，可执行 `docker stop <容器ID或容器名>`。

## 运行模式

| 场景 | 启动命令 | 用途 |
| --- | --- | --- |
| Gazebo 规划仿真 | `./run_gazebo.sh` | RViz 设置目标，经过 `Plan & Execute` 后在 Gazebo 执行 |
| Gazebo VR 遥操作 | `./run_gazebo_vr.sh` | 启动 Gazebo、MoveIt Servo 和 VR 所需服务 |
| Quest 输入桥接 | `./run_vr.sh` | 连接已运行的 Gazebo VR 栈或真机 Servo 栈 |
| Gazebo 键盘控制 | `./run_manual_control.sh` | 连接已运行的 Gazebo 栈；不新建仿真容器 |
| 真机控制 | `ROBOT_IP=<机器人IP> ./run_robot.sh` | 连接 FCI 并启动真机控制栈 |
| 真机键盘控制 | `./run_robot_manual_control.sh` | 在第二终端连接已启动的真机 Servo 栈 |

`run_gazebo.sh` 与 `run_gazebo_vr.sh` 不连接真机，也不需要 `ROBOT_IP`。每次运行都会创建一个临时容器；同一时刻只应保留一个 Gazebo 栈。详细排障见 [question.md](question.md)。

所有启动脚本都会重新构建当前挂载的 `franka_vr` 源码；修改 ROS 包通常无需重建镜像。仅 Dockerfile、系统依赖或上游源码变更后需要重新执行 `docker compose build`。

## 操作方式

### RViz 轨迹规划

1. 在 `MotionPlanning` 面板中将 `Planning Group` 设为 `fr3_arm`，选择 `Planning` 标签页的 `Pose Goal`。
2. 保持 RViz 顶部的 `Interact` 工具，拖动末端六自由度交互标记设置目标。
3. 点击 `Plan & Execute`。目标设置不会立即改变 Gazebo 中的机械臂；只有规划并执行后才会运动。

如未显示面板，选择 `Panels > Add New Panel > Motion Planning`。Gazebo 中的蓝色 `Gripper: right-click for open / close` 控件可右键开合夹爪。

### 键盘手动控制

先启动 Gazebo 或真机栈，再在第二终端启动对应键盘入口。键盘通过 MoveIt Servo 发送世界坐标系目标，保留碰撞、奇异位形和关节边界的安全限制；它不直接写入底层关节命令。

| 按键 | 操作 |
| --- | --- |
| `W` / `S` | 前 / 后（+X / -X） |
| `A` / `D` | 左 / 右（+Y / -Y） |
| `R` / `F` | 上 / 下（+Z / -Z） |
| `↑` / `↓` | 绕 X 轴正 / 负旋转 |
| `→` / `←` | 绕 Y 轴正 / 负旋转 |
| `Z` / `X` | 绕 Z 轴正 / 负旋转 |
| `Space` | 切换夹爪开 / 合 |
| `Q` 或 `Ctrl-C` | 退出 |

真机键盘模式只在急停可达、工作区无人、低速无负载的条件下使用。若自动识别到多个真机容器，可用 `ROBOT_CONTAINER=<容器名或ID> ./run_robot_manual_control.sh` 明确指定。

### Quest 3 遥操作

```bash
# 终端 1：仿真 Servo 栈
./run_gazebo_vr.sh

# 终端 2：确认 Quest 已授权后启动输入桥接
docker compose run --rm franka_vr adb devices
./run_vr.sh
```

右食指扳机 `rightTrig` 达到阈值后进入控制，松开停止跟随；右手握把 `rightGrip` 控制夹爪。启动桥接或佩戴头显时不要扣住扳机。真机 VR 控制请先执行 `ROBOT_IP=<机器人IP> ./run_robot.sh`，再启动 `run_vr.sh`。

## 真机操作

```bash
ROBOT_IP=172.16.0.2 ./run_robot.sh
```

`run_robot.sh` 会使用特权容器、主机网络和 PID namespace，并尝试配置 CPU governor 与 FCI 网卡 IRQ。请按实际主机调整：

```bash
FCI_INTERFACE=<网卡名> FCI_IRQ_CPU=<CPU编号> ./run_robot.sh
```

MoveIt 规划栈和 Quest/Servo 栈都会创建硬件节点，不能同时启动。真机的网络与实时性检查、常见问题及安全边界在 [question.md](question.md) 中维护。

## 模块说明

| 模块 | 位置 | 职责 |
| --- | --- | --- |
| 容器与运行入口 | `Dockerfile`、`docker-compose.yml`、`run_*.sh` | 构建镜像、X11、设备/网络挂载和模式启动 |
| FR3 控制包 | `workspace/src/franka_vr/franka_vr` | 启动编排、Servo、RViz 交互、键盘和 Quest 节点 |
| Gazebo 集成 | `workspace/src/franka_ros2/franka_gazebo` | Gazebo Sim、`ros2_control` 与控制器 |
| 机器人描述 | `workspace/src/franka_description` | FR3 URDF、网格、夹爪与关节定义 |
| 运动规划 | `workspace/src/moveit2`、`workspace/src/franka_vr/franka_vr/config` | MoveIt 2、规划参数、控制器映射和 RViz 配置 |
| Quest 读取器 | `workspace/src/franka_vr/franka_vr/oculus_reader` | ADB/logcat 控制器数据读取与 ROS 桥接 |
| 上游运行时 | `vendor/libfranka` | Franka C++ 库及运行时构建资源 |

## 开发

源码以可写卷挂载到容器。修改 `franka_vr` 包后可单独构建：

```bash
docker compose run --rm franka_vr bash -lc '
  cd /ws_franka_vr &&
  colcon build --symlink-install --packages-select franka_vr
'
```

提交前请至少确认对应启动模式可以构建，且不提交本地 `.env`、容器日志或构建产物。欢迎通过 Issue 报告可复现的问题，并通过 Pull Request 提交独立、说明清晰的改动。

## 文档

- [常见问题与排障](question.md)
- [.env 配置示例](.env.example)
- [MIT 许可证](LICENSE)

## 致谢与许可证

本项目建立在 [Franka ROS 2](https://github.com/frankaemika/franka_ros2)、[libfranka](https://github.com/frankaemika/libfranka)、[MoveIt 2](https://moveit.ai/) / MoveIt Servo 和 [Oculus Reader](https://github.com/rail-berkeley/oculus_reader) 之上。自有代码采用 [MIT License](LICENSE)；仓库中的上游组件仍分别遵循其原有许可证和使用条款。
