# Franka VR Docker

一个基于 ROS 2 Humble、MoveIt Servo 和 Meta Quest 的 Franka FR3 VR 遥操作环境。项目将 Franka ROS 2 工作空间、`libfranka` 运行时和 Quest 控制器读取程序封装在 Docker 中，支持先用假硬件验证，再连接真机运行。

> [!WARNING]
> 这不是安全认证的机器人控制系统。真机运行前，请完成 Franka 官方要求的安装、网络和实时性检查；保持急停可用，并确保机械臂工作区域内无人。首次运行请低速、无负载测试。

## 功能

- Franka FR3 的真机与假硬件启动方式
- MoveIt Servo 在线笛卡尔速度控制与奇异位形降速
- Meta Quest 控制器位姿和按键读取
- 可选 RViz 可视化与 Franka Gripper
- Docker Compose 一键构建和启动

## 环境要求

- Linux 主机、Docker Engine 与 Docker Compose v2
- X11 显示环境（仅在启用 RViz 时需要）
- 真机模式：已按 [Franka Linux 安装指南](https://frankaemika.github.io/docs/installation_linux.html) 配置并验证实时内核、FCI 网络与机器人
- VR 模式：Meta Quest、USB 数据线、已启用开发者模式并允许 USB 调试

## 快速开始

克隆仓库后，按需配置运行参数：

```bash
cp .env.example .env
```

编辑 `.env` 中的 `ROBOT_IP`、`ROS_DOMAIN_ID` 和 `DISPLAY`。首次构建镜像：

```bash
docker compose build
```

## 常用操作命令

以下命令均应在仓库根目录执行。真机操作具有风险；只在完成安全检查、确认急停可用且工作区无人时启动。

| 目的 | 命令 |
| --- | --- |
| 构建或更新镜像 | `docker compose build` |
| 启动 FR3 假硬件与 RViz | `./run_fake.sh` |
| 启动 Gazebo 物理仿真 | `./run_gazebo.sh` |
| 启动可由 Quest 控制的 Gazebo 仿真 | `./run_gazebo_vr.sh` |
| 启动 Quest VR 控制桥接 | `./run_vr.sh` |
| 启动假硬件（不启动 RViz） | `USE_RVIZ=false ./run_fake.sh` |
| 启动假硬件（不加载夹爪） | `LOAD_GRIPPER=false ./run_fake.sh` |
| 启动真机与 RViz | `ROBOT_IP=172.16.0.2 ./run_robot.sh` |
| 启动真机（不启动 RViz） | `ROBOT_IP=172.16.0.2 USE_RVIZ=false ./run_robot.sh` |
| 检查 Quest 的 ADB 连接 | `docker compose run --rm franka_vr adb devices` |
| 进入开发容器 | `docker compose run --rm franka_vr bash` |
| 重新构建 `franka_vr` 包 | `docker compose run --rm franka_vr bash -lc 'cd /ws_franka_vr && colcon build --symlink-install --packages-select franka_vr'` |
| 无缓存完整重建镜像 | `docker compose build --no-cache` |
| 查看 Compose 服务状态 | `docker compose ps` |
| 查看正在运行的全部容器 | `docker ps` |
| 停止 Compose 创建的服务 | `docker compose down` |

默认 FR3 FCI 控制 IP 为 `172.16.0.2`。如设备地址不同，可将该地址写入 `.env`，或在执行命令时以 `ROBOT_IP=<机器人IP>` 覆盖。

### 假硬件模式

这是推荐的首次验证方式。它使用假硬件，不会连接或控制真实机械臂，因此无需设置或连接 `ROBOT_IP`：

```bash
./run_fake.sh
```

默认会启动 RViz 和夹爪。可通过环境变量关闭其中任一项：

```bash
USE_RVIZ=false LOAD_GRIPPER=false ./run_fake.sh
```

### Gazebo 物理仿真

`run_fake.sh` 仅使用 `ros2_control` 假硬件；若需要接触、重力与关节动力学仿真，请使用 Gazebo Sim：

```bash
./run_gazebo.sh
```

该脚本启动带 Franka Hand 夹爪的 FR3、Gazebo Sim、带 MoveIt Motion Planning 面板的 RViz、`joint_state_broadcaster`、MoveIt、机械臂轨迹控制器、夹爪动作控制器，以及位于 `(x=0.45, y=0, z=0.04)` 的橙色抓取长方体。它完全在容器中运行，**不会连接真机，也不需要 `ROBOT_IP`**。首次运行前需要按下面的流程重新构建镜像。

夹爪模型会显示在 Gazebo 和 RViz 中；当前官方关节速度示例控制器不提供夹爪运动控制，因此 Quest 的 `rightGrip` 不会驱动 Gazebo 中的夹爪。

首次配置和启动流程：

```bash
# 1. 允许容器访问本机 X11（仅首次或 X11 权限被撤销后需要）
xhost +local:docker

# 2. 构建带 Gazebo Sim、ROS-GZ 桥接和 Franka Gazebo 插件的镜像
docker compose build franka_vr

# 3. 启动物理仿真；Gazebo 和 RViz 窗口会出现
./run_gazebo.sh
```

终端中出现 `Configured and activated fr3_arm_controller`、`Configured and activated fr3_gripper` 和 `You can start planning now!` 后，即表示 Gazebo 与 MoveIt 都已就绪。使用 `Ctrl+C` 退出仿真。若 GUI 无法创建窗口，可先确认 `echo $DISPLAY` 有值，再重新执行 `xhost +local:docker`；无图形桌面环境暂不支持本项目的 GUI 启动方式。

#### 在 RViz 拖动末端并执行

1. 保持顶部工具栏的 `Interact` 被选中。在底部 `MotionPlanning` 面板中，将 `Planning Group` 设为 `fr3_arm`，并选择 `Planning` 标签页的 `Pose Goal`。
2. 末端会出现青色的六自由度交互标记（平移箭头、平面方块和旋转圆环）。它也在左侧 `Displays > End Effector Drag Control` 中被显式启用；若误关了，可重新勾选该项。
3. 拖动标记调整位置、姿态，随后点击 `Plan & Execute`。若只看到机器人而没有面板，依次点击菜单 `Panels > Add New Panel > Motion Planning` 后选择 `fr3_arm`。

拖动只是在设置目标，Gazebo 不会在拖动过程中实时跟随；`Plan & Execute` 会先做碰撞检查与轨迹规划，再把轨迹发送给 Gazebo。需要连续实时遥操作时，请使用 Quest 的 `./run_gazebo_vr.sh`。

#### 在 RViz 中抓取方块

1. 在 Gazebo 窗口确认橙色长方体已生成。先在 RViz 的 `MotionPlanning` 面板选择 `fr3_arm`，拖动末端标记到方块上方，点击 `Plan & Execute`；再将末端下降到两根手指包住方块的位置并执行。
2. RViz 画面左前方会有一个蓝色方块 `Gripper: right-click for open / close`。右键它，选择 `Close gripper`，即可闭合夹爪；选择 `Open gripper` 即可张开。该交互控件位于 `Displays > Gripper Open Close`，可在这里重新勾选显示。
3. 切回 `fr3_arm`，将末端向上拖动并执行，即可尝试抬起方块。需要释放时，再右键蓝色夹爪控件选择 `Open gripper`。

`fr3_gripper` 使用 Gazebo 的 `GripperActionController`；可控关节为 `fr3_finger_joint1`，另一根手指通过 URDF mimic 关节同步运动。方块具有重力、碰撞和摩擦，抓取成功取决于手指与方块的对准和接触。

如需在另一个终端检查控制器，可将仿真放到后台：

```bash
docker compose run -d --name franka_gazebo franka_vr \
  ros2 launch franka_vr gazebo_vr.launch.py \
  load_gripper:=true use_rviz:=false

docker exec franka_gazebo bash -lc '
  source /ws_franka_vr/install/setup.bash
  ros2 control list_controllers
'

docker stop franka_gazebo
docker rm franka_gazebo
```

期望看到 `joint_state_broadcaster` 和 `fr3_arm_controller` 的状态均为 `active`。

Gazebo 版本使用 Franka ROS 2 工作空间附带的 Gazebo Sim / Ignition Fortress 集成。`run_gazebo.sh` 用于通过 RViz 规划、拖动并执行末端目标；需要 Quest 操控时，使用下面的 `run_gazebo_vr.sh`，不要同时启动两个 Gazebo 栈。

### Quest 控制 Gazebo

需要在 Gazebo 中通过 Quest 操控机械臂时，使用专用的 VR 仿真启动脚本，而不是 `run_gazebo.sh`：

```bash
# 终端 1：Gazebo、FR3 轨迹控制器与 MoveIt Servo
./run_gazebo_vr.sh

# 终端 2：Quest 3 输入桥接
./run_vr.sh
```

等待终端 1 出现 `Configured and activated fr3_arm_controller` 后，再在 Quest 中按住右食指扳机（`rightTrig`）移动机械臂。该模式不连接真机，也不需要 `ROBOT_IP`。它使用 Gazebo 的位置轨迹控制器；夹爪可通过 RViz 中的蓝色交互控件开合。

### 真机模式

先确认 `.env` 中的 `ROBOT_IP` 正确，或在当前终端设置该变量：

```bash
export ROBOT_IP=172.16.0.2
./run_robot.sh
```

`run_robot.sh` 会通过特权容器尝试将 CPU governor 设为 `performance`，并将 FCI 网卡 IRQ 绑到指定 CPU。请根据主机调整：

```bash
FCI_INTERFACE=enp110s0 FCI_IRQ_CPU=2 ./run_robot.sh
```

该脚本会使用 `--privileged`、主机网络和 PID namespace；请只在受信任的本机环境中运行。若主机的 CPU 拓扑不同，还应相应修改 [`docker-compose.yml`](/home/mac/franka_vr_docker/docker-compose.yml) 中的 `cpuset` 设置。

## 启动 VR 控制器

在机械臂启动后，另开一个终端运行：

```bash
./run_vr.sh
```

首次使用时，将 Quest 用 USB 连接到主机，在头显中接受 USB 调试授权。容器会保留 ADB 密钥，以避免每次 `docker compose run --rm` 后重复授权。

控制方式：按住右手扳机（`rightTrig`）控制机械臂运动；按住右手握把（`rightGrip`）闭合夹爪，松开则打开夹爪。

若 RViz 无法连接显示服务器，可在主机执行：

```bash
xhost +local:docker
```

这会放宽本地 Docker 容器的 X11 访问权限；使用完毕后可用 `xhost -local:docker` 撤销。

## Quest 3 遥操作流程

1. 在 Quest 3 中启用开发者模式和 USB 调试；用 USB 连接主机，并在头显中接受“始终允许”调试授权。
2. 启动 FR3 的 Servo 栈（真机使用 `./run_robot.sh`，假硬件使用 `./run_fake.sh`）。
3. 在另一个终端确认 ADB 已识别 Quest：

   ```bash
   docker compose run --rm franka_vr adb devices
   ```

   输出应包含 `<Quest-serial> device`。如显示 `unauthorized`，请回到头显确认 USB 调试授权。

4. 保持 Quest 上的 Oculus Teleop 应用在前台、右手控制器已唤醒，然后启动桥接程序：

   ```bash
   ./run_vr.sh
   ```

   `run_vr.sh` 仅启动 Quest 到 ROS 的输入桥接；在此之前，必须已在另一个终端启动 `./run_fake.sh`、`./run_gazebo_vr.sh` 或 `./run_robot.sh`。

控制器的右食指扳机（`rightTrig`）达到 0.8 后才会进入机械臂控制；按下时会建立相对位姿参考，松开后停止跟随。右手握把（`rightGrip`）大于 0.6 时闭合夹爪，小于 0.4 时执行打开/回零。启动桥接或佩戴头显时请勿扣住扳机。

如需让两个进程在后台运行，可使用以下命令。停止时必须同时停止 Servo 和 Quest 桥接：

```bash
ROBOT_IP=172.16.0.2 docker compose run -d --name franka_vr_servo franka_vr \
  ros2 launch franka_vr franka_twist.launch.py \
  robot_ip:=172.16.0.2 use_fake_hardware:=false load_gripper:=true use_rviz:=false

docker compose run -d --name quest3_bridge franka_vr bash -lc '
  source /ws_franka_vr/install/setup.bash
  cd /ws_franka_vr/src/franka_vr/franka_vr/oculus_reader
  bash ./start_vr.sh
'

docker stop --timeout 2 quest3_bridge franka_vr_servo
```

## MoveIt 轨迹规划（真机）

MoveIt 完整规划栈和 Quest/Servo 栈都包含 `ros2_control` 硬件节点，**不得同时启动**。先停止 VR/Servo 栈，再启动 MoveIt：

```bash
ROBOT_IP=172.16.0.2 docker compose run -d --name franka_moveit franka_vr \
  ros2 launch franka_fr3_moveit_config moveit.launch.py \
  robot_ip:=172.16.0.2 use_fake_hardware:=false load_gripper:=false
```

先只规划、确认终端输出 `PLAN_OK` 且路径安全，再执行。以下示例在 base 坐标系沿 Y 负方向移动 15 cm；它是关节空间规划，不保证中间末端轨迹绝对直线：

```bash
# 仅规划，不运动
ROBOT_IP=172.16.0.2 docker compose run --rm franka_vr \
  ros2 launch franka_vr moveit_offset_client.launch.py \
  robot_ip:=172.16.0.2 dx:=0 dy:=-0.15 dz:=0 execute:=false

# 仅在规划结果安全时执行
ROBOT_IP=172.16.0.2 docker compose run --rm franka_vr \
  ros2 launch franka_vr moveit_offset_client.launch.py \
  robot_ip:=172.16.0.2 dx:=0 dy:=-0.15 dz:=0 execute:=true

docker stop --timeout 2 franka_moveit
```

## 诊断与网络检查

以下 ROS 诊断命令适用于以 `franka_vr_servo` 名称在后台运行的 Servo 容器：

```bash
# 机器人末端 TF、节点、控制器和 Quest 右手 TF
docker exec franka_vr_servo bash -lc '
  source /ws_franka_vr/install/setup.bash
  ros2 run tf2_ros tf2_echo world fr3_link8
'
docker exec franka_vr_servo bash -lc '
  source /ws_franka_vr/install/setup.bash
  ros2 node list
  ros2 control list_controllers
'
docker exec quest3_bridge bash -lc '
  source /ws_franka_vr/install/setup.bash
  ros2 run tf2_ros tf2_echo world oculus_right
'
```

检查 FCI 网络前，将 `<网卡名>` 替换为连接 FR3 控制柜的主机网卡。真机实时控制需要稳定的千兆全双工链路：

```bash
ip -brief address
ip route get 172.16.0.2
ping -c 2 -W 1 172.16.0.2
sudo ethtool <网卡名> | grep -E 'Speed:|Duplex:|Auto-negotiation:|Link detected:'
uname -a
cat /sys/kernel/realtime
```

`/sys/kernel/realtime` 输出 `1` 表示当前内核启用了 PREEMPT_RT。链路速率不足 1000 Mb/s、存在丢包或实时内核未验证时，不应启动真机控制。

## 项目结构

```text
.
├── Dockerfile                 # ROS 2、依赖与工作空间镜像
├── docker-compose.yml         # 运行时权限、显示和设备挂载
├── run_fake.sh                # 假硬件启动脚本
├── run_robot.sh               # 真机启动与实时性辅助设置
├── vendor/                    # libfranka Debian 运行时包
└── workspace/src/
    └── franka_vr/franka_vr/   # 遥操作 ROS 2 包、launch、配置和 Quest reader
```

## 开发

源码目录以可写卷方式挂载到容器。修改 ROS 2 包后，在容器中重新构建：

```bash
docker compose run --rm franka_vr bash
cd /ws_franka_vr
colcon build --symlink-install --packages-select franka_vr
source install/setup.bash
```

`Dockerfile.runtime-fix` 用于在已有 `franka-vr:humble` 镜像上单独重建 `joint_state_broadcaster`：

```bash
docker build -f Dockerfile.runtime-fix -t franka-vr:runtime-fix .
```

## 致谢

本项目建立在 [Franka ROS 2](https://github.com/frankaemika/franka_ros2)、[libfranka](https://github.com/frankaemika/libfranka)、[MoveIt 2](https://moveit.ai/) / MoveIt Servo 与 [Oculus Reader](https://github.com/rail-berkeley/oculus_reader) 之上。请同时遵守这些上游项目各自的许可证和使用条款。

## 许可证

仓库根目录目前尚未声明项目许可证，因此尚不授予本项目自有代码的再分发许可。公开发布前，请添加适合的 `LICENSE` 文件，并同步更新 `workspace/src/franka_vr/franka_vr/package.xml` 中的许可证字段。
