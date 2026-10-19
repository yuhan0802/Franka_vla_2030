# 常见问题与排障

本文件集中维护运行、控制和集成中已经定位的问题。每个主题只保留一处：后续发现相同问题时应补充既有条目，而不是新建重复项。

## 使用前检查

| 检查项 | 命令或要求 |
| --- | --- |
| X11 图形授权 | `xhost +local:docker`，并确认 `echo $DISPLAY` 有值 |
| Gazebo 实例 | 同一时刻仅运行一个 `run_gazebo.sh` 或 `run_gazebo_vr.sh` 容器 |
| Quest 连接 | `docker compose run --rm franka_vr adb devices` 应显示 `<Quest-serial> device` |
| 真机网络 | 千兆全双工 FCI 链路、已验证 PREEMPT_RT、急停可达、工作区无人 |

## Gazebo 控制器没有加载

**现象**：FR3 已出现在 Gazebo，但 `joint_state_broadcaster`、`fr3_arm_controller` 或 `fr3_gripper` 没有激活，MoveIt 无法执行。

**原因**：机器人实体出现不等于 Gazebo 的 `ros2_control` 控制器管理器已经就绪，过早加载会产生竞态。

**处理**：启动文件使用 `controller_manager spawner` 并设置 `--controller-manager-timeout 60`，按关节状态、机械臂和夹爪的顺序加载控制器。启动后检查：

```bash
docker exec <Gazebo容器> bash -lc '
  source /ws_franka_vr/install/setup.bash
  ros2 control list_controllers
'
```

预期 `joint_state_broadcaster`、`fr3_arm_controller` 和 `fr3_gripper` 均为 `active`。

## 重复启动控制栈后 RViz 或控制异常

**现象**：多个 Gazebo、fake hardware 或真机 ROS 栈同时存在，出现话题、服务、控制器或图形资源冲突；真机启动时可能看到 `Controller already loaded`，之后 `Failed to configure controller` 或 `Failed to activate controller`。

**原因**：容器使用 host 网络和相同 ROS domain，多个栈会暴露同名 `/controller_manager`。新的 spawner 可能连到旧的 fake 或 Gazebo 控制器管理器，而不是本次真机栈。

**处理补充**：启动真机前先停止所有同工作区的运行中**控制栈**。`run_robot.sh` 现在会检测当前工作区中启动 `franka_twist.launch.py` 或 Gazebo 的容器并拒绝继续，避免把真机 spawner 接到错误的 `/controller_manager`；RealSense、Quest 等不创建控制器管理器的伴随节点不受影响。2026-09-04 已清理一个遗留的 fake-hardware 测试容器；它正是本次控制器“已加载但无法激活”的直接原因。

**原因**：`docker compose run --rm` 每次执行均创建新的临时容器，不会自动停止已有实例。

**处理**：启动前检查并停止旧实例：

```bash
docker ps --filter "name=franka_vr_docker-franka_vr-run"
docker stop <容器ID或容器名>
```

正常情况下只保留一个 Gazebo 栈。`docker stop` 后，`--rm` 容器会自动清理；仅在无法停止时才使用 `docker kill`。

## RViz 末端标记可见但不能拖动

**现象**：末端平移箭头和旋转圆环可见，但拖动无响应、不稳定，或有重叠的多套末端标记。

**原因**：`MotionPlanning` 已管理末端交互标记；若额外添加 `rviz_default_plugins/InteractiveMarkers` 并订阅同一话题 `/rviz_moveit_motion_planning_display/robot_interaction_interactive_marker_topic`，两套显示会同时渲染和拾取同一标记，造成鼠标事件冲突。

**处理**：在 `gazebo_moveit.rviz` 中只保留 `MotionPlanning` 的末端标记，删除额外的 `End Effector Drag Control` 显示。夹爪控件可保留，因为它使用独立话题 `/franka_gripper_control`。同时保持 `Query Start State: false`，并在 `move_group` 和规划场景就绪后启动 RViz。

## Gazebo 中没有抓取方块

**现象**：只显示机器人，无法验证抓取流程。

**处理**：确认 `workspace/src/franka_vr/franka_vr/models/target_block.sdf` 存在，并使用 `./run_gazebo.sh` 启动。该启动流程会生成位于 `(x=0.45, y=0, z=0.04)` 的橙色动态方块，带重力、碰撞和摩擦。

## Gazebo 夹爪不能开合或两指不同步

**现象**：夹爪模型可见，但 RViz 操作无效，或仅一根手指移动。

**原因**：机械臂轨迹控制器不控制夹爪；Gazebo 还需要独立的 `GripperActionController`，并需要将 mimic 手指注册为 `ros2_control` 硬件关节。

**处理**：确认 `fr3_gripper` 已激活；在 RViz 右键蓝色 `Gripper: right-click for open / close` 控件，选择开合。全开目标为每根手指 `0.04 m`。向 `/fr3_gripper/gripper_cmd` 发送该目标后，两根手指的 `/joint_states` 位置应都接近 `0.04`。

## 键盘长按后仍执行旧动作

**现象**：按键自动重复可能持续产生异步目标，服务短暂变慢后，松键仍像在执行历史指令。

**原因**：若每次按键均直接发送异步服务请求，ROS 请求队列会积压。

**处理**：键盘节点只保留一个在途目标请求；新按键合并为最新目标，当前请求完成后再发送。夹爪动作也在上一个结果返回前拒绝重复提交。长按后松开时，机械臂应只追踪最后的合并目标。

## 真机键盘模式无法启动或连接错误容器

**现象**：未启动 Servo 栈、或同时启动了多个真机栈时，键盘模式无法安全判断连接对象。

**处理**：先启动 `run_robot.sh`，再在第二终端运行 `./run_robot_manual_control.sh`。脚本只连接现有的真机 Servo 栈，不会新建硬件控制器。若有多个实例，先停止多余实例，或使用：

```bash
ROBOT_CONTAINER=<容器名或ID> ./run_robot_manual_control.sh
```

## Gazebo 是否需要配置 `ROBOT_IP`

不需要。`run_gazebo.sh` 与 `run_gazebo_vr.sh` 只使用容器中的 Gazebo 模型，不连接 Franka FCI。只有 `run_robot.sh` 需要 `ROBOT_IP`；项目默认示例为 `172.16.0.2`。

## VR 目标与机器人状态不一致或 Quest 断开后仍追踪

**现象**：VR 目标有时和 Gazebo/RViz 末端不一致；数据延迟、限位或 Quest 断开后可能残留旧目标。

**处理状态**：已实施防护，仍需在 Gazebo 与受保护的真机工位完成回归验证。

**当前处理**：

- `SetTargetPose` 使用带坐标系的 `PoseStamped`；键盘和 VR 发送 `world` 坐标系目标，服务端拒绝空、非有限或非 `world` 的普通目标。
- VR 与 Servo 以 `world -> fr3_link8` 的实际 TF 和最新关节反馈为准；预测窗口失效、碰撞停机或无效 Servo 状态后会清空旧预测并重置平滑器。
- Quest 数据带单调时间戳；超过 `0.20 s` 未收到完整样本时，VR 节点只发送一次停止请求并清除夹持状态。

**验证要求**：在 Gazebo 中分别测试键盘、VR 夹持、碰撞停机和 Quest 断连；真机只允许在低速、急停可达、有人监护的工位验证。

## X11、Quest 与真机网络检查

### RViz/Gazebo 无法创建窗口

确认 `echo $DISPLAY` 有输出，并在主机执行：

```bash
xhost +local:docker
```

### Quest 显示 `unauthorized`

回到头显确认 USB 调试授权，然后重新执行：

```bash
docker compose run --rm franka_vr adb devices
```

### 真机网络或实时性不满足要求

```bash
ip -brief address
ip route get 172.16.0.2
ping -c 2 -W 1 172.16.0.2
sudo ethtool <网卡名> | grep -E 'Speed:|Duplex:|Auto-negotiation:|Link detected:'
cat /sys/kernel/realtime
```

`/sys/kernel/realtime` 应输出 `1`。链路不是 1000 Mb/s 全双工、存在丢包，或实时内核未验证时，不应启动真机控制。

本机在 2026-09-04 检测到 `enp110s0` 为 **100 Mb/s、Full Duplex**，可 ping 通 `172.16.0.2`。这低于建议的千兆链路，通常与网线、网卡协商或中间交换设备有关，但本项目此前可在该链路上启动，因此所有新增网络与 PREEMPT_RT 检查均只作为启动前告警，不会改变原有 `run_robot.sh` 的启动条件。实际是否可稳定运行仍以 libfranka 的连接和通信约束结果为准。

**2026-09-04 实测**：真机栈已成功连接机械臂和夹爪，控制器加载完成后 `FrankaHardwareInterface` 在首次实时读取时抛出 `libfranka: Move command aborted: motion aborted by reflex! ["communication_constraints_violation"]`，`ros2_control_node` 随即退出。此时尚未发送任何键盘、VR 或 Z 轴目标，因此不是手动控制命令造成。已尝试清理抢占 `/controller_manager` 的 fake-hardware 栈；仍需在 Desk 清除本次反射后，用 Franka 官方通信诊断确认丢包、时延和抖动，并优先恢复千兆链路与已验证的实时内核后再进行无负载低速运动测试。

**进一步定位 / 已修复的回归**：RealSense 容器当时占用约 15% CPU，而 Compose 的通用 `franka_vr` 服务把相机和真机都限制在 CPU `4,5`。相机与 1 kHz 控制循环竞争同一 CPU 对是本次反射的直接软件侧风险。现已新增专用 `franka_robot` Compose 服务，只有 `run_robot.sh` 使用 CPU `4,5`；相机、Quest 和构建容器不再继承该限制。相机已按新配置重启并确认不再绑定该 CPU 对；真机重试仍在首次实时读取触发相同反射，因此 CPU 竞争不是唯一根因。

**已排除 / 待继续**：启动脚本的 FCI IRQ 绑定已实测成功写入 CPU `2`，两秒后仍保持，且主机没有运行 `irqbalance`。当前主机为 `7.0.0-30-generic` 的 `PREEMPT_DYNAMIC` 内核，未检测到 PREEMPT_RT；结合 100 Mb/s 链路，这仍是通信约束反射的首要待验证环境因素。下一次尝试前需在 Desk 清除刚刚产生的反射，随后先运行 Franka 官方通信诊断（丢包、时延、抖动），而不是直接发送运动目标。

**最终验证（2026-09-04）**：主机实际上已安装 `6.8.1-1015-realtime`，但此前重启后默认进入了 `7.0.0-30-generic`。将 GRUB 默认启动项切换为 `6.8.1-1015-realtime`、重启并确认 `cat /sys/kernel/realtime` 输出 `1` 后，真机栈可稳定完成机械臂、夹爪、控制器和 MoveIt Servo 初始化；无 GUI 测试期间不再出现 `communication_constraints_violation`，且随后已成功执行世界坐标 Z 轴下移约 `10 cm`。

**为什么切换内核有效**：FCI 控制回路以 `1 kHz` 运行。通用内核不能保证控制线程在每个周期及时获得 CPU；一旦连续约 `20 ms` 未向 FCI 发送有效控制包，机器人就会以 `communication_constraints_violation` 触发保护并停止。PREEMPT_RT 提供可预测的调度延迟和实时优先级，使容器内的 `controller_manager` 能持续完成该循环。当前 FCI 仍协商为 `100 Mb/s`，虽在实时内核下通过本次验证，仍应后续排查并恢复千兆链路以保留通信余量。
