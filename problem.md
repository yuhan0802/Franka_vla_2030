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

## 重复启动 Gazebo 后 RViz 或控制异常

**现象**：多个 Gazebo、RViz 或 MoveIt 实例同时存在，出现话题、服务、控制器或图形资源冲突。

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
