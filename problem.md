# 已解决问题

本文仅记录已验证解决的问题与处理方法；尚未解决或尚未稳定复现的问题不在此列。

## Gazebo 启动后控制器加载失败

**现象**：FR3 已出现在 Gazebo 中，但 `joint_state_broadcaster`、`fr3_arm_controller` 或 `fr3_gripper` 偶发加载失败，导致 MoveIt 无法执行轨迹。

**原因**：机器人实体生成完成并不代表 Gazebo 的 `ros2_control` 控制器管理器已经可用；启动脚本过早调用加载命令产生竞态。

**解决办法**：在 `gazebo_vr.launch.py` 中使用 `controller_manager spawner`，并设置 `--controller-manager-timeout 60`。它会等待 `/controller_manager` 服务就绪后，依次加载并激活关节状态、机械臂轨迹和夹爪控制器。

**验证方式**：

```bash
docker exec <Gazebo容器> bash -lc '
  source /ws_franka_vr/install/setup.bash
  ros2 control list_controllers
'
```

预期 `joint_state_broadcaster`、`fr3_arm_controller` 和 `fr3_gripper` 均为 `active`。

## Gazebo 中没有抓取方块

**现象**：Gazebo 只显示机器人，无法进行抓取流程验证。

**解决办法**：增加 `models/target_block.sdf`，由 Gazebo 启动文件生成位于 `(0.45, 0, 0.04)` 的橙色动态方块。方块已启用重力、碰撞和摩擦。

**使用方式**：启动 `./run_gazebo.sh` 后，在 Gazebo 中确认橙色方块出现，再使用 MoveIt 规划机械臂靠近。

## Gazebo 仿真中的夹爪无法开合

**现象**：夹爪模型可见，但不能通过 RViz 发出开合指令。

**原因**：机械臂轨迹控制器不负责夹爪关节，且 Gazebo 需要独立的 `GripperActionController`。

**解决办法**：

- 配置并启动 `fr3_gripper` 的 `GripperActionController`。
- 增加 RViz 交互式蓝色方块 `Gripper: right-click for open / close`。
- 右键该方块，选择 `Open gripper` 或 `Close gripper`，节点会向 `/fr3_gripper/gripper_cmd` 发送动作目标。

另一根手指通过 URDF mimic 关节同步运动。

## 多次启动 Gazebo 导致控制异常

**现象**：重复执行 `./run_gazebo.sh` 后，可能同时出现多个 Gazebo、RViz 或 MoveIt 实例，造成话题、服务、控制器和图形资源冲突。

**原因**：`docker compose run --rm` 每次调用都会创建新的临时容器，不会自动停止已有实例。

**解决办法**：启动前检查并停止旧实例：

```bash
docker ps --filter "name=franka_vr_docker-franka_vr-run"
docker stop <容器ID或容器名>
```

正常情况下，只保留一个 `run_gazebo.sh` 或 `run_gazebo_vr.sh` 实例运行。

## Gazebo 仿真误以为需要连接真机

**现象**：启动仿真前尝试配置或连接 `ROBOT_IP`。

**解决办法**：`./run_gazebo.sh` 和 `./run_gazebo_vr.sh` 使用容器内的 Gazebo 模型，不连接 Franka FCI，也不需要真实机械臂或 `ROBOT_IP`。仅 `./run_robot.sh` 使用默认地址 `172.16.0.2` 连接真机。
