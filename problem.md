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
- 将夹爪全开目标设为每根手指 `0.04 m`，并收紧控制器目标容差，避免控制器在半开位置过早报告完成。
- 将 `fr3_finger_joint2` 显式注册为 Gazebo `ros2_control` 的 mimic 硬件关节。仅在 URDF 中声明 mimic 不足以让 Gazebo 插件驱动物理上的另一根手指；注册后两根手指会随 `fr3_finger_joint1` 同步运动。

**验证方式**：向 `/fr3_gripper/gripper_cmd` 发送 `position: 0.04` 后，动作应成功完成；`/joint_states` 中主手指和 mimic 手指的位置都应接近 `0.04`，Gazebo 中左右手指同步展开。

## 键盘手动操作在长按时积压旧指令

**现象**：终端按键自动重复时，手动控制可能持续创建异步目标请求。若服务短暂变慢，松开按键后旧目标仍可能排队执行，表现为后续指令不及时响应。

**原因**：每次按键都直接调用异步 `/set_target_pose` 服务，缺少在途请求数量限制和最新目标合并。

**解决办法**：键盘节点最多保留一个在途目标请求；长按产生的新目标只更新本地最新目标，待当前请求完成后再发送。夹爪也限制为上一个动作完成前不重复提交，并使用非阻塞的 action-server 就绪检查。

**验证方式**：长按一个方向后松开，机械臂只会追踪最后一个已合并目标，不会继续消耗历史按键队列；连续按夹爪键不会产生多个并发动作。

## 真机缺少键盘手动模式入口

**现象**：`run_robot.sh` 已启动真机 MoveIt Servo 栈，但原有 `run_manual_control.sh` 只识别 Gazebo 容器，并以仿真时间运行，无法安全地复用到真机。

**解决办法**：新增 `run_robot_manual_control.sh`。它只连接已经由 `run_robot.sh` 启动的容器，以真实时间启动 `keyboard_manual_control.py`；脚本会自动检查恰好一个真机 Servo 栈，发现零个或多个实例时拒绝执行。可用 `ROBOT_CONTAINER=<容器名或ID>` 显式指定目标。

**验证方式**：未启动真机栈时，脚本应输出明确错误且不创建新控制器；启动 `run_robot.sh` 后，在第二终端执行该脚本即可连接已有的 `/set_target_pose` 与夹爪 action 接口。

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

## RViz 末端交互标记显示但无法正常鼠标拖动

**现象**：末端的平移箭头和旋转圆环在 RViz 中可见，但鼠标拖动无响应、响应不稳定，或出现多套重叠的末端控制标记。

**原因**：`MotionPlanning` 插件本身已经管理并显示末端交互标记；配置中又额外添加了一个 `rviz_default_plugins/InteractiveMarkers` 显示，并订阅同一话题：

```text
/rviz_moveit_motion_planning_display/robot_interaction_interactive_marker_topic
```

两套交互显示同时渲染和拾取同一个末端标记，造成鼠标事件与反馈冲突。

**解决办法**：在 `gazebo_moveit.rviz` 中删除额外的 `End Effector Drag Control` 显示，只保留 `MotionPlanning` 管理的末端交互标记。夹爪控制保留为独立的 `Gripper Open Close` 交互标记，因为它使用不同的话题 `/franka_gripper_control`。

同时保持 `Query Start State: false`，避免额外显示并操作 Start State 标记；RViz 在 `move_group` 启动并等待规划场景就绪后再启动，确保交互标记服务已可用。
