# Codex 任务：FrankaVR 真机 VLA 检测脚本落地与测试

日期：2026-10-08

## 任务目标

请在 Ubuntu 控制计算机的实际项目目录中检查、完善并测试 `detect.sh`，确保它能够在
不发送机器人运动命令的前提下，对 FrankaVR 三视角 VLA 部署进行完整的九项前置检测。

实际项目目录预期为：

```text
/home/mac/franka_vr_docker
```

参考文件：

```text
detect.sh
deploy.md
readme.md
run_vla.sh
run_robot.sh
run_vla_cameras.sh
run_franka_vr_vla_bridge.sh
workspace/src/franka_vr/franka_vr/scripts/franka_vr_vla_bridge.py
workspace/src/franka_vr/franka_vr/config/vla_calibration.yaml
```

## 绝对安全约束

1. 本任务只允许只读检测和模型推理，不得发布任何机器人运动目标。
2. 不得调用或向 `/set_target_pose` 发布消息。
3. 不得运行 `./run_vla.sh execute`。
4. 不得通过键盘、VR、MoveIt、Servo 或其他入口驱动机器人运动。
5. 不得修改或放宽 `workspace_bounds_world`、单步限幅、碰撞保护或奇异性保护。
6. 不得同时启动 Gazebo、fake hardware、mock hardware 或第二个真实控制栈。
7. 不得使用 `docker stop`、`SIGKILL` 或短超时强杀正在运行的真实控制栈。
8. 如果检测到状态不明确、多个控制栈、FCI 异常、相机错位或机器人通信异常，立即停止，
   保留日志并报告，不要尝试运动验证。
9. 机器人运动方向、实测工作区和物理安全状态只能由现场操作者确认，Codex 不得代替
   操作者做虚假确认。

## 必须完成的九项检测

1. Intel RealSense D435i 正视角 RGB 图像正常。
2. 两台 Logitech C922 左、右视角 RGB 图像正常，且左右没有交换。
3. 实体机器人当前关节状态正常，`/joint_states` 包含 `fr3_joint1..fr3_joint7`；
   `world -> fr3_link8` TF 可查询且持续更新。
4. AI 服务器与 Ubuntu 控制计算机之间的模型通信正常：
   - 默认服务为 `10.24.116.29:8093`；
   - 确认远端正确进程监听 8093；
   - 不得把单纯 TCP 可达视为完整模型通信成功。
5. Ubuntu 控制计算机与实体机器人之间的控制通信正常：
   - 机器人路由经过专用 FCI 网口；
   - `controller_manager` 服务可用；
   - 所需控制器处于 `active`；
   - `/set_target_pose` 存在有效订阅者，但本任务不得向其发布消息。
6. 只运行一个真实控制栈：没有 Gazebo、fake/mock hardware、旧真机栈或第二个
   `controller_manager`。
7. 使用专用 FCI 网口：检查接口、路由、链路、机器人 IP 和实时控制相关状态；默认
   `ROBOT_IP=172.16.0.2`，实际值以项目配置为准。
8. 模型输入输出及机器人工作区安全限制正常：
   - 使用三路真实图像、当前 8 维状态和任务文本完成一次只读模型请求；
   - 验证响应包含有限数值的 `float32[15,7]` action；
   - 不执行返回的 action；
   - 校验 `vla_calibration.yaml` 中 `workspace_bounds_world` 存在、每轴 `min < max`；
   - 检查桥接器确实应用工作区边界、平移/旋转单步限幅和 Servo 安全保护；
   - 当前文档记录的批准 X 上限为 `0.75 m`，不得自行扩大。
9. 机器人运动方向、工作空间和安全保护设置正常：
   - Codex 只能检查配置和 `dry-run` 输出；
   - xyz/rpy 方向、相机左右对应和实测边界必须由现场操作者确认；
   - 急停、工作区清空、低速空载、碰撞/奇异性保护也必须由现场操作者确认；
   - 未获得现场确认时，此项必须报告为 `BLOCKED`，不能写成通过。

## 实施步骤

### 1. 先阅读实际代码

进入项目目录后，先检查实际文件，不要假设本任务文档中的话题名和容器名完全正确：

```bash
cd /home/mac/franka_vr_docker
pwd
find . -maxdepth 2 -type f -name '*.sh' -o -name 'docker-compose.yml'
sed -n '1,260p' run_vla.sh
sed -n '1,260p' run_robot.sh
sed -n '1,320p' workspace/src/franka_vr/franka_vr/scripts/franka_vr_vla_bridge.py
sed -n '1,220p' workspace/src/franka_vr/franka_vr/config/vla_calibration.yaml
```

从实际代码确认：

- RealSense、左 C922、右 C922 的准确 ROS 话题；
- `/joint_states`、TF frame 和目标话题；
- 真机容器、控制器和 `controller_manager` 名称；
- FCI 网口和机器人 IP 的来源；
- WebSocket 请求、响应 MessagePack schema；
- action 的 dtype、shape 和字段名；
- 工作区和动作限幅实际由哪个模块执行。

### 2. 检查并完善 `detect.sh`

要求：

- 使用实际项目话题作为默认值；仍允许环境变量覆盖。
- 所有 ROS 查询必须有超时，不能无限等待。
- 任一关键项失败时返回非零退出码。
- 三路图像、关节状态或 TF 失败时，不继续进行模型输入输出检查。
- 不得发布运动目标。
- 输出清楚标识 `PASS`、`FAIL`、`WARN`、`BLOCKED`。
- 自动化无法判断的物理安全项必须要求现场人工确认。

先执行静态检查：

```bash
bash -n ./detect.sh
shellcheck ./detect.sh  # 安装了 shellcheck 时执行
bash ./detect.sh --help
```

### 3. 实现真实模型 I/O 探针

不能只检查 8093 端口。优先复用项目已有 WebSocket/MessagePack 客户端；如果没有合适的
只读入口，新建一个小型检测程序，例如：

```text
scripts/detect_vla_model_io.py
```

该程序必须：

1. 从实际 ROS 话题读取三张最新 RGB 图像和当前 8 维状态。
2. 按服务端实际 schema 发送一条 WebSocket 请求到 8093。
3. 设置连接、发送和接收超时。
4. 验证服务返回的模型标识符合 FrankaVR 模型，而不是旧 LIBERO/8092 服务。
5. 验证 action：

```text
dtype = float32
shape = [15, 7]
所有元素均为有限数值
```

6. 打印统计信息即可，例如 shape、dtype、min/max、首行；不得执行 action。
7. 成功返回退出码 0；schema、shape、dtype、NaN/Inf、超时或模型标识错误均返回非零。

然后让 `detect.sh` 自动调用该探针，或通过明确的 `MODEL_IO_CHECK_CMD` 调用。最终日常命令
不应要求用户手工拼接一长串临时命令。

### 4. 检查当前运行环境

只读检查示例：

```bash
./run_vla.sh status
docker ps
ros2 node list
ros2 topic list
ros2 service list
ros2 control list_controllers
ip route get "$ROBOT_IP"
ethtool "$FCI_INTERFACE"
```

如果真机状态栈和三路相机尚未启动，只能使用项目已有的安全入口启动状态/相机部分。
不得为了完成检测而启动 `execute`，不得启动 Gazebo 或第二个真机栈。

### 5. 运行检测

确认急停可达、现场操作者在场、工作区无人且已清空后运行：

```bash
cd /home/mac/franka_vr_docker
FCI_INTERFACE=<实际专用FCI网卡> \
ROBOT_IP=<实际机器人IP> \
bash ./detect.sh 2>&1 | tee "detect_$(date +%Y%m%d_%H%M%S).log"
```

在没有现场操作者确认第 9 项时，允许前 8 项完成，但总结果必须是 `BLOCKED` 或失败，
不得自动设置 `CONFIRM_DIRECTION=true`、`CONFIRM_WORKSPACE=true`、
`CONFIRM_SAFETY=true` 来绕过门禁。

## 验收标准

只有满足以下全部条件，脚本才能返回 0：

- 三路相机均收到新鲜、非空的 RGB 消息。
- 左右相机话题映射已由现场操作者确认。
- 7 个 FR3 关节状态和 `world -> fr3_link8` TF 正常。
- 8093 是正确的 FrankaVR 模型服务。
- 完成一次真实只读模型推理，返回有限的 `float32[15,7]`。
- 专用 FCI 网口、机器人路由和控制器状态正常。
- 只存在一个真实控制栈。
- 工作区配置有效且没有放宽未知区域。
- 现场操作者已确认方向、实测工作区、急停和安全保护。

任何关键条件不满足都必须返回非零，且不得进入 `./run_vla.sh execute`。

## 需要保留的产物

请保留并说明：

```text
detect.sh
scripts/detect_vla_model_io.py       # 如果新增
deploy.md                            # 如有更新
readme.md                            # 如有更新
detect_YYYYMMDD_HHMMSS.log           # 实际检测日志
detect_report_20261008.md            # 最终报告
```

最终报告至少包含：

| 项目 | 结果 | 证据 | 备注 |
| --- | --- | --- | --- |
| D435i 正视角 | PASS/FAIL | 话题、时间戳、编码、尺寸 |  |
| C922 左视角 | PASS/FAIL | 话题、时间戳、编码、尺寸 |  |
| C922 右视角 | PASS/FAIL | 话题、时间戳、编码、尺寸 |  |
| 关节状态与 TF | PASS/FAIL | 关节名、TF 时间戳 |  |
| AI 模型通信 | PASS/FAIL | 主机、端口、模型标识 |  |
| 模型输入输出 | PASS/FAIL | dtype、shape、有限值检查 | 不执行 action |
| 控制通信 | PASS/FAIL | FCI 路由、controller 状态 |  |
| 单一控制栈 | PASS/FAIL | ROS 节点和容器 |  |
| 工作区安全限制 | PASS/FAIL | 配置路径、min/max |  |
| 运动方向与现场安全 | PASS/FAIL/BLOCKED | 操作者确认 |  |

同时列出：

- 修改过的文件及修改原因；
- 实际使用的相机话题、机器人 IP、FCI 网口、模型地址；
- 所有失败、警告和未完成项；
- 是否允许进入下一步 `dry-run`；
- 明确写出本任务未执行任何机器人动作。

## 完成后的回复要求

不要只回复“测试通过”。请给出：

1. 九项检测逐项结果。
2. 实际命令与日志路径。
3. 模型响应的 dtype、shape 和有限值结果。
4. 工作区边界数值及配置来源。
5. 是否发现冲突控制栈。
6. 是否有现场操作者完成第 9 项确认。
7. 是否建议进入下一步 `dry-run`，以及仍然存在的风险。

