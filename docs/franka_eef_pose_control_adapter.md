# Franka 真机 EE Pose 控制端适配说明

> **修订 2026-10-23：Task0 动作的旋转是工具坐标系增量，执行用右乘。**
>
> 原文按 `lerobot_episode_recorder.py` 的注释 `q_delta = q_current * inverse(q_previous)`，把旋转写成
> 基坐标系增量、左乘执行。但录制器代码 `target_delta()` 实际计算的是
> `q_delta = inverse(q_previous) * q_current`，即 **fr3_link8 工具坐标系**下的增量，注释写反了
> （已在代码中改正）。用 Task0 录制的 VR 目标位姿逐帧验证（3 条示范、约 2000 帧）：
>
> | 复现方式 | 单帧误差 | 15 步动作块积分误差 |
> |---|---|---|
> | 右乘 `R_target = R_current × R_delta`（工具坐标系） | ≤ 3e-7 rad | 最大 0.007 rad |
> | 左乘 `R_target = R_delta × R_current`（基坐标系） | 0.023–0.038 rad | 最大 0.75 rad |
>
> 因此本文中凡涉及 **Task0 数据**的旋转，均已改为工具坐标系 / 右乘；**平移**仍是基坐标系，不变。
> DROID 原始数据的 `cartesian_velocity` 是基坐标系命令，DROID 预训练权重仍按左乘执行。
>
> 对训练的影响：Task0 权重由 DROID 权重微调而来，两者共用 embodiment 2 的动作表示，但旋转三维的含义
> 不同（夹爪朝下时工具 z 轴约等于基座 −z，同一 yaw 数值方向相反）。重新训练时建议先把 Task0 旋转标签
> 转换为基坐标系增量 `q_current × inverse(q_previous)`，与 DROID 预训练一致，部署端改为左乘。
> 修订中删除或改写的原文均以「原文：」注明。

## 1. 目标

为当前从 DROID BAR checkpoint 微调的 Franka Task0 模型增加一条匹配训练协议的真机控制链路。

新模型协议：

- 输入状态：EEF 位置 `XYZ(3)`、绝对姿态 rotation vector `rotvec(3)`、夹爪开合 `open(1)`；
- 输出动作：15 Hz、`15 × 7` action chunk；
- 动作前 3 维：基坐标系下的 Cartesian 平移命令；
- 动作中间 3 维：**fr3_link8 工具坐标系**下的一步相对旋转命令（原文：基坐标系，见文首修订）；
- 动作最后 1 维：夹爪，`0=闭合，1=打开`；
- ActionCodec：embodiment 2，`c_franka_droid_15hz`；
- 模型输出顺序：future state，然后 action；
- Action 预测模式：BAR-16，48 个 ActionCodec token。

## 2. 当前仓库中的两类控制路径

仓库中存在两种容易混淆的真机执行方式。

### 2.1 PI0.5 direct 路径

该路径把模型动作转换为 7 个关节速度，并通过 JointJog/direct velocity executor 执行，属于 joint velocity 控制。

这条路径不能直接执行本次模型输出的 EEF pose action。

### 2.2 Qwen ActionCodec Franka bridge

`franka_vr_vla_bridge.py` 已经采用 Cartesian pose target：

1. 读取当前 `world -> fr3_link8` 位姿；
2. 将模型动作转换为位置与旋转增量；
3. 生成新的 EEF pose target；
4. 将目标发送给 Cartesian/MoveIt Servo 控制接口。

旧版本模型的 observation 使用 joint state，但 action 执行本身已经是 EEF target。因此，新控制端适合在这条路径上修改，不需要从零实现 Cartesian 控制器。

## 3. 训练时使用的精确语义

### 3.1 State

状态为 7 维：

```text
state = [x, y, z, rotvec_x, rotvec_y, rotvec_z, gripper_open]
```

其中：

- `x,y,z` 来自 `world -> fr3_link8`；
- quaternion 使用 `xyzw` 顺序；
- quaternion 转换为绝对姿态 rotation vector；
- `gripper_open=0` 表示闭合；
- `gripper_open=1` 表示打开；
- Task0 训练中 `gripper_open` 由实测总宽度二值化：`width >= 0.045 m`（0.01 与 0.08 的中点）为 1。

转换示例：

```python
position = np.asarray([tx, ty, tz], dtype=np.float32)
quat_xyzw = np.asarray([qx, qy, qz, qw], dtype=np.float64)
quat_xyzw /= np.linalg.norm(quat_xyzw)
rotvec = Rotation.from_quat(quat_xyzw).as_rotvec().astype(np.float32)
state = np.concatenate([position, rotvec, [gripper_open]])
```

### 3.2 State 归一化

必须使用 DROID EEF 预训练时的同一份统计量：

```text
droid_qwen3vl_actioncodec/droid_eef_axis_angle_state_stats.json
```

计算方式：

```python
normalized_state = 2 * (state - q01) / (q99 - q01) - 1
normalized_state = np.clip(normalized_state, -1, 1)
```

不能使用真机 Task0 自身统计量替换该文件，否则物理维度虽然相同，但进入已训练 state encoder 的数值坐标系会发生变化。

### 3.3 Action

原始采集数据为 30 Hz，模型动作频率为 15 Hz。训练数据将两个相邻的 30 Hz EEF target 增量合成为一个 15 Hz 动作。

平移：

```python
delta_position_15hz = delta_position_30hz_0 + delta_position_30hz_1
linear_command = delta_position_15hz / 0.075
```

旋转必须通过 SO(3) 合成，不应直接把两个较大的 rotation vector 当普通向量相加：

```python
r0 = Rotation.from_rotvec(delta_rotvec_30hz_0)
r1 = Rotation.from_rotvec(delta_rotvec_30hz_1)
delta_rotation_15hz = r0 * r1          # 工具坐标系增量：先 r0 后 r1，右乘合成
angular_command = delta_rotation_15hz.as_rotvec() / 0.15
```

原文为 `r1 * r0`（基坐标系的合成顺序），现有训练代码
`franka_task0_qwen_actioncodec_droid_pretrained_bar.py:1283` 也是这样实现的。对工具坐标系增量，
两种顺序只差一个二阶小量（每步 < 1.5e-4 rad），已训练的权重不受实际影响，新代码应使用 `r0 * r1`。

录制数据中另有两点需要注意：

- 30 Hz 标签 `eef_delta_*_command` 是限幅后的值（平移 ≤ 3 mm/帧、旋转 ≤ 0.0175 rad/帧），
  未限幅的值在 `eef_delta_*_command_requested`；4–17% 的帧超过限幅；
- 每条示范开头约 40–47 帧还没有 VR 目标位姿（录为原点的单位位姿），这段标签为 0，目标第一次出现的
  那一帧会生成一条错误的限幅步。

夹爪：

```text
目标宽度 0.01 m → 0（闭合）
目标宽度 0.08 m → 1（打开）
```

最终动作：

```text
action = [linear_command(3), angular_command(3), gripper_open(1)]
```

注意：state 中的 rotvec 是绝对姿态；action 中的三维旋转是一步相对旋转/角速度命令。两者都使用 rotation-vector 数学表示，但物理含义不同。

## 4. 模型服务端需要修改的内容

### 4.1 Observation 接口

旧接口中的：

```text
joint_position(7) + gripper(1)
```

应替换为：

```text
eef_position(3) + absolute_orientation_rotvec(3) + gripper_open(1)
```

建议 wire protocol 明确命名，避免继续使用 `joint_state`：

```json
{
  "eef_state": [0, 0, 0, 0, 0, 0, 1],
  "exterior_image": "...",
  "wrist_image": "...",
  "instruction": "..."
}
```

服务端应检查：

- state shape 必须为 `(7,)`；
- 所有数值有限；
- quaternion 转 rotvec 前范数有效；
- gripper 只允许 `[0,1]`；
- state 使用 DROID EEF q01/q99 归一化。

### 4.2 图像与 prompt

训练使用两个视角：

1. 一个外部相机；
2. wrist/front 相机。

部署端必须保持相同的图像数量、顺序和 prompt 结构。不要继续把旧版三个相机图像发送给新模型。

建议固定为：

```text
Main camera: exterior
Wrist camera: front/wrist
```

如果训练时不同轨迹交替使用 left/right 外部相机，部署时应选择与实际标定和任务视角最匹配的一路；不能在一个 action chunk 内切换视角。

### 4.3 ActionCodec 解码

服务端必须采用：

```text
ActionCodec embodiment_id = 2
chunk shape = 15 × 7
token count = 48
```

解码顺序：

1. 生成 48 个 ActionCodec token；
2. 使用 ActionCodec embodiment 2 解码；
3. 前 6 维乘回 DROID action q01/q99 scale；
4. 夹爪由 `[-1,1]` 映射回 `[0,1]`。

伪代码：

```python
decoded = actioncodec.decode(tokens, embodiment_id=2)  # 15×7
scale = np.maximum(np.abs(DROID_Q01), np.abs(DROID_Q99))
decoded[:, :6] *= scale[:6]
decoded[:, 6] = (decoded[:, 6] + 1.0) / 2.0
```

不能把 state 统计量应用到 action，也不能重复执行 ActionCodec 反归一化。

服务端 metadata 建议明确记录：

```json
{
  "state_protocol": "droid_eef_axis_angle6_gripper_open1_singleturn_v1",
  "action_representation": "droid_base_cartesian_velocity_plus_inverted_gripper_v1",
  "action_shape": [15, 7],
  "action_hz": 15,
  "gripper_convention": "0_closed_1_open",
  "rotation_execution": "relative_rotvec_right_multiply"
}
```

`rotation_execution` 取值：Task0 数据训练的权重为 `relative_rotvec_right_multiply`（工具坐标系）；
DROID 预训练权重为 `relative_rotvec_left_multiply`（基坐标系）。原文对 Task0 写的是 left_multiply。

## 5. 真机 bridge 需要修改的内容

### 5.1 动作尺度恢复

ActionCodec 反归一化后的前 6 维仍是 DROID controller command，需要恢复为单步物理增量：

```python
delta_position = action[:3] * 0.075       # metres
delta_rotvec = action[3:6] * 0.15         # radians
```

随后再应用安全限幅。训练和部署的 `0.075/0.15` 必须一致。

### 5.2 旋转执行必须使用 rotvec

旧代码若使用下面的方式，需要修改：

```python
quaternion_from_euler(*delta_rotation)
```

正确方式：

```python
current_rotation = Rotation.from_quat(current_quat_xyzw)
delta_rotation = Rotation.from_rotvec(delta_rotvec)
target_rotation = current_rotation * delta_rotation
target_quat_xyzw = target_rotation.as_quat()
```

Task0 采用右乘：

```text
R_target = R_current × R_delta
```

该顺序与采集器代码实际的相对旋转定义一致：

```text
R_delta = inverse(R_previous) × R_current
```

原文为左乘 `R_target = R_delta × R_current`、`R_delta = R_current × inverse(R_previous)`，
依据的是采集器中写反的注释。DROID 预训练权重仍用左乘。

### 5.3 坐标系

必须确认下列坐标系实际一致：

- 采集代码中的 `world`；
- 部署 bridge 查询的 `world`；
- Franka base frame；
- 模型 action 所代表的 base frame。

本协议中的 translation action 是 base/world frame 命令；Task0 的 rotation action 是 tool-frame
（fr3_link8）命令。原文写的是两者都是 base/world frame。

测试 `+x` 时，夹爪无论朝向如何，都应沿机器人基座 X 轴移动。如果平移方向随夹爪朝向旋转，说明平移被错误地当作 tool-frame action。

测试 `+rot_z` 时，夹爪应绕自身 z 轴（fr3_link8 z 轴，夹爪朝下时约为基座 −z）转动；如果绕基座 +z 转动，说明旋转被错误地当作 base-frame action。

### 5.4 累计目标

推荐一个 action chunk 内维护累计目标：

```python
target_position = measured_position_at_chunk_start
target_rotation = measured_rotation_at_chunk_start

for action in chunk:
    target_position += action[:3] * 0.075
    target_rotation = target_rotation * Rotation.from_rotvec(action[3:6] * 0.15)   # Task0：右乘
    send_pose_target(target_position, target_rotation.as_quat())
```

不推荐每一步都重新以带跟踪滞后的实测位姿为起点，否则控制器尚未到达上一目标时，误差会被带入下一步并造成轨迹缩短或抖动。

每个新 chunk 开始时，可以重新使用当前实测位姿初始化累计目标，限制长期漂移。

### 5.5 控制频率

完整 action chunk 为：

```text
15 steps / 15 Hz = 1 second
```

控制端应每 `1/15 s` 发送一步。不能按采集时的 30 Hz 执行模型输出，否则动作块会在 0.5 秒内执行完，速度与轨迹均不匹配。

第一版建议采用串行 chunk：

1. 采集一次同步 observation；
2. 等待模型输出；
3. 完整执行 15 步；
4. 执行完毕后采集下一次 observation。

后续如改成 receding horizon，需要单独评估观测延迟、chunk 替换和安全停止行为。

### 5.6 夹爪

模型约定固定为：

```text
0 = closed
1 = open
```

建议使用迟滞，避免解码值在阈值附近反复触发：

```python
if gripper >= 0.6:
    request_open()
elif gripper <= 0.4:
    request_close()
else:
    keep_previous_gripper_state()
```

物理目标建议与训练数据一致：

```text
open target   = 0.08 m
closed target = 0.01 m
```

如果闭合使用 Franka `Grasp` action，还需要注意：

- 不要让等待 Grasp result 阻塞机械臂 Cartesian action timer；
- 同一状态不要重复发送目标；
- 使用 measured width 确认开合结果；
- 设置合理 force、speed、epsilon；
- 夹爪失败时决定是停止整个 chunk，还是记录告警继续机械臂动作。

## 6. 安全层

上线时至少保留以下检查。

### 6.1 单步动作限制

在乘回物理尺度后限制：

```python
if np.linalg.norm(delta_position) > max_translation:
    delta_position *= max_translation / np.linalg.norm(delta_position)

if np.linalg.norm(delta_rotvec) > max_rotation:
    delta_rotvec *= max_rotation / np.linalg.norm(delta_rotvec)
```

建议首次真机测试采用比训练上限更保守的限制，并逐步放开。

### 6.2 Workspace

每一步发送前检查：

```text
x_min <= target_x <= x_max
y_min <= target_y <= y_max
z_min <= target_z <= z_max
```

任何目标越界时，不应只裁剪单个坐标后继续长时间执行；建议终止当前 chunk、发送 hold/stop，并记录具体 action 与目标位姿。

### 6.3 数据有效性

拒绝以下输入或输出：

- NaN/Inf；
- 非 15×7 action；
- 不足 48 个 ActionCodec token；
- 无效 quaternion；
- 过期的 TF、图像或夹爪反馈；
- 外部相机与 wrist 图像时间差过大；
- 模型响应超过规定超时；
- 控制器或 Servo 状态异常。

### 6.4 Watchdog 与停止

需要明确：

- action queue 为空时发送 hold/stop；
- policy 超时后不能重复执行旧 chunk；
- 新 chunk 应替换还是追加旧 queue；
- ROS 节点异常退出时机器人进入安全状态；
- 急停触发后必须人工复位，不能自动继续。

## 7. 上线前测试流程

### 阶段 A：离线协议测试

1. 用录制数据构造 15×7 action；
2. 完成 ActionCodec encode/decode；
3. 检查重建 MSE；
4. 检查夹爪二值准确率；
5. 检查 state normalization 与训练完全一致。

当前 64 个动作块的 ActionCodec 审计结果：

```text
归一化空间整体 MSE       4.6862e-4
协议动作空间整体 MSE     2.4353e-4
平移三维 MSE             1.1155e-4
旋转三维 MSE             4.5621e-4
夹爪连续值 MSE           1.4293e-6
夹爪二值准确率           100%
物理平移 RMSE            约 0.79 mm/step
物理旋转 RMSE            约 0.184°/step
```

### 阶段 B：控制端 dry-run

不向机器人发送目标，只记录：

- 原始 observation；
- 归一化 state；
- 48 个 action token；
- ActionCodec decode；
- DROID action 反归一化结果；
- 每一步目标 position/quaternion；
- workspace 与限幅结果；
- 夹爪状态机决策。

### 阶段 C：六方向小动作

不运行 VLA，人工构造小动作：

```text
+x, +y, +z
+rot_x, +rot_y, +rot_z
```

逐项验证：

- 坐标轴方向；
- base frame/tool frame；
- quaternion 乘法顺序；
- 旋转是否被误当作 Euler；
- 单步幅度与 15 Hz timer。

### 阶段 D：低幅度模型执行

1. 限制最大 translation/rotation；
2. 首先禁用夹爪；
3. 每次只执行 1～3 个 action step；
4. 再执行完整 15-step chunk；
5. 最后启用夹爪状态机；
6. 对照模型目标、实际轨迹与 TF 反馈。

## 8. 实现检查清单

- [ ] 模型服务读取 7D EEF state，而不是 8D joint state；
- [ ] quaternion 使用 `xyzw` 顺序；
- [ ] state quaternion 转绝对 rotvec；
- [ ] 使用 DROID EEF state q01/q99；
- [ ] 输入图像为 exterior + wrist 两视角；
- [ ] ActionCodec 使用 embodiment 2；
- [ ] 输出严格为 48 token、15×7；
- [ ] action 前 6 维只进行一次 DROID 反归一化；
- [ ] translation 乘 `0.075`；
- [ ] rotation 乘 `0.15`；
- [ ] rotation 使用 `Rotation.from_rotvec`，不是 Euler；
- [ ] Task0 使用 `R_current × R_delta`（DROID 预训练权重用 `R_delta × R_current`）；
- [ ] 平移在 base/world frame、Task0 旋转在 tool frame 执行；
- [ ] chunk 内采用累计 pose target；
- [ ] action 执行频率为 15 Hz；
- [ ] gripper 为 `0=闭，1=开`；
- [ ] 夹爪使用阈值迟滞；
- [ ] workspace、单步限幅、超时和 watchdog 生效；
- [ ] 先完成 dry-run 和六方向测试，再启用完整 VLA 控制。

## 9. 最容易出现的错误

1. 把 action 的旋转三维当绝对姿态；
2. 把 action rotvec 当 Euler/RPY 执行；
3. quaternion 使用了 `wxyz`，而代码按 `xyzw` 解释；
4. 混淆坐标系：平移是 base-frame 增量，Task0 旋转是 tool-frame 增量（原文写成两者都是 base-frame）；
5. 使用 Task0 自身 state stats，破坏预训练 state encoder 的输入坐标系；
6. 重复执行 ActionCodec 或 DROID action 反归一化；
7. 将 `0=闭，1=开` 错误反转；
8. 以 30 Hz 执行 15 Hz 模型动作；
9. 每一步基于滞后的 measured pose 重新生成目标，导致轨迹缩短或抖动；
10. 继续使用旧三相机 prompt，造成视觉 token 布局与训练不一致。

## 10. 推荐修改范围

最小实现建议包含两个专用文件：

```text
deploy_franka_task0_qwen_actioncodec_eef.py
franka_task0_vla_eef_bridge.py
```

不要直接覆盖旧 joint-state/旧 checkpoint 的部署文件。新旧协议使用独立端口和明确 metadata，能够避免启动错误模型或错误控制端时仍然建立连接。
