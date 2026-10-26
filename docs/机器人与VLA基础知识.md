# 机器人与 VLA 部署基础知识

> 带插图的网页版：同目录下 `机器人与VLA基础知识.html`，下载后用浏览器打开即可（单文件、无外部依赖，支持深色模式）。

本文面向第一次接触机器人控制的同学，解释这个项目里反复出现的概念。每个概念都配一个本项目里的具体例子，
读完后再看 `docs/franka_eef_pose_control_adapter.md`、日志 `policy.jsonl` 和各个 `run_*.sh` 会容易很多。

建议阅读顺序：第 1 章看全貌 → 第 2～4 章（坐标系与旋转，最容易出错）→ 第 5～7 章（机器人怎么动）→
第 8～10 章（模型怎么出动作）→ 第 11 章（怎么排查）。最后有术语表。

---

## 1. 整条链路长什么样

一次推理到机械臂动起来，经过这些环节：

```
 ┌──────────┐  图像   ┌──────────────┐  观测(图像+状态+指令)   ┌───────────────┐
 │ 3 台相机  │ ─────▶ │  adapter      │ ───────────────────▶  │ 模型服务(63)   │
 │ 外部左/右 │        │ (机器人电脑)   │ ◀───────────────────  │ Qwen3-VL VLA  │
 │ 腕部      │        │               │  动作块 15×7            └───────────────┘
 └──────────┘        │  积分成目标位姿 │
 ┌──────────┐ 关节角  │  安全检查      │  目标位姿 / 关节速度
 │ 机械臂    │ ─────▶ │               │ ─────────────────────▶ 控制器 ─▶ 电机
 │ 夹爪      │ 宽度    └──────────────┘
 └──────────┘
```

- **观测（observation）**：模型看到的东西——两张图 + 机器人当前状态 + 任务指令文字。
- **动作（action）**：模型输出的东西——接下来 1 秒内每一小步该怎么动。
- **adapter**：机器人电脑上的"翻译官"，把相机和机器人数据整理成模型要的格式，把模型的输出变成控制器能执行的命令，并做安全检查。本项目里是 `eef_pose_policy_adapter.py` / `eef_pose_direct_adapter.py`。
- **模型服务**：跑在 GPU 服务器（63）上的程序，通过网络（WebSocket）收观测、回动作。本项目里是 `deploy_franka_task0_qwen_actioncodec_eef.py`、`deploy_droid_qwen_actioncodec_eef.py`。

**最重要的原则：训练-部署对齐。** 模型只认识训练时见过的输入格式。部署时任何一处与训练不同（图像尺寸、指令文字、坐标系、单位、频率、夹爪编码……），模型都会"看错"或"做错"，而且通常不报错。本项目前后修的大部分问题都属于这一类。

---

## 2. 坐标系（frame）

### 2.1 为什么需要坐标系

"往前 3 cm" 必须说清楚是**谁的前**。机器人里每个关键部件都有自己的坐标系（三根互相垂直的轴 x/y/z + 一个原点），一个位置或方向只有在指定坐标系后才有意义。

### 2.2 本项目用到的坐标系

| 名称 | 在哪 | 说明 |
|---|---|---|
| `world` / `base` | 机械臂底座 | 固定不动。本项目两者重合。x 朝前、y 朝左、z 朝上 |
| `fr3_link0`～`fr3_link7` | 每节臂 | 随关节转动 |
| `fr3_link8`（法兰 flange） | 最后一个关节末端的安装面 | 本项目的"末端"，状态和目标位姿都用它 |
| `fr3_hand_tcp`（TCP） | 夹爪两指中心 | TCP = Tool Center Point，工具中心点。比法兰再往外约 10 cm |
| `camera_left_optical_frame` 等 | 各相机 | 相机自己的坐标系 |

**位姿（pose）** = 位置（x, y, z）+ 姿态（朝向）。例如日志里的 `armed at [0.5581, -0.0017, 0.3225]` 就是 `fr3_link8` 在 `world` 里的位置，单位米。

### 2.3 TF 树

ROS 用 **TF**（transform）记录所有坐标系之间的关系，组成一棵树：
`world → base → fr3_link0 → … → fr3_link8 → fr3_hand → fr3_hand_tcp`。
关节动了，`robot_state_publisher` 根据关节角实时更新这棵树。查询 `world → fr3_link8` 就得到末端当前位姿。

例子：adapter 日志里的 `waiting for world -> fr3_link8` 表示这棵树还没建好；`world -> fr3_link8 is 1.033s old` 表示查到的数据太旧（可能关节反馈断了）。

### 2.4 法兰 vs TCP 为什么重要

同样"绕某个点转 0.1 rad"，绕法兰转和绕夹爪中心转，指尖的位置差 ≈ 0.1 × 10 cm = 1 cm。所以训练数据用哪个点做末端，部署就必须用哪个点。本项目核实过：Task0 数据和 DROID 数据都用法兰 link8，所以不需要 TCP 外参。

---

## 3. 姿态的几种表示方法

同一个朝向可以用几种不同的数字写出来，混用是最常见的 bug 来源。

### 3.1 欧拉角 / RPY

用三个角度：绕 x 转 roll、绕 y 转 pitch、绕 z 转 yaw。直观，但：

- 转的**顺序**和"绕固定轴还是绕自己的轴"有多种约定（`xyz`、`XYZ`、`zyx`……），约定不同结果完全不同；
- 在某些角度会出现"万向锁"。

DROID 数据存的就是 RPY，训练代码用 `Rotation.from_euler("xyz", …)` 读它（小写 = 绕固定轴）。

### 3.2 旋转矩阵

3×3 矩阵 R，三列分别是物体自身 x/y/z 轴在外部坐标系里的方向。数学上最清楚，但 9 个数表示 3 个自由度，有冗余。

### 3.3 四元数（quaternion）

4 个数 `(x, y, z, w)`，满足 x²+y²+z²+w²=1。ROS 和 TF 都用它。注意：

- **顺序**：ROS/scipy 是 `xyzw`，有些库是 `wxyz`，弄反了姿态全错；
- **q 和 −q 表示同一个姿态**，所以比较时要考虑符号。

日志例子：`quaternion_xyzw: [0.9253, -0.3790, -0.0124, 0.0036]`。

### 3.4 旋转向量（rotation vector / 轴角 axis-angle / rotvec）

3 个数 `r`：方向 = 转轴，长度 = 转角（弧度）。例如 `[0, 0, 0.1]` = 绕 z 轴转 0.1 rad（≈5.7°）。
本项目模型的**状态**里姿态用 rotvec，**动作**里旋转增量也用 rotvec（再除以 0.15）。

**±π 跳变问题**：转角限定在 [0, π]。当姿态接近转了 180° 时（本项目夹爪朝下，起点就是 179.6°），
很小的抖动会让 rotvec 在 `(2.64, -1.62, ·)` 和 `(-2.68, 1.64, ·)` 之间来回跳。物理上几乎没动，数字却跳了一大截。
日志里的 `state rotvec |r|=3.088 rad is near the pi flip boundary` 就是在提示这件事。Task0 训练数据里 49% 的帧都处在"翻过去"的那一侧。

---

## 4. 旋转的合成：左乘与右乘

这是本项目花时间最多的一个问题，值得单独讲。

### 4.1 两种"转一下"

已知当前姿态 R，要再转一个小增量 Δ：

- **左乘** `R_new = Δ × R`：Δ 是绕**世界/基座**的轴转。例如 Δ = 绕 z 转 10°，夹爪绕竖直轴转，不管夹爪本身朝哪。
- **右乘** `R_new = R × Δ`：Δ 是绕**工具自身**的轴转。例如 Δ = 绕 z 转 10°，夹爪绕它自己的 z 轴转。

夹爪朝下时，工具 z 轴 ≈ 世界 −z 轴，所以**同样一个"绕 z 转 +10°"，左乘和右乘的转向正好相反**。

### 4.2 增量是怎么算出来的，就要怎么用回去

录制数据时，从前后两帧姿态算增量：

- `Δ = R_cur × R_prev⁻¹` → 世界系增量 → 部署要左乘；
- `Δ = R_prev⁻¹ × R_cur` → 工具系增量 → 部署要右乘。

本项目的录制器注释写的是前者，代码实际算的是后者。用录制数据验证：按右乘复现误差 ≤3e-7 rad，按左乘 0.02～0.04 rad。
所以 Task0 权重部署用右乘（服务端 metadata `relative_rotvec_right_multiply`，日志里 `rotation_frame=tool`）；
DROID 原始数据是世界系，DROID 权重用左乘（`rotation_frame=world`）。

**平移**没有这个问题：本项目所有平移增量都是世界系 `p_cur − p_prev`。

### 4.3 怎么在真机上确认

逐轴测试（`./task1-act.sh axis-test +z`、`+rz`）：
- `+x` 应该沿底座 x 轴走，不随夹爪朝向变；
- `+rz`（右乘时）应该绕夹爪自身轴转，朝下时看起来是"绕竖直轴反向转"。

---

## 5. 运动学

### 5.1 关节空间 vs 笛卡尔空间

- **关节空间**：7 个关节角 `q1…q7`（弧度）。电机直接控制的是它们。日志 `real_joints`。
- **笛卡尔空间**：末端位姿（位置 + 姿态）。人和模型更容易理解"夹爪往哪走"。

### 5.2 正运动学（FK, forward kinematics）

已知 7 个关节角 → 算出末端位姿。是确定的公式：每节臂的长度和转向（`franka_description/robots/fr3/kinematics.yaml`）连乘。
本项目 `eef_direct_kinematics.forward_kinematics()` 就是它，与机器人 TF 对比误差 0.02 mm。

### 5.3 逆运动学（IK, inverse kinematics）

已知想要的末端位姿 → 算关节角。更难：可能无解，也可能多解（7 个关节控制 6 个自由度，有一个冗余）。

### 5.4 雅可比矩阵（Jacobian）

6×7 矩阵 J，描述"关节转得多快 → 末端动得多快"：`末端速度(6) = J × 关节速度(7)`。
反过来用：想让末端以某速度运动，就求 `关节速度 = J⁻¹ × 末端速度`，这叫**微分逆运动学**。J 不是方阵，用伪逆。

### 5.5 奇异位形与阻尼最小二乘（DLS）

某些姿势下（比如手臂完全伸直），J 会"退化"，某个方向末端怎么都动不了，硬算会得到巨大的关节速度。
**阻尼最小二乘** `q̇ = Jᵀ (J Jᵀ + λ² I)⁻¹ v` 加一点阻尼 λ，接近奇异时自动变保守。
本项目跟踪器在最小奇异值 < 0.02 时直接拒绝（日志 `sigma_min` 字段），实测运行中最小值约 0.136，离奇异很远。

---

## 6. 机械臂怎么被控制

### 6.1 控制层次

```
策略（15 Hz，每块 15 步 = 1 秒）
  └─ 跟踪器（80～100 Hz）：目标位姿 → 末端速度 → 关节速度
       └─ ros2_control 控制器（1 kHz）
            └─ Franka 控制柜 FCI（1 kHz 实时）→ 电机
```

越往下频率越高、越接近硬件。上层只需要给"方向"，下层负责平滑和安全。

### 6.2 MoveIt 与 MoveIt Servo

- **MoveIt**：ROS 的运动规划框架（避障规划、IK、碰撞检测）。
- **MoveIt Servo**：MoveIt 里的**实时**模块。你持续给它"末端往哪个方向以多快速度动"，它实时换算成关节速度，并做平滑、限速、碰撞/奇异检查。
- 本项目 VR 采集用 `franka_vr_vel.cpp`：每周期算 `目标 − 当前` 的误差，乘增益 2/s 得到末端速度（上限 0.30 m/s、0.975 rad/s），交给 Servo。

### 6.3 P 控制器与跟踪滞后

**P（比例）控制器**：速度 = 增益 × 误差。离目标越远走越快，越近越慢，永远"追着"目标跑。
当目标持续移动时，机械臂总落后目标一段距离（**跟踪滞后**），稳态滞后 ≈ 目标速度 / 增益。
本项目增益 2/s，目标 6 cm/s 时滞后约 3 cm。VR 录制数据里实测中位滞后 19～30 mm。日志 `target_lead_m` 就是这个量。

### 6.4 Direct 路径（绕过 Servo）

pi0.5 模型直接输出关节速度，用 `pi05_direct_velocity_executor` 直接下发，不经过 Servo。
EEF 模型输出的是末端动作，所以 `run_eef_direct.sh` 里用自己写的跟踪器（同样的增益 2/s 规律 + DLS 微分逆运动学）把末端目标变成关节速度，再交给 direct 执行器。`task1-act.sh` 默认走这条路，`EEF_CONTROL=servo` 退回 Servo。

### 6.5 安全保护（日志里常见的字样）

| 保护 | 意思 |
|---|---|
| 单步限幅 `translation_clipped` | 每步平移 ≤ 6 mm、旋转 ≤ 0.035 rad，超了按比例缩小 |
| 工作区边界 `workspace_violation` | 目标出了 `eef_workspace_bounds.yaml` 的盒子就锁停 |
| 跟踪违例 `tracking_violation` | 目标领先实测太多（>6 cm），说明机械臂没跟上 |
| 位姿跳变 `pose_jump` | 实测位姿瞬移，说明 TF 数据源异常 |
| 限速/限加速度 | 关节速度 ≤ 0.6 rad/s、加速度 ≤ 2 rad/s²（direct 执行器） |
| 看门狗 watchdog | 命令超过 0.12 s 没刷新就停车 |
| `FAULT LATCHED` | 出现任何异常就"锁定故障"：停车并要求重启，不会自己恢复 |

---

## 7. ROS 2 基础

| 概念 | 一句话 | 本项目例子 |
|---|---|---|
| 节点 node | 一个独立运行的程序 | `eef_pose_policy_adapter`、`franka_gripper_node` |
| 话题 topic | 广播式数据流，谁都能订阅 | `/camera_left/image_rect`、`/franka/joint_states` |
| 服务 service | 一问一答 | `/set_target_pose`（发目标位姿给 Servo） |
| 动作 action | 长时间任务，有过程反馈和结果 | 夹爪 `/fr3_gripper/franka_gripper/grasp`、`move`、`homing` |
| TF | 坐标系关系树 | `world → fr3_link8` |
| QoS | 消息传输策略（可靠/尽力而为） | 相机和关节用 sensor_data（尽力而为） |
| launch | 一次启动一组节点 | `franka_twist.launch.py`、`vla_cameras.launch.py` |

`/joint_states` 与 `/franka/joint_states` 的区别：后者是机械臂真实反馈；前者是 `joint_state_publisher` 把多路反馈合并后的版本，启动初期可能是 URDF 默认值。adapter 要求两者一致后才"armed"（日志 `pose_stable` → `armed`）。

本项目所有 ROS 程序都跑在 Docker 容器里，用 tmux 会话管理（`franka_vr_eef_direct_robot`、`franka_vr_eef_direct_cameras`），`./task1-act.sh stop` 会把它们一起停掉。

---

## 8. 相机

- **内参（intrinsics）**：焦距、光心、畸变系数，描述镜头本身。标定文件 `config/cameras/left_c922.yaml`。
- **`camera_info`**：ROS 里发布内参的话题。
- **去畸变（rectify）**：用内参把鱼眼式弯曲的原图 `image_raw` 校正成 `image_rect`。需要图像和 `camera_info` 配对，缺一不可（20:09 那次就是 `camera_info` 没来，`image_rect` 一直没有输出）。
- **外参（extrinsics）**：相机在世界里的位置和朝向（`extrinsics.yaml`）。
- **自动曝光**：相机刚启动时画面会很暗，训练数据每条开头几帧腕部图像平均亮度只有 16（正常约 100）。
- **分辨率与缩放**：相机出 640×480，模型要 256×256。缩放方式（双线性/双三次、是否裁剪、是否保持比例）必须与训练一致。

---

## 9. 夹爪

- **宽度（width）**：两指间距，FR3 夹爪 0～0.08 m。日志 `gripper_width`。
- **Move**：移动到指定宽度（用于张开）。
- **Grasp**：以一定力闭合到目标宽度，附带容差 epsilon：最终宽度在 `[目标 − inner, 目标 + outer]` 内算成功。本项目 inner 0.1、outer 0.2 m，几乎任何结果都算成功——所以"成功"不代表抓到了东西，要看实测宽度（抓住试管约 0.013 m，空抓约 0）。
- **Homing**：校准。夹爪上电后不 homing，读数可能整体偏移（本项目见过张开读 0.139 m、合拢读 0.071 m）。adapter 现在读数 > 0.085 m 时自动 homing。
- **迟滞（hysteresis）**：模型输出 ≥ 0.6 才张开、≤ 0.4 才闭合，中间保持不变，防止在阈值附近反复开合。
- **编码约定**：本项目统一 `0 = 闭合，1 = 张开`。DROID 原始数据是反的（0 = 开），训练时已取反。

---

## 10. VLA 模型基础

### 10.1 VLA 是什么

**V**ision-**L**anguage-**A**ction：输入图像（视觉）+ 文字指令（语言），输出机器人动作。
本项目两类：**pi0.5**（Physical Intelligence 的模型，用 openpi 微调）和 **Qwen3-VL-2B + ActionCodec**（本组自研）。

### 10.2 状态（state）与动作（action）

| | 状态 = "现在在哪" | 动作 = "接下来怎么动" |
|---|---|---|
| pi0.5 | 7 个关节角 + 夹爪 | 关节速度 |
| Task0 EEF 模型 | 末端位置(3) + rotvec(3) + 夹爪开合(1) | 平移增量(3) + 旋转增量(3) + 夹爪(1) |

**绝对量 vs 增量**：状态里的姿态是绝对的（"现在朝哪"），动作里的旋转是增量（"再转多少"）。两者都写成 rotvec，但含义完全不同。

### 10.3 动作块（action chunk）与频率

模型一次输出 15×7：15 步，每步 7 个数，按 15 Hz 执行 = 1 秒。
**频率必须对**：录制数据是 30 Hz，训练时每 2 帧合成 1 步变成 15 Hz。如果按 30 Hz 执行，动作会快一倍。
**串行执行**：执行完一块 → 等机械臂到位（`settled`）→ 拍新照片 → 推理 → 执行下一块。日志里每块约 3 秒（执行 1 s + 到位等待 + 推理）。
**chunk_start**：每块从哪里开始累积目标。`measured` = 从实测位姿开始（会丢掉跟踪滞后那部分进度）；`last_target` = 从上一块最后的目标继续（`task1-act.sh` 默认，更接近训练时"一直追着目标跑"的状态）。

### 10.4 归一化（normalization）

神经网络喜欢 [-1, 1] 范围的数。训练前把状态按统计量缩放：
`normalized = 2 × (state − q01) / (q99 − q01) − 1`，q01/q99 是数据的 1% 和 99% 分位数。
**部署必须用和训练完全相同的统计文件**（`droid_eef_axis_angle_state_stats.json`），否则模型看到的数字全都偏了。
动作在输出端做反向操作（反归一化），而且**只能做一次**。

### 10.5 动作 token 化：ActionCodec 与 BAR

语言模型只会输出离散的 token（类似"字"）。ActionCodec 是一个编解码器：把一整块 15×7 的连续动作压缩成 48 个离散 token（类似把一段音频压成码本编号），模型输出 token，再解码回连续动作。解码有少量量化误差（文档里实测平移约 0.79 mm/步）。
**BAR-16**（block autoregressive）：48 个 token 分 3 块、每块 16 个并行生成，3 次前向就出结果，比逐个生成快很多（本项目单次推理约 0.4～0.9 s）。

### 10.6 Prompt 与指令

模型输入是一段对话格式的文字（system 提示 + 用户消息 + 图像位置 + 状态占位 token），要和训练时**逐字一致**。
**任务指令**必须和训练数据一致：Task0 训练用 `place the red test tube in the red basket`（`tasks.json` 的 "0"）；之前部署写成 `put the red test tube into the basket`，模型从没见过。DROID 权重见过大量不同指令，可以自由写。

### 10.7 预训练与微调

- **预训练**：在大规模数据（DROID：约 7.6 万条轨迹、多种场景）上学通用能力。
- **微调（fine-tune）**：在自己的小数据（Task0：29 条）上继续训练，适配具体任务。
- **一致性**：微调数据的格式/含义要和预训练一致，才能继承预训练学到的能力。本项目 Task0 的旋转标签是工具系、DROID 是世界系，含义不一致，等于让模型重新学旋转。

### 10.8 泛化与分布外（OOD）

- **训练分布**：训练数据覆盖的情况（试管位置、光照、起点……）。
- **分布外（out-of-distribution）**：部署时遇到训练没见过的情况。数据越少，越容易"一出分布就不会"。
- Task0 的 29 条示范起点完全相同（< 0.3 mm），所以起点偏一点模型就不适应——这就是 `home` 模式的由来。
- **数据增强**：训练时随机改变亮度、对比度、裁剪、换相机视角，让模型对这些变化不敏感。save_8000 没开，save_4000（`task1-act2.sh`）开了。

### 10.9 为什么同样的数据 pi0.5 效果好

1. pi0.5 预训练更强，泛化能力更好；
2. 关节空间没有坐标系歧义，也没有 rotvec 的 ±π 跳变；
3. 动作标签直接用机械臂实际执行的关节速度，没有限幅和"目标领先实测"的偏差。

---

## 11. 数据采集与数据格式

- **遥操作（teleoperation）**：人戴 VR（Quest），手柄位姿实时变成机械臂目标位姿（`fr3_vr_target`），Servo 跟踪它。
- **录制器** `lerobot_episode_recorder.py`：30 Hz 记录图像、关节、末端位姿、夹爪、各种命令。
- **LeRobot 格式**：每条示范（episode）一个目录，数值存 parquet 表格，图像存 mp4 视频。
- **常用列**：`eef_position/eef_quaternion`（实测末端）、`action_legacy_pose`（VR 目标位姿）、`eef_delta_*_command`（相邻两帧目标的增量，已限幅 3 mm）、`*_requested`（未限幅）、`gripper_width`、`gripper_target_width`。
- **训练缓存** `droid_eef_15hz_v1.npz`：训练脚本从 parquet 预先算好的 15 Hz 状态和动作块。

---

## 12. 排查方法

### 12.1 三种运行模式

| 模式 | 动机械臂吗 | 用途 |
|---|---|---|
| `dry-run` | 不动 | 真实观测 + 推理，检查模型输出是否合理 |
| `axis-test +z` | 动一点 | 不用模型，手工构造 5 步 × 3 mm，检查方向和控制链路 |
| `home` | 动 | 回到训练起点 |
| `run`（默认） | 动 | 完整任务 |

### 12.2 看日志 `policy.jsonl`

每行一个事件，按时间顺序：

| 事件 | 含义 | 关注什么 |
|---|---|---|
| `start` | 参数与模型 metadata | checkpoint、指令、rotation_frame 对不对 |
| `pose_stable` / `armed` | 启动检查通过 | 起点位置是否接近训练起点 |
| `gripper_implausible` / `gripper_homed` | 夹爪读数异常 / 已校准 | homing 后张开读数约 0.08 |
| `observation` | 一次观测 | `eef_state`、`gripper_width`；图像在 `observations/` 目录 |
| `chunk` | 一个动作块 | `policy_latency` 推理耗时、动作数值 |
| `action` | 每一步 | 目标位姿、实测位姿、`target_lead_m`、是否限幅 |
| `track` | direct 跟踪器（10 Hz 采样） | 关节速度、`sigma_min` |
| `settled` | 到位等待结束 | `timed_out` 是否超时 |
| `gripper_request` / `gripper_done` | 夹爪命令与结果 | 闭合后宽度：≈0.013 抓住、≈0 空抓 |
| `observation_waiting` | 等不到观测 | `blocker` 写着缺什么 |
| `fault` | 锁停原因 | 直接看 `reason` |
| `complete` | 正常结束 | `reason` |

### 12.3 失败归类

1. **启动就被拒绝**：看终端最后一行 `REFUSED` / `RuntimeError`（服务不匹配、夹爪没张开、指令不一致……）；
2. **运行中 `FAULT LATCHED`**：控制/安全层问题，看 `fault` 事件；
3. **跑完没报错但任务失败**：模型问题，对比 `observations/` 里的图像和训练数据，看模型往哪走、在哪闭合。

---

## 13. 术语表

| 术语 | 解释 |
|---|---|
| action chunk | 动作块，一次推理输出的多步动作 |
| ActionCodec | 把连续动作压成离散 token 的编解码器 |
| adapter | 机器人端连接相机、模型、控制器的程序 |
| axis-angle / rotvec | 旋转向量：方向是转轴，长度是转角 |
| BAR | 分块自回归生成 token 的方式 |
| base / world frame | 机械臂底座坐标系 |
| checkpoint / 权重 | 训练保存下来的模型参数 |
| DLS | 阻尼最小二乘，奇异附近更稳的微分逆运动学 |
| DROID | 大规模公开机器人数据集，Franka + Robotiq 夹爪 |
| dry-run | 只推理不动机械臂 |
| EEF / end-effector | 末端执行器 |
| epsilon (Grasp) | 夹爪抓取成功判定容差 |
| FCI | Franka 控制接口，1 kHz 实时通信 |
| FK / IK | 正/逆运动学 |
| flange | 法兰，本项目 fr3_link8 |
| homing | 夹爪校准 |
| Jacobian | 雅可比矩阵，关节速度到末端速度的映射 |
| left/right multiply | 左乘（世界系转）/右乘（工具系转） |
| metadata | 模型服务启动时声明的协议信息 |
| MoveIt Servo | MoveIt 的实时末端速度控制模块 |
| normalization | 归一化到 [-1, 1] |
| OOD | 分布外，训练没见过的情况 |
| P controller | 比例控制器 |
| pose | 位姿 = 位置 + 姿态 |
| prompt | 送进语言模型的完整输入文字 |
| quaternion | 四元数，ROS 用 xyzw 顺序 |
| rectify | 去畸变 |
| RPY / Euler | 欧拉角 roll-pitch-yaw |
| settle | 等机械臂到达上一块的最终目标 |
| singularity | 奇异位形 |
| TCP | 工具中心点（夹爪中心） |
| TF | ROS 坐标变换树 |
| teleoperation | 遥操作 |
| tool frame | 工具坐标系，随末端一起转 |
| tracking lag | 跟踪滞后 |
| VLA | 视觉-语言-动作模型 |
| watchdog | 超时保护 |

---

## 14. 想深入时可以看

- 坐标变换与旋转：《Modern Robotics》（Lynch & Park）第 3 章，有免费 PDF 和视频课；
- 运动学与雅可比：同书第 4～6 章；
- ROS 2 入门：ROS 2 官方 Tutorials（Beginner: CLI tools、Client libraries）；
- MoveIt Servo：MoveIt 2 文档 "Realtime Arm Servoing"；
- scipy `Rotation`：官方文档里 `from_quat / as_rotvec / from_euler` 的说明，注意"extrinsic vs intrinsic"；
- VLA：pi0 / pi0.5 论文与 openpi 仓库 README；DROID 数据集论文。
