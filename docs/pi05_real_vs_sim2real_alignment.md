# π0.5 红管真机权重与粉管 Sim2Real 权重对齐对比

更新时间：2026-10-28

## 1. 对比对象

| 项目 | 红管真机微调 | 粉管 Sim2Real 微调 |
|---|---|---|
| checkpoint | `pi05_franka_tube_pick_v5_finetune/tube_pick_v5_full_bs32_20k/20000` | `pi05_sim_tube_pick_pink2bluebin_finetune/pink2bluebin_full_bs32_20k/20000` |
| 训练配置 | `pi05_franka_tube_pick_v5_finetune` | `pi05_sim_tube_pick_pink2bluebin_finetune` |
| 数据来源 | FR3 真机示教 | Franka Panda 仿真轨迹 |
| 任务 | 红色试管放入红色篮子 | 粉色试管放入蓝色篮子 |
| 训练 prompt | `put the red test tube into the red basket` | `Place the pink tube into the blue basket.` |
| 当前部署 prompt | `put the red test tube into the red basket` | `Place the pink tube into the blue basket.` |
| prompt 对齐 | 是 | 是，大小写和标点也完全一致 |

结论：粉管任务当前的问题不是 prompt 不一致。两套模型虽然都是 π0.5-DROID 全量微调，但训练数据语义、图像域、夹爪编码和默认执行路径并不相同。

## 2. 总体对齐状态

| 链路 | 红管真机 | 粉管 Sim2Real | 是否对齐 |
|---|---|---|---|
| 模型输出形状 | `16 × 8` | `16 × 8` | 是 |
| 关节顺序 | 7 个 Franka 关节 | 7 个 Franka 关节 | 基本对齐 |
| 关节 action 编码 | `velocity / 3` | `(q[t+2]-q[t]) / 0.2`，15 Hz 下等价于 `velocity / 3` | 数学尺度对齐 |
| 部署关节解码 | `action × 3 rad/s` | `action × 3 rad/s` | 是 |
| prompt | 真机任务文本 | Sim 任务文本 | 各自与训练一致 |
| 夹爪 state 编码 | 全行程 `80→0 mm` | 工作区间 `80→50 mm` | **否** |
| 夹爪 action 解码 | 全行程 `80→0 mm` | 训练语义为 `80→50 mm` | **否** |
| 图像来源 | 真实 C922 + RealSense | 仿真渲染 | **否** |
| 默认控制链 | direct joint velocity | `run_sim2real.sh` 默认 MoveIt Servo JointJog | **否** |
| 已做的对照测试 | direct | 已切换 direct 试跑 | 控制器可对齐 |
| 默认限速/加速度 | 当前 Task1 为 `0.6 / 2.0` | `0.1 / 0.3` | **否** |
| 执行时序 | 16 步 chunk、停车、重新观测 | 同样按 16 步 chunk 执行 | 基本对齐，但与连续训练轨迹不同 |
| norm stats | DROID `droid` 统计 | 相同的 DROID `droid` 统计 | 文件相同，但对 Sim 小动作未必合适 |

## 3. 关节 action 尺度

### 3.1 红管真机数据

真机数据转换保存：

```text
stored_action = sent_joint_velocity_rad_s / 3
```

部署反解：

```text
joint_velocity_rad_s = model_action × 3
```

### 3.2 粉管 Sim 数据

Sim 原始数据为 30 Hz，转换后为 15 Hz。转换脚本使用：

```text
stored_action = (q[t+2] - q[t]) / 0.2
```

因为两个源帧对应 `1/15 s`：

```text
q[t+2] - q[t] ≈ velocity / 15
stored_action ≈ velocity / (15 × 0.2)
              = velocity / 3
```

因此七个机械臂关节的 action 尺度在数学上与真机一致，部署端 `×3` 是正确的。差异在于：Sim action 来自实际位置差分，真机 action 来自已发送速度命令；二者会受到不同动力学和跟踪误差影响。

## 4. 夹爪编码差异

第 8 维同时承担夹爪状态和夹爪动作语义，是目前最明确的不对齐项。

### 4.1 红管真机编码

真机转换把整个 `0～80 mm` 行程映射到 `1～0`：

```text
closedness_real = 1 - width / 0.08
width_real = (1 - closedness) × 0.08
```

| closedness | 真机目标宽度 |
|---:|---:|
| 0 | 80 mm |
| 0.375 | 50 mm |
| 0.875 | 10 mm |
| 1 | 0 mm |

### 4.2 粉管 Sim 编码

Sim 转换脚本把 `80～50 mm` 这个工作区间映射到 `0～1`：

```text
closedness_sim = (0.08 - width) / (0.08 - 0.05)
               = (0.08 - width) / 0.03
```

动作端又使用：

```text
action_sim = 0.875 × closedness_sim
```

对应的部署反解应该是：

```text
width_sim = 0.08 - (action / 0.875) × 0.03
```

| 模型 action | Sim 训练语义 | 普通真机 DROID 解码 |
|---:|---:|---:|
| 0 | 80 mm | 80 mm |
| 0.29 | 约 70.1 mm | 约 56.8 mm |
| 0.50 | 约 62.9 mm | 40 mm |
| 0.875 | 50 mm | 10 mm |

同一个 `0.875` 在训练中表示“闭合到 50 mm”，按普通真机公式却会执行到 10 mm，物理目标相差 40 mm。

状态输入也有同样差异。例如当前宽度为 70 mm：

```text
Sim 训练预期 state = (80 - 70) / 30 = 0.333
真机 DROID state   = 1 - 70 / 80     = 0.125
```

因此不能只修改夹爪输出 action；送入模型的夹爪 state 和模型输出的夹爪 action 必须一起按 Sim 语义处理。

## 5. 图像与场景差异

两套训练都给 π0.5 提供一个外部相机和一个腕部相机，但图像分布不同：

| 项目 | 红管真机训练 | 粉管 Sim 训练 |
|---|---|---|
| 外部图像 | 真实 C922 | 仿真相机 |
| 腕部图像 | 真实 RealSense | 仿真相机 |
| 外部视角选择 | 训练随机选择左/右 | 训练同样随机选择左/右 |
| 当前部署 | 固定真实左相机 + 真实腕部相机 | 固定真实左相机 + 真实腕部相机 |

接口键名已对齐，但仍需核对：

- 相机外参和俯视角；
- 分辨率、FOV、裁剪范围；
- 粉管和蓝篮在画面中的像素位置与尺寸；
- 腕部相机朝向；
- 光照、背景、阴影、材质和颜色；
- 真实物体相对于机器人底座的坐标。

“图像键名相同”不等于视觉分布相同。对于纯 Sim 微调模型，这通常是最大的泛化风险。

## 6. 初始姿态和场景几何

当前公共复位关节姿态为：

```text
[-0.00145, 0.16710, -0.00072, -2.14714, 0.00365, 2.29368, 0.77171]
```

Sim 训练数据的关节范围包含该姿态，整体没有越出训练分布。但 Sim 数据关节中位数约为：

```text
[0.00558, 0.15756, 0.10344, -2.15167, -0.00980, 2.30041, 0.91562]
```

其中关节 3 和关节 7 与公共复位姿态存在较明显差异。更重要的是，粉管、试管架和蓝篮相对于机器人底座的位置目前没有坐标级证据证明与仿真一致。

## 7. 控制链差异

红管成功入口 `task1.sh` 使用 direct joint-velocity 执行器。粉管原入口 `run_sim2real.sh execute` 使用 MoveIt Servo JointJog。两者都会执行关节速度，但中间处理不同：

- direct：模型速度经过限速/限加速度后进入 direct velocity controller；
- Servo：模型速度先发布为 JointJog，再经过 Servo 状态检查、平滑和控制器转换。

2026-10-28 已完成一次粉管权重的 direct 对照试跑，证明该 checkpoint 可以通过相同 direct 协议完成真实观测 dry-run 并进入执行。不过控制参数仍不同：

```text
红管当前配置：max_velocity=0.6 rad/s, max_acceleration=2.0 rad/s²
粉管当前配置：max_velocity=0.1 rad/s, max_acceleration=0.3 rad/s²
```

所以“控制器相同”之后，实际运动包络仍未完全相同。

## 8. 归一化统计差异

两套 checkpoint 的 `assets/droid/norm_stats.json` 当前使用相同 DROID 统计。粉管 Sim 数据的实际关节 action 范围大约为：

```text
最小值约 -0.043
最大值约  0.046
```

而 DROID action 统计的 q01/q99 多数约在 `-0.9～0.8`。因此 Sim 动作只占归一化范围的一小部分，训练信号会集中在接近中心的窄区间。红管模型使用同一套统计可以成功，并不能证明该统计对小幅 Sim action 同样合适。

## 9. 已观察到的运行现象

- 粉管 Servo 流程能够完成真实观测 dry-run 并进入执行；
- 多轮执行中模型长期输出夹爪值约 `0.28～0.29`，没有达到旧适配器的 `0.6` 闭合阈值；
- Servo 流程曾在约 422 和 375 个动作处因 `joint feedback became stale` 安全停止；
- 该错误属于控制/反馈链故障，不能单独证明模型策略失败；
- direct 对照流程通过 dry-run 并进入执行，随后按现场要求人工停止和复位。

## 10. 建议的对齐顺序

1. 为粉管 checkpoint 单独实现 Sim 夹爪 state/action 编解码，不影响红管模型。
2. 导出 Sim 首帧，分别与真实左相机和腕部相机画面并排核对。
3. 用底座坐标测量并复现粉管、试管架和蓝篮的位置。
4. 保持 direct 控制链，先使用保守限速验证抓取阶段。
5. 为 Sim 数据单独计算 norm stats，评估是否需要重新训练。
6. 上述条件固定后，再比较不同 checkpoint step，而不是继续改变 prompt。

## 11. 最终结论

- **已对齐**：prompt、模型结构、`16×8` 协议、七关节 action 的数学尺度、direct 控制器接口。
- **明确未对齐**：夹爪 state/action 物理尺度。
- **尚未证明对齐**：Sim/Real 图像、相机外参、实物坐标和动力学。
- **参数仍不同**：速度/加速度安全包络、Sim 专用 norm stats。

因此，当前不能把两套权重的差异简单归因于“都是 π 模型但一个成功一个失败”。红管是真机同域微调；粉管是纯 Sim 微调，必须额外完成夹爪、视觉、场景和执行参数的 Sim2Real 对齐。
