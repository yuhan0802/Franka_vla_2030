# Task1 EEF 实机失败分析（2026-10-30）

范围：`run_eef_task.sh task1 <model>`（direct 控制）在 2026-10-30 14:54–15:40 的全部实机运行，
模型 `task01-newcodec-10k` 与 `joint8-newcodec-20k`。
证据文件：`experiment_logs/20261030_task1_eef_offline_eval/`（脚本、结果 JSON、图片）。
可复用的离线检查工具：`scripts/offline_eval/`（见"离线检查工具"一节）。
实机日志：`/media/mac/Elements2/franka_vr_docker/logs/eef_task1_*_20261030_*`。

## 结论（按证据强度排序）

1. **主因是模型：它几乎不使用图像，按本体位姿复现"平均轨迹"，闭环时去不到试管所在位置。**
   在 10 个训练场景上离线闭环，闭合点和示教抓取点的误差中位数是 119 mm，0/10 在 2 cm 以内（实验 D）。
   这不是执行精度、映射或单位问题。
2. **执行步数 `EEF_EXECUTE_HORIZON=5` 会让夹爪永远不闭合。** 今天两次 H=5 运行都没有发出闭合。
3. **执行层另有独立的硬故障**：libfranka reflex 导致 `ros2_control_node` abort，表现为
   "real joint feedback stale"。它解释的是"中途停止"，不是"抓不到"。

## 实机运行汇总

| 运行 | H | 结果 | 底层原因 |
|---|---|---|---|
| 150353 joint8-20k | 15 | 中途停止 | joint feedback stale（日志里没有 reflex 记录） |
| 150755 joint8-20k | 15 | 中途停止 | reflex: `joint_motion_generator_velocity/acceleration_discontinuity` |
| 152754 joint8-20k | 15 | 跑完，夹空 | 在 [0.760, -0.127, 0.257] 闭合 |
| 153212 task01-10k | 15 | 跑完，夹空 | 在 [0.763, -0.196, 0.263] 闭合；腕部图像显示试管不在两指之间 |
| 153649 task01-10k | 5 | 中途停止，未闭合 | reflex: `acceleration_discontinuity` + `communication_constraints_violation` |
| 154035 task01-10k | 5 | 中途停止，未闭合 | reflex: `communication_constraints_violation` |

跑完全程的两次：平移/旋转都没有被截断，每个 chunk 结束时的跟踪误差约 1.6 mm / 0.003 rad，执行是准确的。

## 已排除的原因（附核对依据）

| 怀疑点 | 核对结果 |
|---|---|
| 动作反归一化 | `action_norm_stats.json`：`franka_real_q01q99_symmetric_v1`，`clip(a / max(|q01|,|q99|))`；部署端直接乘 `arm_scale` 是对称的逆操作，正确 |
| 单位和坐标系 | 训练 `xyz_delta / 0.075`、`rotvec_delta / 0.15`、world 系左乘；adapter `physical_step` 与之一致 |
| 图像和状态的时间对齐 | 视频 579 帧 = `eef_state` 579 行（都是 30 Hz），每个动作步跨 2 帧，训练时按 `current` 索引读帧是对齐的 |
| 相机话题 | 录制和部署都用 `/camera_left/image_rect` 加腕部 `/camera/camera_front/color/image_raw` |
| RT 环境 | PREEMPT_RT 内核，`isolcpus=2,4,5`，governor=performance，FCI IRQ 绑到 CPU2；`echo: write error` 是内核托管 IRQ 改不了亲和性，无害 |
| 腕部相机亮度 | 训练数据开头几帧很暗（均值 17–29，实机约 99），但接近抓取阶段两边亮度一致（均值约 110–120），不是主因 |

## 离线实验（mac_63，checkpoint 与实机相同）

### A. 开环拟合（`experiment_logs/.../scripts/eval_ol.py`，结果在 `results/ol_task01.json`）
10 条 Task0 示教，共 1083 个时间点，用示教的真实状态和图像作为输入：
- 1 秒位移误差中位数 3.8 mm（真实位移约 48 mm），方向余弦 1.00，速度比 0.96。
- 夹爪闭合时刻基本在 ±1 步以内。

注意：checkpoint 是 `train_all_episodes=true` 训出来的，**没有留出验证集**。开环看起来准，是因为每一步都喂的是示教的真实状态，状态本身就把这条轨迹"告诉"了模型。

### B. 图像替换（`results/swap*.json`）
保留 A 回合的状态，换上 B 回合的图像。A/B 选的是抓取位置相差最远的组合：

| 模型 | 换图后预测变化（状态不变） | 和 B 自己预测的差距（图像相同） |
|---|---|---|
| task01-newcodec-10k | 2.0 mm | 24.3 mm |
| joint8-newcodec-20k | 2.3 mm | 29.3 mm |

预测完全跟着状态走，闭合时刻也不随图像变化。**结论：模型基本不使用图像。**

### C. 从实机起点离线闭环（`results/sim_h15.json`）
从实机起始状态出发，每次执行 15 步，积分出新状态再喂回模型，假设执行完全精确。图像分别用实机观测和 5 条示教：
闭合点都集中在 x 0.69–0.75、y -0.14 到 -0.23 这一小块区域，与试管位置无关（偏差 4–20 cm）。推理是确定性的。

### D. 离线检查基线（`scripts/offline_eval/gate.sh`，结果在 `results/gate_baseline_task01_newcodec_10k/`）
每个场景从示教自己的起点出发，用示教自己的图像：

| 指标 | task01-newcodec-10k |
|---|---|
| image_reliance（0 = 不看图，1 = 完全跟图走） | **0.15** |
| 闭环闭合 xy 误差中位数 | **119 mm** |
| 10 个场景中误差 ≤ 20 mm 的个数 | **0** |
| 10 个场景中误差 > 30 mm 或从未闭合的个数 | **10** |

机理：所有示教都从同一个起始位姿出发，模型不看图像就无法区分该走哪一条，只能走一条混合轨迹；离开示教状态分布后，误差不断累积。

### E. 示教抓取位置分布（Task0，50 条）
平均 [0.673, -0.196, 0.264]，标准差 [0.054, 0.032, 0.003]，x 范围 0.563–0.758。

## 离线检查工具（`scripts/offline_eval/`）

```bash
# 在存有 checkpoint 的 GPU 主机上执行（mac_63 或 mac_20，已同步到 ~/offline_eval/）
bash ~/offline_eval/gate.sh <checkpoint 目录> <GPU 编号> <端口> <输出目录>
```
- 用仓库里的 `deploy_franka_task0_qwen_actioncodec_eef.py` 起服务，跑图像替换（6 组有向配对 × 9 个进度点）和 10 个场景的离线闭环，写出 `summary.json` 后关闭服务。
- **通过标准**：`image_reliance ≥ 0.5`，并且离线闭环 `over_30mm == 0`、`within_20mm ≥ 8/10`。
  这些门限是根据试管直径和指尖开口估的工程门限，不是实测出来的。
- 局限：离线闭环里的图像是示教在同一时刻的帧，不会随模拟的机械臂变化。所以它衡量的是"看到这个场景时能否复现这条示教"，是必要条件，不等于实机一定成功。

## 可直接执行的建议

### 高置信度（证据直接支持，现在就可以做）

1. **task1 实机暂停使用 `task01-newcodec-10k` 和 `joint8-newcodec-20k`。** 实验 B/C/D 表明，即使在训练场景里它们也到不了试管位置。
2. **不要设置 `EEF_EXECUTE_HORIZON=5`。** 运行前执行 `unset EEF_EXECUTE_HORIZON`，使用 profile 默认的 15。
3. **任何新的 EEF checkpoint 上实机前，必须先通过 `gate.sh`。** 没通过就不上实机。
4. **部署脚本已经修复（本次提交）**：16:54 原训练脚本被修改，新增了 `missing_state_embedding`（可学习的状态 dropout）。
   这导致旧版部署脚本加载所有旧 checkpoint 时报错（`Missing key(s): "missing_state_embedding"`）。
   现在部署脚本只允许缺失这一个键，与训练端的规则一致。`run_eef_pose.sh` 起服务时会把仓库里的部署脚本拷到 GPU 主机，所以下次起服务会自动用上修复后的版本。

### 需要先排查（证据只指向方向，还没定位根因）

5. **`communication_constraints_violation`**：1 kHz FCI 通信丢包。RT 配置已经核对过没问题，下一步应该：
   检查 FCI 网线是否直连、网卡 EEE 是否关闭（`ethtool --show-eee enp110s0`）、跑任务期间的 CPU 负载（相机和存图）。
6. **`velocity/acceleration_discontinuity`**：今天有一条提交 "tune direct acceleration"，`pi05_direct_velocity_executor.cpp` 也有改动。
   建议回退到上一个版本做 A/B 对比；但两者之间的关系还没有验证。

## 修复实验：状态输入对照（进行中）

### 设计
mac_20，4 组并行，每组 2 张 A800。除状态处理外配置完全相同：
初始化 joint8 save_11000，Task0 + Task1，每卡 batch 16（全局 32），lr 1e-4，4000 步，每 1000 步保存一次。

| 组 | 状态处理（只动位姿 6 维：xyz + rotvec；夹爪始终保留） | 推理时 |
|---|---|---|
| mask | 训练和推理都把位姿置 0 | checkpoint 带 `state_input.json` = `gripper_only`，部署脚本自动屏蔽 |
| drop | 训练时每个样本有 50% 概率把位姿置 0 | 完整状态 |
| noise | 训练时 xyz 加 N(0, 3 cm)、rotvec 加 N(0, 0.1 rad) | 完整状态 |
| none | 不改（对照组，排除"重新微调本身就有效"的可能） | 完整状态 |

- 训练脚本：`/mnt/82_store/lyh/VLA/Ours_VQAQ_VLA/train/franka_task0_qwen_actioncodec_droid_pretrained_bar_stateaug.py`
  （16:46 从原脚本拷贝，加了 `FRANKA_STATE_AUG` 开关；增强只在训练集上生效）
- 启动脚本：`train-scripts/run_franka_task01_stateaug_from_joint8_11k_20.sh`（参数 `STATE_AUG`、`GPUS`、`MASTER_PORT`）
- 输出：mac_20 `/data2/lyh/vla/runs/franka_task01_stateaug_{mask,drop,noise,none}_from_joint8_11k_20261030/`
- tmux 会话：`claude_stateaug_{mask,drop,noise,none}`

### 为什么同时做"完全屏蔽"和"dropout"
- **mask（完全屏蔽）**：从结构上彻底切断"按位姿背轨迹"这条捷径，最能直接检验"去掉捷径后模型会不会看图"。
  风险是模型完全靠图像判断自己在哪，末段对准精度可能变差，也可能仍然学不会看图。
- **drop（dropout）**：推理时有完整状态，精度上限更高；但在有位姿的那一半样本上，捷径依然存在，部署时可能又退回"背状态"。
- **noise（加噪声）**：保留粗略位置，逼模型靠图像做 3 cm 以内的精细对准，介于两者之间。
- 哪个方案更好事先无法确定，用 `gate.sh` 的指标比较，不凭直觉选。
- 另外，原训练脚本里已经有人加了 `state_dropout_probability`（用可学习的"缺失状态" embedding 替换整个状态），这是 drop 组的另一种实现，可以作为下一轮对照。

### 结果
（每个保存点跑完 `gate.sh` 后补充到这里）

| 组 | step | image_reliance | 闭环误差中位数 | ≤20 mm | >30 mm | 是否通过 |
|---|---|---|---|---|---|---|
| 基线 task01-newcodec-10k | 10000 | 0.15 | 119 mm | 0/10 | 10/10 | 否 |
