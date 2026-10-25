# Task1 v5 版本记录：实机执行成功（2026-10-25）

- **tag**：`task1-v5-real-ok-20261025`
- **分支**：`feat/eef-pose-control`
- **代码**：`554d16e`（EEF adapter 夹爪反馈与 5 步执行修复）之后的提交，代码内容与 `554d16e` 相同，本提交只增加文档。
- **结果**：用户反馈最新代码在实机上执行成功（2026-10-25）。成功运行的日志没有留在仓库里，**具体是 `task1-act.sh`（x3）还是 `task1-act-v2.sh`（maxabs）成功，记录时未注明**，确认后请在下面“成功的脚本”一栏补上。
- **成功的脚本**：待补（`task1-act.sh` / `task1-act-v2.sh`）

## 1. 两套配置（均为 world 帧、左乘、夹爪 `measured`、执行 15 步）

| | `task1-act.sh` | `task1-act-v2.sh` |
|---|---|---|
| 训练 | `v5_x3`：动作整体放大 3 倍（`action_normalization=droid_gain`） | `v5_maxabs`：每维最大绝对值（`action_normalization=task0_maxabs`） |
| 权重 | `save_8000_chkpt` | `save_8000_chkpt` |
| 权重路径 | `/data2/lyh/vla/runs/franka_task0_droid_bar_v5_x3/Qwen3VL2B+Franka-Task0+lr0.0001+v5_droid_x3_first15_all29_no_crop_10k_8gpu_b256/` | `/data2/lyh/vla/runs/franka_task0_droid_bar_v5_maxabs/Qwen3VL2B+Franka-Task0+lr0.0001+v5_maxabs_first15_all29_no_crop_10k_8gpu_b256/` |
| 模型服务器 | 20（`mac_20`，10.24.116.20:8095） | 63（`mac_63`，10.24.116.63:8095） |
| ActionCodec | `/data2/lyh/weights/ActionCodec-Base-RVQft` | `/data2/lyh/vla/assets/ActionCodec-Base-RVQft` |
| 动作缩放（`action_arm_scale`） | `[0.259, 0.193, 0.245, 0.224, 0.235, 0.297]`（DROID 系数的 1/3） | `[0.104, 0.201, 0.196, 0.119, 0.105, 0.283]`（每维最大绝对值） |
| 执行步数 / 最大块数 | 15 / 60 | 15 / 60 |

两者共用：`EEF_ROTATION_FRAME=world`、`EEF_ROTATION_EXECUTION=relative_rotvec_left_multiply`、`EEF_MAX_TRANSLATION=0`（旋转限幅仍是 0.035 rad/步）、`EEF_CHUNK_START=last_target`、`EEF_GRIPPER_STATE_SOURCE=measured`（状态夹爪 = `clip((width-0.01)/0.07, 0, 1)`，动作夹爪仍是 0/1 目标）。

服务端从 checkpoint 的 `training_config.json` 读 `action_arm_scale`，训练和推理的缩放因此一致；缺这个字段会退回旧的 DROID 系数（旧权重用）。

## 2. 夹爪相关的执行端保护（`eef_pose_policy_adapter.py`，554d16e）

- 回零后等 2 秒，判断稳定后的读数是否在 [0.07, 0.085]，不再接受运动途中的瞬时值。
- 运行中夹爪宽度超过 0.085 m（物理最大值）直接 fault。dry-run 只报错。
- 闭合成功后宽度必须低于 0.045 m（`--gripper-verify-timeout`，0.5 s 内），否则 fault。
- 闭合后 5 秒内不接受张开（`--gripper-close-hold`，默认 5.0；演示里闭合到释放最短 7.5 s）。
- 执行步数小于 12 时启动警告：模型把夹爪切换和接近收尾放在块的后半段，只执行前几步会让它们永远落在没执行的未来。
- 离线测试：`scripts/test_eef_pose_adapter_startup.py`，21 个，需要 msgpack/yaml/cv2（服务器 qwenvla 环境里可跑）。

## 3. 运行前检查

1. 先 `./task1-act.sh stop`（或 `./task1-act-v2.sh stop`）停掉服务器上的旧模型服务，否则会复用旧服务，adapter 校验 checkpoint 不一致会拒绝执行。
2. dry-run 日志里确认：元数据的 `checkpoint` 是上面的路径；夹爪张开时每指约 0.0356、总宽约 0.0712、状态约 0.87（训练起点 0.85 到 0.87）。出现 `gripper_implausible` 说明夹爪没有回零，先处理夹爪。
3. 不要把执行步数改成 5（v2 曾经这样导致闭合从未被执行）。
4. 试管、试管架尽量摆在训练时的位置附近（训练抓取位置 x≈0.668±0.054、y≈-0.198±0.047）。

## 4. 变更历史（自 10-24）

| 提交 | 时间 | 内容 |
|---|---|---|
| fe2b5cb | 10-24 18:08 | 调整 EEF world frame 与夹爪启动检查 |
| 5a1fa7e | 10-24 18:16 | Task1 v4：夹爪状态改连续，服务端默认 world 左乘 |
| 3702cb8 | 10-24 18:42 | 新增 task1-act-v2（执行 5 步），task1-act 换 save_9000 |
| a0a1eef | 10-24 19:02 | 新增 task1-act-1k（save_1000 对照） |
| d11d0e6 | 10-24 19:36 | 新增 task1-replay，回放 v5 预处理轨迹到真机 |
| 159fa2d | 10-25 14:31 | 更新 Task1 v5 双服务器推理权重（x3 / maxabs） |
| 554d16e | 10-25 16:24 | adapter：夹爪反馈与 5 步执行的两类执行端问题 |

## 5. 为什么要换成 x3 / maxabs

旧的 v4 权重在试管上方停住、从不闭合。根因是动作编码器在原来的缩放下有分辨率下限：15 步净位移小于约 25 mm 的动作大部分被量化成同一个“几乎不动”的块，训练标签本身就是静止。放大后编码能保住更小的动作。完整的实验、证据和图见 [task0_v4_stall_report_20261024.md](task0_v4_stall_report_20261024.md)。

## 6. 已知问题与未验证

- 实机成功只是一次反馈，没有成功率统计，也没有日志留档。
- 训练用了全部 29 个回合（`all29`），没有留出验证集，在新的试管位置上的泛化没有量化。
- 部分运行开局夹爪读数异常（每指 0.0712，总宽 0.1425，或读到 0），触发回零；回零后仍读到未回零的值的原因没有查清，只是被保护逻辑拦住。出现过的情形是刚重启机器人栈之后，但没有证实。
- v2 曾在 76 秒时出现 `joint feedback timeout`（关节反馈超时），原因没有查清。
