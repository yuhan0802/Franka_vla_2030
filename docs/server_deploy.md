# FrankaVR 服务端部署

本文件只描述三视角 FrankaVR WebSocket 推理服务；服务端不运行 ROS、不连接 FCI、也不控制机械臂。

## 固定配置

- 主机：`mac_29`（`10.24.116.29`）
- 项目根目录：`/mnt/82_store/lyh/VLA/Ours_VQAQ_VLA`
- 服务端口：`8093`；旧 LIBERO 服务使用 `8092`，禁止混用。
- 默认 GPU：`3`；Python：`.conda/qwenvla/bin/python`。

## 最小文件集

下列路径均相对于项目根目录，缺少任何一项不得启动：

- `vla-scripts/run_deploy_franka_simulation.sh`：真机三视角新模型的服务端入口。
- `franka_vr_3view_overfit/franka_vr_qwen_actioncodec.py`：状态编码器、future-state 模块、ActionTokenizer 与动作常量。
- `omnigibson/learning/utils/network_utils.py`：WebSocket 服务实现。
- `actioncodec_libero/weights/ActionCodec-Base-RVQft`：ActionCodec 权重。
- `norm_stats/franka_vr_state.json`：八维关节/夹爪状态归一化统计。
- `runs/franka_vr_actionfix_v2/Qwen3VL2B+FrankaVR+lr6e-05+franka_vr_actionfix_v2_8gpu-singleturn-20260928_161451_f9998af9/save_5000_chkpt/`：真机微调 checkpoint。

checkpoint 至少包含 `config.json`、`processor_config.json`、`tokenizer.json`、`tokenizer_config.json`、`chat_template.jinja`、`model.safetensors`、`franka_state_encoder.pth`、`franka_future_state_predictor.pth`、`training_config.json`、`checkpoint_complete.json`。训练配置必须是三视角、10 步观测、15 步动作；不得替换为 LIBERO 或其他任务 checkpoint。

## 运行要求

- `/mnt/82_store` 能稳定读取 checkpoint，不能在 NFS/I/O 等待中卡住。
- 使用项目自带 Python，环境中必须有 PyTorch、Transformers、Draccus、WebSockets、Msgpack、Pillow 与模型依赖。
- 启动前 GPU 建议至少保留 8 GiB 显存，8093 未被占用，控制电脑能访问该端口。
- 服务端入口不能包含 `from __future__ import annotations`；当前 Draccus 会将 `cfg: Config` 解析成字符串并报 dataclass 错误。

## 启动、确认、停止

控制电脑推荐入口：`./run_vla.sh server`。底层前台入口为 `VLA_CUDA_VISIBLE_DEVICES=3 ./run_franka_vr_vla_server.sh`，仅用于排障。

远端常驻入口：在项目根目录运行 `tmux new -s franka_vr_vla_8093 'exec env CUDA_VISIBLE_DEVICES=3 PORT=8093 bash vla-scripts/run_deploy_franka_simulation.sh'`。

用 `ss -ltn | grep ':8093'` 确认监听；预期为 `0.0.0.0:8093`。若进程为 `D` 状态或显存长期不变，先排查挂载盘 I/O，禁止重复启动多个服务。停止命令：`tmux kill-session -t franka_vr_vla_8093`。

## 协议与边界

客户端连接后先收到 MessagePack metadata：`model_family=qwen3vl2b_franka_vr_actionfix_v2`、`observation_format=franka_vr_front_left_right_and_joint_state_v1`、`action_shape=[15,7]`。

请求必须有三路 `uint8 RGB` 的 `front_image`、`left_image`、`right_image`，八维 `float32` 的 `joint_state`（七关节和固定为零的夹爪状态）以及非空 `instruction`。响应是 `action: float32[15,7]`。

仅服务端响应不能控制真机。本机桥接器仍负责首步选择、单位换算、限幅、工作区边界与 `/set_target_pose`；FCI、ROS、相机和急停不在服务端范围内。

## 最小通信核验与更新

无需机械臂即可验证：客户端读取 metadata 后发送合成三路 RGB 与固定八维关节状态；响应必须是全有限 `(15,7)` 动作块。生产执行前仍需本机完成相机、真实关节状态、`dry-run`、工作区边界和受看护低速测试。

更新入口脚本时，从控制电脑同步 `run_deploy_franka_simulation.sh` 及其依赖的 `deploy_franka_simulation_qwen_actioncodec.py` 至服务器的 `vla-scripts/`，然后在服务器运行 `bash vla-scripts/run_deploy_franka_simulation.sh --help` 检查参数。服务端不会自动重载，更新后必须重启 8093。训练、历史 LIBERO 服务、其他端口与无关数据不属于本服务最小部署集。
