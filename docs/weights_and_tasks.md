# 权重与任务配置：怎么登记、怎么增量扩展

**一句话**：权重、服务器、端口、指令、起始位姿都写在 JSON 注册表里，shell 脚本只负责读取。新增任务或模型 = 在 JSON 里加一个条目 → 离线校验 → `show` → `dry-run` → `run`。**不需要新增或修改任何 shell 脚本。**

## 1. 两份注册表，两种动作协议

| | EEF 位姿（**当前主流程**） | pi0.5 关节速度 |
| --- | --- | --- |
| 注册表 | [`config/eef_tasks.json`](../config/eef_tasks.json) | [`vla_instruction_weights.json`](../vla_instruction_weights.json) |
| 入口 | `./run_eef_task.sh <task> <model> <mode>` | `./run_task.sh <task_id> [run\|stop\|show]`，或快捷脚本 `task1.sh` 等 |
| 解析器 | `scripts/eef_task_config.py` | `scripts/resolve_direct_task.py`（+ `select_vla_task.py`） |
| 模型 | Qwen-VL + ActionCodec，`action[15,7]` 末端增量 | openpi pi0.5-DROID，关节速度动作块 |
| 服务端 | 本仓库 `server/deploy_*.py`，自动 scp 到 GPU 机 | 远端 openpi 工程，由 `scripts/pi05_checkpoint_server.sh` 启动 |
| 结构 | 任务与模型**多对多**：`tasks.<key>.models[]` 引用 `models.<key>` | 一个任务条目绑定一个 checkpoint |

**两种协议的权重不能混用。** 架构背景见 [architecture.md](architecture.md)。

不走注册表的例外（改这些需要直接改脚本，见 [scripts.md](scripts.md)）：

- `tk1.sh` / `tk2.sh` / `tk6.sh` → `scripts/run_tk_direct.sh`：端口、GPU、指令写死在脚本里（66 号机 8101/8102/8106），复用常驻服务。
- `task.sh`：联合权重 `task1_task2_full_bs32_20k` step 15000 写死在脚本里，运行参数借用注册表中的 task2。
- `run_20k.sh`、`run_7k.sh`、`run_sim2real.sh`、`run_vla.sh`：用于复现旧实验，参数固定。

## 2. 任何改动后的验证阶梯

从上到下，每一级都比上一级更接近真机。**agent 默认只做前三级**，第 4 级及以后要由现场操作者决定：

```bash
# 1. 离线：两份注册表全部条目能否被解析（不连任何设备）
python3 scripts/validate_task_registries.py

# 2. 离线：看合并后的实际生效配置
./run_eef_task.sh list
./run_eef_task.sh <task> <model> show
./run_task.sh <task_id> show

# 3. 只读：模型服务是否在听（SSH 到 GPU 机，不动机器人）
./run_eef_task.sh <task> <model> server-status

# 4. 启动服务 + 真实观测 + 推理，不发运动指令
./run_eef_task.sh <task> <model> dry-run

# 5. 真机运动（需操作者在场、急停可达）
./run_eef_task.sh <task> <model> axis-test +z   # direct 后端首次使用前必须做
./run_eef_task.sh <task> <model> run
./run_eef_task.sh <task> <model> stop           # 停止 + 夹爪 homing + 回初始位
```

`validate_task_registries.py` 的输出含义：

- `ERROR`：启动器会拒绝这一对任务/模型，必须修。
- `WARN … serves different checkpoints`：多个模型共用同一 `host:port`，这是合法的，但切换前要先 `stop` 旧服务；否则适配器握手时会发现 checkpoint 不符而拒绝运行。
- `WARN … not listed in any tasks`：模型登记了，但没有任务引用，选不到它。

## 3. EEF 注册表结构（`config/eef_tasks.json`）

下面是字段示意（把多个真实条目的字段合在一起，不是某个模型的原样拷贝）：

```jsonc
{
  "version": 1,
  "defaults": {                      // 所有任务/模型的默认值
    "control": "direct",             // direct（默认）| servo
    "adapter": { ... },              // 观测/夹爪/旋转协议
    "runtime": { ... }               // 执行预算与限幅
  },
  "tasks": {
    "task1": {
      "instruction": "place the red test tube in the red basket",   // 必须与训练时的指令逐字一致
      "models": ["act-v2", "universal-mulaw9"],                     // 本任务允许选的模型 key
      "start_pose": {                                               // 训练首帧的末端位姿，world -> fr3_link8
        "position": [0.5577, -0.0015, 0.3205],                      // m
        "quaternion_xyzw": [0.92537, -0.37885, -0.01236, 0.00364],
        "position_tolerance_m": 0.005,
        "rotation_tolerance_rad": 0.02
      },
      "adapter": { }, "runtime": { }                                // 可选：任务级覆盖（优先级最高）
    }
  },
  "models": {
    "act-v2": {
      "scope": "single_task",                                       // 仅记录：single_task | multi_task | ...
      "training_tasks": ["Task0"],                                  // 仅记录
      "task_key_to_training_task": {"task1": "Task0"},              // 仅记录：入口 key 与训练任务名的映射
      "action_representation": "...", "action_shape": [15, 7],      // 仅记录
      "server": {                                                   // ↓ 以下字段全部必填
        "ssh": "mac_63",                 // ~/.ssh/config 中的 Host 别名
        "host": "10.24.116.63",          // WebSocket 地址
        "port": 8095,
        "gpu": 1,                        // CUDA_VISIBLE_DEVICES
        "checkpoint": "/abs/path/save_8000_chkpt",
        "actioncodec": "/abs/path/step_020000",
        "state_stats": "/abs/path/state_stats.json",
        "project": "/abs/path/Ours_VQAQ_VLA",   // 远端训练工程，作为 PYTHONPATH
        "script": "deploy_franka_task0_qwen_actioncodec_eef.py"   // server/ 下的文件名
      },
      "action_normalization": {          // 可选，强烈建议填写
        "method": "task0_maxabs",        // 见下表
        "params_path": "/abs/path/training_config.json",
        "params_key": "action_arm_scale"
        // franka_mulaw 还需要 "mu": 9
      },
      "adapter": { }, "runtime": { }     // 可选：模型级覆盖
    }
  }
}
```

**合并顺序**：`defaults` ← `models.<key>` ← `tasks.<key>`，后者覆盖前者，按字段浅合并。

### 3.1 `adapter` 字段

| 字段 | 取值 | 默认 | 含义 |
| --- | --- | --- | --- |
| `gripper_state_source` | `measured` \| `binary` \| `command` | `measured` | 状态第 7 维的来源 |
| `exterior_camera` | `left` \| `right` | `left` | 用哪台 C922 作外部视角 |
| `camera_mode` | `raw` \| `calibrated` | `calibrated` | `image_raw` 还是去畸变后的 `image_rect` |
| `model_image_size` | 正整数 | 256 | 必须等于服务端 metadata 声明的尺寸 |
| `rotation_frame` / `rotation_execution` | `world` + `relative_rotvec_left_multiply`，或 `tool` + `relative_rotvec_right_multiply` | world / left | 必须成对出现，且与训练数据一致 |
| `max_tf_age_s` | (0, 1] | 0.3 | TF 交叉校验的新鲜度 |
| `gripper_width_offset` | [0, 0.1] | 0.07 | 固件宽度偏移；`offset + open_width ≤ 0.15` |
| `gripper_open_width` | (0, 0.08] | 0.07 | |
| `gripper_close_max_z` | 0（关闭）或 [0.05, 0.9] m | 0 | 只允许在该高度以下闭合夹爪 |

### 3.2 `runtime` 字段

| 字段 | 取值 | 默认 | 含义 |
| --- | --- | --- | --- |
| `execute_horizon` | 1..15 | 10 | 每块预测 15 步，执行前 N 步。可用 `EEF_EXECUTE_HORIZON` 临时覆盖 |
| `chunk_mode` | `serial` \| `rtc` | `serial` | `rtc` 只支持 direct 后端 |
| `rtc_blend_steps` | 1..8 | 4 | RTC 交接平滑步数 |
| `rtc_observe_after` | 0..14（0 = 自适应） | 0 | RTC 发下一次观测的时机 |
| `action_gain` | 6 个 (0, 1] | `[1]*6` | 前 6 维动作缩放，只能缩小不能放大 |
| `max_translation` | [0, 0.02] m | 0 | 单步平移范数上限；**0 = 不裁剪**（当前默认，靠工作空间与跳变检查兜底） |
| `max_rotation` | (0, 0.15] rad | 0.035 | 单步旋转上限 |
| `max_chunks` / `max_actions` | 0..200 / 0..2000（0 = 不限） | 0 | 可用 `EEF_MAX_CHUNKS` / `EEF_MAX_ACTIONS` 临时覆盖 |
| `chunk_start` | `measured` \| `last_target` | `last_target` | 新动作块从哪里开始积分 |
| `auto_home` | bool | true | 偏离 `start_pose` 时 `run` 是否先自动 home |

### 3.3 `action_normalization.method`

**反归一化只在服务端做**，这里的字段用于记录和展示。其中 `franka_maxabs`、`franka_mulaw`、`franka_quantile` 三种方法还会传给适配器，与服务端 metadata 声明的方法逐一核对，不一致就拒绝运行。

| method | 说明 |
| --- | --- |
| `task0_maxabs` | 训练时前 6 维除以 `training_config.json` 中的 `action_arm_scale`，服务端再乘回 |
| `droid_gain` | DROID 增益缩放 |
| `legacy_droid_symmetric_scale` | 服务端用训练模块的 `DROID_Q01/Q99` 构造对称缩放 |
| `franka_maxabs` | 线性 maxabs，参数随 checkpoint 保存（`action_norm_stats.json` → `arm_scale`） |
| `franka_mulaw` | μ-law 压扩，需填 `mu`。反解：clip 到 [-1,1] → `sign(y)·(exp(|y|·ln(1+μ))−1)/μ` → 乘 scale |
| `franka_quantile` | 分位数缩放（newcodec 系列） |
| `unverified` | 尚未核对 checkpoint，仅作占位，**不要用于正式运行** |

## 4. 增量扩展手册

### 4.1 给已有任务加一个新权重（最常见）

1. 在 GPU 服务器上确认这 4 个路径存在：checkpoint、ActionCodec、状态统计、训练工程。
   ```bash
   ssh mac_20 'ls -d <checkpoint> <actioncodec> <state_stats> <project>'
   ```
2. 选端口和 GPU：
   - 用一个**未被占用**的端口，在两台机器上执行 `grep -n '"port"' config/eef_tasks.json` 查重，再用 `ssh mac_20 'ss -ltn'` 确认没有进程在听。
   - 用 `ssh mac_20 nvidia-smi` 确认 GPU 显存够用。
   - 共用端口是允许的，但同一时刻只能跑一个，切换前要 `stop`。
3. 复制一个**协议相同**的已有模型条目（同一 `script`、同一 `action_representation`），只改 `server.*`、`action_normalization.*` 和记录字段。归一化参数要读 checkpoint 里的 `training_config.json` / `action_norm_stats.json` 核实，**不要猜**。没核实就写 `unverified`。
4. 把新 key 加到对应任务的 `tasks.<task>.models` 列表。
5. 若训练时的执行设置不同（如 `execute_horizon`、`action_gain`、`chunk_mode`），写在**模型**的 `runtime` 里，不要改 `defaults`。
6. 走[验证阶梯](#2-任何改动后的验证阶梯)第 1-3 级，然后在 [§5 模型速查](#5-当前登记的模型速查) 里补一行。
7. 一个模型一次提交，提交信息写明 checkpoint 和 step，例如 `config: add task1 all50noise-2k (mac_20:8107, gpu3)`。

**RTC 变体**：复制一份 serial 条目，key 加 `-rtc` 后缀，`runtime.chunk_mode` 设为 `"rtc"`。它与 serial 版共用同一服务和端口。

### 4.2 新增一个任务

1. 拿到训练时的**逐字**指令文本（大小写、标点都必须一致）。
2. 测量训练首帧的末端位姿（`world → fr3_link8`）作为 `start_pose`，可以从训练数据第一帧读取，或用 `./run_eef_task.sh <已有任务> <模型> probe` 现场读取。目前 task1/2/5/40 共用公共起始位姿 `[0.5577, -0.0015, 0.3205]`，task3 略有不同。
3. 在 `tasks` 中新增条目：`instruction`、`models`、`start_pose`，必要时加任务级 `runtime`（例如 `task40` 的 `action_gain`）。
4. 多任务模型要在模型条目里补充 `task_key_to_training_task`，写明新入口 key 对应的训练任务名。
5. 如需一键快捷入口，在根目录加一个三行脚本（参考 `task1-act-v2.sh`），只转发参数：
   ```bash
   #!/usr/bin/env bash
   set -euo pipefail
   script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
   exec "$script_dir/run_eef_task.sh" <task> <model> "$@"
   ```
   这一步可以省略：通用入口已经足够，快捷脚本越多根目录越乱。

### 4.3 新增一种模型协议或服务端

动作维度、状态定义、图像输入任一项变了，都属于新协议。这已经不是配置改动，而是代码改动：

1. 在 `server/` 新增 `deploy_<name>.py`，实现与现有服务相同的 WebSocket 握手 metadata（checkpoint、actioncodec、state_stats、图像尺寸、旋转协议、归一化方法）。
2. 若状态/动作语义变了，同步修改 `eef_pose_safety.py` 中的协议常量和 [reference/eef_pose_control_adapter.md](reference/eef_pose_control_adapter.md)。
3. 若引入新的归一化方法，把它加入 `eef_task_config.py` 的 `method` 白名单，并在本文 §3.3 中登记。
4. 在 `scripts/test_*.py` 中补离线测试，在 GPU 机上用 `scripts/offline_eval/` 做开环/闭环评估，再上真机。

### 4.4 pi0.5 关节速度任务（`vla_instruction_weights.json`）

```jsonc
{
  "id": "task0-fsp-demo10",                    // run_task.sh 的参数，必须唯一
  "instruction": "put the red test tube into the red basket",
  "instruction_selectable": false,             // 指令与其他条目重复时设为 false（否则按指令查找会冲突）
  "checkpoint_root": "/abs/path/run_dir",      // 远端 66 号机上的绝对路径
  "checkpoint_step": 20000,
  "policy_config": "pi05_franka_task0_fsp_finetune",   // openpi 训练配置名
  "direct": {
    "runtime_from": "task1",                   // 继承另一个条目的 runtime（可链式继承，禁止成环）
    "init_after_stop": true,                   // stop 后自动 fci_init.sh
    "gripper_feedback_open_width": 0.14,
    "gripper_feedback_closed_width": 0.0,
    "gripper_verify_open_width": 0             // 0 | 1
  },
  "runtime": {                                 // 有 runtime_from 时这里会被忽略
    "max_actions": 240,                        // 正整数
    "max_joint_velocity": 0.15,                // rad/s
    "max_joint_acceleration": 0.5,             // rad/s²
    "gripper_force": 10.0                      // N
  }
}
```

新增步骤：复制一个条目 → 改 `id` / `checkpoint_root` / `checkpoint_step` / `policy_config` → `python3 scripts/validate_task_registries.py` → `./run_task.sh <id> show`。最近的 `task0-fsp-demo10/20`（提交 `ecdd8d3`）就是这样加的。可选：加一个快捷脚本 `<id>.sh`，模板同 `task1.sh`。

远端服务器（66 号机、端口 8094、GPU 1）在 `run_task.sh` 顶部用环境变量默认值设定，可以用 `VLA_REMOTE_HOST`、`VLA_SERVER_PORT`、`PI05_SERVER_GPU` 覆盖。

## 5. 当前登记的模型速查

快照时间 2026-11-13（`ecdd8d3`）。**权威来源始终是 `./run_eef_task.sh list` 和 JSON 本身**；改动模型后请同步更新此表。

### EEF（`config/eef_tasks.json`）

| 任务 key | 指令 | 可选模型 |
| --- | --- | --- |
| `task1` | place the red test tube in the red basket | `act-v2`、`all50noise-2k`、`all50noise-1k`、`all50noise-1k-rtc`、`joint9-384-16k`、`all50noises13-1k`、`all50-2k`、`act`、`act-1k`、`act2`、`droid`、`task01-mulaw9-5k`、`task01-maxabs1x-9k`、`joint8-newcodec-20k`、`task01-newcodec-10k`、`universal-mulaw9`、`universal-maxabs-x2` |
| `task2` | place the test tube rack in the black tray | `task2-newcodec-stage1-5k`(-rtc)、`task2-newcodec-stage1-3k`(-rtc) |
| `task3` | store the bottle inside the container and close the lid | `task3-own-p1-7k`、`task3-own-n5k-500` |
| `task5` | press the yellow button to turn on the yellow light | `task5-newcodec-stage1-1k`(-rtc) |
| `task40` | open the lid | `universal-mulaw9`、`universal-maxabs-x2` |

| 模型 key | 服务器:端口 / GPU | 归一化 | 执行 | 备注 |
| --- | --- | --- | --- | --- |
| `act-v2` | 63:8095 / 1 | task0_maxabs | serial | task1 已验证成功的配置（2026-10-25） |
| `act-1k` | 63:8095 / 1 | legacy_droid_symmetric_scale | serial | 与 act-v2 共用端口 |
| `act` | 20:8095 / 1 | droid_gain | serial | |
| `act2` | 20:8096 / 1 | **unverified** | serial | checkpoint 路径核查时不存在 |
| `droid` | 20:8097 / 1 | legacy_droid_symmetric_scale | serial | 使用 `deploy_droid_qwen_actioncodec_eef.py` |
| `task01-mulaw9-5k` | 20:8098 / 1 | franka_mulaw | h=15 | 多任务 |
| `task01-maxabs1x-9k` | 20:8099 / 1 | franka_maxabs | h=6 | 多任务 |
| `joint8-newcodec-20k` | 20:8100 / 1 | franka_quantile | serial | 多任务 |
| `task01-newcodec-10k` | 63:8101 / 1 | franka_quantile | serial | |
| `all50-2k` | 20:8105 / 1 | task0_maxabs | serial | act-v2 续训 |
| `all50noise-1k` / `-rtc` | 20:8106 / 2 | task0_maxabs | serial / rtc | |
| `all50noise-2k` | 20:8107 / 3 | task0_maxabs | serial | |
| `all50noises13-1k` | 20:8108 / 3 | task0_maxabs | serial | |
| `joint9-384-16k` | 20:8109 / 4 | franka_quantile | serial | 多任务 |
| `task3-own-p1-7k` | 63:8110 / 0 | franka_quantile | serial | 与 n5k-500 共用端口 |
| `task3-own-n5k-500` | 63:8110 / 0 | franka_quantile | serial | |
| `task2-newcodec-stage1-5k` / `-rtc` | 20:8111 / 5 | franka_quantile | h=5, gain 0.5 | task2 首选（离线抓取事件评测最优） |
| `task2-newcodec-stage1-3k` / `-rtc` | 20:8111 / 5 | franka_quantile | h=5, gain 0.5 | 对照 |
| `task5-newcodec-stage1-1k` / `-rtc` | 20:8112 / 6 | franka_quantile | h=5, gain 0.5 | task5 首选 |
| `universal-mulaw9` | 63:8098 / 1 | franka_mulaw μ=9 | h=10 | 8 任务联合；task1→Task0、task40→Task40 |
| `universal-maxabs-x2` | 63:8099 / 1 | franka_maxabs | h=5 | 8 任务联合 |

### pi0.5 关节（`vla_instruction_weights.json`，66 号机 8094）

| id | 指令 | checkpoint | 运行参数 |
| --- | --- | --- | --- |
| `task1` | put the red test tube into the red basket | tube_pick_v5_full_bs32_20k @20000 | 继承 task2 |
| `task0-fsp-demo20` / `demo10` | 同上 | task0_fsp_demo20/10 @20000 | 继承 task1 |
| `task2` | place the test tube rack in the black tray | task2_rack_black_tray_full_bs32_20k @20000 | 0.6 rad/s, 2.0 rad/s² |
| `task6` | open the lid | task6_open_lid_full_bs32_20k @20000 | 继承 task2 |
| （无 id） | Place the pink tube into the blue basket. | sim pink2bluebin @20000 | 仅 `run_sim2real.sh` 按指令查找 |

## 6. 常见坑

- **只改环境变量换权重是无效的**：`EEF_CHECKPOINT`、`EEF_SERVER_HOST`、`EEF_INSTRUCTION` 会被 JSON 解析结果覆盖。要换权重就改 JSON，或者用 `EEF_TASK_CONFIG=/path/to/other.json` 指向另一份配置。
- **端口被旧模型占着**：`server` 发现端口已在监听就直接复用，适配器握手时发现 checkpoint 不符会拒绝运行并提示 `stop the stale server first`。先用旧模型的 key 执行 `stop-only`。
- **指令差一个字符**：模型对指令文本敏感。EEF 的 `task1` 是 `place … in …`，pi0.5 的 `task1` 是 `put … into …`，两者不能互换。
- **`action_gain` 只能 ≤ 1**：它是安全缩放，不能用来放大动作。
- **`defaults` 改动会波及所有模型**：只为某个模型调参时，写在该模型的 `runtime` 里。
- **起始位姿**：`start_pose`（EEF 训练起点，`check-start`/`home` 用）与 `config/default_initial_pose.json`（`fci_init.sh` 的关节回位目标）是两回事，分开维护。
