# 通用 EEF 任务入口

入口是 `run_eef_task.sh`，固定模型信息统一保存在 `config/eef_tasks.json`。

```bash
./run_eef_task.sh list
./run_eef_task.sh task1 act-v2 show
./run_eef_task.sh task1 act-v2 dry-run
./run_eef_task.sh task1 act-v2 axis-test +z
./run_eef_task.sh task1 act-v2 axis-test -z
./run_eef_task.sh task1 act-v2 axis-test +rz
./run_eef_task.sh task1 act-v2 run
./run_eef_task.sh task1 act-v2 stop
```

`list`、`show` 只读取配置，不连接机器人、相机或模型服务。`dry-run` 启动观测和推理，但不执行运动。
`axis-test`、`home`、`run` 会执行真机运动。

## 任务与模型 key

目前 task1 对应训练指令 `place the red test tube in the red basket`，可选：

| 模型 key | 服务器 | 权重 | 原快捷入口 |
|---|---|---|---|
| `act-v2` | 63:8095 | v5 maxabs / 8000 | `task1-act-v2.sh` |
| `act` | 20:8095 | v5 x3 / 8000 | `task1-act.sh` |
| `act-1k` | 63:8095 | v4 / 1000 | `task1-act-1k.sh` |
| `act2` | 20:8096 | 增强版 / 4000 | `task1-act2.sh` |
| `droid` | 20:8097 | DROID 预训练 / 20000 | `task1-droid.sh` |
| `universal-mulaw9` | 63:8098 | 8 任务联合训练 / 10000，μ-law μ=9 | 使用通用入口 |
| `universal-maxabs-x2` | 63:8099 | 8 任务联合训练 / 10000，线性 maxabs x2 | 使用通用入口 |

五个快捷脚本都转发到通用入口。例如 `./task1-act-v2.sh stop` 等同于
`./run_eef_task.sh task1 act-v2 stop`。

task2、task6 已留任务配置入口；通用权重的训练集合覆盖它们，但对应 EEF 训练起始位姿与
执行关联尚未登记，因此 `models` 暂为空，补齐后可关联 `universal-mulaw9`。
选取这些未配置组合时会在启动任何服务前报错。原 `vla_instruction_weights.json` 中的
pi0.5 关节速度权重是另一种动作协议，不能直接当作这里的 15×7 EEF 权重使用。

## JSON 里的信息

- `defaults`：控制方式、图像/状态协议和默认执行预算。
- `tasks.<key>`：固定训练指令、允许使用的模型 key、训练起始位姿及偏差容差。
- `models.<key>.server`：SSH 别名、IP、端口、GPU、工程目录、权重、ActionCodec、状态统计和部署脚本。
- `models.<key>.action_normalization`：动作归一化方式 `method`、远端参数文件 `params_path` 和其中的字段 `params_key`。
- 模型和任务可分别提供 `adapter`、`runtime` 覆盖项；任务覆盖模型，模型覆盖 defaults。

`act-v2` 已核对 checkpoint：使用 `task0_maxabs`，参数在该 checkpoint 的
`training_config.json` 的 `action_arm_scale` 字段。训练时前六维动作按各自的 scale 相除，
推理时由服务端乘回同一组 scale。`act` 使用 `droid_gain`；`act-1k`、`droid` 的当前服务端
从训练模块的 `DROID_Q01`/`DROID_Q99` 常量构造对称缩放。`act2` 的已配置 checkpoint 路径
在核查时不存在，因此归一化方式记为 `unverified`，没有推测或改写其权重路径。
这里的归一化字段用于记录和 `show` 展示；服务端继续读取 checkpoint/训练模块中的真实参数，
客户端不额外反归一化。动作归一化参数与 `server.state_stats` 中的状态归一化统计分别记录。

`universal-mulaw9` 的 checkpoint 已在服务器核对：虽然目录含有 `joint8`，实际是 8 个任务
联合训练的 EEF 模型，协议仍为 15×7；任务为 Task0、Task1、Task2、Task4、Task5、Task6、Task40、Task41。
`task_key_to_training_task` 明确记录当前入口映射：task1→Task0、task2→Task2、task6→Task40，
避免把入口 task6（开盖）与训练目录 Task6（按绿色按钮）混淆。

该模型采用 `franka_mulaw`，μ=9。归一化文件优先记录 checkpoint 内随权重保存的
`action_norm_stats.json`；其中 `arm_scale` 是六维缩放，`companding.mu` 是 μ。
`training_config.json` 同时记录 `action_normalization`、`action_mulaw_mu` 和 `action_arm_scale`；
`source_params_path` 保留训练时原始统计文件地址。
正确反解是先 clip ActionCodec 输出到 [-1,1]，再对前六维执行
`sign(y) * (exp(abs(y) * log(1+mu)) - 1) / mu`，最后乘各维 scale；夹爪仍线性还原为 [0,1]。
服务端已实现该顺序，客户端只接收还原后的动作，并核对服务端方法、μ 和逆变换声明；
复用未更新的线性解码服务会拒绝启动。

```bash
./run_eef_task.sh task1 universal-mulaw9 show
./run_eef_task.sh task1 universal-mulaw9 dry-run
```

`start_pose.position` 单位为米，`quaternion_xyzw` 是四元数，均表示 `world -> fr3_link8`。
启动偏差检查和 `home` 共用此配置，不再写死 Task0 位姿。`home` 仍受现有单块小步运动、
工作空间与关节速度保护约束；距离过远时会拒绝，不会直接跳到目标。

增加任务时，在 `tasks` 中提供指令、实测训练起始位姿和允许的模型列表；在 `models` 中录入
对应 EEF 权重的全部服务器信息。无需新增 Shell 启动逻辑。

模型身份信息以 JSON 为准，不采用残留的 `EEF_CHECKPOINT`、`EEF_SERVER_HOST`、
`EEF_INSTRUCTION` 等环境值。可用 `EEF_TASK_CONFIG=/path/to/config.json` 选择另一份配置。
保留 `EEF_CONTROL=servo` 切换控制后端，以及 `EEF_MAX_CHUNKS`、`EEF_MAX_ACTIONS`、
`EEF_RUN_DIR` 等明确的运行预算/日志覆盖方式。`show` 显示生效配置。

## 启动和停止顺序

`run`：逐轴测试记录检查 → 模型服务 → 真实观测 dry-run → 检查训练起始位姿 →
必要时 home → 完整执行动作块，默认每块 15 步、最多 60 块/900 步。

```bash
./run_eef_task.sh task1 act-v2 stop       # 停服务后，夹爪 homing + 机械臂回公共初始姿态
./run_eef_task.sh task1 act-v2 stop-only  # 只停止，不执行任何复位运动
```

`stop` 先停止 EEF 策略适配器，再停止 direct/Servo 控制栈、相机和所选模型服务，等待相关
容器退出后才调用 `fci_init.sh`。任何停止步骤失败时，不启动自动初始化。
FCI 回位目标是 `config/default_initial_pose.json`，与各任务的 EEF 训练起始位姿分别维护。
只有显式 `stop` 会调用 FCI 初始化；任务正常结束或故障退出不会自动移动机械臂回位。

同一服务器端口可能对应不同模型，例如 63:8095 的 `act-v2` 与 `act-1k`。
切换前先用原模型的 `stop` 或 `stop-only` 关闭旧服务。复用服务时，原适配器仍会核对权重、
ActionCodec、状态统计、图像尺寸和旋转协议，发现不匹配就拒绝执行。

日志默认保存在 `$FRANKA_LOG_ROOT/eef_<任务key>_<模型key>_<时间>_<pid>/`，
其中 `selected_profile.json` 留存该次实际配置，`dry_run/`、`home/`、`execute/` 分别记录各阶段。
本次通用化沿用现有 EEF 夹爪状态定义与保护；此前夹爪 reinit 后反馈不稳定的问题仍需独立排查。
