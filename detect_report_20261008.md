# FrankaVR VLA 检测报告

检测时间：2026-10-08 11:08（Asia/Shanghai）

结论：检测脚本的软件缺陷已修复并完成真实只读运行，但当前现场环境不满足通过条件，结果为 **FAIL/BLOCKED**，禁止进入 `execute`。检测过程中没有调用 `/set_target_pose`、没有发布运动目标、没有执行模型 action，也没有启动机器人控制栈。

## 九项检测结果

| 项目 | 结果 | 证据 | 备注 |
| --- | --- | --- | --- |
| 1. D435i 正视角 RGB | FAIL | `/camera/camera_front/color/image_raw` 在 15 秒内无消息 | 相机 ROS 栈未运行；USB 设备本身存在 |
| 2. C922 左、右视角 RGB | FAIL / BLOCKED | `/camera_left/image_raw`、`/camera_right/image_raw` 在 15 秒内无消息；配置序列号 left=`43DDCA6F`、right=`CD7DCA6F` | 两台 USB 设备存在，但 ROS 栈未运行；左右物理位置未经操作者确认 |
| 3. 关节状态与 TF | FAIL | 缺少 `fr3_joint1..fr3_joint7`；未能取得两次递增时间戳的 `world -> fr3_link8` | 真机状态栈未运行 |
| 4. AI 模型通信 | FAIL | `10.24.116.20:8093` TCP 可达；SSH 检查未发现 `deploy_franka_vr_qwen_actioncodec.py --port 8093` 进程及对应监听 | 正确拒绝将 TCP 可达视为模型服务通过 |
| 5. 控制通信 | FAIL | `enp110s0` 存在但 link=no；到 `172.16.0.2` 的路由走 `Meta`，机器人从 FCI 网口不可达；controller manager、`/set_target_pose` service 和两个所需 controller 均不可用 | `/set_target_pose` 经实际代码确认是 `franka_vr/srv/SetTargetPose` service，不是 topic；检测中未调用 |
| 6. 单一真实控制栈 | FAIL | 无 Gazebo/fake/mock 节点或容器，但也没有唯一的真机 `controller_manager` | 未发现冲突栈；当前是“零个真机栈”，不能判 PASS |
| 7. FCI 专用网口与实时状态 | FAIL | `enp110s0` link=no，speed/duplex unknown；路由错误；`/sys/kernel/realtime=1` | PREEMPT_RT 为 PASS，其余关键网络条件失败 |
| 8. 模型 I/O 与安全限制 | BLOCKED / PASS | 因图像、关节、TF 门禁失败，未发送模型请求；工作区及代码安全限制静态检查通过 | 未获得 action，因此 dtype/shape/finite 为 N/A；这是预期安全行为 |
| 9. 运动方向与现场安全 | BLOCKED | 未设置任何 `CONFIRM_*`；无现场操作者确认 | xyz/rpy、实测边界、急停、清场、低速空载、碰撞/奇异性保护均须现场确认 |

## 模型 I/O 结果

本次未执行推理。原因是三路图像、七关节状态和持续 TF 没有通过前置门禁。因而本次没有可报告的真实 action dtype、shape、min/max；脚本明确报告 `BLOCKED`，没有用 TCP 探测伪装成推理成功。

新增的只读探针在条件满足时会严格验证：

- metadata `model_family=qwen3vl2b_franka_vr_actionfix_v2`；
- metadata `observation_format=franka_vr_front_left_right_and_joint_state_v1`；
- 响应 action 必须是原生 `numpy.ndarray`、`dtype=float32`、`shape=[15,7]` 且全部有限；
- 只打印统计信息，不导入或调用 `SetTargetPose`，也不发送夹爪命令。

## 工作区与保护配置

来源：`workspace/src/franka_vr/franka_vr/config/vla_calibration.yaml`

- `min = [0.383737, -0.120000, 0.182611] m`
- `max = [0.750000, 0.120000, 0.786648] m`
- 批准的 X 上限：`0.75 m`（未放宽）
- 桥接器已检查工作区拒绝逻辑、平移单步 `np.clip`、旋转单步 `np.clip` 和 dry-run/execute 门禁。
- Servo 配置 `check_collisions=true`，奇异性阈值为 `10.0 / 30.0`。

## 实际配置与命令

- 正视角：`/camera/camera_front/color/image_raw`
- 左视角：`/camera_left/image_raw`
- 右视角：`/camera_right/image_raw`
- 机器人：`172.16.0.2`
- FCI 网口：`enp110s0`
- 模型：`10.24.116.20:8093`

执行命令：

```bash
cd /home/mac/franka_vr_docker
bash -n ./detect.sh
python3 -m py_compile ./scripts/detect_vla_model_io.py
bash ./detect.sh --help
bash ./detect.sh 2>&1 | tee detect_20261008_110848.log
```

`shellcheck` 未安装，因此本次未执行该工具；`bash -n`、Python 编译、容器内依赖加载及 `--help` 均通过。真实检测退出状态为 1（关键条件失败）。

## 修改内容

- `detect.sh`：改用实际话题默认值；宿主机无 ROS 时自动使用已有 ROS 镜像；修正 `/set_target_pose` service 检查；严格检查所需 controller、单一控制栈、FCI 路由/链路、正确远端进程；增加 BLOCKED 语义和现场确认门禁；自动调用只读模型探针；加强工作区与 Servo 保护检查。
- `scripts/detect_vla_model_io.py`：新增三路新鲜 RGB、七关节/8 维状态、持续 TF 和一次真实 WebSocket/MessagePack 推理探针，严格验证模型身份与有限 `float32[15,7]`。
- `detect_20261008_110848.log`：保留本次真实检测完整日志。

## 下一步判断

当前不建议进入 `dry-run`。需要先由现场人员恢复 `enp110s0` 链路及机器人专用路由，按既有安全入口启动唯一真机状态栈和三相机栈，启动并核实远端 FrankaVR 8093 服务，再重跑检测。即使前八项随后通过，第九项没有现场操作者确认时仍必须保持 BLOCKED。

## 11:19 开机后复测

复测日志：`detect_20261008_111929.log`

开机后 FCI 网络已经恢复：`enp110s0` 为 1000 Mb/s 全双工、链路正常，`172.16.0.2` 路由经该接口且 ping 成功，PREEMPT_RT 正常。本地 `configure_fci_realtime_host.sh` 原先既不能在无交互 sudo 环境运行，也无法在已有 `isolcpus` 时补写缺失的 `irqaffinity`；现已修复为幂等更新，并通过项目已有的特权容器机制成功执行。GRUB 当前包含：

```text
isolcpus=4,5 nohz_full=4,5 rcu_nocbs=4,5 irqaffinity=0-3,6-31
```

`irqaffinity` 将在下一次重启后生效；当前启动已经具备 CPU 4-5 隔离，且 non-FCI IRQ 隔离和禁用 EEE 的 systemd 服务均为 active。

复测仍失败的项目缩小为：真机 ROS 控制栈未启动、三相机 ROS 栈未启动、远端正确的 8093 FrankaVR 模型服务未启动，以及现场物理安全确认未完成。复测统计为 PASS=15、FAIL=7、WARN=1、BLOCKED=5，退出状态 1。由于传感器门禁失败，仍未发送模型推理请求或执行 action。

启动授权后进一步核对发现，本次启动的 `/proc/cmdline` 尚未包含新写入的 `irqaffinity`。NVMe IRQ 171、172 的 effective affinity 分别仍为 CPU 4、5，会与 FR3 的实时控制线程竞争；FCI IRQ 141 则已正确固定到 CPU 2。因此没有启动真机控制栈，必须重启应用新 GRUB 参数后再继续。`detect.sh` 已增加逐 IRQ 检查，今后会把此情况作为 FAIL，而不是只检查 PREEMPT_RT 和 CPU isolation。

远端 8093 FrankaVR 进程已经出现并正在加载权重，但尚未监听 8093；GPU 3 同时存在约 23 GB 的训练任务占用。只有监听建立且应用层 metadata/真实推理探针通过后，才会判定模型服务正常。

## 11:33–11:35 启动与故障定位

- 8093 服务现已监听，WebSocket metadata 实测为 `qwen3vl2b_franka_vr_actionfix_v2`、`franka_vr_front_left_right_and_joint_state_v1`、action shape `[15,7]`，应用层身份通过。
- 真机栈能够连接 FR3、激活 hardware interface 并启动 1000 Hz controller manager，但 `fr3_arm_controller` 切换扭矩模式时被机器人以 `User stopped` 拒绝。launch 随后清理退出，没有留下活动 controller manager。必须由现场操作者在 Franka Desk/操作面板解除 User Stop，不能由检测脚本绕过。
- 初始相机配置下，右 C922 以 `Protocol error (71)` 启流失败，原因是两路未压缩 640×480@30 YUYV 加 D435i 的共享 USB 带宽不足。当前 ROS `v4l2_camera` 不支持 C922 MJPEG 解码，因此最终修复为两台 C922 使用 640×480@15 YUYV 并输出 RGB8。复测三路均收到新鲜图像：front age 0.050 s、left 0.003 s、right 0.071 s。
- `irqaffinity` 已生效，但 NVMe 的 managed MSI-X IRQ 154/155 仍实际运行在 CPU 4/5；启动编译期间分别累计 808/330 次中断。GRUB 已进一步修正为 `isolcpus=domain,managed_irq,4,5`。该修复仍需下一次重启才能生效，重启后必须再次核验 effective affinity。
- 最新完整检测日志为 `detect_20261008_113300.log`。当时结果 PASS=18、FAIL=6、WARN=1、BLOCKED=5；右相机修复发生在该日志之后，并已用独立只读探针验证三路图像。

## 11:39–11:43 第二次重启后复测

- 内核命令行已包含 `isolcpus=domain,managed_irq,4,5`，PREEMPT_RT、CPU 4/5 isolation、IRQ 隔离服务和 EEE 服务均正常。
- NVMe per-CPU managed IRQ 170/171 的 effective affinity 仍显示 4/5，这是该类单 CPU mask 的内核 best-effort 限制；启动前实际计数均为 0。FCI IRQ 165 在启动脚本中成功固定到 CPU 2。该项保留为已知风险，不应伪报完全消除。
- 三路相机以修复后的配置启动成功，D435i 为 640×480@30 RGB8，两台 C922 为 640×480@15 RGB8。
- dry-run 桥成功读取并验证 8093 metadata，日志明确为 `mode=DRY RUN`；没有 `--execute`，没有发送目标。
- 真机硬件接口再次成功连接 `172.16.0.2`、激活并建立 1000 Hz FIFO RT controller manager，但 `fr3_arm_controller` 切换扭矩控制时仍由 FR3 返回 `Move command rejected ... ("User stopped")`。完整日志：`run_robot_attempt_20261008_1142.log`。
- launch 按 required-process 逻辑清理退出，未留下 controller manager。多余 dry-run 桥已通过容器内 SIGINT 干净退出；相机栈继续运行，便于解除 User Stop 后复测。

## 11:53 解锁后最终复测

最终日志：`detect_20261008_115306.log`

- 统计：`PASS=25`、`FAIL=0`、`WARN=1`、`BLOCKED=4`；退出状态为 2，仅因为现场操作者确认项未设置，不是自动检测失败。
- FR3 解锁后真机控制栈成功启动：唯一 `controller_manager`，`joint_state_broadcaster` 和 `fr3_arm_controller` 均为 active，没有 Gazebo/fake/mock 冲突栈。
- `enp110s0` 路由、1000 Mb/s 全双工、FR3 可达、PREEMPT_RT 和 FCI IRQ 165→CPU2 均通过。唯一 WARN 是 NVMe managed IRQ effective mask 仍显示 CPU4/5，但 2 秒采样增量均为 0，保留为已知风险。
- 三路图像均为新鲜 RGB8 640×480；七个 FR3 关节、8 维观测和持续更新的 `world -> fr3_link8` TF 通过。
- 8093 服务元数据与预期完全一致。真实推理返回原生 `float32[15,7]`，全部有限，范围 `[-0.1115998, 0.08527353]`。
- 工作区、平移/旋转裁剪、dry-run 门禁、Servo 碰撞和奇异性保护均通过。
- 检测只读取状态并发送了一次模型推理请求；未调用 `/set_target_pose`，未执行 action，未发送夹爪命令。

仍需现场操作者明确确认四类物理项：C922 左右对应及训练输入顺序、xyz/rpy 方向、实测工作区边界，以及急停/清场/低速空载/碰撞与奇异性保护。在这些确认完成前，不应进入 execute。
