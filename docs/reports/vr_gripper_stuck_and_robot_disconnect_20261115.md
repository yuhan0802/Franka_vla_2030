# VR 夹爪卡死与 `run_robot.sh` 断开排查（2026-11-15）

## 结论

本次夹爪“夹住后不能张开、之后也不能再夹”的直接原因已确认：旧机器人栈断开时，
VR 节点有一个 `CLOSE` action 一直没有收到结果，导致
`start_franka_vr.py` 的 `gripper_goal_in_flight` 永久保持为 `True`。机器人栈重启后
VR 节点没有重启，因此后续 OPEN/CLOSE 都被本地状态机静默跳过。

本次检查时新启动的 `run_robot.sh` 栈正常：机械臂与夹爪均连接成功，控制器激活，
Servo 报 `No warnings`，日志中没有 `communication_constraints_violation`、反射或进程退出。
旧栈已经退出且容器被 `--rm` 删除，现存日志不足以判定它此前为什么断开。

## 证据

- VR 容器启动时间早于当前机器人容器，并跨越了一次机器人栈重启。
- VR 日志中 2026-11-15 12:04:28（Asia/Shanghai）出现
  `Sending gripper CLOSE: width=0.010 m`，之后没有对应的 completed、failed、rejected
  或 result unavailable；此前三组 OPEN/CLOSE 都有成功结果。
- 当前机器人容器于 12:06:18 才启动，说明这个未完成 action 属于旧机器人栈。
- `start_franka_vr.py::_send_gripper_action()` 在发送时把
  `gripper_goal_in_flight=True`；只有 goal response/result 回调会清零。action server 消失时
  没有 watchdog/超时，也没有在新 action server 出现后重置。
- 按钮处理同时要求 `not self.gripper_goal_in_flight`，所以该状态卡住后 OPEN 和 CLOSE
  都不会再次发送，也不会打印错误。

## 当前主机与 FCI 状态

- 内核：`6.8.1-1015-realtime`，`/sys/kernel/realtime=1`。
- 隔离 CPU：`2,4-5`；机器人容器绑定 CPU `4,5`。
- `enp110s0`：1000 Mb/s、Full Duplex、Link detected yes。
- 到机器人路由：`172.16.0.2 dev enp110s0 src 172.16.0.1`。

这排除了仓库 2026-09-04 历史问题中的“误启动通用内核 + 100 Mb/s 链路”作为当前
环境的直接解释，但不能排除旧栈曾发生瞬时通信抖动、Desk 反射或人为终止。

## 现场恢复与复现建议

1. 先退出当前 VR 节点，再在机器人栈稳定后重新启动 VR；不要让 VR 节点跨机器人栈重启。
2. 重新启动后先在无物体、手远离夹爪时验证一次开/合；这是现场动作，由操作者执行。
3. 若 `run_robot.sh` 再断开，保留其前台终端最后 100 行，重点找
   `communication_constraints_violation`、`motion aborted by reflex`、
   `process has died` 和 `franka_gripper_node` 异常。不要只重启后再排查。

## 建议代码修复

给 VR 夹爪 action 增加有限超时/watchdog：超时后记录错误并清除 in-flight 状态；同时用
实测夹爪宽度重新同步 `gripper_state`。还应在 action server 消失/重现时使旧 goal 失效，
避免旧回调晚到后覆盖新状态。该改动需要单元测试覆盖“发送后 server 消失”和“机器人栈
重启后恢复开合”。本次只做诊断，未修改控制代码。
