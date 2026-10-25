# PI05 direct 真机成功验证记录

用户现场确认：task1、task2、task6 均已成功。
本目录保存确认前最近四次运行的完整原始日志，保留时间戳目录名，避免误删或错误归属。
三个任务与具体运行时间的对应关系尚待现场确认，不根据运行顺序猜测。

代码归档基线：`6ab6e8b`（实际运行期间这些改动可能尚未提交）。
日志时间：Asia/Shanghai，2026-10-25。

| 原始日志目录 | 最后记录的动作序号 | 日志观察 |
| --- | ---: | --- |
| pi05_direct_20261025_215218_97920 | 208 | 闭合、打开成功；最后为 chunk_stop |
| pi05_direct_20261025_215426_100433 | 225 | 一次抓取未成功后重新规划；记录后续 OPEN 请求 |
| pi05_direct_20261025_215656_103064 | 282 | 闭合、打开成功；记录后续 CLOSE 请求 |
| pi05_direct_20261025_215931_106058 | 253 | 一次抓取未成功后重新规划；记录后续 OPEN 请求 |

四次 adapter/executor 日志均未发现 `FAULT LATCHED`。executor 均记录 SIGINT/SIGTERM，policy.jsonl
没有 `complete` 事件，因此成功结论来自用户现场确认，并非程序自动判断任务完成。
未抓取成功的记录不代表物体一定没有被夹住；它只表示驱动未报告满足 Grasp 成功条件。

每个 `.tar.gz` 完整包含该次的 adapter.log、robot.log、executor.csv、policy.jsonl、
full_dry_run.log、full_dry_run_policy.jsonl、build.log 和 ros/ 节点日志。
原始本地 logs/ 未修改。文件未记录完整任务指令和权重元数据，不能仅凭日志独立确认具体任务与权重。

解压示例：

```bash
tar -xzf pi05_direct_20261025_215218_97920.tar.gz
```

任务入口：task1.sh、task2.sh、task6.sh。它们的运行配置可在上述代码基线中复现。
