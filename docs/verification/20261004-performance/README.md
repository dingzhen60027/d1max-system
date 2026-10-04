# 导航性能优化验证

2026-10-04，相同输入对比发布基线 `b344f52501851a109c5b55220e7f7b75fd638962` 与性能整改源码。这些记录来自开发机隔离验证，保留实际源码和 ELF 身份；不是新电脑已经完成构建、部署或物理验收的报告。

| 证据 | 内容 |
| --- | --- |
| `native_summary.json` | 三槽缓冲省 10.55 MiB、快照/精确缓存计时、325 个原生用例与四包一致重建 |
| `native_results.json` 和 `native_timing_results.json` | 计数与无 allocator hook 计时分开，原始对照与 checksum |
| `tracker_summary.json` | 同输入曲线输出、热循环零分配、160 个用例；准备几何和 RSS 增加的取舍 |
| `sdk_buffer_benchmark.json` 和 `sdk_performance.md` | MC timing ring、有界 ACK、周期发布，14 组 SDK 逻辑测试；无实际 SDK 连接 |
| `owner_results.json` | 不可变路线和受控 50 ms 剩余帧延迟；不是实测 Zenoh 网络延迟 |
| `python-final-regression.xml` | 2033 通过、1 项大型地图缺失跳过 |
| `owner-final-regression.xml` | 最终通信/参考边界 128 通过，含重复取消 FD/线程退役；与完整回归有重叠，不累加 |
| `publication-owner-regression.xml` | 发布副本重新执行四份通信/参考测试，101 通过，未初始化 ROS 图 |
| `publication-owner-results.json` | 发布副本按上述命令再次对照，geometry/source 输出和完整 packet 相同；时延另行记录，不覆盖开发机计时 |
| `baseline_reference.py` | owner 基准冻结源文件，SHA256 见 owner_results，不是运行模块 |

原始报告中的路径属于开发机，部分复现脚本与构建树没有发布。发布副本可直接重现 owner 对照：

```bash
cd d1max_nav_ws
PYTHONDONTWRITEBYTECODE=1 PYTHONPATH=src/d1max_pct_scan /usr/bin/python3 \
  tools/validation/benchmark_navigation_owner.py \
  --baseline ../docs/verification/20261004-performance/baseline_reference.py \
  --output /tmp/d1max-owner-results.json
```

原生性能探针、跟踪器用例和 SDK 探针源码位于相应 production package 的 test/src；目标机完整构建步骤见 [部署说明](../../DEPLOYMENT.md)。完整服务内存、20 ms 整链截止、NUC 压力、厂商调用和实际制动仍未验证。安全、授权、源时间、默认 release 和物理验收标志保持原值。
