# 9 月 17 日真实 bag 定位回归

本目录是隔离诊断工具，不是实机启动入口，也不是 Nav2 软件运动仿真。
使用现有 Faster-LIO + PCD + 高频输出模块；唯一单独入口是 bag 没有 SDK 机头状态时的一次初值接纳。
不会伪造 SDK、发布速度命令、修改源 bag/地图或自动放宽定位门限。

`run_replay.py` 当前锁定已经核实来源的 bag917 和地图 `grid-0af429985e454a9e99c8aaef`。
更换数据必须重新核对地图坐标、起点和时间轴，不能照搬零位姿。

加载项目 ROS 环境和 nav 工作区后，确认没有其他回放使用 17449，指定一个尚不存在的结果目录：

```bash
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
python3 experiments/localization_replay/run_replay.py \
  --bag /home/dndx/d1max_nav_ws/bags/slam_raw_20260917_171716_fe8f38 \
  --output /home/dndx/d1max_nav_ws/log/localization_replay/NEW_UNIQUE_RUN \
  --duration 1000
```

只回放前后雷达与前雷达 IMU，保持 1x 和真实采样间隔；不重放厂商 TF 或 `/clock`。
沿用单常量时间平移以兼容生产节点的墙钟鲜度保护，不表示硬件时钟标定。
地图/初值采用 tracking 参考点；后续由真实匹配建立 map→odom，不固定为恒等变换。

超过 90 秒尚未首次定位会结束；SIGINT 会清理自身进程。失败不自动循环回放或修改参数。
需要显示实时测试时使用其 RViz 窗口；不要同时展示软件仿真并将其误认为录制数据定位。

`observe_replay.py` 为独立只读观测器，保存状态/诊断 JSONL、位姿 CSV、带失效时间计权的统计；
没有地面真值时不会报告定位精度。配准 RMSE、局部低速区间位移都不能当作绝对精度。

本次报告：`log/localization_replay/20260918_1525_bag917_baseline/REPORT.md`。
测试结论：未通过，原 bag 的 IMU 缺口触发连续 LIO 重置和恢复上限，没有有效全局定位。
