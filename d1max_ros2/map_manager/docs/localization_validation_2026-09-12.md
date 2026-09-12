# 定位后端修改与离线验收 · 2026-09-12

当前默认：Faster-LIO 局部激光惯性里程计 + 已确认 PCD 全局 SE(3) 校正。
Web 启停/初值与 Foxglove 布局保留；不发送任何机器狗运动命令。
配置：`/home/dndx/d1max_nav_ws/src/d1max_localization/config/localization.yaml`。
架构与操作说明：`/home/dndx/d1max_nav_ws/src/d1max_localization/ROBUST_LOCALIZATION.md`。

## 已验证

| 检查 | 结果 |
|---|---|
| ROS 后端构建安装 | Faster-LIO / d1max_localization 均通过 |
| Python 定位单元测试 | 107 项通过 |
| C++ 时间、去畸变、匹配门控、可观测性 | 4 个测试目标，28 项通过 |
| Web mock 生命周期 | 9 项通过；未访问实际控制接口 |
| 前端构建 / 初值数学测试 | 通过 |
| Web 浏览器隔离回归 | 8 类检查通过；含 LIO/恢复/故障状态、5 种视口无横向溢出；无页面异常或未拦截请求 |
| 新后端端到端合成回归 | 通过：静止、运动、匹配暂停/恢复轮次、后雷达缺帧、IMU 长断流 |
| 旧 legacy_ekf 静态全链路 | 通过，保留显式回退 |

## 最终运动合成结果（不是实机精度）

场景为无真实动态遮挡/测量噪声的非对称合成房间。每个点使用自身采样时刻的真实模拟位姿，
并生成对应加速度、角速度；不是加快静态 bag 播放。

- 峰值线速度 2.0616 m/s；峰值 yaw 角速度 1 rad/s。
- 最大位置误差 0.01647 m；位置 RMS 0.00663 m；最大姿态误差 0.128°。
- 最大有效输出间隔 0.1018 s。
- 故意注入 0.035 m/s 的静止 MC 速度偏差，位置跨度约 0.000054 m；新后端不积分该 MC 偏差。
- 暂停匹配进程后撤销 localized；恢复时初值轮次相同且至少 3 次连续确认，才恢复 tracking。
- 后雷达停止发布时，前雷达降级保持定位。
- 约 123 ms IMU 缺口触发 fault / localized=false，之后不继续发布有效轨迹。

完整报告：`/tmp/d1max-localization-smoke-mbu6dpye/report.json`。
旧后端报告：`/tmp/d1max-localization-smoke-01dr20ir/report.json`。

## 原始 rosbag 兼容性（没有位置真值）

读取 `/home/dndx/d1max_rosbag903/slam_raw_20260903_232807` 的前 20 秒，仅 adapter + LIO。
源 IMU 平均约 197.99 Hz，确有 39.999 ms 源时间缺口；后雷达最长缺帧约 0.50 s，前雷达约 0.20 s。
因此同时处理了短 IMU 缺口与后雷达缺帧对前雷达配对的阻塞，而不是仅放宽 ICP 门限。

最终输出约 9.60 Hz，最大里程计间隔 0.2000 s，前雷达降级尝试 28 次，
IMU 硬缺口拒绝 0，LIO 待处理扫描替换 0，结束时 ready=true / fault=false。
这些结果只说明本次输入兼容与连续性，不能证明整包全局定位精度或机器狗的实际高速能力。

完整报告：`/tmp/d1max-lio-bag-check-g4j3cxhf/report.json`。

## 仍需实机验收

机身/雷达/IMU 外参、时间同步、长时间静止、急加减速/转弯、振动、走廊退化和动态人群尚未完成真实运动验收。
不能把上述合成 2.06 m/s 当成已经认证的实机速度上限；navigation_ready 继续为 false。
本次没有启动实机定位服务。下一次 Web 启动采用新配置，仍需静止初始化和正确地图初值。
所有隔离回归使用独立本机 Zenoh 域，测试节点与端口已清理；原始地图和 rosbag 未修改。
