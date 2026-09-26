# 单前雷达点时序与去畸变链离线核查

日期：2026-09-20。对象为 `internal_device_paired_front_180s_20260919`；只读原包 5 对点云（前、后各 5 条），复用已缓存前雷达 IMU 和已完成运行的快照、1641 行前端状态。没有启动 ROS、重新回放、运行 SLAM 或修改生产配置。后雷达消息仅用于重建既有 adapter 的时间基准，后点没有进入本组 Faster-LIO。

## 结论

**有限抽样没有发现点时间单位、首尾时间颠倒或 float 精度导致的明显去畸变错误；确认存在原始 IMU 时间缺口，旧去畸变链会跨缺口积分，极端帧尾需要约 95 ms 外推。** 这是真实的数据连续性风险，但不是“已证实它单独导致整段地图下沉”。物理采样/滤波延迟仍未标定，配置中的 0 ms 不能视作测量结论。

优先级：

1. **先处理/隔离 IMU 连续性风险**。本组正常间隔约 5 ms，但处理窗口存在 78 处大于 50 ms 的源 IMU 间隔，75/1641 个输出扫描区间与这些间隔相交。跨缺口积分和尾端外推都会降低运动补偿可信度；纯粹“等到 IMU 时间超过帧尾”不足以保证中间连续。
2. **保留时间单位链，不凭下沉现象盲改秒/毫秒或固定 offset**。五个实际扫描的重建证据一致支持现有单位和首尾语义。
3. **把物理时间标定列为未解决项**。当前 native front IMU 的零时移只是名义设置，不能套用中心 IMU 的 −13 ms 相对相位估计。

## 实际时序链

| 环节 | 实际解释与验证 |
| --- | --- |
| 原始 `/front_lidar` | `timestamp` 为 FLOAT64 绝对秒；五帧中 header 等于最早点时间，单帧跨度约 100 ms；原始数组无逆序时间。 |
| 双雷达 adapter | `decodePointTime` 命中绝对秒分支；范围筛选、静态坐标旋转后按时间排序。输出 header 为合并有效点中最早点，`time` 为 FLOAT32 相对毫秒。没有额外运动去畸变。 |
| `paired_front` 子集 | 只保留 `ring < 96`，逐点字段和 header 不改。前点相对时间可从 0.001–0.008 ms 而非零开始，因为时间基准偶尔来自稍早的后雷达点；这是合法的公共基准，不是第二次时移。 |
| Faster-LIO VELO32 路径 | `time_scale=1.0`，`curvature=time×1` 保存毫秒；末点时间大于零，使用输入时间而非角度推算。同步/去畸变时再除以 1000 恢复秒。 |
| 扫描末端 | `scan_end = header + last_retained_point.curvature/1000`；五帧按实际盲区筛选后重建，和真实前端 CSV 末端相差不超过 4.1 µs。 |
| IMU | `/front_lidar/imu` → 独立单位适配器 → `/d1max/slam/imu`；时间偏移 0，`time_sync_en=false`。加速度先乘 9.80665，角速度乘 1，无中心 IMU 的 −13 ms 移位。 |

五帧全部满足：重建 adapter header 与实际 `paired_scans.csv` 精确相等、合并及筛出的前点数量一致；相对时间非负有序，重建绝对点时间与源 timestamp 在 FLOAT64 表达精度内一致。没有录制 adapter 中间消息，故这是多项输出交叉验证，**不是逐字节中间消息抓包**。

### 精度不是当前明显瓶颈

- 当前 epoch 下 FLOAT64 绝对秒 ULP 约 238.42 ns。
- 相对毫秒转 FLOAT32 的最大舍入误差约 3.815 ns；这远小于源绝对时间本身的表示步距。
- 前端 CSV 的绝对秒日志保留约 15 位有效数字，epoch 量级下仅数微秒分辨率。因此上述 ≤4.1 µs 的末端差异不能判作软件时延。

## 已定位的 IMU 缺口风险

| 指标 | 结果 |
| --- | ---: |
| 已保存输出扫描 | 1641 |
| 扫描尾端距前一个源 IMU：中位 / P95 / 最大 | 3.643 / 4.857 / **94.681 ms** |
| 跨越扫描尾端的相邻源 IMU 最大间隔 | **105.016 ms** |
| 尾端前 IMU 旧于 10 / 50 ms 的扫描 | 2 / 1 |
| 输出窗口内 >30 ms 源缺口 / 相交扫描 | 105 / 99 |
| 输出窗口内 >50 ms 源缺口 / 相交扫描 | **78 / 75** |
| 实际单位适配器接收 / 发布 | 34915 / 34915 |

最极端是状态第 472 行（相对第一条状态 **52.50003 s**）：源前雷达扫描区间约 `[1772316495.899979, 1772316495.999987]`，约 100 ms 内只有 **2 个 IMU 样本**，尾端前最后样本已旧 94.681 ms。五个检查帧的样本数依次为 20、20、20、10、2。

区间相交统计使用保存的真实 admitted cloud header 与对应状态扫描末端，1641 个映射唯一，实际区间长度 99.952–100.033 ms；不是全部用 `end−0.1s` 虚构起点。适配器完整输入窗口另含初始化期，报告的 >30 ms 缺口为 106，与状态输出窗口的 105 不矛盾。

旧同步代码等待最新 IMU 时间达到扫描末端，但本扫描仅取 `stamp <= scan_end` 的样本，不包含第一个末端之后的样本。`ImuProcess` 用相邻样本的均值和实际时间差传播，然后从最后一个已取样本预测到扫描末端。所以：

- 缺口在扫描内部时，仍会使用跨度很大的相邻样本均值积分；正常 200 Hz 的运动细节已经缺失。
- 缺口跨扫描末端时，等待未来样本抵达并不能消除最后的长外推。
- 这里的 94.681 ms 来自源缓存和实际扫描末端，不是完整的回调消费跟踪。若回调还发生额外延迟/丢失，不能由这份源覆盖统计排除。

**不能把“75 帧相交缺口”直接等同于 75 帧已经匹配失败，也不能把一次 95 ms 外推当作 0.67 m 下沉的唯一因果证据。** 需要结合状态创新/残差和动态区间定位因果；本轮不做新回放。

### 最坏尾端缺口附近的现成状态

查看该帧前后约 1 s 的 19 条已保存更新后状态，没有看到与“95 ms 外推”对应的巨大瞬时跳变：

| 相对该帧 | 估计重力相对世界向下的倾角 | IMU 原点 Z | 匹配残差 RMSE |
| --- | ---: | ---: | ---: |
| −1.000 s | 0.27534° | −0.135867 m | 0.022627 m |
| 0 s | 0.27352° | −0.140620 m | 0.022690 m |
| +1.000 s | 0.27342° | −0.142300 m | 0.023602 m |

这个约 2 s 窗口的估计高度变化是 **−6.43 mm**，重力向量方向变化约 **0.001946°**；残差范围 0.022449–0.023602 m。当帧相对前一个状态的 Z 改变量约 −0.497 mm。它表明这一次极端输入缺口在现成更新后日志中**没有形成明显突变**，因此不能凭该事件认定全程缓慢下沉的根因已经查明。上述均是估计状态，不是独立地面高度 GT；匹配后的平滑也不能证明缺口没有危害。

## 固定时间偏移与时钟边界

本组 `timestamp_offset_sec=0.0`、前端 `time_sync_en=false`，软件链没有偷偷施加中心 IMU 时移。原始点时间和 IMU header 在同一个 sensor-time 域内对齐；bag 记录接收时间属于另一时间域，不能直接相减当成物理采样延迟。

缺少专用动态时间标定，尚不能证明激光发射时间和 IMU 内部采样/数字滤波输出不存在固定延迟。雷达与其内置 IMU 使用关联时钟，也不等于二者物理延迟恰好为零。中心↔前内置 IMU 的相对相位估计只约束那两路信号，不能自动证明激光点与 IMU 的相位。

## 运行版本与源码边界

真实运行的 `/proc/PID/maps` 快照记录：

- `libfaster_lio_lib.so` SHA256：`8f7a66b4a545a24fce4ba36cbc9764e16e8a2b9f5361833a376738831111f069`。
- `run_mapping_online` SHA256：`90005feb780bdf79c20f8e1090e59c834ddc42ed96891e9ce9f4543876bdd762`。

当前工作区后来加入的点时间 guard/预处理排序、`mapping_imu_guard` 不属于本次已加载默认库，**不能把未编译源码说成实际保护已经启用**。本审计按旧版预处理“末点时间为正则采用输入时间”的分支重建；adapter 已排序，子集保持顺序，因此抽样帧即使没有新增预处理排序也保持正确。adapter 当前与旧版的差异为后加单雷达模式，原来的双雷达时间解码/排序/时间写出路径未改，且重建结果与实际快照相符。

加速度常数特别更正：`common::G_m_s2=9.81`，`imu_processing.hpp` 用 **9.81/初始化均值模长** 归一化；前内置初始化模长约 9.82997769 m/s²，对应系数约 0.99796768。`use-ikfom.hpp` 的 S2 重力状态模长才是 **9.809**，不是归一化分子。不能混用，也没有证据说明两者 0.001 m/s² 的差异就是此次下沉根因。

## 复现与证据入口

- 数值结果：[timing_audit.json](/home/dndx/d1max_nav_ws/experiments/lio_frontend_reliability_20260919/timing_bias_geometry_20260920/timing_audit.json)。
- 有界只读脚本：[audit_point_timing.py](/home/dndx/d1max_nav_ws/experiments/lio_frontend_reliability_20260919/timing_bias_geometry_20260920/audit_point_timing.py)。只需 ROS Python 消息类型环境，不调用 `rclpy.init`；输出采用独占新建，拒绝覆盖既有 JSON。
- 实际参数：[frontend.yaml](/home/dndx/d1max_nav_ws/experiments/lio_frontend_reliability_20260919/runs/internal_device_paired_front_180s_20260919/config/frontend.yaml)、[calibration.yaml](/home/dndx/d1max_nav_ws/experiments/lio_frontend_reliability_20260919/runs/internal_device_paired_front_180s_20260919/config/calibration.yaml)。
- 实际运行记录：[central_imu_adapter.json](/home/dndx/d1max_nav_ws/experiments/lio_frontend_reliability_20260919/runs/internal_device_paired_front_180s_20260919/central_imu_adapter.json)、[runtime_binary_check.json](/home/dndx/d1max_nav_ws/experiments/lio_frontend_reliability_20260919/runs/internal_device_paired_front_180s_20260919/runtime_binary_check.json)、[frontend_state.csv](/home/dndx/d1max_nav_ws/experiments/lio_frontend_reliability_20260919/runs/internal_device_paired_front_180s_20260919/frontend_state.csv)。
- 源码相关位置：[adapter 时间解释](/home/dndx/d1max_nav_ws/src/d1max_slam/src/dual_lidar_adapter.cpp:194)、[同步基本路径](/home/dndx/d1max_nav_ws/src/faster_lio/src/laser_mapping.cc:788)、[积分/尾端外推](/home/dndx/d1max_nav_ws/src/faster_lio/include/imu_processing.hpp:234)。这些链接指当前工作区，阅读时必须保留上面的已编译版本边界。

没有引入平地 GT、地面锁高、回环或更改外参来掩盖本次问题。有限点云抽样不能排除全包其他帧的坏时间字段；源 IMU 缺口是在已有完整缓存和真实运行统计中确认的。
