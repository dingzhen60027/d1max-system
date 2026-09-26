# D1 Max：LIO-SAM 隔离回放试验

日期：2026-09-20。原 Faster-LIO 下沉问题暂时保留；本试验不继续修改或替换原系统。

## 使用范围

- Bag：`/home/dndx/d1max_nav_ws/bags/slam_raw_20260917_171716_fe8f38`，约 985 s。
- 输入：当前默认 **前、后 AIRY96 点云 + 中心 IMU**；`--lidar-mode front` 保留单前雷达对照。不使用机器人定位/SDK 位姿。
- 上游：官方 [LIO-SAM ROS2 分支](https://github.com/TixiaoShan/LIO-SAM/tree/ros2)，固定 commit `08af3f32f01725372d4269838dc44c19c6d9e76b`。
- 运行：ROS2 Humble，**rmw_zenoh_cpp**，隔离 Domain 220、仅 localhost:7448，无实机发现/连接。
- 启动方式：独立脚本；**完成的地图已接入 Web 结果列表**，未替换生产建图启动选项、未修改默认 Faster-LIO 外参或二进制。

**这是“官方 LIO-SAM + 六轴输入实验适配”，不是原版九轴硬件输入，也不是一次新标定。** 当前 bag 三路 IMU 的 orientation 全程恒等，不能直接当真实姿态。适配依据和限制见 [INPUT_COMPATIBILITY.md](INPUT_COMPATIBILITY.md)。

## 做了哪些必要适配

| 项目 | 本试验处理 |
| --- | --- |
| 每点时间 | AIRY 的 FLOAT64 绝对秒转为 `FLOAT32 time` 相对秒，保留真实 ring、按点时间排序 |
| 扫描投影 | 点和状态使用 N 轴；保留各自原生 `column`、`sensor_range`，前 ring 0–95、后 ring 96–191，分别投影；不假装两个光学原点共心 |
| 双源配对 | 原始 header 差 ≤5 ms 才配对，每条输入只用一次；统一起始时间、按真实逐点秒排序。等待已发布的 IMU 覆盖扫描末尾，不按消息到达时刻伪造时间 |
| 特征边界 | 修正上游扫描线起始内区少 1 点的问题；曲率、遮挡和邻域抑制均限于本扫描线，避免前后扫描线跨界影响 |
| 输入吞吐 | IMU 与点云使用独立互斥回调组；PointCloud2 使用 typed byte array，避免 Python 对数百万字节逐项校验造成抽帧 |
| 六轴姿态 | 起始约 2 s 通过低动态门槛后做重力初始化、减去启动 gyro 均值并积分；不声称磁航向或独立姿态真值 |
| 绝对姿态反馈 | `imuRPYWeight=0`，新增 `imuIncrementalRPYWeight=0`，避免把积分姿态当独立绝对倾角再次融合 |
| 外参 | 中心 IMU 继续沿用实验候选；双雷达使用 `device_front_plus_runtime_rear.yaml` 中 9 月 17 日实机已加载的前后雷达矩阵。仅实验选择，未覆盖生产默认，也未重新标定 |
| 上游崩溃保护 | TransformFusion 清空 IMU 队列后直接读 front/back，gdb 复现后加 empty guard；不更换 GTSAM |
| TF 消息头 | 雷达与机体 frame 同名时，上游遗漏父 frame 与 stamp；已补全。完整回放运行中发现，未中断重启，因此该次地图对应的二进制仍未含此显示链路修复 |
| 保存安全 | 原服务含 shell 删除目录；隔离版仅允许创建新的绝对输出目录，不覆盖、不删除已有结果 |

上游补丁都在本目录 `ws/src/LIO-SAM`，运行快照保存 `source.patch`。没有改动工作区主 `src/faster_lio`。GTSAM 使用现有 `/opt/ros/humble` 的 4.2.0；没有全局安装/升级依赖。系统 OpenCV 与 GTSAM 链接两种 TBB 的警告仍在，本次启动崩溃的堆栈定位在空队列读取，而非据该警告猜测 ABI 错误。

## 当前结果记录

| 运行 | 用途 | 状态 |
| --- | --- | --- |
| `runs/20260920_165926_smoke60` | 首次接通 | 第 2 个位姿后上游队列读取崩溃，已清理，不是有效地图结果 |
| `runs/20260920_170050_debug_imu` | gdb 定位 | 重现相同问题；保留堆栈，已清理 |
| `runs/20260920_170210_smoke60_fixed` | 修复后 60 s 短试验，无回环 | 完成，563 个位姿，66,759 点原生特征地图，无速度/偏置重置告警 |
| `runs/20260920_170433_full_central_native_loop` | 完整包，原生回环，1×，RViz | 完成；9,341 个 mapping 位姿，402 个关键帧，1,173,224 点特征地图 |
| `runs/20260920_173923_dual_smoke60_typed_buffer` | 双雷达 60 s，2×短测 | 完成；541 个 mapping 位姿，544 对输入全部发布；后雷达点确实进入独立投影区 |
| `runs/20260920_174130_full_dual_central_native_loop` | 双雷达完整包，2×，原生回环 | 运行结果以该目录 manifest/status 为准；完整性门槛通过后自动登记 Web |

短段结果：[GlobalMap.pcd](runs/20260920_170210_smoke60_fixed/map/GlobalMap.pcd)、[summary.json](runs/20260920_170210_smoke60_fixed/summary.json)。短段 estimated ΔZ 约 −0.097 m，不是独立真值误差；不能据它宣称下沉问题修好。

### 上一轮单前雷达完整包结果

- [完整地图 GlobalMap.pcd](runs/20260920_170433_full_central_native_loop/map/GlobalMap.pcd)：1,173,224 点，18,771,776 字节；XYZ 均有限。
- [统计 summary.json](runs/20260920_170433_full_central_native_loop/summary.json)：原始 IMU 192,971 条、点云 9,364 帧均被适配器收到。起始重力初始化跳过 20 帧；投影器保留末尾 2 帧缓冲，另有 1 次等待 IMU，最终输出 9,341 帧。
- mapping 轨迹跨度 982.60 s，平均 9.51 Hz；原包约 985 s，其开头用于初始化。频率不是 IMU 预积分频率。
- ICP 接受并显示 355 条历史约束，**不是 355 次绕圈，也不保证末尾每条均已融合**。
- 最终优化关键帧的估计 Z 范围为 **−1.468～+1.570 m**，跨度约 **3.04 m**。这是估计量而非独立真值误差，但不能据本次结果宣称高程漂移已解决。
- 在线 mapping 轨迹最大相邻位置变化约 **6.71 m**，包含末段回环校正；不能将其解释成机器人瞬移或纯前端跳变。
- 无大速度、大偏置重置或特征不足告警；存在下面说明的 TF 消息头错误，不隐去原始日志。
- 回放、适配器及四个算法节点最终均 exit 0。之后 RViz 退出，监督进程和本地 router 也已退出；[cleanup.json](runs/20260920_170433_full_central_native_loop/cleanup.json) 记录所有本轮进程均已清理（exit 0），没有继续回放或后台建图。

本次完成的是兼容性/运行试验。原有外参候选、六轴姿态适配以及上游噪声参数限制仍在；**不能以这一次开启回环的结果，判断 LIO-SAM 前端优于或劣于 Faster-LIO，更不应立即替换现有正式链路。**

### 双雷达与 Web 纳管

Web：[3D 地图 / 原始点云](http://127.0.0.1:8766/#/3d/maps)。上一轮已明确标为「LIO-SAM · 单前雷达」；本轮成功导出后为「LIO-SAM · 前后双雷达」。仅登记最终 `GlobalMap.pcd`，不把 `trajectory.pcd`、`transformations.pcd`、子特征图或关键帧当地图重复列出。原实验 PCD 保留，纳管副本放在 `maps/lio_sam/<run>/`，当前选用地图不变。

输入适配验证：[audit_dual_sample.json](audit_dual_sample.json) 检查两个真实前后样本对：前后 ring 区间互斥；后点逆变换误差约 1 μm；原生距离与列号一致；绝对逐点时间重建误差不超过 238 ns（float64 epoch 分辨率）。这些数值验证实现，不代表物理标定精度。

2×短测 60.1 s：前 585 帧、后 583 帧、中心 IMU 12,299 条收到；初始化略过前 20、后 19 帧；544 对成功发布，下游处理 541 帧（末尾 2 帧缓存及 1 帧起始等待）。未配对的前 21、后 19 帧单独计数，不声称零丢帧。早期故障短测和有抽帧的短测保留日志用于诊断，**未登记到 Web**。

[全包原始 header 基线](bag_pair_audit.json)（仅查询 12 字节消息前缀）确认：前 9,364 / 后 9,325，初始化略过前 20 / 后 19 后应得到 **9,051 对**；原包无匹配前 293 / 后 255。相同短测前缀的 544 对、21/19 无匹配与运行统计完全一致。后续完整结果与这个精确基线核验，不用武断的“必须 98% 配对”等比例替代源数据证据。

性能检查同一真实 123,097 点合并帧：转换约 17.25 ms，`data=bytes` 约 161.72 ms，`data=array('B',...)` 约 0.273 ms。typed buffer 保存同一字节内容，没有用降采样掩盖吞吐问题。

整包导出前检查 raw 前/后/中心 IMU 接收数量与 bag metadata 完全一致、IMU 覆盖队列排空、配对帧均发布、投影/建图处理数量匹配；检查失败会保留诊断文件并拒绝自动发布到 Web。配对丢弃数量继续明确保留。

完整运行结束后，脚本自动保存地图、轨迹和参数，并关闭回放、适配器及四个 LIO-SAM 节点。`--hold` 仅保留 RViz、离线地图预览发布器和本地 Zenoh 路由；关闭 RViz 窗口后也会关闭这套预览进程。

完整运行原始日志含 `TF_NO_FRAME_ID`：TransformFusion 发布的高频 TF 被拒收。该路径不反馈进 mapping；去畸变消费增量 Odometry 消息，全局点云与路径直接使用 `odom`。因此本次不能称“日志无错误”或“高频 TF 链路已验证”；保存地图仍有效用于此次实验观察。源码修复与本次实际运行的快照分开记录，不把之后修复冒充本次已运行代码。

完整回放结束后已重新编译 TF 消息头修复，并用独立 Domain 221 / localhost:7449 做合成输入回归：[result.json](tf_header_tests/20260920_172215_60705/result.json)。三项检查通过：旧/相等时间输入不会访问空队列；两条 `odom → lio_lidar` TF 的父 frame 和非零原始时间戳精确保留。测试自己的 router 与节点均已退出；**没有再次回放整包**。修复后二进制 SHA256：`e1a0988b6ed06bd41273cb0bc438a6d3f7be9f04ffe393dfb3c3f930b3f55c6a`。

## 查看内容与保存位置

RViz 显示实时全局特征地图、黄色当前扫描、细线轨迹和原生回环约束，不加载之前的 PCD，不使用 Foxglove。

- `map/GlobalMap.pcd`：原生 LIO-SAM 关键帧角点/平面特征地图，**不是所有原始雷达点的密集累加**。
- `map/CornerMap.pcd`、`map/SurfMap.pcd`：按导出请求 0.1 m 体素处理的分层地图；上游 `GlobalMap.pcd` 组合的是关键帧特征原集合，并非强制整体 0.1 m 下采样。
- `map/trajectory.pcd`、`map/transformations.pcd`：关键帧位置及优化位姿。
- `odometry.tum`：本次运行实时输出的 mapping 位姿；回环时可以跳变，不等同于最终全部重优化关键帧轨迹。
- `params.yaml`、`manifest.json`、`adapter.json`、`status.json`、`code/`、`source.patch`：参数、输入/运行证据与代码快照。
- 各节点 `.log`：原始日志；`cleanup.json` 在本次整个监督进程退出时写入。

约束数量不等于真正绕楼一圈的次数。原生空间邻近 + ICP 后端也可接受沿途有重叠视野的历史扫描约束；不能仅凭“有连线”证明闭环正确。连线表示 ICP 已接受并记录的约束；上游仅在新增关键帧时将队列约束加入图，末尾可能仍有待融合项，因此不把连线数称为“已融合因子数”。

统计中的在线 TUM 路程和最大步长含回环校正跳变，不是实走距离或纯前端跳变；输出频率仅指 mapping 位姿，不代表高频 IMU 预积分频率。最终优化关键帧的统计单列为 `optimized_keyframes`，不混用两种轨迹。

## 复现

```bash
cd /home/dndx/d1max_nav_ws/experiments/lio_sam_20260920
python3 -m unittest -v test_adapter.py
bash run.sh --duration 60 --rate 2 --lidar-mode dual --loop --no-rviz --label dual_smoke60
bash run.sh --duration 0 --rate 2 --lidar-mode dual --loop --no-rviz --publish-web --label full_dual_central_native_loop
# 单前雷达对照（不覆盖历史运行）
bash run.sh --duration 0 --rate 1 --lidar-mode front --loop --hold --label front_comparison
```

每次自动创建独立时间戳结果目录。独占锁和端口检查会拒绝重复启动，绝不通过广泛 pkill 清理已有任务。前一次 `--hold` 的预览仍开着时，先关闭它或向其监督进程发送 SIGINT，再新开下一次。

完成后可对输出做只读统计：

```bash
python3 summarize.py runs/对应运行目录
```

`summary.json` 排他创建，不覆盖旧统计。七项输入计算测试覆盖：初始重力对齐、ring/秒单位/时间排序/投影逆变换、杆臂方向、双传感器区块与原生距离、过期扫描配对拒绝、重复消费防护、无效外参拒绝。

## 结论边界

本轮先检验链路、可视化和算法实际运行，不证明导航精度。中心外参/时间偏移仍是旧实验候选，IMU 噪声参数仍为上游默认，未宣称适配 D1 Max 的标定噪声；未施加平地锁高。上游仍只进行旋转去畸变，未启用平移去畸变。完整试验开启原生回环，所以不能把回环后的首尾重合解释成前端无漂移，也不能直接与另一套输入/无回环结果做公平优劣排名。六轴、多雷达投影与边界保护均为披露的实验适配，不冒充上游原版直接支持双雷达。
