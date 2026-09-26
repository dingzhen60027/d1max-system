# D1 Max rosbag → LIO-SAM 输入适配边界

日期：2026-09-20。对象：`/home/dndx/d1max_nav_ws/bags/slam_raw_20260917_171716_fe8f38`。

**这是中央六轴 IMU 的隔离适配试验，不是官方 LIO-SAM 九轴输入的开箱运行，也不是一次新标定。** 原有 Faster-LIO 配置、默认外参和 Zenoh 不因本试验改变。

## 1. 为什么不能原样输入

[官方 ROS2 README](https://github.com/TixiaoShan/LIO-SAM/blob/ros2/README.md) 的 `Prepare lidar data` 要求每点相对秒时间和 ring；`Prepare IMU data` 要求带姿态估计的 IMU，并建议至少 200 Hz。对应本地副本：[README.md](ws/src/LIO-SAM/README.md)。

读取既有全量 IMU 缓存，结果如下；本次没有重新遍历全包：

| 原始话题 | 消息数 | orientation 不同值数 | 全程值 |
|---|---:|---:|---|
| `/front_lidar/imu` | 189,626 | 1 | `[0,0,0,1]` |
| `/imu_driver/imu_central` | 192,971 | 1 | `[0,0,0,1]` |
| `/rear_lidar/imu` | 190,011 | 1 | `[0,0,0,1]` |

这些四元数不是有效的随动姿态。三路平均记录频率约 193–196 Hz，均存在间隔缺口，不能等同于连续无缺口的硬件采样。metadata 仅包含上述 IMU、前后点云、`/tf` 和 `/tf_static`，没有独立 MC 或机身姿态话题；未经来源证明的 TF 也不充当真实九轴姿态。

证据：[fields.json](../central_imu_quality_20260918/fields.json)、[原始质量报告](../central_imu_quality_20260918/REPORT.md)、同目录 `front.npz` / `central.npz` / `rear.npz`。质量报告早于本轮中央 IMU 试验，其中“当时建图使用哪颗 IMU”的描述只适用于该报告所核查的历史运行。

## 2. 两帧点云的投影核查

只读取了两条前雷达完整 payload，header 分别为 `1772316441199985981` 和 `1772316485699981213` ns。只读 SQLite 查询，未启动 ROS。两帧均为 `height=900`、`width=96`、86,400 点、`ring=0..95`，按 96 通道交错排列；每点 `timestamp` 是 FLOAT64 绝对秒，时间有序，扫描跨度约 0.099986 s。

使用有限且距离 0.35–120 m 的点，按 LIO-SAM 的 `atan2(x,y)`、96 行、900 列计算重复单元率：

| 计算方位角所用坐标 | 第一帧（60,553 有效点） | 第二帧（59,094 有效点） |
|---|---:|---:|
| 原始雷达坐标 L | 0.0215% | 0.0102% |
| 既有归一化坐标 N | **71.7372%** | **71.2424%** |

原始坐标每 ring 仰角标准差的中位数约 0.65°，归一化后约 31°。这说明**不能直接把 Faster-LIO 的归一化 xyz 交给原版方位角分列**；重复 range-image 单元会令大量点被跳过。上述比例是两帧投影审计，不是全包实际丢点率，也不代表归一化旋转本身错误。

本试验采用：

- 状态、点云 xyz、去畸变与特征处理使用 N 轴，避免用本机斜装雷达原始轴作为 LOAM 姿态轴带来的接近 90° pitch 风险。
- 仅计算方位角列号时，通过 `projectionRot = R_N_Lᵀ` 将 N 坐标临时旋回原始 L 坐标；ring 保持原始。
- `N_SCAN=96`、`Horizon_SCAN=900`；单前雷达起步，不把前后雷达相同 ring/column 直接塞入同一 range image。
- 点时间转换为 FLOAT32 **相对秒**：`time = timestamp - header_time`，按时间排序。不是 Faster-LIO curvature 所用的相对毫秒。

实现位置：[adapter.py](adapter.py) 的 `convert_cloud()`、[imageProjection.cpp](ws/src/LIO-SAM/src/imageProjection.cpp) 的 `projectPointCloud()`、[utility.hpp](ws/src/LIO-SAM/include/lio_sam/utility.hpp) 的 `projectionRot` 参数校验。

## 3. 六轴姿态适配，不伪装为硬件 AHRS

当前 [adapter.py](adapter.py) 将中央加速度按已有配置由 g 尺度转换为 m/s²，并将六轴转到 N 轴。启动约 2 s 数据需通过低动态门限，随后由起始重力方向初始化 roll/pitch、任意置零 yaw，以减去启动陀螺均值后的角速度传播辅助姿态。

这不是磁航向、不是持续加速度校正的九轴 AHRS，也没有使用 Faster-LIO 位姿、机器人定位结果或地面水平先验来伪造姿态。初始低动态门限不等于有外部静止真值；启动均值不能证明全程零偏恒定。

为了不把陀螺积分姿态反过来当作独立绝对姿态观测，[run.py](run.py) 明确设置：

```yaml
useImuHeadingInitialization: false
imuRPYWeight: 0.0
imuIncrementalRPYWeight: 0.0
```

辅助姿态仍用于初始化和相对姿态备用初值。两处绝对 roll/pitch 混合均关闭：一处是 `transformUpdate()`，另一处是增量里程计输出，见 [mapOptmization.cpp](ws/src/LIO-SAM/src/mapOptmization.cpp)。后者原有固定权重已改为独立可配置参数。**仅关闭第一处不足以避免输出再次向辅助姿态倾斜。**

## 4. 坐标与外参约定

列向量约定：`p_C = R_C_N p_N + t_C_N`；C 是中央 IMU，N 与前雷达原点相同，仅旋转了坐标轴。当前 adapter 已将六轴与辅助姿态完整表示在 N 轴，但测量原点仍在中央 IMU。

因此本轮实际配置为：

```text
extrinsicRot   = I
extrinsicRPY   = I
extrinsicTrans = -R_C_Nᵀ t_C_N
projectionRot = R_N_Lᵀ
```

若将来改为直接输入原始 C 轴六轴及 `R_W_C` 姿态，则必须重新配置：`extrinsicRot=R_C_Nᵀ`、`extrinsicRPY=R_C_N`，不是机械地给两者同一矩阵。依据是 [utility.hpp](ws/src/LIO-SAM/include/lio_sam/utility.hpp) 中 `v_out=extRot*v_in` 与 `q_final=q_from*extQRPY` 的不同乘法语义。平移符号依据 [imuPreintegration.cpp](ws/src/LIO-SAM/src/imuPreintegration.cpp) 的 `lidarPose.compose(lidar2Imu)`。

采用的旧候选是 [device_front_rotation.yaml](../lio_frontend_reliability_20260919/extrinsic_trials_20260919/calibrations/device_front_rotation.yaml)：中央—前雷达旋转包含间接陀螺拟合关系，中央时间补偿 −13 ms 可能包含滤波相位，杆臂包含中央原点的名义位置假设；精确录包驱动坐标处理仍有不确定性。**这些是未完整验证的实验外参，不因更换算法而变成厂商已确认标定。**

## 5. 可以与不可以得出的结论

本试验可以验证这套显式输入适配下 LIO-SAM 是否运行、地图形状和轨迹是否改善。它不能单独证明原来下沉已修复、中央 IMU 外参正确、原版 LIO-SAM 在真实九轴输入下的性能，也不能把回环拉回后的结果当成前端无漂移。

两帧投影统计和三路 orientation 统计是本次只读输入审计结论；运行是否成功、完整结果和回环事件应以各次 `runs/` 下的配置、日志及保存产物为准。
