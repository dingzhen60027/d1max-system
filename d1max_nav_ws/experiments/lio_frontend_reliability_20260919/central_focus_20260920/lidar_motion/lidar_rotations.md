# 中央 IMU 专项：独立前雷达相对旋转观测

2026-09-20。只读离线诊断；没有 ROS 节点、SLAM 回放、机器人通信或生产参数改动。

## 结果与使用边界

从 4 个相互分离的运动窗口取得 **36 对连续收到的原始前雷达扫描，27 对通过预先写定的质量筛选**。9 对未通过也完整保留，不可仅挑成功样本隐藏失败。

这些是 **LiDAR-only 的独立配准结果**：中央 IMU 的 gyro 仅选择窗口；配准不使用任何 IMU/LIO 位姿、外参或运动去畸变，三个配准方向/设置均从单位变换开始。点坐标一直保持 raw `rslidar_head`，没有隐藏坐标规范化。

**可以作为中央 IMU 相位/旋转假设的独立诊断观测，但不是生产标定结果。** 每帧原始扫描跨约 100 ms，帧内运动并未补偿。将整帧配准视作某个单一瞬间的姿态本身是近似；`midpoint` 只给出扫描中点估计，不保证它是真实有效配准时刻。环境可见性、点分布及运动畸变都可能造成有效时间偏移，因此不能把拟合所得时移不加区分地叫作物理传感器延迟。

## 窗口与质量

相对时间以 `central.npz` 第一条 header 为零（绝对时间 `1772316441.270134` 秒）；窗口编号与 JSON 的 `window_index` 一致。

| 组 | 中心相对时间 | 选择目的 | 通过 / 总数 | 通过观测旋转角范围 | 前向 fitness 中位 | 欧氏内点 RMSE 中位 |
| --- | ---: | --- | ---: | ---: | ---: | ---: |
| 0 | 179.75 s | 前 180 s 内角运动较强 | 8 / 9 | 0.771–2.647° | 0.9856 | 0.0426 m |
| 1 | 576.75 s | 中央 gyro X 分量较强 | 3 / 9 | 0.086–0.364° | 0.9889 | 0.0454 m |
| 2 | 590.25 s | 中央 gyro Z 分量较强 | 8 / 9 | 0.931–2.106° | 0.9501 | 0.0628 m |
| 3 | 892.75 s | 中央 gyro Y 分量较强，兼有 Z | 8 / 9 | 0.105–2.727° | 0.9915 | 0.0474 m |

组名描述的是中央 IMU 的窗口选择量，并不宣称已经证明它对应雷达坐标的哪个 roll/pitch/yaw。尤其组 1 的独立 LiDAR 运动较弱，只有 3 对通过；不能假定已获得平衡的三轴激励。

27 对通过结果的质量范围：

- 前向/反向闭合旋转误差：0.0117–0.0991°，中位 0.0324°。
- 两种 voxel 设置的旋转差：0.0064–0.0873°，中位 0.0267°。
- 前向 fitness：0.9284–0.9976；欧氏内点 RMSE：0.03485–0.06601 m。
- 点到面残差 RMSE：0.00918–0.03341 m，中位 0.01782 m。
- 尺度归一化 6×6 信息矩阵最小/最大特征值比：0.01096–0.04446。
- 消去平移后的旋转 Schur 信息矩阵特征值比：0.02681–0.25607。

局部信息谱只是固定对应关系下的几何可观测性代理，不是标定后的测量协方差；正的谱和很高的重叠率均不能证明匹配到唯一正确位置。小角度观测的闭合/分辨率误差占比可能很高，后续拟合需要做按窗口留出验证，而不是把全部 27 对视为同等精度。

## 明确保留的失败

所有索引从 0 开始。

| 组 / 对 | 原因 |
| --- | --- |
| 0 / 7 | 相邻已记录消息的 header 间隔约 0.200 s，不是约 0.100 s 的连续扫描，拒绝进入严格相邻扫描拟合。 |
| 2 / 2 | header 间隔约 0.300 s，同上。 |
| 1 / 1 | 前后向闭合旋转约 1.110°，且平移闭合超过阈值；拒绝。 |
| 1 / 3、5、6、7、8 | 独立估计的旋转角不足 0.08°，有效角运动不足。 |
| 3 / 8 | 旋转角不足 0.08°。 |

间隔跳步并不证明这些长间隔配准一定错误；只是明确不把它们当作已知的相邻 100 ms 观测混入当前相位诊断。

## 配准方向与时间字段

`previous` 为较早扫描，`next` 为较晚扫描。配准 source=`next`、target=`previous`：

```text
p_previous = R_prev_from_next * p_next + t_prev_from_next
```

这也是下一时刻 LiDAR 坐标系在前一时刻 LiDAR 坐标系中的相对姿态。若采用机器人运动的相对姿态积分，不能再无依据地取逆。旋转向量 `rotation_vector_prev_axes_rad` 用前一扫描的 raw LiDAR 轴表达。`reverse_T_next_from_prev` 是另一次独立 identity 初值配准，不是简单对前向矩阵求逆。

每对 JSON 提供：

- `window_index`、`pair_index`。
- `previous` / `next`：`header_ns`、`point_time_min_sec`、`point_time_max_sec`、`scan_midpoint_estimate_sec`、范围过滤后点时间均值/中位数、消息 ID、bag 接收时间及 frame_id。
- `header_delta_sec`、`midpoint_delta_sec`，以及 R、t、4×4 T、旋转向量和旋转角。
- 独立反向与另一套 voxel 的 T、多尺度 fitness / RMSE、闭合误差、信息谱、`gates`、`accepted` 和 `rejected_reasons`。

`header_ns` 为整数传感器首点时间，点时间为原始 FLOAT64 绝对秒。bag 接收时间只用于 SQLite 有界检索，不参与物理旋转拟合。所有姿态时刻应显式选用 header 或 midpoint 假设；二者相差约 50 ms，不能在拟合中悄悄混用。还应按同一样本集合比较 lag，避免时间覆盖改变导致目标函数不公平。

## 方法与筛选阈值

1. 从已有 `central.npz` 找 0.5 s gyro RMS 较强窗口，不扫描原包 IMU。X/Y/Z 三个窗口至少间隔 10 s，另加前 180 s 最大角运动窗口。
2. 借助已有 `front.npz` 的 header→record 时间映射及 metadata 中的分包范围，只查询目标附近 ±1.2 s 的至多 50 个 12-byte CDR header；随后只读每窗口 10 个原始完整前雷达消息，形成 9 对。没有全包点云扫描。
3. 仅过滤非有限 XYZ 和半径不在 0.75–30 m 的点；不人为抽取地面、不做坐标旋转、不用 IMU 去畸变。
4. Open3D 多尺度 point-to-plane ICP：主配置 voxel 为 0.30/0.15/0.08 m，对应最大距离 0.75/0.40/0.20 m，迭代 30/25/20；独立验证配置为 0.25/0.12/0.06 m，距离 0.65/0.32/0.16 m，迭代 30/25/20。法线半径为 `max(0.25,3×voxel)`，最多 30 邻居。
5. 正向、反向、独立 voxel 配准全部 identity 初值。几何信息采用最终主配置近邻对应点构造点到面 Jacobian `[p×n,n]`，并报告用几何 RMS 半径归一化后的谱和消去平移后的旋转 Schur 谱。

阈值在第一次完整运行前写入脚本：前/反向 fitness≥0.55、欧氏内点 RMSE≤0.10 m、前后向闭合旋转≤0.20°且平移≤0.03 m、两套 voxel 的旋转差≤0.20°且平移差≤0.03 m、归一化完整谱比≥1e−7、旋转 Schur 谱比≥0.002、旋转角≥0.08°、扫描间隔在 0.08–0.12 s。

## 文件与执行记录

- [原始旋转观测及所有失败](/home/dndx/d1max_nav_ws/experiments/lio_frontend_reliability_20260919/central_focus_20260920/lidar_motion/lidar_rotations.json)
- [可复现脚本](/home/dndx/d1max_nav_ws/experiments/lio_frontend_reliability_20260919/central_focus_20260920/lidar_motion/extract_lidar_rotations.py)

最终成功执行只读取 40 条完整点云消息，配准及读取约 4.7 s，OpenMP/BLAS 限 2 线程。最初两次开发执行在 Open3D 只读数组兼容、bag 多 SQLite 分卷路由处停止，各读取首窗口 10 条消息；修复后重新运行。**本任务全部尝试合计 60 次完整 payload 读取、40 个独立消息，未超过 80 条预算。** 不将开发报错冒充算法质量失败，也没有丢掉最终配准中的失败观测。

复现时加载 ROS 消息类型环境即可，不需要 ROS 节点：

```bash
source /opt/ros/humble/setup.bash
OMP_NUM_THREADS=2 OPENBLAS_NUM_THREADS=2 python3 /home/dndx/d1max_nav_ws/experiments/lio_frontend_reliability_20260919/central_focus_20260920/lidar_motion/extract_lidar_rotations.py --output /tmp/central_lidar_rotations_recheck.json
```

脚本拒绝覆盖已有输出；JSON 内记录脚本 SHA256 和 Open3D/NumPy 版本。该成果仅提供独立旋转观测，不包含中央 IMU 外参或时间偏移拟合，也不声称已经修复地图下沉。
