# AIRY 原始点云坐标基底离线核验

## 结论

只读抽查本包前、后雷达各两条原始 `PointCloud2` 消息，分别位于开头约 0.65 秒和 1.5 秒。点云不经过任何配准、地面提取或姿态修正，直接满足对应设备 DIFOP 的原生射线锥面模型，角残差 RMS 为 **1.25～1.30 × 10⁻⁶ 度**，达到 float32 坐标舍入量级。

在可观测的 roll、pitch 和三维平移方向上，没有发现额外点云坐标变换：拟合旋转分量小于约 10⁻⁷ 度，平移为数纳米。这些极小值仅说明生成点坐标的公式一致，**不是传感器真实测量精度或外参标定精度**。

如果假设记录点云已经使用 DIFOP 的雷达到内置 IMU 四元数、平移进行过变换，将其逆变换回原生雷达坐标后，锥面残差反而达到 113～116 度。该“已经应用完整 DIFOP IMU 外参”的假设与抽查数据不符。

### 设备扫描角指纹交叉检查

对同样四条消息交换两个设备的标定角数组，不进行任何参数拟合：

| 点云话题 | 匹配的 DIFOP 设备 | 对应设备残差 RMS | 交换设备后残差 RMS |
|---|---|---:|---:|
| `/front_lidar` | `192.168.1.200` / `3009bede1530` | 约 0.00000125° | 0.3236～0.3238° |
| `/rear_lidar` | `192.168.2.200` / `300dbeda0056` | 约 0.00000130° | 0.3217～0.3220° |

交换候选时的绝对角误差 P95 约为 0.59°，最大约 0.77°。这支持当前话题与设备标定指纹的对应关系，不支持“两颗设备的扫描角数据互换”的假设。但它是点云内容的指纹核验，**不是直接读取录包时驱动配置的结果**，也不能据此保证任何物理外参正确。

## 方法与独立性

官方 AIRY 解码器以每条激光通道的固定垂直角、水平角修正和量测距离生成 XYZ，随后才进入可选 `transformPoint` 路径。`ring` 为垂直角升序排名。

本分析从同机 DIFOP 原始 hex 解码 96 条线的垂直/水平角，不把某个扫描环当作地面。模型保留官方镜心径向偏移 `sqrt(0.0075² + 0.00664²)` 米与 Z 偏移 `0.04532` 米。先从记录点的 XY 半径解析消除镜心偏移，再检验其射线仰角是否等于对应 ring 的标定仰角。

因此，地面是否水平、机器人是否静止、扫描的物体形状，均不是本检查成立所依赖的场景约束。抽查开头只是为了缩小读取范围。

## 不能据此证明的内容

- 绕原生雷达 Z 轴的任意 yaw 不改变射线锥面，因此这里不能排除额外 yaw 旋转，不能单独完全认证 XYZ 轴方向。
- 不证明内置 IMU 的 ROS 消息未单独转轴，也不证明 DIFOP q/t 的物理方向约定。
- 不证明双雷达之间的安装关系或中心 IMU 的外参正确。
- 只有四条消息，不是整包每帧认证。
- 当前上游源码是公式依据，不是录包机器二进制的版本鉴定。

## 结果与复现

- [全部指标](beam_frame_report.json)
- [只读核验脚本](audit_beam_frame.py)
- [交换设备标定角结果](cross_device_report.json)
- [交换设备核验脚本](audit_cross_device.py)

脚本只使用 ROS 消息反序列化，不初始化 ROS、不连机器狗、不启动回放。输出使用排他创建；重复运行前应指定新输出路径或保留既有结果，不应覆盖证据。

```bash
source /opt/ros/humble/setup.bash
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 python3 audit_beam_frame.py
```

## 官方公式依据

- [AIRY 解码与可选变换](https://github.com/RoboSense-LiDAR/rs_driver/blob/main/src/rs_driver/driver/decoder/decoder_RSAIRY.hpp)：`internDecodeMsopPkt` 的点坐标生成、`decodeDifopPkt` 的可选安装角处理。
- [标定角与 ring 对应关系](https://github.com/RoboSense-LiDAR/rs_driver/blob/main/src/rs_driver/driver/decoder/chan_angles.hpp)：`loadFromDifop`、`genUserChan`。
- [镜心径向偏移](https://github.com/RoboSense-LiDAR/rs_driver/blob/main/src/rs_driver/driver/decoder/decoder_mech.hpp)：`lidar_lens_center_Rxy_`。
