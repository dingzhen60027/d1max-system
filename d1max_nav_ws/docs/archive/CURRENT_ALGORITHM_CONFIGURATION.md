# D1 Max 当前算法配置汇总

更新日期：2026-08-23  
工作空间：`/home/dndx/d1max_nav_ws`  
环境：ROS 2 Humble + Zenoh

## 1. 系统架构模块

```text
前后 Airy96 点云 + IMU
          |
          v
 dual_lidar_adapter
          |
 /d1max/slam/points + /d1max/slam/imu
          |
          v
 Faster-LIO 或 FAST-LIO2
          |
 连续里程计 + 世界系当前扫描
          |
          v
     SCAN-Planner
          |
 3D 占据地图 + B 样条轨迹
```

可选回环链路：`Faster-LIO -> SC-PGO`。FAST-LIO2 当前没有回环。

安全边界：

- SDK 桥只读取状态，不发送控制指令。
- SCAN-Planner 不启动开环或闭环控制器。
- 当前不发布 `/cmd_vel`。

## 2. 传感器模块

### 双雷达

| 项目 | 前雷达 | 后雷达 |
|---|---|---|
| 话题 | `/front_lidar` | `/rear_lidar` |
| frame | `rslidar_head` | `rslidar_tail` |
| 线数 | 96 | 96 |
| 实测频率 | 约 9.913 Hz | 约 9.826 Hz |
| 单帧槽位 | 86400 | 86400 |
| 有效点中位数 | 约 30560 | 约 29621 |
| ring | 0 到 95 | 0 到 95 |
| 单帧时间跨度 | 约 99.986 ms | 约 99.986 ms |

消息类型为 `sensor_msgs/msg/PointCloud2`，字段包括 `x/y/z`、`intensity`、`ring` 和 `timestamp`。前后雷达配对时间差中位数约 `0.005 ms`。

### 双 IMU

- `/front_lidar/imu`：约 200 Hz。
- `/rear_lidar/imu`：约 200 Hz。
- 原始加速度量级约为 `g`。
- 适配器乘以 `9.80665` 转换为 `m/s^2`。
- 同一 LIO 只使用一套经过正确外参转换的 IMU 状态输入。

### 时钟

传感器时钟与 PC 时钟观测到约 `17,320,250.685 s` 的差值，约 200.47 天。传感器消息不能未经转换就和 PC `now()`、PC 动态 TF 或实时系统时间混用。

## 3. 双雷达适配模块

功能包：`src/d1max_slam`  
配置：`src/d1max_slam/config/dual_lidar.yaml`  
节点：`dual_lidar_adapter`

输入：

- `/front_lidar`
- `/rear_lidar`
- `/front_lidar/imu`
- `/rear_lidar/imu`

输出：

| 话题 | 类型 | 用途 |
|---|---|---|
| `/d1max/slam/points` | `sensor_msgs/msg/PointCloud2` | 双雷达融合点云 |
| `/d1max/slam/imu` | `sensor_msgs/msg/Imu` | 坐标和单位转换后的 IMU |

处理规则：

- 根据传感器时间戳配对前后雷达。
- 将两颗雷达转换到统一 `d1max_lidar` 坐标系。
- 后雷达 ring 增加 96，融合 ring 范围为 `0 到 191`。
- 保留逐点相对时间，输出 `time` 字段单位为浮点毫秒。
- IMU 和融合点云使用相同参考坐标约定。

## 4. SLAM 模块

### Faster-LIO

- 功能包：`src/faster_lio`
- 配置：`src/d1max_slam/config/faster_lio_airy96.yaml`
- 启动入口：`start_slam.sh`
- 输入：`/d1max/slam/points`、`/d1max/slam/imu`
- 当前主要实际建图前端。
- 可以连接 SC-PGO。
- SCAN 默认接口：`/Odometry`、`/cloud_registered`。

如果实际输出被重映射，启动 SCAN 时传入 `odom_topic` 和 `cloud_topic`。

### FAST-LIO2

- 功能包：`src/fastlio2`
- 配置：`src/d1max_slam/config/fastlio2_airy96.yaml`
- 启动入口：`start_fastlio2.sh`

| 参数 | 当前值 |
|---|---|
| `lidar_input_type` | `pointcloud2` |
| `lidar_topic` | `/d1max/slam/points` |
| `imu_topic` | `/d1max/slam/imu` |
| `body_frame` | `d1max_lidar` |
| `world_frame` | `map` |
| `imu_acc_scale` | `1.0` |
| `point_time_scale_to_ms` | `1.0` |
| `lidar_queue_depth` | `20` |
| `max_scan_duration` | `0.15 s` |
| `lidar_filter_num` | `1` |
| `scan_resolution` | `0.15 m` |
| `map_resolution` | `0.25 m` |
| IMU 初始化 | `400` 个样本，约 2 秒 |

输出：

- `/d1max/fastlio2/odometry`
- `/d1max/fastlio2/world_cloud`

FAST-LIO2 使用原生 PointCloud2 输入，不再伪装为 Livox CustomMsg。

## 5. 回环模块

功能包：`src/sc_pgo`

- 当前只连接 Faster-LIO。
- FAST-LIO2 当前没有回环。
- 回环用于全局地图和全局定位。
- 局部规划继续使用连续 LIO 里程计，避免回环校正跳变进入控制链。
- 回环不能修复外参、逐点时间、IMU 单位或坐标系错误。

## 6. SCAN-Planner 模块

- 上游算法：`src/scan_planner_vendor`
- D1 适配包：`src/d1max_scan_planner`
- 配置：`src/d1max_scan_planner/config/d1max_scan_planner.yaml`
- launch：`src/d1max_scan_planner/launch/scan_planner.launch.py`
- RViz：`src/d1max_scan_planner/rviz/d1max_scan_planner.rviz`

### 输入接口

| 后端 | odometry | 世界系当前扫描 |
|---|---|---|
| Faster-LIO | `/Odometry` | `/cloud_registered` |
| FAST-LIO2 | `/d1max/fastlio2/odometry` | `/d1max/fastlio2/world_cloud` |

| 导航模式 | 输入 |
|---|---|
| `navi_mode=1` | `/goal_pose` |
| `navi_mode=2` | 参数文件预设关键点 |
| `navi_mode=3` | `/d1max/scan/initial_path`，类型为 `nav_msgs/msg/Path` |

### 3D 滚动地图

| 参数 | 当前值 |
|---|---|
| frame | `map` |
| 分辨率 | `0.08 m` |
| 地图尺寸 | `12.0 x 12.0 x 6.4 m` |
| 局部更新范围 | `6.0 x 6.0 x 3.2 m` |
| 滑动阈值 | `0.32 m` |
| 最大射线长度 | `8.0 m` |
| 初始地图下边界 | `-0.8 m` |
| `cloud_is_world` | `true` |
| `need_extrinsic` | `false` |

### D1 Max 碰撞包络

| 参数 | 当前值 |
|---|---|
| 双圆柱半径 | `0.29 m` |
| 圆柱前后偏移 | `+/-0.20 m` |
| 近似水平包络 | `0.98 x 0.58 m` |
| 身体高度 | `0.55 m` |
| 向上膨胀 | `0.15 m` |
| 向下膨胀 | `0.45 m` |

这是根据当前 URDF 设置的保守初始值，需要通过门、楼梯和狭窄通道实测标定。

### 轨迹约束

| 参数 | 当前值 |
|---|---|
| 最大速度 | `0.45 m/s` |
| 最大加速度 | `0.35 m/s^2` |
| 最大 jerk | `2.0 m/s^3` |
| 控制点间距 | `0.20 m` |
| 规划范围 | `6.0 m` |
| 碰撞权重 | `1.5` |
| 安全距离 `dist0` | `0.28 m` |
| B 样条阶数 | `3` |

### 输出接口

- `/d1max/scan/planning/bspline`
- `/d1max/scan/grid_map/occupancy`
- `/d1max/scan/grid_map/occupancy_inflate`
- `/d1max/scan/optimal_list`
- `/d1max/scan/a_star_list`
- `/d1max/scan/global_list`

## 7. 运行模块

Faster-LIO + SCAN：

```bash
cd /home/dndx/d1max_nav_ws
./start_scan_planner.sh faster_lio true
```

FAST-LIO2 + SCAN：

```bash
cd /home/dndx/d1max_nav_ws
./start_fastlio2.sh
./start_scan_planner.sh fastlio2 true
```

外部 3D 路径模式：

```bash
ros2 launch d1max_scan_planner scan_planner.launch.py backend:=fastlio2 navi_mode:=3 use_rviz:=true
```

原始输入录包：

```bash
ros2 bag record /front_lidar /rear_lidar /front_lidar/imu /rear_lidar/imu
```

实测数据率约 `42 MiB/s`，约 `148 GiB/h`。

## 8. 验证与风险模块

已验证：

- 双雷达和双 IMU 发布频率、点云字段及帧配对精度。
- FAST-LIO2 原生 PointCloud2 适配可以构建，三项核心测试通过。
- SCAN-Planner 六个上游包和 D1 适配包构建成功。

待验证：

- SCAN 3D 地图与实时 LIO 点云对齐。
- LIO odometry 原点是否位于机身中心。
- 碰撞包络是否适合实际楼梯、门宽和腿部摆动。
- FAST-LIO2 双 96 线实时输入是否产生队列积压。
- SCAN 轨迹的稳定性、连续性和安全距离。
- 多楼层全局路径和机器狗控制均未接入。

已知风险：

- `body_pose` 和 `sensor_pose` 当前使用同一个 LIO odometry；若原点不在机身中心，需要固定外参转换。
- `start_fastlio2.sh` 的进程检测可能受 Linux 15 字符进程名限制而漏检 Faster-LIO。
- 上游 `PolynomialTraj::getMeanVel()` 存在非 void 函数缺少返回值的编译警告。
- 机器人传感器与 PC 时钟尚未统一。

## 9. 跨楼层导航模块

推荐架构：

```text
每层 2D 地图 + 楼层拓扑图
              |
       跨楼层全局路径
              |
       SCAN 3D 局部规划
              |
     安全控制桥（未实现）
              |
            D1 SDK
```

- 每层 2D 地图负责平层全局规划。
- 楼层拓扑图表达楼梯、电梯和楼层连接关系。
- SCAN 负责复杂区域的局部 3D 规划。
- 电梯需要独立状态机。

## 10. 配置文件索引

| 模块 | 文件 |
|---|---|
| 双雷达适配 | `src/d1max_slam/config/dual_lidar.yaml` |
| Faster-LIO | `src/d1max_slam/config/faster_lio_airy96.yaml` |
| FAST-LIO2 | `src/d1max_slam/config/fastlio2_airy96.yaml` |
| SLAM launch | `src/d1max_slam/launch/mapping.launch.py` |
| SCAN 参数 | `src/d1max_scan_planner/config/d1max_scan_planner.yaml` |
| SCAN launch | `src/d1max_scan_planner/launch/scan_planner.launch.py` |
| SCAN RViz | `src/d1max_scan_planner/rviz/d1max_scan_planner.rviz` |
| SCAN 启动 | `start_scan_planner.sh` |

本文档是当前唯一的算法配置汇总。修改运行参数后，应同步更新对应模块章节。
