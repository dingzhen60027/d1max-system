# 导航定位输出 · 当前默认架构

2026-09-12。默认 `lio_pcd`，启用独立高频输出层。Web 操作入口和 Foxglove 布局不变。
定位进程不连接 SDK、不控制机器人、不启动 Nav2；通信仍为 Zenoh。

## 分层与职责

| 层 | 模块 | 唯一职责 | 不负责 |
|---|---|---|---|
| 输入 | `dual_lidar_adapter` | 雷达/IMU 轴向、单位、公共时间轴、逐点时间 | 定位、控制权 |
| 局部估计 | Faster-LIO | 原生误差状态滤波、零偏/重力、逐点去畸变、局部几何约束 | 地图全局坐标、公开 TF |
| 高频传播 | `estimation/prediction.py` + `lio_predictor.py` | 扫描末端后验 + 新 IMU → 有界 50 Hz 局部预测 | 匹配、修改 LIO 状态、SDK 速度积分 |
| 地图校正 | FastGICP + `lio_localizer.py` | PCD 匹配、初值/轮次确认、地图锚点和恢复策略 | 高频导航滤波、公开动态 TF |
| 全局融合 | `robot_localization/ekf_navigation` | 高频 twist + 已验证 6D PCD pose、延迟观测回放 | SDK、外参修补、输出有效性授权 |
| 输出边界 | `estimation/navigation.py` + `navigation_output.py` | 重置握手、同时间插值、时效/跳变门控、机身参考、唯一公开动态 TF | 地图匹配、Web 文件管理、运动控制 |
| 展示/会话 | 现有 Web + Foxglove | 选图、启停、初值、状态与可视化 | 传感器融合计算 |

`estimation/` 纯数值/状态模块不导入 ROS、SDK、Web 或文件管理器。
`contracts.py` 负责输入契约及杆臂/协方差转换，`configuration.py` 只把一个 YAML 映射到节点参数。
`estimation_ros.py` 仅转换消息；ROS 端点和服务调用留在节点适配层。

这不是三个叠加的 EKF：局部保留 Faster-LIO 原生滤波；新增传播器只是从不可变后验重放 IMU，
不构造第二套局部滤波、不反馈修改 LIO；全局才使用独立 robot_localization EKF。

## 高频究竟来自哪里

- Faster-LIO 几何更新仍随实际有效扫描，当前 Airy 链路通常约 10 Hz；没有把点云匹配改成 50 Hz。
- 后验额外导出世界速度、陀螺/加速度零偏、重力和原生加速度缩放系数。
- 高频传播使用扫描结束之后真实接收的 IMU，包含完整三维旋转、速度和位置积分。
  新的延迟 LIO 后验到达后，丢弃上次预测、从新后验重放有界 IMU 历史，不重复积分。
- 允许末端最多 25 ms 零阶保持传播，明确上报 `extrapolation_sec`；不是补造 IMU 消息。
  LIO 后验最多 250 ms、IMU 间隔硬限 50 ms；不能无限惯性漂航。
- SDK OnMcData 保留真实遥测/频率。默认链路不积分 MC 偏置，不用 OnSpeedData 或状态速度代替。

robot_localization 配置：全三维、`predict_to_current_time=true`、`smooth_lagged_data=true`、
2 s 历史、`publish_tf=false`。只融合预测器的六轴 twist 和已确认 PCD 的六轴 pose，
不再加入同源 LIO pose、原始 IMU 或另一份 MC 作为“独立”运动观测。
LIO twist 与 PCD pose 仍共享部分激光信息；该 EKF 没有完整跨源协方差模型，
使用保守协方差下限/传播增长，不宣称是严格独立、最优或已实机标定的统计估计。

## 下游接口和坐标

统一前缀 `/d1max/localization/`：

| 接口 | 参考点/坐标 | 目标速率 | 使用者 |
|---|---|---|---|
| `odometry/local` | `d1max_loc_odom` 中的机身中心；child=`d1max_loc_base_link` | 50 Hz | 后续局部控制器、局部代价图 |
| `odometry/global` | `d1max_loc_map` 中的机身中心；相同 child | 50 Hz | 全局导航/记录；只在门控通过时输出 |
| `pose` | 地图中的 tracking 雷达参考点，保留原语义 | 50 Hz | 原 Foxglove 定位箭头 |
| `trajectory` | 地图中的 tracking 轨迹 | 5 Hz / 1800 点 | 原 Foxglove 轨迹 |
| `navigation/status` | 有效性、实际频率、源龄、重置/故障、标定状态 | 5 Hz | 输出层诊断；汇入 Web health.navigation |
| `status` | 整体匹配/定位会话状态 | 5 Hz | Web 与原 Foxglove 状态面板 |

TF：`d1max_loc_map → d1max_loc_odom → d1max_loc_base_link → d1max_loc_tracking → d1max_loc_lidar`。
前两条动态 TF 仅由 navigation_output 发送；Faster-LIO 与 RL 可以创建空闲广播端点但禁止发 TF。
body→tracking 是现有配置的固定刚体关系；tracking→lidar 保持归一化后的恒等关系。
这不是厂商原始雷达坐标、足底坐标或“自动重力校平”。没有改 PCD、压平地面、丢弃真实俯仰/高度。

同一公开时刻 t，从两份短历史插值到共同覆盖的真实时间，不把旧 pose 改时间戳：

`T_map_odom(t) = T_map_tracking_filtered(t) × inverse(T_odom_tracking_predicted(t))`

机身 pose 乘 `inverse(T_body_tracking)`；twist 同时旋转轴向并减去 `omega × 杆臂`，
协方差按相应参考点变换。不能只修改 child_frame 名称就把雷达里程计冒充机身里程计。
局部 odom 不受全局地图校正直接跳变；LIO 自身的小幅扫描更新仍存在，异常大跳变停止输出。

未确认全局初值时可以有局部 odom，但没有 global odom / map→odom / pose。
局部和全局都有效时使用同一时间戳；正常有约一个调度周期的对齐延迟。
规划应使用 map，局部控制应使用连续 odom；不能让局部控制直接追随可能重定位跳变的 map 位姿。

## 契约和失效处理

- 内部 `lio/local_sample`、`prediction/local_sample`、`map_alignment` 为 schema=1 的有序 JSON 契约。
  携带 epoch、valid/fault、源/接收时间；纳秒用十进制字符串避免 JS 精度损失。
- 地图契约另有 seed_id 和确认数，至少三次同初值连续确认。用户画箭头不直接重置全局滤波。
- 新确认的 `(epoch, seed_id)` 触发一次异步 `SetPose`，等待回执后重新建立滤波历史。
  2 s 回执不明则锁定 `filter_reset_unknown`，禁止自动重复重置，需停止并重启定位。
- 首次重置将已验证地图锚点传播到最新局部状态时刻；重置前的 PCD 不再重复融合。
  后续迟到 PCD 保留原始扫描时间，由 RL 回退/回放。旧轮次/旧初值不能授权新输出。
- 公开输出同时检查局部和 IMU 新鲜度、地图校正/契约年龄、EKF 新鲜度、同时间历史覆盖、
  相对已验证锚点误差和单步地图修正。私有 EKF 仍在预测不等于公开定位有效。
- 默认最大单步额外地图修正 0.10 m / 0.08 rad；相对锚点误差 0.5 m / 0.3 rad。
  这是异常拒绝阈值，不是靠钳制坐标掩盖错误，也不是精度承诺。
- LIO 因严重 IMU 缺口重建局部轮次时清空种子/轨迹/全局滤波授权；重新静止初始化并给新初值。
  局部恢复预算、时钟回跳保护仍保留。任何节点退出会停整个定位 launch，避免半套链路继续运行。
- 断流后停止公开输出，实际频率归零、localized=false。TF/Foxglove 可能仍缓存最后画面；
  下游必须同时检查 status 和时间戳，不能把“有一条旧 TF”当作定位仍有效。

## 一个配置文件

`config/localization.yaml` 是模板；Web 每次启动保存会话配置快照，修改只作用于新会话。

- `navigation_estimation.output_rate_hz`：唯一输出频率源，映射预测器、输出器和 RL；默认 50。
- `navigation_estimation.prediction.*`：传播时长、末端保持、IMU 缺口、缓冲长度。
- `navigation_estimation.limits.*`：新鲜度、异常/重置门限。
- `navigation_estimation.filter_process_noise_diagonal`：15 维 RL 状态过程噪声，启动时展开矩阵。
- `ekf_navigation`：测量选择、观测队列、延迟历史、异常观测门限。
- `lio_localizer`：唯一 map/odom/tracking、外参和标定状态配置；输出层复用，不复制另一套外参。

未知参数拼写、无界传播、重复融合源、2D 模式或冲突 TF 发布设置会在启动前拒绝。
`navigation_estimation.enabled=false` 可显式回退到原始 LIO 扫描频率输出；其 odometry 仍是旧 tracking 参考。
`legacy_ekf` 保留更早的 MC 双 EKF 测试后端。切换后端会改变接口语义，不应运行中修改。

## 验收边界

离线：`test_navigation_estimation.py` 检查纯估计/契约，`test_navigation_output.py` 检查 ROS 消息/重置边界；
`D1MAX_QA_BACKEND=lio_pcd D1MAX_QA_MOTION=1 python3 test/smoke_localization.py` 跑真实 LIO + RL 节点、
合成扫描/IMU，只使用本机隔离 Zenoh 域。覆盖静止偏置、约 2.06 m/s 和 1 rad/s 模拟运动、
实际高频输出、TF/机身参考/同时间、匹配暂停恢复、长 IMU 缺口新轮次、重新给初值和最终断流。

这不等于实机高速验收。外参/时间仍为未验收，`navigation_ready=false`；
下次需现场核对静止漂移、加减速/转弯、振动/丢包和地图几何退化，再定义可用速度范围。
本模块没有实现楼层识别、跨层地图切换、楼梯导航或运动控制。

依据当前安装的 robot_localization 3.5.4 / Humble：
[状态估计配置](https://github.com/cra-ros-pkg/robot_localization/blob/humble-devel/doc/state_estimation_nodes.rst)、
[输入选择和重复观测](https://github.com/cra-ros-pkg/robot_localization/blob/humble-devel/doc/configuring_robot_localization.rst)。
尤其注意 `sensor_timeout` 仅让 RL 继续预测，不会自动撤销可信输出；这由独立输出层完成。
