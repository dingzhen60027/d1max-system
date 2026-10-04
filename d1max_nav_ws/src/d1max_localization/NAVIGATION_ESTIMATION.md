# 导航定位输出 · 当前默认架构

2026-09-24。默认 `lio_pcd`，连续运动是下游的数据契约，不以界面是否变绿为验收标准。
Web 只负责启停；初始位姿和三维目标在 RViz 中指定。
定位进程不连接 SDK、不控制机器人、不启动 Nav2；通信仍为 Zenoh。

## 分层与职责

| 层 | 模块 | 唯一职责 | 不负责 |
|---|---|---|---|
| 输入 | `dual_lidar_adapter` | 雷达/IMU 轴向、单位、公共时间轴、逐点时间 | 定位、控制权 |
| 局部估计 | Faster-LIO | 原生误差状态滤波、零偏/重力、逐点去畸变、局部几何约束 | 地图全局坐标、公开 TF |
| 高频传播 | `estimation/prediction.py` + `lio_predictor.py` | 扫描末端后验 + 新 IMU → 有界 50 Hz 局部预测 | 匹配、修改 LIO 状态、SDK 速度积分 |
| 地图校正 | FastGICP + `lio_localizer.py` | PCD 匹配、初值/轮次确认、地图锚点和恢复策略 | 高频导航滤波、公开动态 TF |
| 全局融合 | `robot_localization/ekf_navigation` | 高频 twist + 已验证 6D PCD pose、延迟观测回放 | SDK、外参修补、输出有效性授权 |
| 连续输出 | `estimation/continuity.py` + `estimation/navigation.py` | 单一运动时钟、原始/公开历史、校正目标与平滑、时效及轮次契约 | ROS、SDK、任务管理 |
| ROS 边界 | `navigation_output.py` | 重置握手、消息/机身参考转换、唯一公开动态 TF、实际频率诊断 | 地图匹配、界面、运动控制 |
| 展示/任务 | Web + RViz、独立规划会话 | 启停、三维初值/目标、质量可视化、有界暂停与重新规划 | 传感器融合计算 |

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
- 真实 IMU 末端最多保持 25 ms；之后转为明确标记的恒速/恒角速短时预测，
  从最后真实 IMU 到输出时刻**总计最多 100 ms**，不是额外再预测 100 ms。
  位置和姿态真正按运动模型推进，不重发旧 pose、不补造 IMU、不更改其源时间戳。
  `prediction_mode=coasting`、`degraded=true`，协方差随未观测运动时间增长。
  高动态时还受 0.08 rad 未支撑转角限制，不能把静止场景的容忍用于任意高速转动。
- LIO 后验最多 250 ms；真实双端支撑的内部 IMU 间隔最多 100 ms，超过 15 ms
  降级并增加不确定性。内部缺口保护与末端预测预算是两个不同约束，彼此不能绕过。
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
`navigation/status.quality` 统一报告 tracking / coasting / paused / initializing / fault；
`*_observed_hz` 按实际发布回调时间统计，`*_timestamp_hz` 单独统计源时间间隔，
`global_max_publish_gap_sec` 暴露近 2 秒最大发布间隔，防止成批迟到仍显示“50 Hz”。
前两条动态 TF 仅由 navigation_output 发送；Faster-LIO 与 RL 可以创建空闲广播端点但禁止发 TF。
body→tracking 是现有配置的固定刚体关系；tracking→lidar 保持归一化后的恒等关系。
这不是厂商原始雷达坐标、足底坐标或“自动重力校平”。没有改 PCD、压平地面、丢弃真实俯仰/高度。

原始 LIO/IMU 估计用于运动观测、地图锚点校验和滤波器初始化；连续输出历史用于公开 TF。
两种历史均保留真实时间戳，不能把地图对原始 LIO 的锚点直接用于平滑后的估计。

设 EKF 样本的真实时刻为 k，先在该历史时刻求校正目标：

`C_target(k) = T_map_tracking_EKF(k) × inverse(T_odom_tracking_continuous(k))`

每个新的局部运动样本 t 都产生自己的全局估计：

`T_map_tracking_target(t) = C_target(k) × T_odom_tracking_continuous(t)`

公开输出只由新局部样本触发。EKF 回调更新校正目标，既不生成第二条输出时钟，也不把旧 EKF
位姿直接盖成当前时间。目标 k 仍需小于等于 t、历史覆盖、数据龄不超过 80 ms；
当前 IMU、LIO、地图契约也各自检查，新的 global 时间戳绝不延长旧数据的寿命。

实际运动先传播，只有估计校正按真实 dt 平滑：局部与全局各限制额外校正速度
0.15 m/s、0.10 rad/s，时间常数 0.30 s。**不是把机器人真实运动限制为 0.15 m/s**。
残差进入协方差与诊断；超过允许残差暂停，而非长期“追着错误慢慢走”。
首次给定初值/新轮次允许建立新位置，不能跨重定位强行拼成一条假连续轨迹。

最终公开 TF 由同一时刻的两份公开位姿确定：

`T_map_odom(t) = T_map_tracking_output(t) × inverse(T_odom_tracking_continuous(t))`

机身 pose 乘 `inverse(T_body_tracking)`；twist 同时旋转轴向并减去 `omega × 杆臂`，
协方差按相应参考点变换。不能只修改 child_frame 名称就把雷达里程计冒充机身里程计。
局部 odom 不受地图校正直接跳变，LIO 后验切换的小修正也受连续性约束；异常大跳变停止输出。
EKF 和公开 body twist 始终来自原始运动估计，不能把姿态平滑/地图修正当作新的真实速度。

未确认全局初值时可以有局部 odom，但没有 global odom / map→odom / pose。
局部和全局都有效时使用同一新运动样本的时间戳；校正观测有自己的历史时间，不阻塞输出时钟。
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
- 历史支撑按历史时刻检查，当前源龄另查。地图匹配身份 `map_localized` 与运动输出质量分开：
  正常跟踪、短时预测、输出暂停、失锁/故障。短时预测仍是有界估计，不是“等待重新定位”。
  输出暂停时规划不得使用旧轨迹；界面只可橙色保留原时间戳的机器人标记至原 TTL 0.5 s。
  新轮次/新初值/硬故障立即撤销，不能用视觉保留作为规划授权。
- 规划任务可以在同一 session/map/epoch/confirmed seed 下有界保留目标意图，当前参考路径和
  局部样条立即撤销。恢复必须连续新数据确认并从当前位姿重新计算；不复活旧轨迹。
- 默认最大单步额外地图修正 0.10 m / 0.08 rad；相对锚点误差 0.5 m / 0.3 rad。
  这是原始校正异常拒绝阈值，与正常校正的平滑速率分开；不能用平滑隐藏大跳变。
  局部平滑残差上限 0.25 m / 0.15 rad，全局 0.5 m / 0.3 rad，均不是精度承诺。
- 输出间断不是坐标轮次故障：同一 epoch 的真实新鲜 LIO 状态到达后，利用两端原始估计的
  实际相对位移恢复公开位姿，保留已有公开偏移；不跨间断盲积分、不补发缺失时刻的假位姿。
  恢复时丢弃旧全局校正授权，等待屏障之后的新 EKF 校正，再恢复全局输出。
  超过 250 ms 的输出空窗不再永久锁成 `local_continuity_gap`；真实大跳变、时钟和 epoch 错误仍拒绝。
- 全局协方差保守包含校正边际、当前局部协方差上界、校正目标数据龄和剩余平滑误差。
  不把共享激光/运动信息当作独立观测，不用两个边际相减冒充增量协方差。
  这是工程不确定性模型，不宣称是含跨时刻相关项的完整 SE(3) 统计最优传播。
- LIO 因严重 IMU 缺口重建局部轮次时清空种子/轨迹/全局滤波授权；重新静止初始化并给新初值。
  局部恢复预算、时钟回跳保护仍保留。任何节点退出会停整个定位 launch，避免半套链路继续运行。
- 断流后停止公开输出，实际频率归零、localized=false。TF/Foxglove 可能仍缓存最后画面；
  下游必须同时检查 status 和时间戳，不能把“有一条旧 TF”当作定位仍有效。

## 一个配置文件

`config/localization.yaml` 是模板；Web 每次启动保存会话配置快照，修改只作用于新会话。

- `navigation_estimation.output_rate_hz`：唯一输出频率源，映射预测器、输出器和 RL；默认 50。
- `navigation_estimation.prediction.*`：后验时长、末端保持、短时预测、未观测运动噪声、IMU 缺口。
- `navigation_estimation.limits.*`：源龄、校正速率/时间常数/残差、异常/重置门限。
- `navigation_estimation.filter_process_noise_diagonal`：15 维 RL 状态过程噪声，启动时展开矩阵。
- `ekf_navigation`：测量选择、观测队列、延迟历史、异常观测门限。
- `lio_localizer`：唯一 map/odom/tracking、外参和标定状态配置；输出层复用，不复制另一套外参。

未知参数拼写、无界传播、重复融合源、2D 模式或冲突 TF 发布设置会在启动前拒绝。
`navigation_estimation.enabled=false` 可显式回退到原始 LIO 扫描频率输出；其 odometry 仍是旧 tracking 参考。
`legacy_ekf` 保留更早的 MC 双 EKF 测试后端。切换后端会改变接口语义，不应运行中修改。

## 验收边界

离线：`test_navigation_estimation.py` 检查纯估计/契约，`test_navigation_output.py` 检查 ROS 消息/重置边界；
`test_prediction_coast.py` 与 `test_navigation_continuity.py` 检查运动中的数据延迟、恢复、
25/50/75/100 ms 到达空窗、20 ms 输出时钟、真实时间戳、局部/全局校正速率与长期断流。
合成到达测试通过不等于真实网络的实收 50 Hz；还需隔离 ROS 节点实收验证和现场验收。
`D1MAX_QA_BACKEND=lio_pcd D1MAX_QA_MOTION=1 python3 test/smoke_localization.py` 跑真实 LIO + RL 节点、
合成扫描/IMU，只使用本机隔离 Zenoh 域。覆盖静止偏置、约 2.06 m/s 和 1 rad/s 模拟运动、
实际高频输出、TF/机身参考/同时间、匹配暂停恢复、长 IMU 缺口新轮次、重新给初值和最终断流。

这不等于实机高速验收。外参/时间仍为未验收，`navigation_ready=false`；
下次需现场核对静止漂移、加减速/转弯、振动/丢包和地图几何退化，再定义可用速度范围。
本模块没有实现楼层识别、跨层地图切换、楼梯导航或运动控制。

### 2026-09-24 连续输出验收

真实 ROS predictor + robot_localization EKF + navigation_output，在仅回环网络的独立 Zenoh 域运行。
输入为合成 200 Hz IMU、延迟 100 ms 的 10 Hz LIO 后验、5 Hz 地图校正，运动 22 s 后全部断流 2 s。
没有运行真实 LIO 点云前端，也没有连接机器人，不能据此宣称实机精度或网络问题已修复。

- 预热后局部/全局各 950 条：实收 50.003 / 50.005 Hz，最大回调间隔 26.34 / 25.57 ms。
- 三次 75 ms IMU 交付空窗确实进入短时预测，运动阶段状态有效率 100%，无 fault。
- 位姿与时间戳单调、局部/全局同源时间戳、两条动态 TF 同时间，无重复样本。
- 全部输入停止后最后公开输出约在 43 ms，之后无位姿/TF伪续发；最终有效性为 false。
- 16 项验收通过，所有 4 个子进程正常退出，测试端口释放。

复验入口 `tools/validation/smoke_navigation_continuity.py`；证据见
`log/offline_navigation_continuity/20260924_183940_ede3acec/report.json`。
这是边界与实时发布验证，不是“永远不断流”的保证；剩余实机验收是独立门槛。

依据当前安装的 robot_localization 3.5.4 / Humble：
[状态估计配置](https://github.com/cra-ros-pkg/robot_localization/blob/humble-devel/doc/state_estimation_nodes.rst)、
[输入选择和重复观测](https://github.com/cra-ros-pkg/robot_localization/blob/humble-devel/doc/configuring_robot_localization.rst)。
尤其注意 `sensor_timeout` 仅让 RL 继续预测，不会自动撤销可信输出；这由独立输出层完成。
局部连续运动与地图全局校正的职责参考
[ROS REP-105](https://github.com/ros-infrastructure/rep/blob/master/rep-0105.rst)。
