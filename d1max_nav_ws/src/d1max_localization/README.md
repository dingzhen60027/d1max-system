# D1 Max 单楼层定位（调试版）

**2026-09-12：当前默认为 Faster-LIO / IMU 高频传播 + PCD + robot_localization 全局 EKF。**
导航输出 50 Hz、分层接口及失败策略请看 [导航输出架构](NAVIGATION_ESTIMATION.md)。
使用、配置、坐标系、故障恢复与验收边界请看 [当前实现说明](ROBUST_LOCALIZATION.md)。
下文双 EKF / MC 积分说明仅适用于显式选择 `legacy_ekf` 的历史后端，不代表当前默认链路。

Web 启动、设置初始位姿和停止；Foxglove 只看地图、实时扫描、位姿与轨迹。**不启动 Nav2、不申请 SDK 控制权、不发送速度、姿态或急停解除命令。仅 Zenoh，不使用 Fast DDS。**

## 历史后端的数据与算法（仅 legacy_ekf）

参考 `/home/dndx/go2_nav/src/go2_localization` 的双 EKF / 独立 GICP 实现，替换 Go2 专有传感器接入：

| 模块 | 输入 | 作用 |
|---|---|---|
| D1 SDK 监看进程 | `OnMcData`；SDK 0.1.0 文档定频 50 Hz，界面显示实收 Hz | `v_body` / `omega_body`，不新建第二个 SDK 连接 |
| 双 Airy96 适配 | 前后 PointCloud2、前雷达 IMU | 沿用 D1 建图标定，合并点云、统一轴向 / IMU 单位、保留逐点时间 |
| 局部 EKF · 50 Hz | MC 三轴速度 + 统一去零偏的 IMU 三轴角速度 | 连续局部里程计；不积分振动加速度，不融合来源不明的厂商绝对位姿 |
| 独立 FastGICP | 完整定位 PCD + 旋转 / MC 平移去畸变扫描 + 局部里程计预测 | 初始 3 次稳定匹配；时间、重叠率、RMSE、fitness、位移、yaw、Z / 倾角门限 |
| 全局 EKF · 30 Hz | 局部 **twist only** + 门控 ICP XYZ / roll / pitch / yaw | 保留真实高度和完整姿态约束；不把局部位置和速度重复作为独立观测 |
| Supervisor | 新鲜度、初值、融合诊断、两路 EKF | 首次验证后才发布全局 TF / 位姿；断流与校正超时撤销 localized |

GICP 获取第一组已确认匹配后才重置全局 EKF；Web 初值本身不会重置 EKF。匹配预测使用上次被接受的绝对位姿 + 局部 odom 增量，并按扫描时刻对齐。丢弃滞后、未来、乱序扫描和完成计算时已经过期的结果。去畸变使用同一固定 tracking 基底的三轴 IMU / MC 速度；MC 速度先做完整 omega×杆臂补偿，再积分平移。IMU 或速度没有覆盖整帧、跨越过大缺口时拒绝该帧，不以零速度顶替。MC 与雷达相对时间仍待实机标定。

两路 EKF 均使用三维模式，局部仅融合三轴线速度 / 角速度，局部位姿是启动时刻为零的连续相对里程计，会漂移；不冒充重力绝对姿态。全局六个位姿轴必须全部有 ICP 观测，不能沿用二维滤波的仅 yaw 姿态配置。Supervisor 在发布可信 TF / pose / trajectory 前，将已接受 ICP 用局部里程计增量传播到同一个有插值覆盖的 EKF 时刻，再比较 Z 轴方向：相差超过 10°、插值缺口超过 0.20 秒、ICP 超过 0.75 秒或姿态非有限时拒绝输出，Web 降为 `degraded` / `localized=false`。门限在同一 YAML 的 `max_output_*`，诊断在 `health.output_guard`；不强行把地图坡度或 Z 设为零。重新给初值后清除旧 ICP 校验缓存。仅检查相对倾角，不代表已完成位置 / yaw 精度或导航安全验证。

SDK 速度唯一来源为 `IDataCallback::OnMcData`。不接受 `OnSpeedData`、RobotState 速度或无来源的数据；兼容参数 `allow_sdk_state_velocity_fallback` 即使设为 true 也不再启用降级。RobotState 仅提供前向、安全、电量等状态。`SetMcConfig(true, 0)` 的第二项为超时，不是频率。Web 从 `health.speed_report.observed_hz` 显示实收频率。

桥保留原始 `source_timestamp_ns`（十进制字符串，避免 JS 精度损失），拒绝零值、重复、倒退和过期样本。`stamp_unix` 为首次本机接收时刻加 MC 源时间增量；保留源采样间隔，但不是硬件同步，不保证消除网络延迟或与雷达的时钟误差。时差异常停止接纳，重新连接建立新会话；不会偷偷重置地图 TF。定位融合 MC 的三轴机身线速度，经完整三轴杆臂补偿后送入跟踪帧，MC 世界坐标位置、四元数不直接当作地图位姿。`HeadDirection` 必须为机头前向（1）且状态新鲜；尾部前向或未知时停止速度融合，不自行切换机器狗状态。IMU 三轴零偏统一由 Supervisor 用 100 个连续静止样本估计，供 EKF 与去畸变共用；不再单独估计第二套偏置或只给点云施加启动重力旋转。

## 使用

静止标定参数：`stationary_speed=0.05`（m/s）、`stationary_angular_rate=0.025`（rad/s）分别限制 MC 线速度与角速度，归一化 IMU 还须满足 `gyro_bias_max_rate=0.04`（rad/s），连续采集 100 个合格样本。2026-09-11 用户确认静止时 MC 速度范数约 0.027–0.038 m/s，旧 0.025 m/s 阈值会一直重置标定。新阈值仅用于陀螺标定，不将 MC 原始速度置零，也不表示完成速度偏差或低速运动精度标定。

新鲜度与接收检查统一使用现有最多 100 ms 的未来时间容差，避免共享时差估计产生的小幅超前被误报成断流；MC 还同时检查原始接收时间，断流不能靠未来时间戳延长有效期。原始时间戳、时差参数及过期/乱序保护不变，时间同步仍未实机标定。

1. 打开 `http://127.0.0.1:8766/#/2d/navigation`，先在地图版本中选用单层地图。
2. 连接机器狗数据，等待 SDK / 双雷达在线；启动定位并保持静止。
3. 与 RViz2 一样：按下选机身位置、拖动指向机头、松开提交，Esc 取消。离线只记下箭头，不自动启动；就绪后可手动提交。+X=0°、+Y=90°，精确 XYZ / 角度放在高级设置中。
4. Foxglove 数据源 `ws://127.0.0.1:8769`，左侧 `PCD · LOCALIZATION`：地图、青色扫描、橙色未验证初值预览、黄色位姿 / 轨迹。Web 按钮旁定位图标：绿=锁定，黄=运行但未锁定，灰=无新鲜状态。图标锁定不代表 Nav2 就绪。
5. Web 停止定位清理全部定位节点，保留共享 SDK / 感知连接；Stop Web 也会停止其定位进程组。Foxglove Disconnect 关闭数据链路，不是机器人急停；定位会因断流失效，需要另行停止。

定位使用所选版本目录中的 **`localization.pcd`**，不是高度切片后的点云或栅格图。2D 涂改仅影响 PGM，不会挖掉定位 PCD。启动后锁住版本，禁止切换、取消选用、归档或删除它；不覆盖任何源 PCD。停止后解锁。

## 唯一参数文件与记录

### 实机输入时间轴（2026-09-10 链路修订）

`dual_lidar_adapter.input_clock_mode` 支持 `strict`（要求真实时钟同步）和
`estimate_shared_epoch`（当前调试配置）。后者从至少 100 个、跨至少 1 秒的前 IMU
递增时间戳估计一次共享时差，将同一常量用于前后雷达、扫描 timebase 和 IMU，
保持逐点相对时间不变。原始传感器 topic 不改、不逐帧用接收时间重盖时间戳。
估计包含未知的最小单向网络延迟，**不是 NTP/PTP 或精确时间标定**；仅用于静态 / 低速调试，
`navigation_ready=false` 和未标定外参约束不变。先修复网络拥塞再启动这一校准。

收到 `/clock`、传感器时钟大幅跳变、主机时钟跳变时锁定拒收，必须核对并重启；
过期、未来、重复 IMU 不进入新鲜输入。IMU 断流不再发布新的转换扫描；恢复后不重新估计
时差，不能把旧数据自动变成“新鲜数据”。两路雷达仍必须满足原有 5 ms 同步门限。
`/diagnostics` 的 `d1max_localization/input_clock` 和 Web health 的 `input_clock`
报告模式、时差、样本数及就绪状态。真实主机/传感器时钟同步后应使用 `strict`。

已通过人为 17,320,211 秒时差的隔离全链路测试（含断流撤销可信输出）；
2026-09-10 新版已启动实机：SDK / IMU / 点云输入就绪，共享时差约 17,320,211 秒，等待用户重新给初值。不能据此声称实机定位成功或运动精度合格。长延迟包直接丢弃，不会仅因网络积压误锁时间轴；真实回跳、前跳、主机时钟跳变、回放保护不变。

参数：`config/localization.yaml`。涵盖双雷达标定、输入 topic、轴向 / 单位、速度协方差、两路 EKF、去畸变、GICP、门限、新鲜度和静止零偏。每次启动拷贝一份到 Web `data/localization/<session>/localization.yaml`，同时保存 `session.json`、`status.json`、`runtime.log` 和初值回执。修改模板只对下一次启动生效。

固定 systemd 用户单元 `d1max-localization-managed.service`，唯一会话标识校验、文件锁、进程组归属检查、防重入。任意定位节点退出会终止整个 launch；停止 SIGINT 等待 12 秒，超时清理同一 cgroup，包含另建会话的子孙进程。未知占用拒绝接管；无自动重启 / 重发初值。

## 帧与显示接口

独立树 **`d1max_loc_map → d1max_loc_odom → d1max_loc_tracking → d1max_loc_lidar`**。最后一条是定位包归一化后的同原点、同基底恒等变换；不是原始雷达的外参。tracking 原点为前 Airy 归一化参考点，不是机身中心或足底。全局保留建图 Z；不假造与厂商 / Foxglove 旧显示树之间的单位变换。无 URDF。

所有定位 topic 在 `/d1max/localization/`：`map_cloud`、`scan_leveled`、`scan_initial_preview`、`odometry/local`、`odometry/global`、`pose`、`trajectory`、`status`。**只有 pose / trajectory 和 Supervisor 的全局 TF 是经新鲜度及匹配确认后的输出**；原始 global odometry 在初始化前可能已经存在，不能据此判定定位成功。暂停时 Foxglove 可能保留旧画面，以定位图标和 Web 新鲜度为准。

## 初值契约与职责边界（Go2 对齐）

- `frontend/src/lib/pose-estimate.js`：像素 / 地图坐标和角度的纯函数，支持带旋转的地图 origin；`PoseEstimateMap.jsx` 只处理手势，不访问 ROS / API。
- Web `POST /api/localization/initial-pose` 明确传 `reference: body`，XYZ 和 yaw 全部属于机身。内部 `tracking` 入口供离线测试使用，不重复转换；旧 mailbox 保持旧语义，不重解释。
- `d1max_localization/initial_pose.py`：按 Go2 的 SE(3) 方式计算 `T_map_tracking = T_map_body × T_body_tracking`。机身到 tracking 的位置与 SDK 速度转换共用 YAML 的 `tracking_offset_body` / `sdk_to_tracking_yaw`，只组合一次；ACK 返回转换后的坐标便于核对。
- IMU 负责短时三轴旋转 / 去畸变，GICP 负责地图中的 XYZ / 完整朝向匹配；**没有自动地面估计、固定地面 Z 补偿、地图改写或新增区域搜索**。局部 GICP 仍需在正确楼层给大致正确的位置、方向和高度。
- `scan_initial_preview` 仅把最新扫描按未验证种子 / 候选变换到地图坐标，供叠加查看。不发布初值 TF，不进入任何 EKF，不代表已定位；锁定时发空云清除，显示端仅保留 0.5 秒。
- `/diagnostics → d1max_localization/matcher → health.matcher` 报告匹配尝试、未收敛次数、耗时、点数、连续确认数及拒绝原因；HTTP 排队、节点接收、匹配锁定是三个不同状态。

## 2026-09-11 修复后的时序 / 质量契约

- 固定 tracking 坐标系：点云、MC 线速度、IMU 三轴角速度一致；`scan_leveled` 只保留历史 topic 名以兼容唯一 Foxglove 布局，内容不再做额外“校平”。
- 预测 / TF 全部使用 XYZ 线性插值 + 四元数 SLERP；禁止用最近邻或接收时刻伪装量测时间。初次全局重置先将延迟 ICP 传播到最新有覆盖的局部位姿时间，按该时间发布。全局 EKF 开启 2 秒延迟量测历史。
- 任意已处理扫描失败都打断初始连续确认；跟踪失败不自动解锁、放宽搜索或触发重定位。
- 匹配新增默认 0.50 m 内对应点比例 ≥ 0.35、RMSE ≤ 0.30 m。PCL fitness 截断参数正确使用平方米。协方差按残差和重叠率保守放大，融合桥不再覆盖为更小的固定噪声；这不是标定过的统计置信区间，仍无重复结构唯一性 / 完整退化检测保证。
- 可信 pose / TF 独立 30 Hz 输出调度（仅发新、有完整时间覆盖的状态，实际频率看 `output_observed_hz`），轨迹 5 Hz、健康 / 文件写入 5 Hz。30 Hz 是上限配置，不是实测承诺。
- `mc_time_offset_sec` 为加到 MC 源时间对齐值的固定标定修正量，默认 0；不猜时差。`time_alignment_verified=false`、`extrinsics_verified=false` 保留。只修改 YAML 模板对下次启动生效；不覆盖旧会话配置。
- 原有地图没有改写。PCD 候选地面约 2° 的倾斜未确认是真实坡度还是建图误差，禁止自动压平。旧 2D 版本空白标自由的问题仍需现场复核，新生成版本已经默认 unknown；未授权其用于自动行走。

## 实机限制（必须核对）

- 雷达内部 / 双雷达标定沿用本机建图参数；**SDK 机身速度到前雷达的杆臂 / 朝向仍采用 CAD 近似，`extrinsics_verified: false`**。现场须核对 SDK 速度轴定义、头部角度影响、外参和时钟同步；不能仅把开关改为 true 当作已标定。
- 当前任务仍限单楼层，使用三维传感器运动模型以保留机身侧倾、俯仰与上下运动；没有跨层、楼梯、自动重定位、可通行性验证或 Nav2 执行。平面 / 重复走廊可能误匹配，连续确认及门限不是全局唯一性保证。
- Web 初始 Z 使用原地图坐标中的机身高度；不是固定离地高度。机身到前雷达的固定杆臂由后台转换，不能再手动叠加一次。
- `navigation_ready` 始终为 false。2026-09-10 已通过隔离模拟链路并恢复实时输入；**未验证实机精度、运动中的融合质量或导航安全**。

## 构建与离线测试

先 source D1 `d1max_ros2_env.sh` 和 `/home/dndx/d1max_nav_ws/install/setup.bash`；在 nav 工作区：

```bash
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
colcon build --packages-select d1max_localization --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON
colcon test --packages-select d1max_localization
/usr/bin/python3 src/d1max_localization/test/smoke_localization.py
```

完整 smoke 测试自建仅回环的 Zenoh 路由（17447，Domain 214），生成临时非对称房间 PCD，模拟双原始雷达 / IMU / SDK，不访问机器狗，不修改用户地图。验证无初值时无全局 TF、粗初值连续匹配锁定、双 EKF、断流后停止可信输出、退出清理。合成数据误差不代表真实精度。测试报告在其打印的 `/tmp/d1max-localization-smoke-*` 目录。

源实现 / 依赖说明见 `UPSTREAM.md`。robot_localization 官方配置依据：[双滤波坐标系](https://github.com/cra-ros-pkg/robot_localization/blob/ros2/doc/state_estimation_nodes.rst)、[融合变量和重复观测](https://github.com/cra-ros-pkg/robot_localization/blob/ros2/doc/configuring_robot_localization.rst)。
