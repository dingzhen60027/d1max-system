# D1 Max 单楼层定位（调试版）

**2026-09-12：当前默认为 Faster-LIO / IMU 高频传播 + PCD + robot_localization 全局 EKF。**
导航输出 50 Hz、分层接口及失败策略请看 [导航输出架构](NAVIGATION_ESTIMATION.md)。
使用、配置、坐标系、故障恢复与验收边界请看 [当前实现说明](ROBUST_LOCALIZATION.md)。
下文双 EKF / MC 积分说明仅适用于显式选择 `legacy_ekf` 的历史后端，不代表当前默认链路。

## 2026-10-02：启动全局重定位

新版开发入口在 `lio_pcd` 中加入 **全图搜索 → 初值 → 原生 GICP 连续确认 → 连续融合位姿 → BT 规划**。
`single_floor_session` 仍是正式会话入口。新开发会话调用 `global_localization.launch.py`，它运行
`GlobalLioLocalizer` 替代（不同时运行）旧种子节点，复用旧节点的匹配确认和融合合同。
已封存的发布包没有自动更新；必须整套构建、验收、封装后切换，不能只替换 XML 或 Python。
Web 启停、RViz 手动初值的职责不变。此功能不连接 SDK、不申请控制权，也不授权运动。

启动时使用同一份**原坐标定位 PCD**和 Faster-LIO 已去畸变的双雷达 `lio/deskewed`（tracking 坐标）。
与 PCT 的平整规划地图无关，不借用建图轨迹或用户初值进行全局搜索，不压平地图、不增加假 TF。
原始 XYZ 的三维窗口建立 FPFH 索引，RANSAC 提出候选，ICP 在原图上精配准。
默认检查全部有几何证据的有界子图；描述子仅排序，不以“前八个最相似”代替全图检查。
多候选按 XYZ 和姿态去重；不同楼层/远处相似走廊仍单独竞争。
重叠率、残差、几何可观测性及候选分差不足时不提交种子，保留手动入口。全局唯一性不是数学保证。
算法方法参考 [Open3D 官方全局配准流程](https://www.open3d.org/docs/release/tutorial/pipelines/global_registration.html)。

唯一参数入口是 `lio_localizer.global_relocalization.*`，见 `config/global_relocalization.yaml`；
`.enabled: false` 可恢复只用手动初值。`registration.backend: fpfh_ransac` 与纯算法模块分离，
上层不依赖 FPFH/PCT/SCAN 内部实现。当前仅此全局搜索后端可选，不声称已集成其他算法。
默认两线程、一个独立进程、一个请求、一个扫描槽；索引以地图内容/配置/库版本哈希缓存，
首建 120 秒、每次搜索 25 秒上限，超时终止 worker，不阻塞现有高频估计或无界堆积。

- 初始化要求 LIO 就绪、机头状态新鲜且前向、静止至少 0.6 秒。保存扫描时刻的插值 LIO 位姿；返回后按真实 LIO 增量传播一次，不能用最新 TF 配旧点云。
- 地图/会话/局部 epoch/请求身份必须一致；搜索中明显移动则拒绝结果。tracking 初值不再叠加机身高度或 SDK 朝向。
- `lio_localizer` 仍是唯一种子接收者；候选经原有 `fused_icp/initialpose` 进入匹配器，至少三次新扫描确认，且融合输出有效后才显示 ready。候选不直接发布机器人 TF。
- 手动 RViz 初值优先，立即退役 worker 和旧请求。已有种子/任务失锁后不自动进行全图跳转；LIO 重置后需重新给手动初值，不复活原运动授权。
- BT 的 `InitialLocalization` 节点有 60 秒任务等待期限；失败不计算路线。成功后记忆推进，普通地图校正不会重回此节点。
- 诊断 `/d1max/localization/global_relocalization/status` 和会话 `status.json.global_relocalization`。初次搜索失败可显式调用 `global_relocalization/retry` (`std_srvs/srv/Trigger`)，最多两次搜索；已有种子后只能手动给初值。

### 离线复现与已验证边界

工具只读 PCD / sqlite rosbag，不初始化 ROS、不发布初值/目标、不连接机器人：

```bash
source /opt/ros/humble/setup.bash
OMP_NUM_THREADS=2 OPENBLAS_NUM_THREADS=2 /usr/bin/python3 tools/validation/global_relocalization_probe.py \
  --map '/home/dndx/智元四足机器人D1 Max二次开发文档资料包v0.1.0/d1max_ros2/map_manager/data/processed/20260923_141005_sc-pgo-0923-crossfloor-structure-v1_2c7218/processed_map.pcd' \
  --bag experiments/near_field_feasibility/20260927_live_static_W2CBQT/capture01/bag \
  --topic /d1max/localization/perception/rays_raw --static-raw \
  --cache experiments/global_relocalization_20261002/cache \
  --output experiments/global_relocalization_20261002/recorded_static_reproduce.json
```

该历史包未录 deskewed，以上**仅静止原始扫描**测试先核验录制的 lidar→tracking 恒等变换、
LIO 速度和实际位移；不能用于运动中的去畸变验收。动态输入应使用 `--bag ...` 的默认 deskewed 话题。
`--scan query.pcd` / `--scan query.npz` 支持独立查询云；`--crop-center X Y Z` 是地图裁剪夹具，不冒充实机证据。

`experiments/global_relocalization_20261002/recorded_static_04.json`：未使用初值搜索，结果
tracking XYZ 约 `(0.545, 1.732, -0.022)`，重叠率 1.0、RMSE 0.088 m、候选分差 0.124，热缓存总耗时约 20.9 秒。
与历史 TF / LIO 记录位置一致，**不是定位精度 GT，也未完成本轮实机或动态自动初始化验收**。
原生 Open3D 另有未知刚体姿态正例及完全重复上下层拒绝反例；协议测试覆盖手动抢占、epoch/地图错配、迟到结果、运动/断流和确认超时；BT 覆盖等待、取消、超时及不重入。
稀疏扫描的特征一致率与几何重叠率分别统计；不能拿 Open3D 0.14 RANSAC 的对应点一致率直接筛掉正确的几何候选。

本轮编译输出只在 `experiments/global_relocalization_20261002/{build,install}`，未更新生产 install / 已封存发布包，未启动生产服务。正式自动初始化仍须现场静止/不同楼层/重复走廊验收。
旧参数 `config/localization.yaml`、旧定位节点/launch 和旧 XML 保持封存内容不变；新开发会话由 `single_floor_session`
选择 `navigate_with_global_relocalization.xml`，新的 localization launch 先载入重定位参数再载入会话参数。
会话可显式覆盖 `.enabled: false`。不要把新 XML 配旧 BT ELF，或把旧定位安装配新 launch。

### 使用真实行为树的原始录包回放

```bash
bash tools/validation/bt_localization_replay_entry.sh \
  --bag bags/slam_raw_20260923_010248_eb79d8 \
  --output experiments/global_relocalization_20261002/bt_replay_reproduce \
  --seconds 45 --rviz
```

该入口运行真正的 BehaviorTree.CPP `navigate_with_global_relocalization.xml`、Action 适配器、
Faster-LIO、原生 GICP 和 robot_localization。Python 只负责原始消息回放和进程生命周期，
不替代 BT、不伪造健康状态。仅回放前后雷达和前雷达 IMU，不使用录包里的旧位姿、TF 或控制。
使用域 219、独立且仅本机回环的 Zenoh 路由，不访问 SDK。索引预热后，BT 在定位前收到一个
只预览的地图支持目标；全图搜索时暂停虚拟时间，搜索完成后续播，不能循环旧扫描冒充新输入。
源消息时间保持原值；固定录包接收时钟到传感器时钟的偏移只用于 `/clock`，不是物理时钟标定。
`recorded_sim_time` 输入时钟模式必须同时满足隔离标识、指定域、Zenoh 和 `use_sim_time`；
不能用于实机。录包没有 SDK 机头状态，仅隔离边界允许 tracking 初值，实机前向检查不变。

开发版 `causal_lio_predictor` 复用封存的传播核心，只使用不晚于当前目标时刻的真实 IMU；
略晚的样本保留到下一拍，不修改时间戳，也不放宽源龄检查。原发布入口和传播核心未替换。
这解决了回放中“预测状态携带稍晚 IMU、原子 NavigationState 因源时间顺序错误拒收”的问题。
BT 的跟随准备预算只在路线完成且 FollowRoute 已发出后开始，不占用初始定位等待预算。

`bt_replay_03` 与修复后的 `bt_replay_04` 使用同一录包区间：自动重定位都经实际 GICP/融合确认，
BT 的 `InitialLocalization` 成功并进入 `ComputeRouteOnce`。修复后无原子对时间顺序拒收，
45 秒回放收到 1891 个融合位姿及 1891 个类型化导航状态，约 45 Hz。
**连续性仍未验收**：最长输出间隔约 0.30 秒，仍有源龄相关的可用/等待转换。
`report.json` 分别记录初始化与连续性结果，`passed` 不再以“有输出”代替连续性通过。
本入口未启动局部地图、SCAN 或运动链，BT 跟随阶段会等待地图并按预算超时；不代表整链导航成功。
回放结束后 RViz 保留最后一帧供检查，关闭窗口会清理此入口拥有的全部进程。

Web 启动、设置初始位姿和停止；Foxglove 只看地图、实时扫描、位姿与轨迹。**不启动 Nav2、不申请 SDK 控制权、不发送速度、姿态或急停解除命令。仅 Zenoh，不使用 Fast DDS。**

## 2026-10-03：实时导航状态整改（开发包，未达到连续性验收）

要求分开度量：输出更新间隔、节点计算耗时、源数据龄及下游实际接收/使用时的数据龄。
50 Hz 定时器不等于 20 ms 最坏延迟，也不代表获得了 50 Hz 的新激光观测。
不再增加一个把同源 IMU/LIO 当作独立观测的 EKF；Faster-LIO 已进行惯性/激光状态估计，
robot_localization 保留全局融合职责。参考 [robot_localization 官方状态估计配置](https://github.com/cra-ros-pkg/robot_localization/blob/ros2/doc/state_estimation_nodes.rst)。

开发链路：真实扫描末端后验和 IMU → `RealtimeNavigationOutput` 中的因果预测 →
连续局部 odom；原生地图匹配 → 私有全局 EKF → 有限速率的 map 校正 →
同时间 local/global 原子状态 → BT/下游。地图校正不控制高频输出节拍。
预测与导航输出是分离的纯算法核心，只共用一个串行 ROS 调度所有者，去掉一次进程间
转发和等频定时器相位差；全局搜索、匹配、EKF 不在这个高频节点求解。

改动及边界：

- 新入口 `realtime_navigation_output` 取代开发 launch 内独立的 predictor/output 两节点；禁止同时运行两套预测器或公共 TF 写者。旧封存节点与传播核心不改。
- `config/realtime_navigation.yaml` 记录 50 Hz 调度和 350 ms 后验预测上限，复制进入新会话并记录哈希；350 ms 不是延长 IMU 有效期。仍保留 100 ms IMU/coast 上限、真实积分缺口、旋转、不确定度、epoch 和故障限制；现有消费者后验源龄上限仍为 400 ms。
- 原生 LIO 的非故障 `valid=false/reason=tracking` 扫描过期通知，只在同 epoch、`last_admitted_scan` 且扫描末端身份与保留后验一致时保留原后验。不能更新源时间、接收时间或续期。过期仍停止预测；退化、重置、传感器故障和未知状态立即失效。
- 仅使用不晚于目标时刻的真实 IMU，稍晚样本留到下一拍。没有填造 IMU、发布零运动观测、重复旧姿态刷新租约或修改未验收标志。
- 输出目标先选为不晚于 ROS 整数时钟、经浮点核心编解码仍不向前漂移的可表示时刻，再进行积分。当前 epoch 最多相差 1791 ns；不是给旧姿态重新盖接收时间。该措施消除浮点往返造成的几百纳秒“未来样本”误拒收，未放宽任何未来时间判断。
- `/d1max/localization/navigation/realtime` 提供有限长度统计：计算耗时、调度间隔、超期计数、后验/IMU 原始源龄和 LIO 通知。`posterior_processing_sec` 是历史字段名，实际为后验源时到发布的总龄，包含输入延迟；不能据此声称测得纯 LIO 求解耗时。
- 隔离回放增加原生 LIO 事件与真实 BT 输入诊断，不改变 BT 的接纳条件。SIGTERM/取消/ROS 关闭异常不再跳过该入口拥有的子进程清理。

同包同 45 秒区间、相同地图的对照（`bt_replay_04` → `bt_replay_09_causal_wire`）：

| 指标 | 分离旧链路 | 合并调度＋有界后验衔接 |
|---|---:|---:|
| 类型化状态数量 | 1891 | 2049 |
| 输出平均频率 | 45.04 Hz | 48.86 Hz |
| 最长输出间隔 | 299 ms | 199 ms |
| 5 Hz 诊断采样的不可用转换 | 25 次 | 1 次 |

新链路更新间隔 P50/P95/P99 约 20.0/22.0/38.8 ms，计算耗时 P95/P99 约
5.7/6.8 ms、最大 9.6 ms，未超过 20 ms 计算预算。真正的下游接收间隔 P95 约 24.5 ms，
不能拿节点计算耗时代替端到端更新间隔。这是本机短回放测量，不是硬实时或 NUC 验收。
5 Hz 诊断没有记录每一次短暂停顿，须同时查看状态间隔；本次仍有 16 个状态间隔超过 40 ms。
**整段连续性仍失败，不能凭接近 50 Hz 宣称满足要求。**

独立只读审计原始包发现：前雷达 IMU 源间隔中位数约 5 ms，但接收间隔 P99 约
90.3 ms、最大 116.3 ms；前后雷达的源帧间隔最大约 400 ms、记录接收间隔约 401 ms。
因此“200 Hz 采样”不等于每 5 ms 能交付一次数据。记录缺帧的位置不能靠新的滤波器补回。
这还不能区分驱动丢包、传输拥塞与录制漏收，须在实机的驱动/传输/接收/记录各边界对照源序号和时间。
中心 IMU 录制接收间隔最大约 93.5 ms，没有超过 100 ms 的接收空窗；但其共享时钟、
外参和作为 LIO 输入的动态质量未在本轮验收，不自动替换或混用偏置。

`bt_replay_08_realtime_evidence` 捕获 212 个“未来样本”误拒收，最差只超前 639 ns，
确认为浮点往返精度问题，不是毫秒级传感器时钟错误。修复后的 `bt_replay_09_causal_wire`
未来拒收为零，源龄检查保持不变；仍有 223 个 BT 诊断使用时刻超过 IMU 源龄上限，
以及 9 个抵达时过龄的状态。它们需要同时核对输入成批交付和上一状态的龄。
最长断档处仍是后验源龄超过 350 ms（原包中前后雷达均存在约 400 ms 帧间隔）。
真实机器人时钟/网络延迟上界尚未标定，不能放宽所有消费者来掩盖剩余输入问题。

复现（输出目录须不存在或为空；仅本机隔离 Zenoh，无 SDK/运动）：

```bash
bash tools/validation/bt_localization_replay_entry.sh \
  --bag bags/slam_raw_20260923_010248_eb79d8 \
  --output experiments/global_relocalization_20261002/realtime_reproduce \
  --seconds 45 --timing-backend integrated
```

`--timing-backend split` 可单独复现 250 ms 分离节点对照。
新报告保存源/接收更新间隔、IMU/后验源龄、原始输入审计、原生 LIO 通知、BT 使用时龄及清理结果。
初始化成功与连续性通过分别判定；没有启动局部规划/控制，不能拿本回放冒充导航到达。
开发 install 在 `experiments/global_relocalization_20261002/install`；正式发布入口仍核验
原封存 563 文件，未自动替换、部署、连接 SDK 或 push。下一步必须先解决/验收原始采集交付，
然后在同一 BT 输入上重新测量连续性、最坏间隔和端到端数据龄。
定位包 19 个 CTest 目标及 BT 4 个 CTest 目标通过，覆盖因果样本、原后验到期、故障/重置不保留、
IMU/旋转边界、时间戳精度往返和原始包只读审计；组件测试通过不代表实机连续定位通过。

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

## 可选的原始双雷达射线元数据

`perception_rays.enabled` 默认关闭；独立保留每点的雷达来源、原点与采集时间，不改变现有 LIO 输入。接口与限制见 [PERCEPTION_RAYS.md](PERCEPTION_RAYS.md)。该支路尚未去畸变或接入 SCAN，不能据此宣称地面误阻塞已经解决。
